"""模型计算线程模块
包含高斯模型计算的线程实现
"""
import os
import math
import json
import tempfile
import traceback
from PyQt5.QtCore import QThread, pyqtSignal

from gaussian_plume_model import run_gaussian_model
from map_utils import download_baidu_static_image, overlay_contours_on_basemap

class ModelRunnerThread(QThread):
    """模型计算线程类"""
    # 定义完成信号：最终图片URL, 边界, 数据(x,y,conc), 透明覆盖层路径, 底图路径
    finished = pyqtSignal(str, dict, tuple, str, str)
    
    def __init__(self, center_lng, center_lat, wind_speed, wind_dir, 
                 calculate_bounds_func, source_radius=500.0, download_map_func=None, overlay_func=None,
                 gm_center=None, gm_radius=None, non_gm_center=None, non_gm_radius=None,
                 wind_mode='constant', temperature=None, humidity=None, pollen_factor=1.0, height_diff=3.0,
                 grid_km=1.0):
        super().__init__()
        self.center_lng = center_lng
        self.center_lat = center_lat
        self.wind_speed = wind_speed
        self.wind_dir = wind_dir
        self.source_radius = source_radius
        self.calculate_bounds = calculate_bounds_func
        # 种植区域信息
        self.gm_center = gm_center
        self.gm_radius = gm_radius
        self.non_gm_center = non_gm_center
        self.non_gm_radius = non_gm_radius
        self.wind_mode = wind_mode
        # 新增环境与源参数
        self.temperature = temperature
        self.humidity = humidity
        self.pollen_factor = float(pollen_factor) if pollen_factor is not None else 1.0
        self.height_diff = float(height_diff) if height_diff is not None else 3.0
        # 使用传入的函数或默认使用导入的函数
        self.download_map = download_map_func if download_map_func else download_baidu_static_image
        self.overlay_contours = overlay_func if overlay_func else overlay_contours_on_basemap
        # 动态网格边长（公里）
        try:
            self.grid_km = float(grid_km)
        except Exception:
            self.grid_km = 1.0

    def run(self):
        """运行高斯模型并生成结果"""
        try:
            print("[模型线程] 开始运行")
            # 根据传入的grid_km计算边界
            bounds = self.calculate_bounds(self.center_lng, self.center_lat, self.grid_km, self.grid_km)
            print(f"[模型线程] 计算边界: {bounds}")
            
            # 计算转基因区域相对于地图中心（非转基因区域）的偏移
            gm_offset_x = 0.0  # 默认偏移
            gm_offset_y = 0.0
            
            if self.gm_center and self.non_gm_center:
                import math
                # 计算经纬度差值并转换为米
                gm_lat = float(self.gm_center.get('lat', 0.0))
                meters_per_deg_lat = 111000.0
                meters_per_deg_lng = 111000.0 * math.cos(math.radians(gm_lat)) if gm_lat else 111000.0
                
                d_lng = float(self.gm_center['lng']) - float(self.non_gm_center['lng'])
                d_lat = float(self.gm_center['lat']) - float(self.non_gm_center['lat'])
                gm_offset_x = d_lng * meters_per_deg_lng
                gm_offset_y = d_lat * meters_per_deg_lat
                
                print(f"[模型线程] 转基因区域偏移: ({gm_offset_x:.2f}, {gm_offset_y:.2f}) 米")
            
            # 1. 运行高斯模型 (GM区域作为源)
            fig, x, y, conc, overlay_path = run_gaussian_model(
                wind_speed_value=self.wind_speed,
                wind_dir_value=self.wind_dir,
                grid_km=self.grid_km, dxy=10, source_radius=self.source_radius,
                gm_offset_x=gm_offset_x, gm_offset_y=gm_offset_y,
                wind_mode=self.wind_mode,
                temperature_value=self.temperature,
                humidity_value=self.humidity,
                pollen_factor=self.pollen_factor,
                height_diff=self.height_diff
            )
            
            # 定义 final_url (用于信号发射)
            final_url = overlay_path
            
            # 2. 获取 Non-GM 区域扩散数据 (用于导出)
            # 优化：如果非转基因区域大小与转基因区域一致，直接复用结果
            recipient_data = {}
            target_radius = self.non_gm_radius if self.non_gm_radius else 50.0
            
            if abs(target_radius - self.source_radius) < 0.1:
                print("[模型线程] Non-GM区域大小与GM区域一致，复用扩散数据 (跳过二次计算)")
                recipient_data = {
                    "x": x,
                    "y": y,
                    "conc": conc
                }
            else:
                print("[模型线程] Non-GM区域大小不同，开始独立计算 Non-GM 区域扩散数据...")
                _, x_rec, y_rec, conc_rec, _ = run_gaussian_model(
                    wind_speed_value=self.wind_speed,
                    wind_dir_value=self.wind_dir,
                    grid_km=self.grid_km, dxy=10, 
                    source_radius=target_radius, 
                    gm_offset_x=0.0, gm_offset_y=0.0, # 居中
                    wind_mode=self.wind_mode,
                    temperature_value=self.temperature,
                    humidity_value=self.humidity,
                    pollen_factor=self.pollen_factor, 
                    height_diff=self.height_diff
                )
                recipient_data = {
                    "x": x_rec,
                    "y": y_rec,
                    "conc": conc_rec
                }
            
            if not overlay_path or not os.path.exists(overlay_path):
                raise ValueError("模型计算失败，未返回有效结果")
                
            # 下载底图 - 使用模拟的实际中心点（非转基因区域中心）
            # 不要从bounds重新计算中心，避免浮点误差导致的对齐问题
            basemap_path = self.download_map(self.center_lng, self.center_lat, bounds=bounds)
            
            if not basemap_path or not os.path.exists(basemap_path):
                raise ValueError("底图下载失败")
            
            # Debug: 输出中心点信息用于验证对齐
            print(f"[模型线程] 底图中心: ({self.center_lng}, {self.center_lat})")
            print(f"[模型线程] Bounds: SW({bounds['sw']['lng']:.6f}, {bounds['sw']['lat']:.6f}), NE({bounds['ne']['lng']:.6f}, {bounds['ne']['lat']:.6f})")
            if self.gm_center:
                print(f"[模型线程] 转基因中心: ({self.gm_center['lng']:.6f}, {self.gm_center['lat']:.6f}), 半径{self.gm_radius}m")
            if self.non_gm_center:
                print(f"[模型线程] 非转基因中心: ({self.non_gm_center['lng']:.6f}, {self.non_gm_center['lat']:.6f}), 半径{self.non_gm_radius}m")
                
            # 叠加图层
            final_path = self.overlay_contours(basemap_path, overlay_path)
            
            # 在最终图像上绘制种植区域矩形
            if (self.gm_center and self.gm_radius) or (self.non_gm_center and self.non_gm_radius):
                from map_utils import draw_planting_areas_on_image
                final_path = draw_planting_areas_on_image(
                    final_path, bounds,
                    gm_center=self.gm_center, gm_radius=self.gm_radius,
                    non_gm_center=self.non_gm_center, non_gm_radius=self.non_gm_radius
                )
            
            # 发送完成信号（包含 recipient_data）
            final_url = os.path.abspath(final_path).replace("\\", "/")
            overlay_url = os.path.abspath(overlay_path).replace("\\", "/")
            basemap_url = os.path.abspath(basemap_path).replace("\\", "/")
            self.finished.emit(final_url, bounds, (x, y, conc, recipient_data), overlay_url, basemap_url)
            
            print("[模型线程] 计算完成")
            
        except Exception as e:
            print(f"[模型线程] 错误: {str(e)}")
            print(traceback.format_exc())
            self.finished.emit("", {}, (None, None, None), "", "")


class DriftCalculatorThread(QThread):
    """基因漂移概率计算线程"""
    # 信号：x_sub, y_sub, G_percent
    finished = pyqtSignal(object, object, object)
    error = pyqtSignal(str)

    def __init__(self, x_full, y_full, conc_donor_full, non_gm_radius,
                 kp, L, vd, cp, wind_speed, wind_dir, grid_km, dxy, pollen_factor):
        super().__init__()
        self.x_full = x_full
        self.y_full = y_full
        self.conc_donor_full = conc_donor_full
        self.non_gm_radius = non_gm_radius
        self.kp = kp
        self.L = L
        self.vd = vd
        self.cp = cp
        self.wind_speed = wind_speed
        self.wind_dir = wind_dir
        self.grid_km = grid_km
        self.dxy = dxy
        self.pollen_factor = pollen_factor

    def run(self):
        try:
            print("[漂移线程] 开始计算...")
            import drift_model
            x_sub, y_sub, G_percent = drift_model.calculate_drift_probability(
                self.x_full, self.y_full, self.conc_donor_full,
                self.non_gm_radius,
                self.kp, self.L, self.vd, self.cp,
                self.wind_speed, self.wind_dir, self.grid_km,
                dxy=self.dxy,
                pollen_factor=self.pollen_factor,
                return_full_grid=True  # 总是返回完整网格以便绘制全局图
            )
            print("[漂移线程] 计算完成")
            self.finished.emit(x_sub, y_sub, G_percent)
        except Exception as e:
            print(f"[漂移线程] 错误: {e}")
            self.error.emit(str(e))

