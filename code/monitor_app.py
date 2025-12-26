"""监测系统主窗口模块
包含主窗口UI、地图交互、数据展示和模型可视化的实现
"""
import os
import csv
import datetime
import math
import json
import numpy as np
import matplotlib as mpl
# 统一 Matplotlib 中文字体与负号显示
mpl.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'WenQuanYi Micro Hei', 'DejaVu Sans', 'Arial']
mpl.rcParams['axes.unicode_minus'] = False
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
from PIL import Image
import requests
import io
import tempfile
import time
from collections import deque

from PyQt5.QtCore import Qt, QUrl, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QComboBox, QGroupBox, QGridLayout, QMessageBox,
    QFileDialog, QTabWidget, QTextEdit, QSplitter, QLineEdit, QCheckBox
)
from PyQt5.QtGui import QFont, QColor, QPalette
from PyQt5.QtWebEngineWidgets import QWebEngineView

from model_thread import ModelRunnerThread, DriftCalculatorThread
from serial_thread import SerialThread, SensorData
from hardware_serial import HardwareSerialThread, HardwareSensorData, find_ch340_ports
from map_utils import download_baidu_static_image, overlay_contours_on_basemap
import drift_model  # 新增导入


class MonitorApp(QMainWindow):
    """监测系统主窗口类"""
    def __init__(self):
        super().__init__()
        self.serial_thread = None
        self.hardware_serial_thread = None  # 硬件串口线程
        self.data_log = []
        self.hardware_data_log = []  # 硬件数据日志
        self.latest_speed = 5.0
        self.latest_dir = 10.0
        self.latest_hardware_data = None  # 最新硬件数据
        self.model_map_view = None  # 地图显示组件
        self.model_canvas = None  # 模型结果画布
        
        # 动画定时器
        self.loading_timer = QTimer(self)
        self.loading_timer.timeout.connect(self.update_loading_animation)
        self.loading_dots = 0
        
        # 种植区域相关属性
        self.gm_center = None  # 转基因种植区域中心坐标
        self.gm_radius = None  # 转基因种植区域半径
        self.current_dynamic_size_km = 1.0  # 当前动态边长（公里），默认1km
        self.non_gm_center = None  # 非转基因种植区域中心坐标
        self.non_gm_radius = None  # 非转基因种植区域半径
        # 1分钟滚动平均缓冲与窗口
        self.avg_window_seconds = 60
        self.wind_samples = deque()
        
        self.init_ui()

    def update_loading_animation(self):
        self.loading_dots = (self.loading_dots + 1) % 4
        dots = "." * self.loading_dots
        
        # 更新 model tab 的 label
        try:
            if hasattr(self, 'model_status_label'):
                text = self.model_status_label.text()
                if "正在模拟" in text:
                    self.model_status_label.setText(f"正在模拟{dots}")
        except Exception:
            pass
            
        # 更新 drift tab 的 label
        try:
            if hasattr(self, 'drift_status_label'):
                drift_text = self.drift_status_label.text()
                if "正在计算热力图" in drift_text:
                    self.drift_status_label.setText(f"正在计算热力图{dots}")
        except Exception:
            pass

    def init_ui(self):
        """初始化UI界面"""
        self.setWindowTitle("风速风向温湿度监测系统（含地图 + 模型计算）")
        self.setGeometry(100, 100, 1000, 650)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        serial_layout = QHBoxLayout()
        
        # 数据源选择
        self.data_source_combo = QComboBox()
        self.data_source_combo.addItems(["模拟数据", "硬件设备"])
        self.data_source_combo.currentTextChanged.connect(self.on_data_source_changed)
        
        self.port_combo = QComboBox()
        self.port_combo.addItems([f"COM{i}" for i in range(1, 17)])
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600", "115200", "230400"])
        self.baudrate_combo.setCurrentText("115200")
        
        # 刷新端口按钮
        self.refresh_ports_btn = QPushButton("刷新端口")
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)
        
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        serial_layout.addWidget(QLabel("数据源:"))
        serial_layout.addWidget(self.data_source_combo)
        serial_layout.addWidget(QLabel("串口号:"))
        serial_layout.addWidget(self.port_combo)
        serial_layout.addWidget(QLabel("波特率:"))
        serial_layout.addWidget(self.baudrate_combo)
        serial_layout.addWidget(self.refresh_ports_btn)
        serial_layout.addWidget(self.connect_btn)
        serial_layout.addStretch()
        main_layout.addLayout(serial_layout)

        self.statusBar().showMessage("未连接")

        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)

        self.create_main_tab()
        self.create_blank_tab()
        self.create_full_model_tab()  # 添加完整模型标签页
        self.create_model_tab()



    def create_main_tab(self):
        """创建主监测数据标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
    
        html_content = """
        <!DOCTYPE html>
        <html><head><meta charset='utf-8'><title>地图</title>
        <style>html, body, #container {height: 100%; margin: 0;}</style>
        <script type="text/javascript" src="https://api.map.baidu.com/api?v=3.0&ak=sXHZZmCrjOTisFCbeJnCFFNkUbfd4nF9"></script>
        </head>
        <body><div id="container"></div>
        <script>
        var map = new BMap.Map("container");
        var point = new BMap.Point(116.404, 39.915);
        map.centerAndZoom(point, 15);
        map.enableScrollWheelZoom(true);
        var centerMarker = new BMap.Marker(point);
        map.addOverlay(centerMarker);
    
        // 通过经纬度输入定位地图中心，并更新中心标记
        window.setMapCenter = function(lng, lat) {
            try {
                var p = new BMap.Point(lng, lat);
                map.setCenter(p);
                if (centerMarker) { try { map.removeOverlay(centerMarker); } catch(e){} }
                centerMarker = new BMap.Marker(p);
                map.addOverlay(centerMarker);
                return true;
            } catch (e) {
                console.error("setMapCenter错误:", e);
                return false;
            }
        }
        </script></body></html>
        """
        with open("temp_map.html", "w", encoding="utf-8") as f:
            f.write(html_content)
    
        self.map_view = QWebEngineView()
        self.map_view.load(QUrl.fromLocalFile(os.path.abspath("temp_map.html")))
    
        self.data_group = QGroupBox("风速风向温湿度")
        data_layout = QGridLayout(self.data_group)
        self.temp_label = self.create_data_display(data_layout, "温度:", 0, 0)
        self.humi_label = self.create_data_display(data_layout, "湿度:", 0, 1)
        self.wind_speed_label = self.create_data_display(data_layout, "风速:", 1, 0)
        self.wind_dir_label = self.create_data_display(data_layout, "风向:", 1, 1)
    
        # 使用可拖动分隔条，上方为地图（含手动定位控件），下方为数据面板
        left_container_main = QWidget()
        left_vbox_main = QVBoxLayout(left_container_main)
        left_vbox_main.setContentsMargins(0, 0, 0, 0)
        left_vbox_main.setSpacing(6)
        left_vbox_main.addWidget(self.map_view, 1)
    
        coord_row = QHBoxLayout()
        coord_row.addWidget(QLabel("经度(lng):"))
        self.main_lng_input = QLineEdit()
        self.main_lng_input.setPlaceholderText("如 116.404")
        self.main_lng_input.setFixedWidth(140)
        coord_row.addWidget(self.main_lng_input)
        coord_row.addSpacing(8)
        coord_row.addWidget(QLabel("纬度(lat):"))
        self.main_lat_input = QLineEdit()
        self.main_lat_input.setPlaceholderText("如 39.915")
        self.main_lat_input.setFixedWidth(140)
        coord_row.addWidget(self.main_lat_input)
        coord_row.addSpacing(8)
        self.main_center_btn = QPushButton("定位")
        self.main_center_btn.setFixedWidth(60)
        self.main_center_btn.clicked.connect(self.center_main_map_to_input)
        coord_row.addWidget(self.main_center_btn)
        coord_row.addStretch()
        left_vbox_main.addLayout(coord_row)
    
        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(left_container_main)
        splitter.addWidget(self.data_group)
        splitter.setSizes([300, 200])
    
        layout.addWidget(splitter)
        self.tab_widget.addTab(tab, "监测数据")


    def move_planting_area_advanced(self):
        """根据高级模式（坐标或角度）移动非转基因区域（基于边界间距计算）"""
        def _do_move(areas):
            try:
                if not areas or not isinstance(areas, dict):
                    self.show_error_message("无法获取当前选择区域信息")
                    return
                gm = areas.get('gm')
                if not gm:
                    self.show_error_message("请先选择转基因区域作为基准")
                    return
                
                lat_gm = float(gm['lat'])
                lng_gm = float(gm['lng'])
                
                # 1. 获取两区域半径 (优先用真实值，否则用默认/输入值)
                try:
                    r_gm = float(gm.get('radius', 50.0))
                except:
                    r_gm = 50.0
                    
                non = areas.get('non')
                try:
                    if non:
                        r_non = float(non.get('radius', 50.0))
                    else:
                        # 尝试从输入框获取（如果非GM尚未创建，通常使用设定值）
                        input_val = self.radius_input.text() if hasattr(self, 'radius_input') else "100"
                        r_non = float(input_val) / 2.0
                except:
                    r_non = 50.0

                # 2. 计算目标中心距与位移分量
                d_x = 0.0 # 最终经度方向偏移(m)
                d_y = 0.0 # 最终纬度方向偏移(m)
                gap_dist = 0.0 # 边界间距
                center_dist = 0.0 # 中心间距
                description = ""
                
                mode_idx = self.pos_tabs.currentIndex()
                
                # 辅助函数：计算正方形在指定方向上的有效半径（中心到边界的距离）
                # angle_rad: 0=正北, 顺时针
                def get_square_effective_radius(base_r, angle_rad):
                    # 正方形边界方程：max(|x|, |y|) = base_r
                    # x = r * sin(angle), y = r * cos(angle)
                    # r * max(|sin|, |cos|) = base_r => r = base_r / max(...)
                    sin_v = math.sin(angle_rad)
                    cos_v = math.cos(angle_rad)
                    # 避免除以零
                    denominator = max(abs(sin_v), abs(cos_v))
                    if denominator < 1e-9: return base_r
                    return base_r / denominator

                if mode_idx == 0:
                    # Tab 1: 直角坐标
                    try:
                        in_x = float(self.offset_e_input.text())
                        in_y = float(self.offset_n_input.text())
                        
                        # 1. 计算Gap的大小和方向
                        gap_dist = math.sqrt(in_x**2 + in_y**2)
                        
                        if gap_dist < 1e-6:
                            angle_rad = 0.0
                            d_x, d_y = 0.0, 0.0
                            center_dist = 0.0
                            description = "重叠"
                        else:
                            # 计算方向 (0=North, Clockwise) -> atan2(x, y)
                            angle_rad = math.atan2(in_x, in_y)
                            
                            # 计算在此方向上两个正方形的有效半径
                            eff_r_gm = get_square_effective_radius(r_gm, angle_rad)
                            eff_r_non = get_square_effective_radius(r_non, angle_rad)
                            
                            # 中心距 = Gap + R1 + R2
                            center_dist = gap_dist + eff_r_gm + eff_r_non
                            
                            # 分解回 d_x, d_y (Center Offset)
                            d_x = center_dist * math.sin(angle_rad)
                            d_y = center_dist * math.cos(angle_rad)
                            
                        description = f"直角输入(Gap={gap_dist:.1f}m)"
                            
                    except ValueError:
                        self.show_error_message("坐标输入错误")
                        return
                else:
                    # Tab 2: 极坐标
                    try:
                        in_dist = float(self.polar_dist_input.text())
                        if in_dist < 0: 
                            self.show_error_message("距离不能为负")
                            return
                        angle_deg = float(self.polar_angle_input.text())
                        
                        # 1. Gap大小
                        gap_dist = in_dist
                        
                        # 2. 方向
                        angle_rad = math.radians(angle_deg)
                        
                        # 3. 计算有效半径
                        eff_r_gm = get_square_effective_radius(r_gm, angle_rad)
                        eff_r_non = get_square_effective_radius(r_non, angle_rad)
                        
                        # 4. 中心距
                        center_dist = gap_dist + eff_r_gm + eff_r_non
                        
                        d_x = center_dist * math.sin(angle_rad)
                        d_y = center_dist * math.cos(angle_rad)
                        
                        description = f"极坐标输入(Gap={gap_dist:.1f}m)"
                    except ValueError:
                        self.show_error_message("极坐标输入错误")
                        return

                # 3. 执行移动 (共享逻辑)
                delta_lat = d_y / 111000.0
                delta_lng = d_x / (111000.0 * math.cos(math.radians(lat_gm)))
                
                target_lat = lat_gm + delta_lat
                target_lng = lng_gm + delta_lng
                
                js = f"window.updateSelectionPosition('non_gm', {target_lat}, {target_lng})"
                self.run_js(js, description=description)
                
                # 4. 更新UI显示
                info_text = f"计算结果: 中心距 {center_dist:.1f}m, 边界距 {gap_dist:.1f}m"
                if mode_idx == 0:
                    self.calc_info_label_xy.setText(info_text)
                    self.calc_info_label_polar.setText("上次移动结果: 在直角坐标页操作")
                else:
                    self.calc_info_label_polar.setText(info_text)
                    self.calc_info_label_xy.setText("上次移动结果: 在极坐标页操作")
                    
                self.statusBar().showMessage(f"已移动: {description} -> {info_text}")
                
            except Exception as e:
                self.show_error_message(f"移动失败: {e}")
                import traceback
                print(traceback.format_exc())
        
        self.run_js("window.getSelectedAreas()", callback=_do_move, description="获取基准区域位置")

    def export_simulation_csv(self):
        """导出当前模拟结果和参数为CSV"""
        try:
            if not hasattr(self, 'latest_model_data') or self.latest_model_data is None:
                self.show_error_message("尚无模拟结果，请先运行模拟")
                return
            
            data = self.latest_model_data
            params = data.get('params', {})
            x = data.get('x')
            y = data.get('y')
            conc = data.get('conc')
            
            if x is None or y is None or conc is None:
                self.show_error_message("模拟结果数据不完整")
                return
            
            # 选择保存路径
            file_path, _ = QFileDialog.getSaveFileName(self, "导出模拟数据", "", "CSV Files (*.csv)")
            if not file_path:
                return
                
            with open(file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                
                # 1. 写入环境和区域参数
                writer.writerow(["=== 模拟参数记录 ==="])
                
                # 种植区域信息
                gm_center = params.get('gm_center', {}) or {}
                gm_radius = params.get('gm_radius', 0)
                non_gm_center = params.get('non_gm_center', {}) or {}
                non_gm_radius = params.get('non_gm_radius', 0)
                
                writer.writerow(["【转基因区域(花粉源)】"])
                writer.writerow(["中心经度", gm_center.get('lng', 'N/A')])
                writer.writerow(["中心纬度", gm_center.get('lat', 'N/A')])
                writer.writerow(["边长(m)", gm_radius * 2])
                
                writer.writerow(["【非转基因区域(受体)】"])
                writer.writerow(["中心经度", non_gm_center.get('lng', 'N/A')])
                writer.writerow(["中心纬度", non_gm_center.get('lat', 'N/A')])
                writer.writerow(["边长(m)", non_gm_radius * 2])
                
                # 环境参数
                writer.writerow(["【环境与源参数】"])
                writer.writerow(["风速(m/s)", params.get('wind_speed', 'N/A')])
                writer.writerow(["风向(度)", params.get('wind_dir', 'N/A')])
                writer.writerow(["风场模式", "盛行风" if params.get('wind_mode') == 'prevailing' else "恒定风"])
                writer.writerow(["温度(C)", params.get('temp', 'N/A')])
                writer.writerow(["相对湿度(%)", params.get('humidity', 'N/A')])
                writer.writerow(["源强系数(粒/m2/s)", params.get('q_factor', 'N/A')])
                writer.writerow(["高度差(m)", params.get('height_diff', 'N/A')])
                
                writer.writerow([])
                writer.writerow(["=== 花粉浓度扩散网格数据 (矩阵格式: 行=Y坐标, 列=X坐标) ==="])
                
                # 2. 写入网格数据 (矩阵形式)
                # 矩阵模式：第一行为X坐标，第一列为Y坐标
                # 为了直观对应地图，建议Y轴从上到下即从最大值到最小值（行序倒序）
                
                rows, cols = x.shape
                
                # 获取 X 轴坐标（第一行的所有列）
                x_coords = x[0, :]
                # 写入表头：[ "Y \ X" , x1, x2, ... ]
                header = ["Y \ X (m)"] + [f"{val:.2f}" for val in x_coords]
                writer.writerow(header)
                
                # 逐行写入数据 (GM)
                # 原数据 row 0 是 y_min (底部)。为了在Excel中看起来像地图(上北下南)，我们从最后一行(y_max)开始写
                for r in range(rows - 1, -1, -1):
                    y_val = y[r, 0]
                    # 构建当前行: [y_val, conc_r_0, conc_r_1, ...]
                    row_data = [f"{y_val:.2f}"] + [f"{val:.4e}" for val in conc[r, :]]
                    writer.writerow(row_data)
                
                # 3. 写入 Non-GM 区域扩散网格数据 (如果存在)
                recipient_data = data.get('recipient_data')
                if recipient_data:
                    writer.writerow([])
                    writer.writerow([])
                    writer.writerow(["=== 非转基因区域(受体) 花粉浓度扩散网格数据 (矩阵格式) ==="])
                    writer.writerow(["此数据系假设非转基因区域同样作为花粉源进行模拟的结果"])
                    
                    xr = recipient_data.get('x')
                    yr = recipient_data.get('y')
                    cr = recipient_data.get('conc')
                    
                    if xr is not None and yr is not None and cr is not None:
                        rx_coords = xr[0, :]
                        r_header = ["Y \ X (m)"] + [f"{val:.2f}" for val in rx_coords]
                        writer.writerow(r_header)
                        
                        r_rows, r_cols = xr.shape
                        for r in range(r_rows - 1, -1, -1):
                            ry_val = yr[r, 0]
                            r_row_data = [f"{ry_val:.2f}"] + [f"{val:.4e}" for val in cr[r, :]]
                            writer.writerow(r_row_data)
            
            self.statusBar().showMessage(f"模拟数据已导出至: {file_path}")
            QMessageBox.information(self, "导出成功", f"模拟数据已成功导出！\n路径: {file_path}")
            
        except Exception as e:
            self.show_error_message(f"导出失败: {e}")
            import traceback
            print(traceback.format_exc())

    def setup_model_map(self):
        """设置模型地图，修复JS返回值并将默认框选改为 1km x 1km，同时提供 addModelImageOverlay 接口"""
        html_content = f"""
         <!DOCTYPE html>
         <html>
         <head>
             <meta charset="utf-8">
             <title>模型区域选择</title>
             <style>
                 html, body, #container {{ width: 100%; height: 100%; margin: 0; }}
                 #info {{ position: absolute; top: 10px; left: 10px; background: white; padding: 5px; z-index: 100; }}
                 #debug {{ position: absolute; bottom: 10px; left: 10px; right:10px; background: rgba(255,255,255,0.9); padding: 5px; z-index: 100; color: red; font-size:12px; }}
             </style>
             <script type="text/javascript" src="https://api.map.baidu.com/api?v=3.0&ak=sXHZZmCrjOTisFCbeJnCFFNkUbfd4nF9"></script>
         </head>
         <body>
             <div id="info">拖动地图到目标位置，点击"框选区域"按钮创建1km×1km范围</div>
             <div id="debug">调试信息：未执行操作</div>
             <div id="container"></div>
             <script>
                 var map = new BMap.Map("container");
                 var centerPoint = new BMap.Point(116.404, 39.915); // 默认北京
         
                 function initMap() {{
                     try {{
                         map.centerAndZoom(centerPoint, 15);
                         map.enableScrollWheelZoom(true);
                         map.setMapStyleV2({{styleId: "2b2e585dec698fa62f732b976f0ea1ac"}});
                         document.getElementById("debug").textContent = "地图初始化成功，已应用干净样式";
                     }} catch (e) {{
                         document.getElementById("debug").textContent = "地图初始化失败: " + e.message;
                         console.error("地图初始化错误:", e);
                     }}
                 }}
                 initMap();
         
                 // 1km 对应的经度/纬度近似度数
                 var km1InDegree = 1.0 / 111.0;
                 var mainRectangle = null;  // 重命名避免与种植区域矩形冲突
                 var centerMarker = null;
                 var currentCenter = centerPoint;
                 
                 // 初始化种植区域矩形变量
                 window._gmRectangle = null;
                 window._nonGmRectangle = null;
                 console.log('种植区域矩形变量已初始化');
         
                 function calculateRectangle(center) {{
                     var lat = center.lat;
                     var lng = center.lng;
                     var lngAdjust = km1InDegree / Math.cos(lat * Math.PI / 180);
                     var p1 = new BMap.Point(lng - lngAdjust/2, lat + km1InDegree/2);
                     var p2 = new BMap.Point(lng + lngAdjust/2, lat - km1InDegree/2);
                     return new BMap.Bounds(p1, p2);
                 }}
         
                 function updateRectangle(center) {{
                     try {{
                         if (mainRectangle) map.removeOverlay(mainRectangle);
                         if (centerMarker) map.removeOverlay(centerMarker);
         
                         centerMarker = new BMap.Marker(center);
                         map.addOverlay(centerMarker);
         
                         var bounds = calculateRectangle(center);
                         var sw = bounds.getSouthWest();
                         var ne = bounds.getNorthEast();
                         var nw = new BMap.Point(sw.lng, ne.lat);
                         var se = new BMap.Point(ne.lng, sw.lat);
                         mainRectangle = new BMap.Polygon([
                             sw,
                             nw,
                             ne,
                             se
                         ], {{strokeColor:"blue", strokeWeight:2, strokeOpacity:0.8, fillOpacity:0.1}});
                         map.addOverlay(mainRectangle);
                         currentCenter = center;
                         map.setViewport(bounds);
         
                         document.getElementById("debug").textContent =
                             "区域创建成功: " + center.lng.toFixed(6) + ", " + center.lat.toFixed(6);
                         return center;
                     }} catch (e) {{
                         document.getElementById("debug").textContent = "区域创建失败: " + e.message;
                         console.error("区域创建错误:", e);
                         return null;
                     }}
                 }}
         
                 // 返回纯 JSON（number），保证 PyQt 能序列化
                 window.createRectangleAtCenter = function() {{
                     try {{
                         var center = map.getCenter();
                         if (!center) {{
                             throw new Error("无法获取地图中心");
                         }}
                         updateRectangle(center);
                         var result = {{lng: parseFloat(center.lng), lat: parseFloat(center.lat)}};
                         console.log("返回中心坐标(JSON):", result);
                         document.getElementById("debug").textContent =
                             "返回中心坐标(JSON): " + result.lng.toFixed(6) + ", " + result.lat.toFixed(6);
                         return result;
                     }} catch (e) {{
                         document.getElementById("debug").textContent = "createRectangleAtCenter错误: " + e.message;
                         console.error("createRectangleAtCenter错误:", e);
                         return null;
                     }}
                 }}
         
                 window.getModelAreaCenter = function() {{
                     var center = map.getCenter();
                     updateRectangle(center);
                     var result = {{lng: parseFloat(center.lng), lat: parseFloat(center.lat)}};
                     return result;
                 }}
         
                 /* ------- 新增：通过输入定位地图中心 ------- */
                 window.setMapCenter = function(lng, lat) {{
                     try {{
                         var point = new BMap.Point(lng, lat);
                         map.setCenter(point);
                         updateRectangle(point); // 更新矩形
                         document.getElementById("debug").textContent = "已通过输入定位到: " + lng.toFixed(6) + ", " + lat.toFixed(6);
                         return true;
                     }} catch (e) {{
                         document.getElementById("debug").textContent = "setMapCenter错误: " + e.message;
                         console.error("setMapCenter错误:", e);
                         return false;
                     }}
                 }}
         
                 /* ------- 自定义图片覆盖物（根据经纬度矩形定位） ------- */
                 function ImageOverlay(bounds, imageUrl) {{
                     this._bounds = bounds; // {{sw: {{lng,lat}}, ne: {{lng,lat}}}}
                     this._imageUrl = imageUrl;
                 }}
                 ImageOverlay.prototype = new BMap.Overlay();
                 ImageOverlay.prototype.initialize = function(map) {{
                     this._map = map;
                     var div = document.createElement("div");
                     div.style.position = "absolute";
                     div.style.pointerEvents = "none";
                     var img = document.createElement("img");
                     img.src = this._imageUrl;
                     img.style.position = "absolute";
                     img.style.left = "0px";
                     img.style.top = "0px";
                     img.style.width = "100%";
                     img.style.height = "100%";
                     img.style.opacity = "0.85";
                     div.appendChild(img);
                     this._div = div;
                     map.getPanes().floatPane.appendChild(div);
                     return div;
                 }};
                 ImageOverlay.prototype.draw = function() {{
                     var sw = new BMap.Point(this._bounds.sw.lng, this._bounds.sw.lat);
                     var ne = new BMap.Point(this._bounds.ne.lng, this._bounds.ne.lat);
                     var swPixel = this._map.pointToOverlayPixel(sw);
                     var nePixel = this._map.pointToOverlayPixel(ne);
                     var left = Math.min(swPixel.x, nePixel.x);
                     var top = Math.min(swPixel.y, nePixel.y);
                     var width = Math.abs(nePixel.x - swPixel.x);
                     var height = Math.abs(nePixel.y - swPixel.y);
                     this._div.style.left = left + "px";
                     this._div.style.top = top + "px";
                     this._div.style.width = width + "px";
                     this._div.style.height = height + "px";
                 }};
         
                 // 将图片按给定经纬度矩形加载到地图
                 window.addModelImageOverlay = function(imageUrl, sw, ne) {{
                     try {{
                         if (window._modelImageOverlay) {{
                             try {{ map.removeOverlay(window._modelImageOverlay); }} catch(e){{}}
                             window._modelImageOverlay = null;
                         }}
                         var bounds = {{ sw: {{lng: sw.lng, lat: sw.lat}}, ne: {{lng: ne.lng, lat: ne.lat}} }};
                         window._modelImageOverlay = new ImageOverlay(bounds, imageUrl);
                         map.addOverlay(window._modelImageOverlay);
                         document.getElementById("debug").textContent = "已加载模型图片（大小与位置已对齐）";
                         return true;
                     }} catch (e) {{
                         document.getElementById("debug").textContent = "addModelImageOverlay错误: " + e.message;
                         console.error("addModelImageOverlay错误:", e);
                         return false;
                     }}
                 }}

                 // ===== 选择逻辑：支持左键点击为主，右键为辅；容器像素坐标作为兜底 =====
                 // 禁用默认右键菜单，避免拦截地图事件
                 document.addEventListener('contextmenu', function(ev){{ ev.preventDefault(); return false; }});
                 
                 // 选择状态与接口
                 window._selectionMode = null; // 'gm' 或 'non_gm'
                 window._selectionRadius = 50.0;
                 window._gmSelection = null;
                 window._nonGmSelection = null;

                 window.setSelectionRadius = function(r){{
                     try {{ window._selectionRadius = (typeof r === 'number' && r > 0) ? r : 50.0; return true; }} catch(e){{ return false; }}
                 }};
                 
                 window.setSelectionMode = function(mode){{
                     window._selectionMode = mode;
                     var modeName = (mode === 'gm') ? '转基因区域(红)' : ((mode === 'non_gm') ? '非转基因区域(蓝)' : '无');
                     document.getElementById('debug').textContent = '当前框选模式: ' + modeName + '。请在地图上右键框选。';
                 }};

                 window.resetSelection = function(){{
                     try {{
                         window._selectionMode = null;
                         window._gmSelection = null;
                         window._nonGmSelection = null;
                         if (window._gmRectangle) {{ try {{ map.removeOverlay(window._gmRectangle); }} catch(e){{}} window._gmRectangle = null; }}
                         if (window._nonGmRectangle) {{ try {{ map.removeOverlay(window._nonGmRectangle); }} catch(e){{}} window._nonGmRectangle = null; }}
                         document.getElementById('debug').textContent = '已重置选择。请点击上方按钮激活框选模式。';
                         return true;
                     }} catch(e){{ return false; }}
                 }};

                window.updateSelectionPosition = function(mode, lat, lng){{
                    try {{
                        var center = new BMap.Point(lng, lat);
                        var r = 50.0;
                        if (mode === 'gm' && window._gmSelection) r = window._gmSelection.radius;
                        if (mode === 'gm' && window._gmSelection == null) r = window._selectionRadius || 50.0;
                        
                        if (mode === 'non_gm' && window._nonGmSelection) r = window._nonGmSelection.radius;
                        if (mode === 'non_gm' && window._nonGmSelection == null) r = window._selectionRadius || 50.0;

                        if (mode === 'gm') {{
                            if (window._gmRectangle) {{ try {{ map.removeOverlay(window._gmRectangle); }} catch(ex){{}} window._gmRectangle = null; }}
                            var rect = createAreaRectangle(center, r, 'red');
                            map.addOverlay(rect);
                            window._gmRectangle = rect;
                            window._gmSelection = {{ lng: parseFloat(lng), lat: parseFloat(lat), radius: parseFloat(r) }};
                            document.getElementById('debug').textContent = '已对齐转基因区域位置。';
                        }} else if (mode === 'non_gm') {{
                            if (window._nonGmRectangle) {{ try {{ map.removeOverlay(window._nonGmRectangle); }} catch(ex){{}} window._nonGmRectangle = null; }}
                            var rect = createAreaRectangle(center, r, 'blue');
                            map.addOverlay(rect);
                            window._nonGmRectangle = rect;
                            window._nonGmSelection = {{ lng: parseFloat(lng), lat: parseFloat(lat), radius: parseFloat(r) }};
                            document.getElementById('debug').textContent = '已对齐非转基因区域位置。';
                        }}
                        return true;
                    }} catch(err) {{
                        console.error(err);
                        return false;
                    }}
                }};
                 
                 window.getSelectedAreas = function(){{
                     try {{
                         var gm = window._gmSelection ? {{lng: parseFloat(window._gmSelection.lng), lat: parseFloat(window._gmSelection.lat), radius: parseFloat(window._gmSelection.radius)}} : null;
                         var non = window._nonGmSelection ? {{lng: parseFloat(window._nonGmSelection.lng), lat: parseFloat(window._nonGmSelection.lat), radius: parseFloat(window._nonGmSelection.radius)}} : null;
                         return {{ gm: gm, non: non }};
                     }} catch(e){{ return {{ gm: null, non: null }}; }}
                 }};

                 function createAreaRectangle(center, radius, color){{
                     var sideLength = radius * 2;
                     var latOffset = sideLength / 111000.0;
                     var lngOffset = sideLength / (111000.0 * Math.cos(center.lat * Math.PI / 180));
                     var sw = new BMap.Point(center.lng - lngOffset/2, center.lat - latOffset/2);
                     var ne = new BMap.Point(center.lng + lngOffset/2, center.lat + latOffset/2);
                     var nw = new BMap.Point(sw.lng, ne.lat);
                     var se = new BMap.Point(ne.lng, sw.lat);
                     return new BMap.Polygon([sw, nw, ne, se], {{
                         strokeColor: color,
                         strokeWeight: 4,
                         strokeOpacity: 1.0,
                         fillColor: color,
                         fillOpacity: 0.35
                     }});
                 }}

                 // 选择防抖
                 window._selectionGuard = false;
                 function performSelection(center, source){{
                     try {{
                         if (window._selectionGuard) return;
                         window._selectionGuard = true;
                         setTimeout(function(){{ window._selectionGuard = false; }}, 200);
                         
                         if (!window._selectionMode) {{
                             document.getElementById('debug').textContent = '请先点击上方按钮激活框选模式！';
                             return;
                         }}

                         var r = window._selectionRadius || 500.0;
                         
                         if (window._selectionMode === 'gm') {{
                             if (window._gmRectangle) {{ try {{ map.removeOverlay(window._gmRectangle); }} catch(ex){{}} window._gmRectangle = null; }}
                             var rect = createAreaRectangle(center, r, 'red');
                             map.addOverlay(rect);
                             window._gmRectangle = rect;
                             window._gmSelection = {{ lng: parseFloat(center.lng), lat: parseFloat(center.lat), radius: parseFloat(r) }};
                             document.getElementById('debug').textContent = '已更新转基因区域（红）。';
                         }} else if (window._selectionMode === 'non_gm') {{
                             if (window._nonGmRectangle) {{ try {{ map.removeOverlay(window._nonGmRectangle); }} catch(ex){{}} window._nonGmRectangle = null; }}
                             var rect2 = createAreaRectangle(center, r, 'blue');
                             map.addOverlay(rect2);
                             window._nonGmRectangle = rect2;
                             window._nonGmSelection = {{ lng: parseFloat(center.lng), lat: parseFloat(center.lat), radius: parseFloat(r) }};
                             document.getElementById('debug').textContent = '已更新非转基因区域（蓝）。';
                         }}
                     }} catch(err) {{
                         document.getElementById('debug').textContent = '选择失败: ' + err.message;
                     }}
                 }}

                 // 显式开启地图点击事件
                 if (map.enableMapClick) {{ map.enableMapClick(true); }}
                 
                 if (map.addEventListener) {{
                     // 统一使用右键
                     map.addEventListener('rightclick', function(e){{
                         try {{
                             var center = e && e.point ? e.point : map.getCenter();
                             performSelection(center, 'rightclick');
                         }} catch(err) {{
                             document.getElementById('debug').textContent = '地图rightclick选择失败: ' + err.message;
                         }}
                     }});
                 }}

                 // 容器层点击作为后备
                 (function(){{
                     var containerEl = document.getElementById('container');
                     function selectAtPixel(ev){{
                         try {{
                             return; // 暂禁用容器点击，避免冲突
                         }} catch(err) {{
                             document.getElementById('debug').textContent = '容器click选择失败: ' + err.message;
                         }}
                     }}
                     containerEl.addEventListener('click', function(ev){{ selectAtPixel(ev); }});
                     containerEl.addEventListener('contextmenu', function(ev){{ ev.preventDefault(); selectAtPixel(ev); }});
                 }})();
             </script>
         </body>
         </html>
         """

        try:
            with open("model_map.html", "w", encoding="utf-8") as f:
                f.write(html_content)
            print("地图HTML文件生成成功")
        except Exception as e:
            print(f"生成地图HTML失败: {str(e)}")

        self.model_map_view.load(QUrl.fromLocalFile(os.path.abspath("model_map.html")))



    def create_blank_tab(self):
        """创建实时记录标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        layout.addWidget(QLabel("实时数据记录："))
        layout.addWidget(self.log_display)
        
        # 添加导出CSV按钮
        export_btn = QPushButton("导出CSV数据")
        export_btn.clicked.connect(self.export_csv)
        layout.addWidget(export_btn)
        
        self.tab_widget.addTab(tab, "实时记录")


    def create_full_model_tab(self):
        """创建完整模型运行标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
    




        # 精确位置控制 (使用 Tab 切换直角坐标与极坐标)
        # 精确位置控制 (使用 Tab 切换直角坐标与极坐标)
        # 精确位置控制 (使用 Tab 切换直角坐标与极坐标)

        



        
        # 地图和结果显示区域（使用可拖动分隔条）
        splitter = QSplitter(Qt.Horizontal)
    
        # 地图显示区域（左侧）
        left_container = QWidget()
        left_vbox = QVBoxLayout(left_container)
        left_vbox.setContentsMargins(0, 0, 0, 0)
        left_vbox.setSpacing(6)
        
        # ================= 左侧顶部：定位 Tabs =================
        self.pos_tabs = QTabWidget()
        self.pos_tabs.setFixedHeight(180) 
        # 注意：不再设置固定宽度(setFixedWidth)，使其自适应容器宽度

        # Tab 1: 直角坐标 (XY)
        tab_xy = QWidget()
        vbox_xy = QVBoxLayout(tab_xy)
        vbox_xy.setContentsMargins(10, 10, 10, 5)
        
        row_e = QHBoxLayout()
        row_e.addWidget(QLabel("向东+/西-(m):"))
        self.offset_e_input = QLineEdit("0")
        self.offset_e_input.setPlaceholderText("负数=向西")
        self.offset_e_input.setFixedWidth(200)
        row_e.addWidget(self.offset_e_input)
        row_e.addStretch()
        vbox_xy.addLayout(row_e)
        
        row_n = QHBoxLayout()
        row_n.addWidget(QLabel("向北+/南-(m):"))
        self.offset_n_input = QLineEdit("-150")
        self.offset_n_input.setPlaceholderText("负数=向南")
        self.offset_n_input.setFixedWidth(200)
        row_n.addWidget(self.offset_n_input)
        row_n.addStretch()
        vbox_xy.addLayout(row_n)
        
        # 移动按钮 (直角坐标)
        btn_layout_xy = QHBoxLayout()
        self.btn_move_xy = QPushButton("设置位置")
        self.btn_move_xy.setFixedWidth(80) 
        self.btn_move_xy.clicked.connect(self.move_planting_area_advanced)
        btn_layout_xy.addWidget(self.btn_move_xy)
        btn_layout_xy.addStretch()
        vbox_xy.addLayout(btn_layout_xy)
        
        self.calc_info_label_xy = QLabel("尚未操作")
        self.calc_info_label_xy.setStyleSheet("color: blue; font-size: 13px; font-weight: bold;") 
        vbox_xy.addWidget(self.calc_info_label_xy)
        vbox_xy.addStretch()
        
        # Tab 2: 极坐标 (距离+角度)
        tab_polar = QWidget()
        vbox_polar = QVBoxLayout(tab_polar)
        vbox_polar.setContentsMargins(10, 10, 10, 5)
        
        row_d = QHBoxLayout()
        row_d.addWidget(QLabel("边界间距:"))
        self.polar_dist_input = QLineEdit("150")
        self.polar_dist_input.setFixedWidth(200)
        row_d.addWidget(self.polar_dist_input)
        row_d.addStretch()
        vbox_polar.addLayout(row_d)
        
        row_a = QHBoxLayout()
        row_a.addWidget(QLabel("角度(0°=北，顺时针):"))
        self.polar_angle_input = QLineEdit("180")
        self.polar_angle_input.setFixedWidth(200)
        row_a.addWidget(self.polar_angle_input)
        row_a.addStretch()
        vbox_polar.addLayout(row_a)
        
        # 移动按钮 (极坐标)
        btn_layout_polar = QHBoxLayout()
        self.btn_move_polar = QPushButton("设置位置")
        self.btn_move_polar.setFixedWidth(80)
        self.btn_move_polar.clicked.connect(self.move_planting_area_advanced)
        btn_layout_polar.addWidget(self.btn_move_polar)
        btn_layout_polar.addStretch()
        vbox_polar.addLayout(btn_layout_polar)
        
        self.calc_info_label_polar = QLabel("尚未操作")
        self.calc_info_label_polar.setStyleSheet("color: blue; font-size: 13px; font-weight: bold;")
        vbox_polar.addWidget(self.calc_info_label_polar)
        vbox_polar.addStretch()
        
        self.pos_tabs.addTab(tab_xy, "直角坐标")
        self.pos_tabs.addTab(tab_polar, "极坐标")
        
        # 将Tabs添加到左侧布局的最上方
        left_vbox.addWidget(self.pos_tabs)
        
        self.model_map_view = QWebEngineView()
        # 关闭右键菜单以避免干扰百度地图的 rightclick 事件
        try:
            self.model_map_view.setContextMenuPolicy(Qt.NoContextMenu)
        except Exception:
            pass
        self.setup_model_map()
        left_vbox.addWidget(self.model_map_view, 1)
    
        # 参数设置区域 (网格布局)
        params_grid = QGridLayout()
        params_grid.setContentsMargins(0, 10, 0, 0)
        params_grid.setSpacing(8)
        
        # Row 0: 经纬度定位
        params_grid.addWidget(QLabel("经度:"), 0, 0)
        self.lng_input = QLineEdit()
        self.lng_input.setPlaceholderText("116.404")
        self.lng_input.setFixedWidth(120)
        params_grid.addWidget(self.lng_input, 0, 1)
        
        params_grid.addWidget(QLabel("纬度:"), 0, 2)
        self.lat_input = QLineEdit()
        self.lat_input.setPlaceholderText("39.915")
        self.lat_input.setFixedWidth(120)
        params_grid.addWidget(self.lat_input, 0, 3)
        
        self.center_btn = QPushButton("定位")
        self.center_btn.setFixedWidth(60)
        self.center_btn.clicked.connect(self.center_map_to_input)
        params_grid.addWidget(self.center_btn, 0, 4)
        
        # Row 1: 种植区域边长 & 风速
        params_grid.addWidget(QLabel("边长(m):"), 1, 0)
        self.radius_input = QLineEdit("100")
        self.radius_input.setFixedWidth(120)
        params_grid.addWidget(self.radius_input, 1, 1)
        
        params_grid.addWidget(QLabel("风速(m/s):"), 1, 2)
        self.wind_speed_input = QLineEdit()
        self.wind_speed_input.setPlaceholderText("传感器/空")
        self.wind_speed_input.setFixedWidth(120)
        params_grid.addWidget(self.wind_speed_input, 1, 3)

        # Row 2: 风向 & 风场模式
        params_grid.addWidget(QLabel("风向(0°=正北风，顺时针):"), 2, 0)
        self.wind_dir_input = QLineEdit()
        self.wind_dir_input.setPlaceholderText("传感器/空")
        self.wind_dir_input.setFixedWidth(120)
        params_grid.addWidget(self.wind_dir_input, 2, 1)
        
        params_grid.addWidget(QLabel("风模式:"), 2, 2)
        self.wind_mode_combo = QComboBox()
        self.wind_mode_combo.addItems(["恒定风", "盛行风"])
        self.wind_mode_combo.setCurrentText("恒定风")
        self.wind_mode_combo.setFixedWidth(120)
        params_grid.addWidget(self.wind_mode_combo, 2, 3)
        
        # Row 3: 温度 & 湿度
        params_grid.addWidget(QLabel("温度(°C):"), 3, 0)
        self.temp_input = QLineEdit()
        self.temp_input.setPlaceholderText("传感器/空")
        self.temp_input.setFixedWidth(120)
        params_grid.addWidget(self.temp_input, 3, 1)
        
        params_grid.addWidget(QLabel("湿度(%):"), 3, 2)
        self.humi_input = QLineEdit()
        self.humi_input.setPlaceholderText("传感器/空")
        self.humi_input.setFixedWidth(120)
        params_grid.addWidget(self.humi_input, 3, 3)
        
        # Row 4: 源强 & 高度差
        params_grid.addWidget(QLabel("源强Q:"), 4, 0)
        self.q_factor_input = QLineEdit()
        self.q_factor_input.setPlaceholderText("默认5e6")
        self.q_factor_input.setFixedWidth(120)
        params_grid.addWidget(self.q_factor_input, 4, 1)
        
        params_grid.addWidget(QLabel("高度差(m):"), 4, 2)
        self.height_diff_input = QLineEdit("3.0")
        self.height_diff_input.setPlaceholderText("默认3.0")
        self.height_diff_input.setFixedWidth(120)
        params_grid.addWidget(self.height_diff_input, 4, 3)

        left_vbox.addLayout(params_grid)
        left_vbox.addStretch() # 让地图占用剩余空间

        # 逻辑：半径变更时同步到地图
        try:
            def _on_radius_changed(text):
                try:
                    side_len = float(text) if text.strip() else 100.0
                    if side_len <= 0: side_len = 100.0
                    r = side_len / 2.0
                    js = f"window.setSelectionRadius({r})"
                    self.run_js(js, description="同步选择半径")
                except: pass
            self.radius_input.textChanged.connect(_on_radius_changed)
        except: pass
    
        splitter.addWidget(left_container)
    
        # 模型结果显示区域（右侧）
        self.result_container = QWidget()
        result_layout = QVBoxLayout(self.result_container)
        # ================= 右侧顶部：控制面板 =================
        right_panel = QWidget()
        right_vbox = QVBoxLayout(right_panel)
        right_vbox.setContentsMargins(5, 5, 5, 5)

        # 1. 状态行
        status_row = QHBoxLayout()
        self.model_info_label = QLabel("最新风速风向：尚未获取")
        self.model_info_label.setStyleSheet("font-weight: bold;")
        status_row.addWidget(self.model_info_label)
        
        status_row.addStretch()
        
        # 状态指示灯
        self.status_indicator = QLabel("●")
        self.status_indicator.setStyleSheet("color: red; font-size: 20px;")
        self.status_indicator.setToolTip("模拟状态")
        status_row.addWidget(self.status_indicator)
        
        self.model_status_label = QLabel("")
        self.model_status_label.setStyleSheet("color: blue; font-weight: bold;")
        status_row.addWidget(self.model_status_label)
        
        right_vbox.addLayout(status_row)

        # 2. 按钮网格
        btns_grid = QGridLayout()
        btns_grid.setSpacing(10)
        
        # Row 0
        self.reset_selection_btn = QPushButton("重置选择")
        self.reset_selection_btn.clicked.connect(self.reset_map_selection)
        btns_grid.addWidget(self.reset_selection_btn, 0, 0)
        
        self.gm_area_btn = QPushButton("框选转基因区域")
        self.gm_area_btn.clicked.connect(self.activate_gm_selection)
        btns_grid.addWidget(self.gm_area_btn, 0, 1)

        self.non_gm_area_btn = QPushButton("框选非转基因区域")
        self.non_gm_area_btn.clicked.connect(self.activate_non_gm_selection)
        btns_grid.addWidget(self.non_gm_area_btn, 0, 2)
        
        # Row 1
        self.run_model_btn = QPushButton("运行模拟")
        self.run_model_btn.setStyleSheet("font-weight: bold; background-color: #e3f2fd;")
        self.run_model_btn.clicked.connect(self.run_model_with_latest_data)
        btns_grid.addWidget(self.run_model_btn, 1, 0)
        
        self.export_sim_btn = QPushButton("导出模拟数据")
        self.export_sim_btn.clicked.connect(self.export_simulation_csv)
        btns_grid.addWidget(self.export_sim_btn, 1, 1)
        
        right_vbox.addLayout(btns_grid)
        # 不要 addStretch，让它高度自适应内容
        
        result_layout.addWidget(right_panel)

        self.canvas_container = QWidget()
        self.canvas_layout = QVBoxLayout(self.canvas_container)
        result_layout.addWidget(self.canvas_container, 1) # 图表占据剩余空间

        # 图例区域 (添加到结果图下方)
        legend_widget = QWidget()
        legend_layout = QHBoxLayout(legend_widget)
        legend_layout.setAlignment(Qt.AlignCenter)

        # GM 图例 (红色)
        gm_color = QLabel()
        gm_color.setFixedSize(20, 20)
        gm_color.setStyleSheet("background-color: red; border: 1px solid black;")
        legend_layout.addWidget(gm_color)
        legend_layout.addWidget(QLabel("转基因种植区域"))
        
        legend_layout.addSpacing(20)

        # Non-GM 图例 (蓝色)
        non_gm_color = QLabel()
        non_gm_color.setFixedSize(20, 20)
        non_gm_color.setStyleSheet("background-color: blue; border: 1px solid black;")
        legend_layout.addWidget(non_gm_color)
        legend_layout.addWidget(QLabel("非转基因种植区域"))
        
        result_layout.addWidget(legend_widget)
        splitter.addWidget(self.result_container)
    
        # 设置初始大小比例
        splitter.setSizes([500, 500])
    
        layout.addWidget(splitter)
        self.tab_widget.addTab(tab, "花粉浓度扩散")

    def reset_map_selection(self):
        """重置地图右键选择的两区域，并清空当前选择的中心与半径"""
        try:
            # 调用JS重置选择状态与矩形覆盖物
            self.run_js("window.resetSelection()", description="重置地图选择")
            # 清空当前记录的中心与半径（让后续运行前必须重新选择）
            self.gm_center = None
            self.gm_radius = None
            self.non_gm_center = None
            self.non_gm_radius = None
            # 重置按钮样式
            self.gm_area_btn.setStyleSheet("")
            self.non_gm_area_btn.setStyleSheet("")
            self.statusBar().showMessage("已重置选择：请点击上方按钮激活框选模式")
        except Exception as e:
            self.show_error_message(f"重置选择失败：{e}")
    
    def activate_gm_selection(self):
        """激活转基因种植区域框选模式"""
        try:
            # 调用JavaScript设置框选模式为GM
            self.run_js("window.setSelectionMode('gm')", description="激活GM框选模式")
            # 更新按钮样式显示当前激活状态
            self.gm_area_btn.setStyleSheet("background-color: lightblue; font-weight: bold;")
            self.non_gm_area_btn.setStyleSheet("")
            self.statusBar().showMessage("已激活转基因区域框选模式，请在地图上右键框选")
        except Exception as e:
            self.show_error_message(f"激活GM框选失败：{e}")
    
    def activate_non_gm_selection(self):
        """激活非转基因种植区域框选模式"""
        try:
            # 调用JavaScript设置框选模式为非GM
            self.run_js("window.setSelectionMode('non_gm')", description="激活非GM框选模式")
            # 更新按钮样式显示当前激活状态
            self.non_gm_area_btn.setStyleSheet("background-color: lightgreen; font-weight: bold;")
            self.gm_area_btn.setStyleSheet("")
            self.statusBar().showMessage("已激活非转基因区域框选模式，请在地图上右键框选")
        except Exception as e:
            self.show_error_message(f"激活非GM框选失败：{e}")
    
    def run_model_with_latest_data(self):
        """通过右键选择的区域获取中心并启动模型流程（回调式）"""
        # 先取右键选择的两个区域（GM、非GM），再获取模型区域中心
        def _after_selected_areas(areas):
            try:
                if areas and isinstance(areas, dict):
                    gm = areas.get('gm')
                    non = areas.get('non')
                    if gm and 'lng' in gm and 'lat' in gm:
                        self.gm_center = {"lng": float(gm['lng']), "lat": float(gm['lat'])}
                        # 优先使用 JS 返回的 radius，如果为空则使用输入框边长/2
                        input_side = float(self.radius_input.text() or '100')
                        self.gm_radius = float(gm.get('radius', input_side / 2.0))
                        self.statusBar().showMessage(
                            f"已选择转基因种植区域，中心: {self.gm_center['lng']:.6f}, {self.gm_center['lat']:.6f}, 半径: {self.gm_radius}m")
                    if non and 'lng' in non and 'lat' in non:
                        self.non_gm_center = {"lng": float(non['lng']), "lat": float(non['lat'])}
                        input_side = float(self.radius_input.text() or '100')
                        self.non_gm_radius = float(non.get('radius', input_side / 2.0))
                        self.statusBar().showMessage(
                            f"已选择非转基因种植区域，中心: {self.non_gm_center['lng']:.6f}, {self.non_gm_center['lat']:.6f}, 半径: {self.non_gm_radius}m")
                # 校验是否已完成两次右键选择
                if not self.gm_center:
                    self.show_error_message("请先右键选择转基因种植区域")
                    return
                if not self.non_gm_center:
                    self.show_error_message("请右键选择非转基因种植区域")
                    return
                # 继续获取模型区域中心
                self.run_js(
                    "window.getModelAreaCenter()",
                    callback=self.process_map_center,
                    description="获取地图中心"
                )
            except Exception as e:
                self.show_error_message(f"读取右键选择区域失败：{e}")
        # 发起JS调用获取已选择的区域
        self.run_js("window.getSelectedAreas()", callback=_after_selected_areas, description="读取右键选择区域")


    def process_map_center(self, center_data):
        """处理地图中心点数据并启动模型线程"""
        print("[DEBUG] process_map_center 被调用")
        print(center_data)
        try:
            # 检查是否已选择了转基因和非转基因区域
            if not hasattr(self, 'gm_center') or not self.gm_center:
                self.show_error_message("请先选择转基因种植区域")
                return
            
            if not hasattr(self, 'non_gm_center') or not self.non_gm_center:
                self.show_error_message("请先选择非转基因种植区域")
                return

            # 使用非转基因区域作为地图中心（用户观察重点）
            center_lng = self.non_gm_center["lng"]
            center_lat = self.non_gm_center["lat"]

            # 记录转基因和非转基因区域中心（用于右侧绘图坐标换算）
            self.last_gm_center = {"lng": float(self.gm_center["lng"]), "lat": float(self.gm_center["lat"])}
            self.last_non_gm_center = {"lng": float(self.non_gm_center["lng"]), "lat": float(self.non_gm_center["lat"])}

            # 从已有属性获取最新值（可能为 None）
            manual_wind_speed = self.wind_speed_input.text().strip()
            manual_wind_dir = self.wind_dir_input.text().strip()
            manual_temp = self.temp_input.text().strip() if hasattr(self, 'temp_input') else ''
            manual_humi = self.humi_input.text().strip() if hasattr(self, 'humi_input') else ''
            manual_q_factor = self.q_factor_input.text().strip() if hasattr(self, 'q_factor_input') else ''
            
            if manual_wind_speed:
                try:
                    used_speed = float(manual_wind_speed)
                    print(f"[INFO] 使用手动输入的风速: {used_speed} m/s")
                except ValueError:
                    print("[警告] 手动输入的风速格式错误，使用传感器数据或默认值")
                    manual_wind_speed = ""
            
            if manual_wind_dir:
                try:
                    used_dir = float(manual_wind_dir)
                    print(f"[INFO] 使用手动输入的风向: {used_dir} 度")
                except ValueError:
                    print("[警告] 手动输入的风向格式错误，使用传感器数据或默认值")
                    manual_wind_dir = ""
            
            # 如果没有手动输入或输入格式错误，则使用传感器数据或默认值
            if not manual_wind_speed:
                wind_speed = getattr(self, 'latest_speed', None)
                if wind_speed is None:
                    print("[警告] 没有收到实时风速数据，使用默认值")
                    used_speed = 5.0  # 默认风速 m/s
                else:
                    used_speed = float(wind_speed)
                    print(f"[INFO] 使用传感器风速数据: {used_speed} m/s")
            
            if not manual_wind_dir:
                wind_dir = getattr(self, 'latest_dir', None)
                if wind_dir is None:
                    print("[警告] 没有收到实时风向数据，使用默认值")
                    used_dir = 10.0  # 默认风向 °
                else:
                    used_dir = float(wind_dir)
                    print(f"[INFO] 使用传感器风向数据: {used_dir} 度")

            # 处理温度/湿度/源强系数
            used_temp = None
            used_humi = None
            used_q_factor = 5000000.0  # 默认源强 5e6
            used_height_diff = 3.0     # 默认高度差 3m

            if manual_temp:
                try:
                    used_temp = float(manual_temp)
                    print(f"[INFO] 使用手动输入的温度: {used_temp} °C")
                except ValueError:
                    print("[警告] 手动输入的温度格式错误，使用传感器或平均值")
                    used_temp = None
            if manual_humi:
                try:
                    used_humi = float(manual_humi)
                    print(f"[INFO] 使用手动输入的湿度: {used_humi} %")
                except ValueError:
                    print("[警告] 手动输入的湿度格式错误，使用传感器或平均值")
                    used_humi = None
            if manual_q_factor:
                try:
                    used_q_factor = float(manual_q_factor)
                    print(f"[INFO] 使用手动输入的源强 Q: {used_q_factor}")
                except ValueError:
                    print("[警告] 源强 Q 格式错误，改为默认 5e6")
                    used_q_factor = 5000000.0
            
            # 读取高度差
            manual_height = self.height_diff_input.text().strip() if hasattr(self, 'height_diff_input') else ''
            if manual_height:
                try:
                    used_height_diff = float(manual_height)
                    print(f"[INFO] 使用手动输入的高度差: {used_height_diff} m")
                except ValueError:
                    print("[警告] 高度差格式错误，使用默认值 3.0 m")
                    used_height_diff = 3.0

            # 若温/湿均未手动输入，则优先使用过去1分钟滚动平均
            try:
                if not manual_temp and not manual_humi:
                    avg_env = self.compute_average_env()
                    if avg_env is not None:
                        at, ah, n_env = avg_env
                        used_temp = float(at)
                        used_humi = float(ah)
                        print(f"[INFO] 使用过去{self.avg_window_seconds}秒平均温湿度: {used_temp:.1f} °C, {used_humi:.1f} % (n={n_env})")
            except Exception as e:
                print(f"[process_map_center] 计算平均温湿度失败: {e}")

            # 若仍为空则使用最新传感器或默认值
            if used_temp is None:
                latest_temp = None
                try:
                    latest_temp = getattr(self.latest_hardware_data, 'temperature', None) if self.latest_hardware_data else None
                except Exception:
                    latest_temp = None
                if latest_temp is None:
                    used_temp = 25.0
                    print("[INFO] 温度默认 25 °C")
                else:
                    used_temp = float(latest_temp)
                    print(f"[INFO] 使用传感器温度: {used_temp:.1f} °C")
            if used_humi is None:
                latest_humi = None
                try:
                    latest_humi = getattr(self.latest_hardware_data, 'humidity', None) if self.latest_hardware_data else None
                except Exception:
                    latest_humi = None
                if latest_humi is None:
                    used_humi = 50.0
                    print("[INFO] 湿度默认 50 %")
                else:
                    used_humi = float(latest_humi)
                    print(f"[INFO] 使用传感器湿度: {used_humi:.1f} %")
            
            # 获取种植区域半径
            # 注意：source_radius 应始终代表转基因区域（花粉源）的半径
            try:
                # 优先使用已保存的转基因区域半径
                if hasattr(self, 'gm_radius') and self.gm_radius is not None:
                    source_radius = float(self.gm_radius)
                else:
                    # 如果没有，尝试从输入框读取（作为回退）
                    source_radius = float(self.radius_input.text() or "50")
                
                if source_radius <= 0:
                    source_radius = 50.0
            except ValueError:
                source_radius = 50.0
                if not hasattr(self, 'gm_radius') or self.gm_radius is None:
                    self.radius_input.setText("50")

            # 计算两个区域之间的距离
            import math
            gm_lng, gm_lat = self.gm_center["lng"], self.gm_center["lat"]
            non_gm_lng, non_gm_lat = self.non_gm_center["lng"], self.non_gm_center["lat"]
            
            # 使用Haversine公式计算距离（单位：米）
            def haversine_distance(lat1, lng1, lat2, lng2):
                R = 6371000  # 地球半径（米）
                lat1_rad = math.radians(lat1)
                lat2_rad = math.radians(lat2)
                delta_lat = math.radians(lat2 - lat1)
                delta_lng = math.radians(lng2 - lng1)
                
                a = (math.sin(delta_lat/2)**2 + 
                     math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lng/2)**2)
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                return R * c
            
            distance = haversine_distance(gm_lat, gm_lng, non_gm_lat, non_gm_lng)

            # 动态规则：根据两区域距离与矩形尺寸，按500m倍数取整，最小1000m
            try:
                gm_r = float(self.gm_radius) if self.gm_radius is not None else float(source_radius)
            except Exception:
                gm_r = float(source_radius)
            try:
                non_r = float(self.non_gm_radius) if self.non_gm_radius is not None else float(source_radius)
            except Exception:
                non_r = float(source_radius)

            # 矩形边长（米）取较大者的两倍，确保能完全覆盖两个种植区域
            max_rect_side_m = 2.0 * max(gm_r, non_r)
            # 需求边长：两个中心之间的距离 + 较大矩形边长，再加500m缓冲
            required_len_m = distance + max_rect_side_m + 500.0
            # 以500m为单位向上取整，最小1000m
            dynamic_size_m = max(1000.0, 500.0 * math.ceil(required_len_m / 500.0))
            dynamic_size_km = dynamic_size_m / 1000.0
            
            # 保存动态边长到类属性，供结果图显示使用
            self.current_dynamic_size_km = dynamic_size_km
            
            print(f"[DEBUG] 转基因区域中心: ({gm_lng:.6f}, {gm_lat:.6f})")
            print(f"[DEBUG] 非转基因区域中心: ({non_gm_lng:.6f}, {non_gm_lat:.6f})")
            print(f"[DEBUG] 两区域距离: {distance:.2f} m")
            print(f"[DEBUG] 动态模拟区域: {dynamic_size_km:.2f} km × {dynamic_size_km:.2f} km（按500m取整，最小1km）")
            # 优先使用过去1分钟滚动平均风速/风向（若无手动输入）
            try:
                if not manual_wind_speed and not manual_wind_dir:
                    avg = self.compute_average_wind()
                    if avg is not None:
                        avg_speed, avg_dir, n = avg
                        used_speed = float(avg_speed)
                        used_dir = float(avg_dir)
                        print(f"[INFO] 使用过去{self.avg_window_seconds}秒平均风速/风向: {used_speed:.2f} m/s, {used_dir:.1f} ° (n={n})")
            except Exception as e:
                print(f"[process_map_center] 计算平均风参数失败: {e}")
            print(f"[DEBUG] 构造线程对象之前，用于本次运行的风速/风向: {used_speed:.3f} m/s, {used_dir:.2f} °")
            print(f"[DEBUG] 用于本次运行的温湿度: {used_temp:.1f} °C, {used_humi:.1f} %；源强系数 ×{used_q_factor}")
            print(f"[DEBUG] 种植区域半径: {source_radius} m")
            
            # 创建一个新的calculate_bounds函数，使用动态大小
            def dynamic_calculate_bounds(center_lng, center_lat, width_km=None, height_km=None):
                return self.calculate_bounds(center_lng, center_lat, dynamic_size_km, dynamic_size_km)
            
            # 传入真正的方法引用（不要传 None）
            # 读取风场模式
            wind_mode_text = self.wind_mode_combo.currentText() if hasattr(self, 'wind_mode_combo') else '恒定风'
            wind_mode_key = 'prevailing' if wind_mode_text == '盛行风' else 'constant'

            self.model_thread = ModelRunnerThread(
                center_lng, center_lat,
                used_speed, used_dir,
                dynamic_calculate_bounds,
                source_radius,
                download_map_func=None,
                overlay_func=None,
                gm_center=self.gm_center,
                gm_radius=self.gm_radius,
                non_gm_center=self.non_gm_center,
                non_gm_radius=self.non_gm_radius,
                wind_mode=wind_mode_key,
                temperature=used_temp,
                humidity=used_humi,
                pollen_factor=used_q_factor,
                height_diff=used_height_diff,
                grid_km=dynamic_size_km
            )

            # 更新UI状态
            self.run_model_btn.setEnabled(False)
            self.status_indicator.setStyleSheet("color: red; font-size: 20px;")  # 指示灯设为红色
            self.model_status_label.setText("正在模拟")
            self.model_status_label.setStyleSheet("color: blue; font-weight: bold;")
            self.loading_timer.start(500)
            
            # 保存模拟参数供基因漂移计算使用
            self.latest_wind_speed = used_speed
            self.latest_wind_dir = used_dir
            self.current_dynamic_size_km = dynamic_size_km

            # 在地图上显示模拟区域边界（蓝色矩形和中心标记）
            try:
                center_lng = float(center_lng)
                center_lat = float(center_lat)
                grid_km = float(dynamic_size_km)
                
                # 生成 JavaScript 代码来绘制边界
                js_code = f"""
                (function() {{
                    try {{
                        // 移除旧的模拟边界和标记
                        if (window._simBoundaryRect) {{
                            try {{ map.removeOverlay(window._simBoundaryRect); }} catch(e) {{}}
                            window._simBoundaryRect = null;
                        }}
                        if (window._simCenterMarker) {{
                            try {{ map.removeOverlay(window._simCenterMarker); }} catch(e) {{}}
                            window._simCenterMarker = null;
                        }}
                        
                        // 移除旧的mainRectangle和centerMarker（蓝色小方框）
                        if (typeof mainRectangle !== 'undefined' && mainRectangle) {{
                            try {{ map.removeOverlay(mainRectangle); }} catch(e) {{}}
                            mainRectangle = null;
                        }}
                        if (typeof centerMarker !== 'undefined' && centerMarker) {{
                            try {{ map.removeOverlay(centerMarker); }} catch(e) {{}}
                            centerMarker = null;
                        }}
                        
                        var centerLng = {center_lng};
                        var centerLat = {center_lat};
                        var gridKm = {grid_km};
                        var center = new BMap.Point(centerLng, centerLat);
                        
                        // 添加中心标记
                        window._simCenterMarker = new BMap.Marker(center);
                        map.addOverlay(window._simCenterMarker);
                        
                        // 计算矩形边界
                        var halfGridKm = gridKm / 2.0;
                        var latOffset = halfGridKm / 111.0;
                        var lngOffset = halfGridKm / (111.0 * Math.cos(centerLat * Math.PI / 180));
                        
                        var sw = new BMap.Point(centerLng - lngOffset, centerLat - latOffset);
                        var ne = new BMap.Point(centerLng + lngOffset, centerLat + latOffset);
                        var nw = new BMap.Point(sw.lng, ne.lat);
                        var se = new BMap.Point(ne.lng, sw.lat);
                        
                        // 绘制蓝色边界矩形
                        window._simBoundaryRect = new BMap.Polygon([sw, nw, ne, se], {{
                            strokeColor: "blue",
                            strokeWeight: 3,
                            strokeOpacity: 0.8,
                            fillColor: "blue",
                            fillOpacity: 0.05
                        }});
                        map.addOverlay(window._simBoundaryRect);
                        
                        // 自动调整地图视野
                        var bounds = new BMap.Bounds(sw, ne);
                        map.setViewport(bounds, {{margins: [50, 50, 50, 50]}});
                        
                        document.getElementById("debug").textContent = 
                            "已显示模拟区域边界: " + gridKm.toFixed(2) + " km × " + gridKm.toFixed(2) + " km";
                    }} catch(e) {{
                        console.error("显示模拟边界失败:", e);
                    }}
                }})();
                """
                
                self.run_js(js_code, description="显示模拟边界")
            except Exception as e:
                print(f"[DEBUG] 显示模拟边界失败: {e}")

            print("[DEBUG] 构造线程对象之后")

            self.model_thread.finished.connect(self.on_model_finished)
            self.model_thread.start()

        except Exception as e:
            self.show_error_message(f"模型流程失败：{str(e)}")
            self.run_model_btn.setEnabled(True)
            self.model_status_label.setText("模拟启动失败")
            self.model_status_label.setStyleSheet("color: red;")
            import traceback
            print(traceback.format_exc())
    
    def calculate_bounds(self, center_lng, center_lat, width_km=1.0, height_km=1.0):
        """计算给定中心点和尺寸的地理边界"""
        # 经纬度到弧度
        lat_rad = math.radians(center_lat)
    
        # 每度纬度的距离 ~ 111 km
        lat_deg_per_km = 1 / 111.0
        # 每度经度的距离 ~ 111 km * cos(latitude)
        lng_deg_per_km = 1 / (111.0 * math.cos(lat_rad))
    
        half_width_deg = (width_km / 2) * lng_deg_per_km
        half_height_deg = (height_km / 2) * lat_deg_per_km
    
        sw = {
            "lat": center_lat - half_height_deg,
            "lng": center_lng - half_width_deg
        }
        ne = {
            "lat": center_lat + half_height_deg,
            "lng": center_lng + half_width_deg
        }
        return {"sw": sw, "ne": ne}
    
    def on_model_finished(self, final_url, bounds, data, overlay_path, basemap_path):
        """模型计算完成后的回调处理"""
        self.loading_timer.stop()
        try:
            if not final_url or not bounds or not data:
                print("[警告] 模型计算返回的数据不完整")
                self.model_status_label.setText("模拟失败：数据不完整")
                self.model_status_label.setStyleSheet("color: red;")
                self.status_indicator.setStyleSheet("color: red; font-size: 20px;")  # 指示灯保持红色
                self.run_model_btn.setEnabled(True)
                return

            # 右边绘制:使用相同的图片和数据
            # data tuple expanded: x, y, conc, recipient_data
            if len(data) == 4:
                x, y, conc, recipient_data = data
            else:
                x, y, conc = data
                recipient_data = None

            # 缓存最近一次模型结果，供“基因漂移概率”页使用
            self.latest_model_data = {
                "x": x,
                "y": y,
                "conc": conc,
                "recipient_data": recipient_data,
                "bounds": bounds,
                "overlay_path": overlay_path,
                "final_url": final_url,
                "basemap_path": basemap_path,
                "params": {
                    "gm_center": getattr(self.model_thread, 'gm_center', None),
                    "gm_radius": getattr(self.model_thread, 'gm_radius', 0),
                    "non_gm_center": getattr(self.model_thread, 'non_gm_center', None),
                    "non_gm_radius": getattr(self.model_thread, 'non_gm_radius', 0),
                    "wind_speed": getattr(self.model_thread, 'wind_speed', 0),
                    "wind_dir": getattr(self.model_thread, 'wind_dir', 0),
                    "wind_mode": getattr(self.model_thread, 'wind_mode', 'constant'),
                    "temp": getattr(self.model_thread, 'temperature', 0),
                    "humidity": getattr(self.model_thread, 'humidity', 0),
                    "q_factor": getattr(self.model_thread, 'pollen_factor', 0),
                    "height_diff": getattr(self.model_thread, 'height_diff', 0)
                }
            }

            # 左侧在线地图：叠加本次计算得到的透明覆盖层（随所选区域变化）
            try:
                if overlay_path and os.path.exists(overlay_path) and bounds and 'sw' in bounds and 'ne' in bounds:
                    overlay_url = QUrl.fromLocalFile(overlay_path).toString()
                    js = f"window.addModelImageOverlay({json.dumps(overlay_url)}, {json.dumps(bounds['sw'])}, {json.dumps(bounds['ne'])})"
                    self.run_js(js, description="叠加模型图层")
            except Exception as e:
                print(f"[on_model_finished] 地图叠加失败: {e}")

            # 使用模型线程已经生成的图片，确保左右一致
            self.plot_local_result(x, y, conc, basemap_path=None, model_image_path=final_url)

            # 如果基因漂移页已创建，提示可直接计算
            try:
                if hasattr(self, 'drift_ready_label') and self.drift_ready_label:
                    self.drift_ready_label.setText("已获取最近一次模拟结果，可计算概率-距离曲线")
            except Exception:
                pass

            self.model_status_label.setText("模拟完成")
            self.model_status_label.setStyleSheet("color: green; font-weight: bold;")
            self.status_indicator.setStyleSheet("color: green; font-size: 20px;")  # 指示灯设为绿色
            self.run_model_btn.setEnabled(True)

        except Exception as e:
            print(f"[on_model_finished] 出错: {e}")
            self.model_status_label.setText("模拟出错")
            self.model_status_label.setStyleSheet("color: red;")
            self.status_indicator.setStyleSheet("color: red; font-size: 20px;")  # 指示灯保持红色
            self.run_model_btn.setEnabled(True)
            import traceback
            print(traceback.format_exc())

    def create_model_tab(self):
        """创建基因漂移概率标签页（基于最近一次“运行模拟”的浓度结果，计算概率-距离曲线）"""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 顶部：参数与按钮
        h_top = QHBoxLayout()
        h_top.addWidget(QLabel("冠层拦截系数 k_p:"))
        self.kp_input = QLineEdit()
        self.kp_input.setFixedWidth(80)
        self.kp_input.setText("0.3")
        h_top.addWidget(self.kp_input)

        h_top.addSpacing(8)
        h_top.addWidget(QLabel("叶面积指数 L:"))
        self.L_input = QLineEdit()
        self.L_input.setFixedWidth(80)
        self.L_input.setText("2.0")
        h_top.addWidget(self.L_input)

        h_top.addSpacing(8)
        h_top.addWidget(QLabel("沉降速度 v_d:"))
        self.vd_input = QLineEdit()
        self.vd_input.setFixedWidth(80)
        self.vd_input.setText("0.02")
        h_top.addWidget(self.vd_input)

        h_top.addSpacing(8)
        h_top.addWidget(QLabel("竞争参数 c_p (0~0.5):"))
        self.cp_input = QLineEdit()
        self.cp_input.setFixedWidth(80)
        self.cp_input.setText("0.5")
        h_top.addWidget(self.cp_input)

        h_top.addSpacing(8)
        h_top.addWidget(QLabel("受体花粉源强度 Q_recipient:"))
        self.qr_input = QLineEdit()
        self.qr_input.setFixedWidth(120)
        self.qr_input.setText("0")
        h_top.addWidget(self.qr_input)

        compute_btn = QPushButton("生成热力图")
        compute_btn.setFixedWidth(160)
        compute_btn.clicked.connect(self.calculate_drift)
        h_top.addWidget(compute_btn)
        
        # 运行状态指示灯（红色=运行中/错误，绿色=完成）
        self.drift_status_indicator = QLabel("●")
        self.drift_status_indicator.setStyleSheet("color: red; font-size: 20px;")
        self.drift_status_indicator.setToolTip("热力图计算状态指示灯")
        h_top.addWidget(self.drift_status_indicator)

        # 状态提示标签
        self.drift_status_label = QLabel("")
        self.drift_status_label.setStyleSheet("color: blue; font-weight: bold;")
        h_top.addWidget(self.drift_status_label)
        
        h_top.addSpacing(8)
        export_drift_btn = QPushButton("输出csv形式的基因漂移概率")
        export_drift_btn.setFixedWidth(200)
        export_drift_btn.clicked.connect(self.export_drift_csv)
        h_top.addWidget(export_drift_btn)
        
        h_top.addStretch()
        layout.addLayout(h_top)

        # 提示/状态
        self.drift_ready_label = QLabel("请先在‘花粉浓度扩散’页运行模拟并选定两中心，然后点击“生成热力图”")
        layout.addWidget(self.drift_ready_label)

        # 结果文本（更新说明）
        self.drift_result = QLabel("在此显示热力图摘要信息")
        self.drift_result.setFont(QFont("Arial", 11))
        layout.addWidget(self.drift_result)

        # 画布（自定义布局：使用嵌套 GridSpec 实现左侧平行热力图，右侧独立上下统计图）
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
        from matplotlib.figure import Figure
        import matplotlib.gridspec as gridspec
        
        self.drift_fig = Figure(figsize=(18, 9), dpi=100)
        
        # 1. 主网格：分为左右两部分
        # width_ratios=[2, 1.2]: 左侧占约 62.5% 宽度，右侧占 37.5%
        gs_main = self.drift_fig.add_gridspec(1, 2, width_ratios=[2, 1.2], wspace=0.15)
        
        # 2. 左侧子网格 (嵌套在 gs_main[0,0])：1行2列，用于两个热力图
        # wspace=0.3: 两个热力图之间的间距
        gs_left = gs_main[0, 0].subgridspec(1, 2, wspace=0.3)
        
        # 3. 右侧子网格 (嵌套在 gs_main[0,1])：2行1列，用于两个统计图
        # hspace=0.4: 上下两个统计图之间的垂直间距
        gs_right = gs_main[0, 1].subgridspec(2, 1, hspace=0.4)
        
        # 初始化4个子图
        # ax1: 左边第一个热力图
        self.drift_ax1 = self.drift_fig.add_subplot(gs_left[0, 0])
        # ax2: 左边第二个热力图
        self.drift_ax2 = self.drift_fig.add_subplot(gs_left[0, 1])
        # ax3: 右上统计图
        self.drift_ax3 = self.drift_fig.add_subplot(gs_right[0, 0])
        # ax4: 右下统计图
        self.drift_ax4 = self.drift_fig.add_subplot(gs_right[1, 0])
        
        # 调整全局边距
        self.drift_fig.subplots_adjust(left=0.05, right=0.98, top=0.92, bottom=0.15)
        
        self.drift_canvas = FigureCanvas(self.drift_fig)
        layout.addWidget(self.drift_canvas)

        tab.setLayout(layout)
        self.tab_widget.addTab(tab, "基因漂移概率")


    def calculate_drift(self):
        """计算并绘制非转基因区域内的基因漂移概率热力图。"""
        try:
            # 更新状态：开始计算
            if hasattr(self, 'drift_status_indicator'):
                self.drift_status_indicator.setStyleSheet("color: red; font-size: 20px;")
            if hasattr(self, 'drift_status_label'):
                self.drift_status_label.setText("正在计算热力图")
                self.drift_status_label.setStyleSheet("color: blue; font-weight: bold;")
            
            self.loading_timer.start(500)
            QApplication.processEvents()
            
            # 1) 校验：最近一次模型数据
            if not hasattr(self, 'latest_model_data') or self.latest_model_data is None:
                self.show_error_message("尚无模拟结果。请先在'花粉浓度扩散'页点击'运行模拟'。")
                self.loading_timer.stop()
                return
            x_full = self.latest_model_data.get("x")
            y_full = self.latest_model_data.get("y")
            conc_donor_full = self.latest_model_data.get("conc")
            basemap_path = self.latest_model_data.get("basemap_path")

            if x_full is None or y_full is None or conc_donor_full is None:
                self.show_error_message("模拟结果不完整，无法计算热力图。")
                self.loading_timer.stop()
                return

            # 2) 校验：非转基因区域
            if not hasattr(self, 'non_gm_radius') or self.non_gm_radius is None:
                self.show_error_message("尚未选定非转基因种植区。")
                self.loading_timer.stop()
                return

            # 3) 读取参数
            try:
                kp = float((self.kp_input.text() or "0.55").strip())
                L = float((self.L_input.text() or "3.5").strip())
                vd = float((self.vd_input.text() or "0.3").strip())
                cp = float((self.cp_input.text() or "0.5").strip())
                Q_rec = float((self.qr_input.text() or "0.1").strip())
                if kp < 0 or L < 0 or vd <= 0 or not (0.0 < cp < 1.0) or Q_rec < 0:
                    raise ValueError
            except Exception:
                self.show_error_message("请输入有效参数：k_p≥0, L≥0, v_d>0, 0<cp<1, Q_recipient≥0。")
                self.loading_timer.stop()
                return

            # 4) 调用 drift_model 计算
            wind_speed = getattr(self, 'latest_wind_speed', 5.0)
            wind_dir = getattr(self, 'latest_wind_dir', 10.0)
            grid_km = getattr(self, 'current_dynamic_size_km', 1.0)
            
            try:
                # 假设分辨率 dxy=10，这应该与 gaussian_plume_model 中的默认值一致
                # 或者从 x_full 中推断分辨率
                dxy = 10.0
                if x_full.shape[1] > 1:
                    dxy = float(x_full[0, 1] - x_full[0, 0])

                # 获取源强 Q (用于供体和受体，假设两者单株产量相似)
                # 优先从输入框获取，默认为 5e6
                try:
                    pollen_Q = float(self.q_factor_input.text().strip())
                except:
                    pollen_Q = 5000000.0

                # 4) 启动后台线程计算
                self.drift_thread = DriftCalculatorThread(
                    x_full, y_full, conc_donor_full,
                    self.non_gm_radius,
                    kp, L, vd, cp, 
                    wind_speed, wind_dir, grid_km,
                    dxy=dxy,
                    pollen_factor=pollen_Q
                )
                self.drift_thread.finished.connect(self.on_drift_calculation_finished)
                self.drift_thread.error.connect(self.on_drift_error)
                self.drift_thread.start()

            except Exception as e:
                self.show_error_message(f"启动计算失败: {e}")
                self.loading_timer.stop()
                return

        except Exception as e:
            self.loading_timer.stop()
            # 更新状态：计算出错
            if hasattr(self, 'drift_status_indicator'):
                self.drift_status_indicator.setStyleSheet("color: red; font-size: 20px;")
            if hasattr(self, 'drift_status_label'):
                self.drift_status_label.setText("热力图计算出错")
                self.drift_status_label.setStyleSheet("color: red; font-weight: bold;")
            self.show_error_message(f"热力图计算出错：{e}")
            import traceback
            print(traceback.format_exc())

    def on_drift_calculation_finished(self, x_sub, y_sub, G_percent):
        """基因漂移计算完成回调"""
        try:
            # 保存计算结果供导出
            self.latest_drift_data = {
                "x": x_sub,
                "y": y_sub,
                "G_percent": G_percent
            }

            # 重新获取完整数据用于底图范围（因为线程只返回了局部结果）
            if not hasattr(self, 'latest_model_data') or self.latest_model_data is None:
                self.show_error_message("模型数据丢失，无法绘制底图。")
                self.loading_timer.stop()
                return
            
            x_full = self.latest_model_data.get("x")
            y_full = self.latest_model_data.get("y")
            basemap_path = self.latest_model_data.get("basemap_path")

            # 5) 绘图
            self.drift_ax1.clear()
            self.drift_ax2.clear()
            self.drift_ax3.clear()
            self.drift_ax4.clear()
            
            extent_sub = [x_sub.min(), x_sub.max(), y_sub.min(), y_sub.max()]
            print(f"[DEBUG] Plotting extent: {extent_sub}")
            
            # 动态调整颜色尺度
            g_max_val = np.max(G_percent)
            vmax_val = 1.0 if g_max_val < 1.0 else g_max_val
            
            # --- 左图1：纯热力图 (Ax1) ---
            # 使用 contourf 平滑显示
            levels = np.linspace(0, vmax_val, 100)
            im1 = self.drift_ax1.contourf(
                x_sub, y_sub, G_percent,
                levels=levels,
                cmap='Reds'
            )
            
            # 添加关键等高线
            try:
                # 1% 线 - 紫色
                if g_max_val >= 1.0:
                    cs1_purple = self.drift_ax1.contour(
                        x_sub, y_sub, G_percent,
                        levels=[1.0],
                        colors='purple',
                        linewidths=2.0,
                        linestyles='solid'
                    )
                    self.drift_ax1.clabel(cs1_purple, inline=True, fmt='1%%', fontsize=10, colors='purple')
                
                # 0.1% 线 - 黑色
                if g_max_val >= 0.1:
                    cs1_black = self.drift_ax1.contour(
                        x_sub, y_sub, G_percent,
                        levels=[0.1],
                        colors='black',
                        linewidths=1.5,
                        linestyles='solid'
                    )
                    self.drift_ax1.clabel(cs1_black, inline=True, fmt='0.1%%', fontsize=10, colors='black')
            except: pass
            
            self.drift_ax1.set_xlabel("相对 X 坐标 (m)")
            self.drift_ax1.set_ylabel("相对 Y 坐标 (m)")
            self.drift_ax1.set_title(f"热力图 (半径={self.non_gm_radius}m)")
            # 设置显示范围与右图一致
            radius = float(self.non_gm_radius)
            self.drift_ax1.set_xlim(-radius, radius)
            self.drift_ax1.set_ylim(-radius, radius)
            self.drift_ax1.set_aspect('equal', adjustable='box')
            
            # --- 左图2：底图 + 热力图 (Ax2) ---
            # 1. 绘制底图（如果有）
            if basemap_path and os.path.exists(basemap_path):
                try:
                    img = plt.imread(basemap_path)
                    # 底图对应的是整个模拟区域
                    if x_full is not None and y_full is not None:
                        extent_full = [x_full.min(), x_full.max(), y_full.min(), y_full.max()]
                        self.drift_ax2.imshow(img, extent=extent_full, aspect='auto', alpha=1.0)
                except Exception as e:
                    print(f"[WARN] 无法加载底图: {e}")
            
            # 2. 叠加热力图
            self.drift_ax2.contourf(
                x_sub, y_sub, G_percent,
                levels=levels,
                cmap='Reds',
                alpha=0.6
            )
            
            # 添加关键等高线（叠加在地图底图上）
            try:
                # 1% 线 - 紫色粗线
                if g_max_val >= 1.0:
                    cs2_purple = self.drift_ax2.contour(
                        x_sub, y_sub, G_percent,
                        levels=[1.0],
                        colors='purple',
                        linewidths=2.5,
                        linestyles='solid'
                    )
                    self.drift_ax2.clabel(cs2_purple, inline=True, fmt='1%%', fontsize=11, 
                                         colors='purple', inline_spacing=10,
                                         manual=False)
                
                # 0.1% 线 - 黑色实线
                if g_max_val >= 0.1:
                    cs2_black = self.drift_ax2.contour(
                        x_sub, y_sub, G_percent,
                        levels=[0.1],
                        colors='black',
                        linewidths=2.0,
                        linestyles='solid'
                    )
                    self.drift_ax2.clabel(cs2_black, inline=True, fmt='0.1%%', fontsize=11, 
                                         colors='black', inline_spacing=10,
                                         manual=False)
            except: pass
            
            # 3. 设置显示范围为非转基因区域（Zoom in）
            radius = float(self.non_gm_radius)
            self.drift_ax2.set_xlim(-radius, radius)
            self.drift_ax2.set_ylim(-radius, radius)
            self.drift_ax2.set_aspect('equal', adjustable='box')
            
            self.drift_ax2.set_xlabel("相对 X 坐标 (m)")
            self.drift_ax2.set_title(f"热力图 + 地图背景")
            
            # --- 右上：平均值统计图 (Ax3) ---
            # X轴为相对位置，Y轴为该X处的平均漂移概率
            # 获取 X 轴坐标 (取第一行)
            x_axis = x_sub[0, :]
            # 对每一列求平均 (axis=0)
            y_avg = np.mean(G_percent, axis=0)
            
            self.drift_ax3.plot(x_axis, y_avg, color='blue', linewidth=2)
            self.drift_ax3.set_title("平均漂移概率分布")
            self.drift_ax3.set_xlabel("相对 X 坐标 (m)")
            self.drift_ax3.set_ylabel("平均概率 (%)")
            self.drift_ax3.grid(True, linestyle='--', alpha=0.7)
            # 标出最大值点
            max_idx = np.argmax(y_avg)
            self.drift_ax3.plot(x_axis[max_idx], y_avg[max_idx], 'ro')
            self.drift_ax3.text(x_axis[max_idx], y_avg[max_idx], f"{y_avg[max_idx]:.2f}%", 
                               fontsize=9, verticalalignment='bottom')

            # --- 右下：总和统计图 (Ax4) ---
            # X轴为相对位置，Y轴为该X处的概率总和
            y_sum = np.sum(G_percent, axis=0)
            
            self.drift_ax4.plot(x_axis, y_sum, color='green', linewidth=2)
            self.drift_ax4.set_title("漂移概率总和分布")
            self.drift_ax4.set_xlabel("相对 X 坐标 (m)")
            self.drift_ax4.set_ylabel("概率总和 (%)")
            self.drift_ax4.grid(True, linestyle='--', alpha=0.7)
             # 标出最大值点
            sum_max_idx = np.argmax(y_sum)
            self.drift_ax4.plot(x_axis[sum_max_idx], y_sum[sum_max_idx], 'ro')
            self.drift_ax4.text(x_axis[sum_max_idx], y_sum[sum_max_idx], f"{y_sum[sum_max_idx]:.1f}%", 
                               fontsize=9, verticalalignment='bottom')
            
            # 添加颜色条 (水平放置在左侧两热力图下方)
            if hasattr(self, 'drift_cbar') and self.drift_cbar:
                try:
                    self.drift_cbar.remove()
                except:
                    pass
            
            # 放在 ax1 和 ax2 的下方，水平方向
            self.drift_cbar = self.drift_fig.colorbar(im1, ax=[self.drift_ax1, self.drift_ax2], 
                                                    orientation='horizontal', 
                                                    fraction=0.08, pad=0.15, aspect=40)
            self.drift_cbar.set_label("基因漂移概率 (%)")
            
            self.drift_canvas.draw()

            # 6) 摘要
            g_max = float(np.max(G_percent))
            g_avg = float(np.mean(G_percent))
            self.drift_result.setText(
                f"热力图生成完成：区域大小 {radius*2:.0f}x{radius*2:.0f} m。区域内最大概率≈{g_max:.2f}%，平均概率≈{g_avg:.2f}%。"
            )

            if hasattr(self, 'drift_ready_label') and self.drift_ready_label:
                self.drift_ready_label.setText("已基于最近模拟结果计算完成（热力图）")
            
            # 更新状态：计算完成
            if hasattr(self, 'drift_status_indicator'):
                self.drift_status_indicator.setStyleSheet("color: green; font-size: 20px;")
            if hasattr(self, 'drift_status_label'):
                self.drift_status_label.setText("热力图计算完成")
                self.drift_status_label.setStyleSheet("color: green; font-weight: bold;")
            
            self.loading_timer.stop()

        except Exception as e:
            self.show_error_message(f"绘图处理失败: {e}")
            self.loading_timer.stop()
            import traceback
            print(traceback.format_exc())

    def on_drift_error(self, err_msg):
        """基因漂移计算错误回调"""
        self.show_error_message(f"热力图计算出错: {err_msg}")
        self.loading_timer.stop()
        
        # 更新状态：计算出错
        if hasattr(self, 'drift_status_indicator'):
            self.drift_status_indicator.setStyleSheet("color: red; font-size: 20px;")
        if hasattr(self, 'drift_status_label'):
            self.drift_status_label.setText("热力图计算出错")
            self.drift_status_label.setStyleSheet("color: red; font-weight: bold;")

    def export_drift_csv(self):
        """导出基因漂移概率数据为CSV"""
        if not hasattr(self, 'latest_drift_data') or self.latest_drift_data is None:
            self.show_error_message("尚无基因漂移概率数据，请先点击“生成热力图”。")
            return

        try:
            # 获取保存路径
            file_path, _ = QFileDialog.getSaveFileName(
                self, "保存基因漂移概率数据", "", "CSV Files (*.csv);;All Files (*)"
            )
            if not file_path:
                return

            x = self.latest_drift_data['x']
            y = self.latest_drift_data['y']
            g = self.latest_drift_data['G_percent']

            import csv
            with open(file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                
                writer.writerow(["=== 基因漂移概率数据 (矩阵格式: 行=Y坐标, 列=X坐标) ==="])
                writer.writerow(["数值单位: %"])
                
                rows, cols = x.shape
                
                # 获取 X 轴坐标（第一行的所有列）
                x_coords = x[0, :]
                # 写入表头：[ "Y \ X" , x1, x2, ... ]
                header = ["Y \ X (m)"] + [f"{val:.2f}" for val in x_coords]
                writer.writerow(header)
                
                # 逐行写入数据
                # 为了直观对应地图，从最后一行(y_max)开始写到第一行(y_min)
                for r in range(rows - 1, -1, -1):
                    y_val = y[r, 0]
                    # 构建当前行: [y_val, g_0, g_1, ...]
                    row_data = [f"{y_val:.2f}"] + [f"{val:.4f}" for val in g[r, :]]
                    writer.writerow(row_data)

            QMessageBox.information(self, "导出成功", f"数据已保存至：\n{file_path}")

        except Exception as e:
            self.show_error_message(f"导出CSV失败：{e}")


    def create_data_display(self, layout, label_text, row, col):
        label = QLabel(label_text)
        value_label = QLabel("---")
        value_label.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(label, row, col * 2)
        layout.addWidget(value_label, row, col * 2 + 1)
        return value_label

    def toggle_connection(self):
        """切换串口连接状态"""
        data_source = self.data_source_combo.currentText()
        
        # 断开现有连接
        if (self.serial_thread and self.serial_thread.running) or \
           (self.hardware_serial_thread and self.hardware_serial_thread.running):
            if self.serial_thread and self.serial_thread.running:
                self.serial_thread.close()
            if self.hardware_serial_thread and self.hardware_serial_thread.running:
                self.hardware_serial_thread.close()
            self.connect_btn.setText("连接")
            self.statusBar().showMessage("未连接")
            
            # 重置为默认状态，以便断开后使用默认参数运行模拟
            self.latest_speed = None
            self.latest_dir = None
            self.latest_hardware_data = None
            self.model_info_label.setText("最新风速风向：尚未获取")
            # 清空滚动平均缓冲
            self.wind_samples = deque()
            self.env_samples = deque()
            
        else:
            port = self.port_combo.currentText()
            baudrate = int(self.baudrate_combo.currentText())
            
            if data_source == "硬件设备":
                # 使用硬件串口线程
                self.hardware_serial_thread = HardwareSerialThread(port, baudrate)
                self.hardware_serial_thread.signal.update_signal.connect(self.update_hardware_data)
                self.hardware_serial_thread.start()
                self.connect_btn.setText("断开")
                self.statusBar().showMessage(f"已连接到硬件设备 {port}, 波特率 {baudrate}")
            else:
                # 使用模拟数据串口线程
                self.serial_thread = SerialThread(port, baudrate)
                self.serial_thread.signal.update_signal.connect(self.update_data)
                self.serial_thread.start()
                self.connect_btn.setText("断开")
                self.statusBar().showMessage(f"已连接到模拟设备 {port}, 波特率 {baudrate}")

    def update_data(self, sensor_data):
        """更新传感器数据显示"""
        if sensor_data is None:
            self.show_error_message("串口通信错误")
            self.connect_btn.setText("连接")
            self.statusBar().showMessage("未连接")
            return
        self.temp_label.setText(f"{sensor_data.temp:.1f} °C")
        self.humi_label.setText(f"{sensor_data.humi} %")
        self.wind_speed_label.setText(f"{sensor_data.wind_speed:.2f} m/s")
        self.wind_dir_label.setText(f"{sensor_data.wind_direction:.1f} °")

        # 实时保存风速风向以供模型使用
        self.latest_speed = sensor_data.wind_speed
        self.latest_dir = sensor_data.wind_direction
        self.model_info_label.setText(
            f"最新风速: {self.latest_speed:.2f} m/s，风向: {self.latest_dir:.1f} °"
        )

        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.data_log.append([
            now, sensor_data.temp, sensor_data.humi,
            sensor_data.wind_speed, sensor_data.wind_direction
        ])
        self.log_display.append(
            f"[{now}] 温度: {sensor_data.temp:.1f} °C, 湿度: {sensor_data.humi} %, 风速: {sensor_data.wind_speed:.2f} m/s, 风向: {sensor_data.wind_direction:.1f} °"
        )
        # 追加到平均缓冲
        try:
            self.append_wind_sample(sensor_data.wind_speed, sensor_data.wind_direction)
            # 同步追加温湿度样本
            self.append_env_sample(sensor_data.temp, sensor_data.humi)
        except Exception as e:
            print(f"[update_data] 追加平均缓冲失败: {e}")

    def update_hardware_data(self, hardware_data):
        """更新硬件传感器数据显示"""
        if hardware_data is None:
            self.show_error_message("硬件串口通信错误")
            self.connect_btn.setText("连接")
            self.statusBar().showMessage("未连接")
            return
            
        # 更新显示标签
        self.temp_label.setText(f"{hardware_data.temperature:.1f} °C")
        self.humi_label.setText(f"{hardware_data.humidity:.1f} %")
        self.wind_speed_label.setText(f"{hardware_data.wind_speed:.2f} m/s")
        self.wind_dir_label.setText(f"{hardware_data.wind_direction:.1f} °")

        # 实时保存风速风向以供模型使用
        self.latest_speed = hardware_data.wind_speed
        self.latest_dir = hardware_data.wind_direction
        self.latest_hardware_data = hardware_data
        self.model_info_label.setText(
            f"最新风速: {self.latest_speed:.2f} m/s，风向: {self.latest_dir:.1f} °"
        )

        # 记录数据
        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.hardware_data_log.append([
            now, hardware_data.temperature, hardware_data.humidity,
            hardware_data.wind_speed, hardware_data.wind_direction,
            hardware_data.latitude, hardware_data.longitude
        ])
        
        # 显示在日志区域
        gps_info = ""
        if hardware_data.latitude != 0 and hardware_data.longitude != 0:
            gps_info = f", GPS: {hardware_data.latitude:.6f}, {hardware_data.longitude:.6f}"
        
        self.log_display.append(
            f"[{now}] 温度: {hardware_data.temperature:.1f} °C, 湿度: {hardware_data.humidity:.1f} %, "
            f"风速: {hardware_data.wind_speed:.2f} m/s, 风向: {hardware_data.wind_direction:.1f} °{gps_info}"
        )
        # 追加到平均缓冲
        try:
            self.append_wind_sample(hardware_data.wind_speed, hardware_data.wind_direction)
            # 同步追加温湿度样本
            self.append_env_sample(hardware_data.temperature, hardware_data.humidity)
        except Exception as e:
            print(f"[update_hardware_data] 追加平均缓冲失败: {e}")

    def append_wind_sample(self, speed, direction):
        """将一条风速/风向样本加入滚动窗口，并裁剪到最近avg_window_seconds秒"""
        try:
            ts = time.time()
            # 初始化缓冲
            if not hasattr(self, 'wind_samples') or self.wind_samples is None:
                self.wind_samples = deque()
            # 追加样本
            self.wind_samples.append((ts, float(speed), float(direction)))
            # 裁剪过期样本
            cutoff = ts - float(getattr(self, 'avg_window_seconds', 60))
            while self.wind_samples and self.wind_samples[0][0] < cutoff:
                self.wind_samples.popleft()
        except Exception as e:
            print(f"[append_wind_sample] 失败: {e}")

    def append_env_sample(self, temperature, humidity):
        """将一条温度/湿度样本加入滚动窗口，并裁剪到最近avg_window_seconds秒"""
        try:
            ts = time.time()
            if not hasattr(self, 'env_samples') or self.env_samples is None:
                self.env_samples = deque()
            self.env_samples.append((ts, float(temperature), float(humidity)))
            cutoff = ts - float(getattr(self, 'avg_window_seconds', 60))
            while self.env_samples and self.env_samples[0][0] < cutoff:
                self.env_samples.popleft()
        except Exception as e:
            print(f"[append_env_sample] 失败: {e}")

    def compute_average_wind(self):
        """计算最近avg_window_seconds秒内的平均风速与风向（向量平均）
        返回 (avg_speed, avg_dir, n) 或 None
        """
        try:
            if not hasattr(self, 'wind_samples') or not self.wind_samples:
                return None
            ts_now = time.time()
            cutoff = ts_now - float(getattr(self, 'avg_window_seconds', 60))
            x_sum = 0.0
            y_sum = 0.0
            n = 0
            for ts, s, d in list(self.wind_samples):
                if ts >= cutoff:
                    rad = math.radians(float(d))
                    x_sum += float(s) * math.cos(rad)
                    y_sum += float(s) * math.sin(rad)
                    n += 1
            if n == 0:
                return None
            avg_speed = math.sqrt(x_sum * x_sum + y_sum * y_sum) / n
            avg_dir = (math.degrees(math.atan2(y_sum, x_sum)) + 360.0) % 360.0
            return float(avg_speed), float(avg_dir), int(n)
        except Exception as e:
            print(f"[compute_average_wind] 失败: {e}")
            return None

    def compute_average_env(self):
        """计算最近avg_window_seconds秒内的平均温度与湿度
        返回 (avg_temp, avg_humi, n) 或 None
        """
        try:
            if not hasattr(self, 'env_samples') or not self.env_samples:
                return None
            ts_now = time.time()
            cutoff = ts_now - float(getattr(self, 'avg_window_seconds', 60))
            t_sum = 0.0
            h_sum = 0.0
            n = 0
            for ts, t, h in list(self.env_samples):
                if ts >= cutoff:
                    t_sum += float(t)
                    h_sum += float(h)
                    n += 1
            if n == 0:
                return None
            return float(t_sum / n), float(h_sum / n), int(n)
        except Exception as e:
            print(f"[compute_average_env] 失败: {e}")
            return None

    def on_data_source_changed(self, source):
        """数据源切换时的处理"""
        if source == "硬件设备":
            # 自动刷新并选择CH340端口
            self.refresh_ports()
        
    def refresh_ports(self):
        """刷新串口列表，优先显示CH340端口"""
        current_port = self.port_combo.currentText()
        self.port_combo.clear()
        
        # 获取所有可用端口
        available_ports = [f"COM{i}" for i in range(1, 17)]
        
        # 如果是硬件设备模式，尝试找到CH340端口
        if self.data_source_combo.currentText() == "硬件设备":
            try:
                ch340_ports = find_ch340_ports()
                if ch340_ports:
                    # 将CH340端口放在前面
                    for port in ch340_ports:
                        if port in available_ports:
                            available_ports.remove(port)
                    available_ports = ch340_ports + available_ports
                    self.port_combo.addItems(available_ports)
                    self.port_combo.setCurrentText(ch340_ports[0])
                    self.statusBar().showMessage(f"检测到CH340设备: {', '.join(ch340_ports)}")
                else:
                    self.port_combo.addItems(available_ports)
                    self.statusBar().showMessage("未检测到CH340设备，请检查连接")
            except Exception as e:
                self.port_combo.addItems(available_ports)
                self.statusBar().showMessage(f"端口检测失败: {e}")
        else:
            self.port_combo.addItems(available_ports)
            
        # 尝试恢复之前选择的端口
        if current_port in [self.port_combo.itemText(i) for i in range(self.port_combo.count())]:
            self.port_combo.setCurrentText(current_port)

    def plot_local_result(self, x, y, conc, basemap_path=None, model_image_path=None):
        """在右侧绘制：
        - 优先：直接显示模型线程生成的叠加后成品图（与左侧一致）
        - 回退：背景 = 框选区域地图，前景 = 等高线（带坐标轴）
        """
        # 保障中文与负号正常显示（Windows常见中文字体）
        mpl.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
        mpl.rcParams['axes.unicode_minus'] = False

        # 清理旧画布
        for i in reversed(range(self.canvas_layout.count())):
            widget = self.canvas_layout.itemAt(i).widget()
            if widget:
                widget.deleteLater()

        fig = Figure(figsize=(5, 5), dpi=100)
        ax = fig.add_subplot(111)

        # 准备颜色映射和范围，无论哪个分支都需要
        vmax = float(np.percentile(conc, 95))
        if vmax <= 0 or not np.isfinite(vmax):
            vmax = np.max(conc) if np.max(conc) > 0 else 1.0
        step = vmax / 5
        levels = np.arange(0, vmax + step, step)

        # 计算动态地图背景显示范围
        if hasattr(self, 'current_dynamic_size_km') and self.current_dynamic_size_km:
            # 将公里转换为米，并设置为边长的一半作为地图背景显示范围
            half_range_m = int(self.current_dynamic_size_km * 1000 / 2)
            map_extent = [-half_range_m, half_range_m, -half_range_m, half_range_m]
            print(f"[DEBUG] 使用动态地图背景范围: {map_extent} (对应 {self.current_dynamic_size_km:.2f}km × {self.current_dynamic_size_km:.2f}km)")
        else:
            # 回退到固定范围
            map_extent = [-500, 500, -500, 500]
            print(f"[DEBUG] 使用固定地图背景范围: {map_extent}")

        # ===== 优先显示模型叠加后的最终图片（确保与左侧一致） =====
        if model_image_path and os.path.exists(model_image_path):
            try:
                img = Image.open(model_image_path)
                ax.imshow(img, extent=map_extent, origin="upper")

                # 为已有的叠加图手动添加颜色条
                norm = mpl.colors.Normalize(vmin=levels.min(), vmax=levels.max())
                sm = mpl.cm.ScalarMappable(cmap='jet', norm=norm)
                sm.set_array([])

                divider = make_axes_locatable(ax)
                cax = divider.append_axes("right", size="5%", pad=0.08)
                cbar = fig.colorbar(sm, cax=cax, ticks=levels, spacing='proportional')
                cbar.set_label("花粉浓度 (μg/m³)", fontsize=10)
                cbar.ax.tick_params(labelsize=9)

            except Exception as e:
                print(f"[plot_local_result] 加载模型叠加图失败: {e}")
        else:
            # ===== 背景地图 =====
            if basemap_path and os.path.exists(basemap_path):
                img = Image.open(basemap_path)
                ax.imshow(img, extent=map_extent, origin="upper")

            # ===== 填色等高线（去掉线条，仅分级填色） =====
            # 对无扩散区域（≤0）不着色，保持背景显示
            data = np.ma.masked_less_equal(conc, 0.0)
            cfs = ax.contourf(x, y, data, levels=levels, cmap='jet', alpha=0.40, antialiased=True)

            # === 更细的、靠右的颜色条，使用填色对象使色块更明显 ===
            divider = make_axes_locatable(ax)
            cax = divider.append_axes("right", size="5%", pad=0.08)
            cbar = fig.colorbar(cfs, cax=cax, ticks=levels, spacing='proportional')
            cbar.set_label("花粉浓度 (μg/m³)", fontsize=10)
            cbar.ax.tick_params(labelsize=9)

        # ===== 坐标轴和种植区域（公共部分） =====
        # 获取当前设置的种植区域半径（作为回退值）
        # 获取当前设置的种植区域半径（作为回退值）
        try:
            # 输入为边长，需除以2
            input_side = float(self.radius_input.text() or "100")
            if input_side <= 0:
                input_side = 100.0
            input_radius = input_side / 2.0
        except (ValueError, AttributeError):
            input_radius = 50.0

        # 分别确定非转基因与转基因的半径（优先使用选择时的半径）
        try:
            non_radius = float(self.non_gm_radius) if getattr(self, 'non_gm_radius', None) else input_radius
        except Exception:
            non_radius = input_radius
        try:
            gm_radius = float(self.gm_radius) if getattr(self, 'gm_radius', None) else input_radius
        except Exception:
            gm_radius = input_radius

        # 绘制非转基因种植区域边界（受体，位于中心）- 矩形
        non_side = non_radius * 2  # 矩形边长为直径
        # 修改颜色为蓝色
        non_gm_rect = plt.Rectangle((-non_radius, -non_radius), non_side, non_side,
                                   fill=False, color='blue', linewidth=2, linestyle='--')
        ax.add_patch(non_gm_rect)
        # 移除非转基因区域文字注释
        # ax.text(0, non_radius + 50, '非转基因区域\n(受体)', ...)

        # 如果已选定转基因种植区域，则在其对应位置绘制红色矩形（使用转基因半径）
        try:
            if hasattr(self, 'last_gm_center') and hasattr(self, 'last_non_gm_center') and self.last_gm_center and self.last_non_gm_center:
                gm_lat = float(self.last_non_gm_center.get('lat', 0.0))
                meters_per_deg_lat = 111000.0
                meters_per_deg_lng = 111000.0 * math.cos(math.radians(gm_lat)) if gm_lat else 111000.0
                d_lng = float(self.last_gm_center['lng']) - float(self.last_non_gm_center['lng'])
                d_lat = float(self.last_gm_center['lat']) - float(self.last_non_gm_center['lat'])
                dx_m = d_lng * meters_per_deg_lng
                dy_m = d_lat * meters_per_deg_lat

                gm_side = gm_radius * 2
                gm_rect = plt.Rectangle((dx_m - gm_radius, dy_m - gm_radius), gm_side, gm_side,
                                       fill=False, color='red', linewidth=3, linestyle='-')
                ax.add_patch(gm_rect)
                # 移除转基因区域文字注释
                # ax.text(dx_m, dy_m + gm_radius + 50, '转基因区域\n(花粉源)', ...)
        except Exception as e:
            print(f"[plot_local_result] 绘制转基因矩形失败: {e}")

        # 使用动态边长设置坐标轴范围
        if hasattr(self, 'current_dynamic_size_km') and self.current_dynamic_size_km:
            # 将公里转换为米，并设置为边长的一半作为坐标轴范围
            half_range_m = int(self.current_dynamic_size_km * 1000 / 2)
            ax.set_xlim(-half_range_m, half_range_m)
            ax.set_ylim(-half_range_m, half_range_m)
            
            # 动态设置刻度，根据范围大小调整刻度间隔
            if half_range_m <= 1000:  # 1km以内，每250m一个刻度
                tick_interval = 250
            elif half_range_m <= 2500:  # 2.5km以内，每500m一个刻度
                tick_interval = 500
            elif half_range_m <= 5000:  # 5km以内，每1km一个刻度
                tick_interval = 1000
            else:  # 5km以上，每2km一个刻度
                tick_interval = 2000
            
            # 生成刻度
            ticks = list(range(-half_range_m, half_range_m + 1, tick_interval))
            ax.set_xticks(ticks)
            ax.set_yticks(ticks)
            
            print(f"[DEBUG] 结果图使用动态范围: ±{half_range_m}m, 刻度间隔: {tick_interval}m")
        else:
            # 回退到固定范围（兼容性）
            ax.set_xlim(-500, 500)
            ax.set_ylim(-500, 500)
            ticks = [-500, -250, 0, 250, 500]
            ax.set_xticks(ticks)
            ax.set_yticks(ticks)
            print("[DEBUG] 结果图使用固定范围: ±500m")
        
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("花粉浓度扩散分布（从转基因区域向外传播）")
        
        # 设置x轴标签倾斜显示，避免数字重叠；y轴保持正常显示
        ax.tick_params(axis='x', rotation=45, labelsize=9)
        ax.tick_params(axis='y', labelsize=9)

        canvas = FigureCanvas(fig)
        self.canvas_layout.addWidget(canvas)

    def export_csv(self):
        """导出数据为CSV文件"""
        path, _ = QFileDialog.getSaveFileName(self, "保存CSV", "监测数据.csv", "CSV Files (*.csv)")
        if path:
            with open(path, 'w', newline='', encoding='utf-8-sig') as f:
                writer = csv.writer(f)
                writer.writerow(["时间", "温度", "湿度", "风速", "风向"])
                writer.writerows(self.data_log)
            QMessageBox.information(self, "成功", "数据已导出")

    def show_error_message(self, message):
        """显示错误消息对话框"""
        QMessageBox.critical(self, "错误", message)

    def run_js(self, js_code, callback=None, description=None):
        """统一封装 JS 调用与错误处理"""
        try:
            page = self.model_map_view.page()
            if callback is not None:
                page.runJavaScript(js_code, callback)
            else:
                page.runJavaScript(js_code)
        except Exception as e:
            self.show_error_message(f"{description or '执行脚本'}失败：{e}")
    
    def center_main_map_to_input(self):
        """读取经纬度输入并让监测页地图定位到该位置"""
        try:
            lng_text = self.main_lng_input.text().strip()
            lat_text = self.main_lat_input.text().strip()
            if not lng_text or not lat_text:
                QMessageBox.warning(self, "提示", "请输入经度和纬度")
                return
            lng = float(lng_text)
            lat = float(lat_text)
            if not (-180 <= lng <= 180) or not (-90 <= lat <= 90):
                QMessageBox.warning(self, "提示", "请输入有效的经纬度范围：经度[-180,180]，纬度[-90,90]")
                return
            js = f"window.setMapCenter({lng}, {lat})"
            try:
                page = self.map_view.page()
                page.runJavaScript(js, lambda ok: self.statusBar().showMessage(
                    f"监测地图已定位到 经度:{lng:.6f}, 纬度:{lat:.6f}"
                ) if ok else QMessageBox.warning(self, "提示", "地图定位失败，请检查经纬度是否为百度坐标系（BD-09）"))
            except Exception as e:
                self.show_error_message(f"监测地图执行脚本失败：{e}")
        except Exception as e:
            self.show_error_message(f"监测定位出错：{e}")

    def center_map_to_input(self):
        """读取经纬度输入并让左侧地图定位到该位置，同时更新1km×1km矩形"""
        try:
            lng_text = self.lng_input.text().strip()
            lat_text = self.lat_input.text().strip()
            if not lng_text or not lat_text:
                QMessageBox.warning(self, "提示", "请输入经度和纬度")
                return
            lng = float(lng_text)
            lat = float(lat_text)
            if not (-180 <= lng <= 180) or not (-90 <= lat <= 90):
                QMessageBox.warning(self, "提示", "请输入有效的经纬度范围：经度[-180,180]，纬度[-90,90]")
                return
            js = f"window.setMapCenter({lng}, {lat})"
            self.run_js(
                js,
                callback=(lambda ok: self.statusBar().showMessage(
                    f"已定位到 经度:{lng:.6f}, 纬度:{lat:.6f}"
                ) if ok else QMessageBox.warning(self, "提示", "地图定位失败，请检查经纬度是否为百度坐标系（BD-09）")),
                description="地图定位"
            )
        except Exception as e:
            self.show_error_message(f"定位出错：{e}")

    def calculate_drift_heatmap(self):
        """计算并绘制基因漂移概率的二维热力图（基于最近一次模拟的浓度场）。
        概率公式：G = (A * D_d) / (A * D_d + D_r)，其中 D = e^{-k_p L} * C * v_d。
        """
        try:
            # 1) 最近一次模型数据
            if not hasattr(self, 'latest_model_data') or self.latest_model_data is None:
                self.show_error_message("尚无模拟结果。请先在‘花粉浓度扩散’页点击‘运行模拟’。")
                return
            x = self.latest_model_data.get("x")
            y = self.latest_model_data.get("y")
            conc = self.latest_model_data.get("conc")
            if x is None or y is None or conc is None:
                self.show_error_message("模拟结果不完整，无法计算热力图。")
                return

            # 2) 读取参数
            try:
                kp = float((self.kp_input.text() or "0.3").strip())
                L = float((self.L_input.text() or "2.0").strip())
                vd = float((self.vd_input.text() or "0.02").strip())
                cp = float((self.cp_input.text() or "0.5").strip())
                C_rec = float((self.crec_input.text() or "10.0").strip())
                if kp < 0 or L < 0 or vd <= 0 or not (0 < cp <= 0.5) or C_rec < 0:
                    raise ValueError
            except Exception:
                self.show_error_message("请输入有效参数：k_p≥0, L≥0, v_d>0, 0< c_p ≤0.5, C_rec≥0。")
                return

            # 3) 概率场
            decay = np.exp(-kp * L)
            C = np.maximum(conc, 0.0)
            D_d = decay * C * vd
            D_r = decay * max(C_rec, 0.0) * vd
            A = (cp / (1.0 - cp))**2
            prob_grid = (A * D_d) / (A * D_d + D_r + 1e-12)
            prob_grid = np.clip(prob_grid, 0.0, 1.0)

            # 4) 绘制热力图
            self.drift_ax.clear()
            xmin, xmax = float(np.min(x)), float(np.max(x))
            ymin, ymax = float(np.min(y)), float(np.max(y))
            im = self.drift_ax.imshow(
                prob_grid, origin='lower', extent=[xmin, xmax, ymin, ymax],
                cmap='inferno', vmin=0.0, vmax=1.0, aspect='equal'
            )
            self.drift_ax.set_xlabel("x (m)")
            self.drift_ax.set_ylabel("y (m)")
            self.drift_ax.set_title("基因漂移概率热力图 G(x,y)")

            # 色标
            divider = make_axes_locatable(self.drift_ax)
            cax = divider.append_axes("right", size="5%", pad=0.05)
            cb = self.drift_fig.colorbar(im, cax=cax)
            cb.set_label("概率")

            self.drift_canvas.draw()

            # 5) 文本摘要
            pmax = float(np.max(prob_grid))
            self.drift_result.setText(
                f"热力图生成完成：最大概率≈{pmax*100:.2f}%（k_p={kp:g}, L={L:g}, v_d={vd:g}, c_p={cp:g}, C_rec={C_rec:g}）。上风向区域概率将接近 0，这是正常现象。"
            )

            if hasattr(self, 'drift_ready_label') and self.drift_ready_label:
                self.drift_ready_label.setText("已基于最近模拟结果计算完成（热力图）")

        except Exception as e:
            self.show_error_message(f"热力图计算出错：{e}")
