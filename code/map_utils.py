"""地图工具模块
包含地图下载、图像叠加等功能
"""
import os
import tempfile
import requests
from PIL import Image, ImageDraw
import io
import math


def download_baidu_static_image(center_lng, center_lat, bounds=None,
                              zoom=16, maptype="normal", width=800, height=800):
    """下载百度静态图（干净版本，无POI标注），返回保存路径"""
    os.makedirs("temp_maps", exist_ok=True)
    save_path = os.path.join("temp_maps", f"basemap_{center_lat:.6f}_{center_lng:.6f}.png")

    ak = "sXHZZmCrjOTisFCbeJnCFFNkUbfd4nF9"  # 你的 AK
    # 使用与左侧地图一致的干净样式ID
    style_id = "2b2e585dec698fa62f732b976f0ea1ac"

    # 如果提供了bounds参数，动态计算合适的zoom级别
    if bounds:
        # 计算bounds的经纬度跨度
        lat_span = bounds['ne']['lat'] - bounds['sw']['lat']
        lng_span = bounds['ne']['lng'] - bounds['sw']['lng']
        
        # 根据跨度计算合适的zoom级别
        # 百度地图zoom级别与覆盖范围的关系（大致估算）
        # zoom=18: ~0.001度, zoom=17: ~0.002度, zoom=16: ~0.004度, 
        # zoom=15: ~0.008度, zoom=14: ~0.016度, zoom=13: ~0.032度
        max_span = max(lat_span, lng_span)
        
        if max_span <= 0.002:
            calculated_zoom = 17
        elif max_span <= 0.004:
            calculated_zoom = 16
        elif max_span <= 0.008:
            calculated_zoom = 15
        elif max_span <= 0.016:
            calculated_zoom = 14
        elif max_span <= 0.032:
            calculated_zoom = 13
        elif max_span <= 0.064:
            calculated_zoom = 12
        elif max_span <= 0.128:
            calculated_zoom = 11
        else:
            calculated_zoom = 10
            
        zoom = calculated_zoom
        print(f"[DEBUG] 根据bounds动态计算zoom级别: 跨度={max_span:.6f}度, zoom={zoom}")

    # 构建基础URL，直接使用干净的样式ID
    base_url = (
        f"http://api.map.baidu.com/staticimage/v2?"
        f"ak={ak}"
        f"&center={center_lng},{center_lat}"
        f"&zoom={zoom}"
        f"&width={width}"
        f"&height={height}"
        f"&styles={style_id}"
    )

    print("[DEBUG] 使用与左侧地图一致的干净样式")

    try:
        print(f"[DEBUG] 正在下载底图: {base_url}")
        r = requests.get(base_url, timeout=15)
        r.raise_for_status()

        # 检查返回内容
        if len(r.content) < 1000:
            print(f"[警告] 返回内容太小: {len(r.content)} bytes，可能是错误响应")
            # 如果样式ID失败，尝试使用自定义样式来隐藏POI和标签
            custom_style = '[{"featureType":"poi","elementType":"all","stylers":{"visibility":"off"}},{"featureType":"road","elementType":"labels","stylers":{"visibility":"off"}},{"featureType":"administrative","elementType":"labels","stylers":{"visibility":"off"}}]'
            fallback_url = (
                f"http://api.map.baidu.com/staticimage/v2?"
                f"ak={ak}"
                f"&center={center_lng},{center_lat}"
                f"&zoom={zoom}"
                f"&width={width}"
                f"&height={height}"
                f"&styles={custom_style}"
            )
            print(f"[DEBUG] 尝试自定义样式URL: {fallback_url}")
            r = requests.get(fallback_url, timeout=15)
            r.raise_for_status()

        img = Image.open(io.BytesIO(r.content)).convert("RGBA")
        img.save(save_path)
        print(f"[DEBUG] 下载并保存底图成功: {save_path}")
        return save_path

    except Exception as e:
        print(f"[地图下载错误] {e}")
        print(f"[地图下载错误] URL: {base_url}")

        # 最后的降级方案：使用自定义样式隐藏不需要的元素
        try:
            # 更全面的自定义样式，隐藏POI、标签、行政区划等
            clean_style = '[{"featureType":"poi","elementType":"all","stylers":{"visibility":"off"}},{"featureType":"road","elementType":"labels","stylers":{"visibility":"off"}},{"featureType":"administrative","elementType":"labels","stylers":{"visibility":"off"}},{"featureType":"administrative","elementType":"geometry","stylers":{"visibility":"off"}},{"featureType":"building","elementType":"all","stylers":{"visibility":"off"}}]'
            clean_url = (
                f"http://api.map.baidu.com/staticimage/v2?"
                f"ak={ak}"
                f"&center={center_lng},{center_lat}"
                f"&zoom={zoom}"
                f"&width={width}"
                f"&height={height}"
                f"&styles={clean_style}"
            )
            print(f"[DEBUG] 尝试完全干净样式: {clean_url}")
            r = requests.get(clean_url, timeout=15)
            r.raise_for_status()
            img = Image.open(io.BytesIO(r.content)).convert("RGBA")
            img.save(save_path)
            print(f"[DEBUG] 干净样式下载成功: {save_path}")
            return save_path
        except Exception as e2:
            print(f"[地图下载完全失败] {e2}")
            return None


def overlay_contours_on_basemap(basemap_path, overlay_path, output_path=None, transparent=True):
    """将模型浓度覆盖层叠加到底图上"""
    try:
        base_img = Image.open(basemap_path).convert("RGBA")
        overlay_img = Image.open(overlay_path).convert("RGBA")

        # 调整 overlay 尺寸与底图一致
        overlay_img = overlay_img.resize(base_img.size, resample=Image.Resampling.LANCZOS)

        # 合成
        base_img.alpha_composite(overlay_img)

        # 如果没有指定输出路径，就生成一个临时文件
        if output_path is None:
            fd, output_path = tempfile.mkstemp(suffix=".png", prefix="final_map_")
            os.close(fd)

        base_img.save(output_path, format="PNG")
        return output_path

    except Exception as e:
        print("[叠加失败]", e)
        raise


def draw_planting_areas_on_image(image_path, bounds, gm_center=None, gm_radius=None, 
                                non_gm_center=None, non_gm_radius=None, output_path=None):
    """在图像上绘制种植区域矩形"""
    try:
        img = Image.open(image_path).convert("RGBA")
        draw = ImageDraw.Draw(img)
        
        # 图像尺寸
        img_width, img_height = img.size
        
        # 地图边界
        sw_lng, sw_lat = bounds['sw']['lng'], bounds['sw']['lat']
        ne_lng, ne_lat = bounds['ne']['lng'], bounds['ne']['lat']
        
        def coord_to_pixel(lng, lat):
            """将经纬度坐标转换为图像像素坐标"""
            x = int((lng - sw_lng) / (ne_lng - sw_lng) * img_width)
            y = int((ne_lat - lat) / (ne_lat - sw_lat) * img_height)
            return x, y
        
        def draw_rectangle(center_lng, center_lat, radius, color, label):
            """绘制矩形区域"""
            # 计算矩形边界（以半径为边长的一半）
            side_length = radius * 2  # 矩形边长为直径
            lat_offset = side_length / 111000  # 纬度偏移
            lng_offset = side_length / (111000 * math.cos(math.radians(center_lat)))  # 经度偏移
            
            # 矩形四个角的坐标
            sw_corner = (center_lng - lng_offset/2, center_lat - lat_offset/2)
            ne_corner = (center_lng + lng_offset/2, center_lat + lat_offset/2)
            
            # 转换为像素坐标
            x1, y1 = coord_to_pixel(sw_corner[0], ne_corner[1])  # 左上角
            x2, y2 = coord_to_pixel(ne_corner[0], sw_corner[1])  # 右下角
            
            # 绘制矩形边框
            draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
            
            # 绘制半透明填充
            overlay = Image.new('RGBA', img.size, (0, 0, 0, 0))
            overlay_draw = ImageDraw.Draw(overlay)
            fill_color = color + (80,)  # 添加透明度
            overlay_draw.rectangle([x1, y1, x2, y2], fill=fill_color)
            img.alpha_composite(overlay)
            
            print(f"[绘制] {label}矩形: 中心({center_lng:.6f}, {center_lat:.6f}), 半径{radius}m")
        
        # 绘制转基因种植区域（红色）
        if gm_center and gm_radius:
            draw_rectangle(gm_center['lng'], gm_center['lat'], gm_radius, (255, 0, 0), "转基因")
        
        # 绘制非转基因种植区域（改为蓝色）
        if non_gm_center and non_gm_radius:
            draw_rectangle(non_gm_center['lng'], non_gm_center['lat'], non_gm_radius, (0, 0, 255), "非转基因")
        
        # 保存结果
        if output_path is None:
            fd, output_path = tempfile.mkstemp(suffix=".png", prefix="map_with_areas_")
            os.close(fd)
        
        img.save(output_path, format="PNG")
        print(f"[绘制完成] 保存到: {output_path}")
        return output_path
        
    except Exception as e:
        print(f"[绘制种植区域失败] {e}")
        raise