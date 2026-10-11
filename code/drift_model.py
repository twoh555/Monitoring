import numpy as np
import math
import sys
from gaussian_plume_model import run_gaussian_model

def calculate_drift_probability(
    x_full, y_full, conc_donor_full,
    non_gm_radius,
    beta1, beta2, vd, cp,
    wind_speed, wind_dir, grid_km,
    dxy=10.0,
    pollen_factor=5000000.0,   # 这里代表 Q_base（基础源强）
    Rd=0.50,                   # 逃逸系数
    return_full_grid=False
):
    """
    计算非转基因区域内的基因漂移概率。
    
    参数:
    - x_full, y_full: 完整网格的坐标矩阵 (meshgrid)
    - conc_donor_full: 完整网格的供体花粉浓度 (from GM to everywhere)
    - non_gm_radius: 非转基因区域半径 (m)
    - beta1, beta2, vd, cp: 模型参数
    - wind_speed, wind_dir: 环境参数 (用于计算受体自身浓度)
    - grid_km: 模拟区域大小 (km)
    - dxy: 网格分辨率 (m)
    - pollen_factor: 基础源强 (Q_base)
    - Rd: 逃逸系数 (0~1)
    
    返回:
    - x_sub, y_sub: 裁剪后的局部坐标
    - G_percent: 基因漂移概率矩阵 (%)
    """
    
    print(f"[DriftModel] 开始计算基因漂移概率...")
    print(f"[DriftModel] 参数: R={non_gm_radius}, beta1={beta1}, beta2={beta2}, vd={vd}, cp={cp}, Q_base={pollen_factor}, Rd={Rd}")
    print(f"[DriftModel] 环境: Wind={wind_speed}m/s @ {wind_dir}deg")
    
    # 计算 Rc (非逃逸比例)
    Rc = 1.0 - Rd
    
    # 根据 Q_base 和逃逸系数，计算各自的源强
    Q_donor = pollen_factor * Rd           # 供体逃逸飘散源强
    Q_recipient = pollen_factor * Rd       # 受体逃逸飘散源强
    Q_recipient_direct = pollen_factor * Rc # 受体直接落回源强
    
    # 源区面积 (假设每个网格点代表一个源区，面积为 dxy^2)
    source_area = dxy * dxy
    
    # 1. 计算受体自身花粉浓度 (Source at 0,0)
    # 受体自身是非转基因作物，其产生的逃逸花粉在自身区域内的分布
    # 假设受体区域中心为原点 (0,0)
    print("[DriftModel] 计算受体自身花粉浓度 (Recipient Self-Pollination)...")
    try:
        _, x_rec, y_rec, conc_recipient_full, _ = run_gaussian_model(
            wind_speed_value=wind_speed,
            wind_dir_value=wind_dir,
            grid_km=grid_km,
            dxy=dxy,
            source_radius=non_gm_radius,
            gm_offset_x=0.0,  # 源点在原点
            gm_offset_y=0.0,
            wind_mode='constant',
            temperature_value=None,
            humidity_value=None,
            pollen_factor=pollen_factor,  # 使用受体逃逸飘散源强
            height_diff=3.0
        )
    except Exception as e:
        print(f"[DriftModel] Error calculating recipient concentration: {e}")
        raise e

    # 2. 提取非转基因区域内的子网格 (如果 return_full_grid=False)
    # 假设模型坐标系原点(0,0)即为非转基因区域中心
    if return_full_grid:
        print("[DriftModel] 返回完整网格数据")
        x_sub = x_full
        y_sub = y_full
        conc_donor_sub = conc_donor_full
        conc_recipient_sub = conc_recipient_full
    else:
        radius = float(non_gm_radius)
        
        # 找到索引范围
        x_indices = np.where((x_full[0, :] >= -radius) & (x_full[0, :] <= radius))[0]
        y_indices = np.where((y_full[:, 0] >= -radius) & (y_full[:, 0] <= radius))[0]
        
        if len(x_indices) == 0 or len(y_indices) == 0:
            raise ValueError("非转基因区域内没有网格点 (分辨率不足或半径太小)")

        # 使用 np.ix_ 提取子块
        ix_grid = np.ix_(y_indices, x_indices)
        
        x_sub = x_full[ix_grid]
        y_sub = y_full[ix_grid]
        conc_donor_sub = conc_donor_full[ix_grid]
        conc_recipient_sub = conc_recipient_full[ix_grid]
    
    # --- 调试信息 ---
    print(f"[DriftModel] Subgrid shape: {x_sub.shape}")
    print(f"[DriftModel] X range: {x_sub.min():.2f} to {x_sub.max():.2f}")
    print(f"[DriftModel] Y range: {y_sub.min():.2f} to {y_sub.max():.2f}")
    
    rows, cols = x_sub.shape
    
    print("[DriftModel] --- Corner Check ---")
    print(f"BL (0,0)   : x={x_sub[0,0]:.1f}, y={y_sub[0,0]:.1f}, Donor={conc_donor_sub[0,0]:.2e}")
    print(f"BR (0,-1)  : x={x_sub[0,-1]:.1f}, y={y_sub[0,-1]:.1f}, Donor={conc_donor_sub[0,-1]:.2e}")
    print(f"TL (-1,0)  : x={x_sub[-1,0]:.1f}, y={y_sub[-1,0]:.1f}, Donor={conc_donor_sub[-1,0]:.2e}")
    print(f"TR (-1,-1) : x={x_sub[-1,-1]:.1f}, y={y_sub[-1,-1]:.1f}, Donor={conc_donor_sub[-1,-1]:.2e}")
    
    # 3. 计算 G (基因漂移概率)
    exp_b1 = math.exp(-beta1)
    exp_b2 = math.exp(-beta2)
    
    #    供体、受体的浓度都是基于 Q_base 算出来的（同一基准），
    #    因此都需要乘以 Rd 换算为"真实逃逸源强对应的浓度"。
    #    受体自身的直接落回量基于 Q_base * Rc，不经过扩散，不需要再乘 Rd。
    D_donor = exp_b1 * (conc_donor_sub * Rd) * vd 
    
    # 受体沉积量 = 直接落回量 + 外部飘入量
    # 直接落回量 = Q_recipient_direct / source_area
    D_recipient_direct = Q_recipient_direct / source_area
    D_recipient = exp_b2 * D_recipient_direct + exp_b1 * (conc_recipient_sub * Rd) * vd
    
    denominator = D_donor + D_recipient

    with np.errstate(divide='ignore', invalid='ignore'):
        x = np.where(denominator != 0, D_donor / (denominator + 1e-12), 0.0)

    A = ((1.0 - cp) / cp) ** 2 if cp != 0 else 1e10

    if abs(A - 1.0) < 1e-7:
        # cp ≈ 0.5 时，A ≈ 1，极限形式
        G = x
    else:
        G = (np.power(A, x) - 1.0) / (A - 1.0)

    # 4. 掩码：如果返回全网格，将非转基因区域外的概率设为 0
    if return_full_grid:
        radius = float(non_gm_radius)
        mask = (np.abs(x_sub) > radius) | (np.abs(y_sub) > radius)
        G[mask] = 0.0
    
    return x_sub, y_sub, G
