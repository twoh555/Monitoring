import numpy as np
import math
import sys
from gaussian_plume_model import run_gaussian_model

def calculate_drift_probability(
    x_full, y_full, conc_donor_full,
    non_gm_radius,
    kp, L, vd, cp,
    wind_speed, wind_dir, grid_km,
    dxy=10.0,
    pollen_factor=5000000.0,
    return_full_grid=False
):
    """
    计算非转基因区域内的基因漂移概率。
    
    参数:
    - x_full, y_full: 完整网格的坐标矩阵 (meshgrid)
    - conc_donor_full: 完整网格的供体花粉浓度 (from GM to everywhere)
    - non_gm_radius: 非转基因区域半径 (m)
    - kp, L, vd, cp: 模型参数
    - wind_speed, wind_dir: 环境参数 (用于计算受体自身浓度)
    - grid_km: 模拟区域大小 (km)
    - dxy: 网格分辨率 (m)
    - pollen_factor: 源强 (Q)，用于计算受体自身浓度
    
    返回:
    - x_sub, y_sub: 裁剪后的局部坐标
    - G_percent: 基因漂移概率矩阵 (%)
    """
    
    print(f"[DriftModel] 开始计算基因漂移概率...")
    print(f"[DriftModel] 参数: R={non_gm_radius}, kp={kp}, L={L}, vd={vd}, cp={cp}, Q_source={pollen_factor}")
    print(f"[DriftModel] 环境: Wind={wind_speed}m/s @ {wind_dir}deg")
    
    # 1. 计算受体自身花粉浓度 (Source at 0,0)
    # 受体自身是非转基因作物，其产生的花粉在自身区域内的分布
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
            pollen_factor=pollen_factor,  # 使用传入的源强
            height_diff=3.0 # 受体通常假设高度差较小？或者保持一致。这里保持默认。
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
        
        # x_full 和 y_full 应该是 meshgrid 产生的
        # ... (保留原有注释)
        
        # 找到索引范围
        x_indices = np.where((x_full[0, :] >= -radius) & (x_full[0, :] <= radius))[0]
        y_indices = np.where((y_full[:, 0] >= -radius) & (y_full[:, 0] <= radius))[0]
        
        if len(x_indices) == 0 or len(y_indices) == 0:
            # 如果选中区域太小或分辨率不够，至少返回中心点附近 3x3
            # 或者抛出异常。这里尝试回退到完整网格或抛出异常
            # 暂时保持抛出异常
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
    
    # 打印四角坐标和浓度，确认方向
    # row 0, col 0 -> Bottom-Left
    # row -1, col -1 -> Top-Right
    rows, cols = x_sub.shape
    
    print("[DriftModel] --- Corner Check ---")
    print(f"BL (0,0)   : x={x_sub[0,0]:.1f}, y={y_sub[0,0]:.1f}, Donor={conc_donor_sub[0,0]:.2e}")
    print(f"BR (0,-1)  : x={x_sub[0,-1]:.1f}, y={y_sub[0,-1]:.1f}, Donor={conc_donor_sub[0,-1]:.2e}")
    print(f"TL (-1,0)  : x={x_sub[-1,0]:.1f}, y={y_sub[-1,0]:.1f}, Donor={conc_donor_sub[-1,0]:.2e}")
    print(f"TR (-1,-1) : x={x_sub[-1,-1]:.1f}, y={y_sub[-1,-1]:.1f}, Donor={conc_donor_sub[-1,-1]:.2e}")
    
    # 3. 计算 G
    # 公式更新：D_recipient = exp(-kp*L)*Q + exp(-kp*L)*Conc_rec*vd
    # 其中 Q 是供体源强 (pollen_factor)，而非受体的 Q_rec
    decay = math.exp(-kp * L)
    D_donor = decay * conc_donor_sub * vd
    D_recipient = decay * pollen_factor + decay * conc_recipient_sub * vd
    A = (cp / (1.0 - cp)) ** 2
    G = (A * D_donor) / (A * D_donor + D_recipient + 1e-12)
    
    G_percent = G * 100.0
    
    return x_sub, y_sub, G_percent
