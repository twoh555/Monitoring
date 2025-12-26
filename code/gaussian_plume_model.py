import matplotlib
matplotlib.use('Agg')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from gauss_func import gauss_func
import tempfile, os
import sys
from multiprocessing import Pool, cpu_count

def compute_time_step_worker(args):
    """计算单个时间步的浓度场"""
    i, wind_dir_i, Q_arr, xs_arr, ys_arr, H_arr, wind_speed, x_grid, y_grid, z_grid, kp_val, L_val, Dy_val, Dz_val, stab = args
    C_time = np.zeros_like(x_grid)
    for j in range(len(Q_arr)):
        C = gauss_func(Q_arr[j], wind_speed, wind_dir_i,
                       x_grid, y_grid, z_grid, xs_arr[j], ys_arr[j], H_arr[j], 
                       kp_val, L_val, Dy_val, Dz_val, stab)
        C_time += C
    return i, C_time

def run_gaussian_model(wind_speed_value=5.0, wind_dir_value=10.0,
                       grid_km=1.0, dxy=1, source_radius=500.0,
                       gm_offset_x=0.0, gm_offset_y=0.0,
                       wind_mode='constant',
                       temperature_value=None, humidity_value=None,
                       pollen_factor=1.0, height_diff=3.0):
    """
    标准版模型（对齐用户训练脚本）：
    - 网格：grid_km×grid_km，默认1km，分辨率dxy（默认1m）
    - 面源：默认10×10规则方阵，间距1m，居中于(gm_offset_x, gm_offset_y)
    - 叠加：遍历24个时间步的恒定风向风速，累加得到C_donor
    返回: fig, x, y, C_donor, overlay_png_path
    """
    print(f"[DEBUG] 模型启动(标准版): wind_speed={wind_speed_value}, wind_dir={wind_dir_value}")
    if temperature_value is not None or humidity_value is not None:
        print(f"[DEBUG] 环境: 温度={temperature_value if temperature_value is not None else 'N/A'} °C, 湿度={humidity_value if humidity_value is not None else 'N/A'} %")
    print(f"[DEBUG] 源强系数: ×{pollen_factor}, 高度差: {height_diff} m")
    sys.stdout.flush()

    # 域设置
    Lm = float(grid_km) * 1000.0
    half = Lm / 2.0
    dxy = float(dxy)
    x_vals = np.arange(-half, half + dxy, dxy)
    y_vals = x_vals.copy()
    x, y = np.meshgrid(x_vals, y_vals)
    z = np.zeros_like(x)

    # 规则方阵面源（动态数量），居中于gm_offset
    # 使用 source_radius 确定区域大小 (边长 = 2 * radius)
    side_length = float(source_radius) * 2.0
    # 避免除以零
    if side_length <= 0:
        side_length = 10.0
    
    # 用户要求：固定10m间距，数量动态计算
    # 例如：边长10m -> 1个点；边长100m -> 100个点 (10x10)；边长1000m -> 10000个点 (100x100)
    grid_size = int(side_length / 10.0)
    if grid_size < 1: 
        grid_size = 1
        
    rows, cols = grid_size, grid_size
    spacing = 10.0
    width = (cols - 1) * spacing
    height = (rows - 1) * spacing
    xs_grid = np.linspace(-width / 2.0, width / 2.0, cols)
    ys_grid = np.linspace(-height / 2.0, height / 2.0, rows)
    xs_mesh, ys_mesh = np.meshgrid(xs_grid, ys_grid)
    xs = xs_mesh.ravel() + gm_offset_x
    ys = ys_mesh.ravel() + gm_offset_y
    stacks = len(xs)

    # 源参数（按标准脚本）
    # 源参数（按标准脚本）
    # pollen_factor 现在代表输入的源强总数 Q (例如 5e6)
    # 用户指示：输入值已包含逃逸率，无需再乘 0.1582
    base_Q = float(pollen_factor)
    Q = np.full(stacks, base_Q)
    H = np.full(stacks, float(height_diff))
    kp = 0.55
    L = 3.0
    Dy = 10.0
    Dz = 10.0
    STABILITY = 4

    # 时间步（全天24个小时）
    times = np.arange(1, 24 + 1) / 24.0
    # 风场模式：恒定风或盛行风（高斯扰动），盛行风以输入风向为均值
    if str(wind_mode).lower() in ['prevailing', '盛行风']:
        rng = np.random.default_rng()
        wind_dir_seq = rng.normal(loc=float(wind_dir_value), scale=40.0, size=len(times))
        wind_dir_seq = np.mod(wind_dir_seq, 360.0)
    else:
        wind_dir_seq = np.full(len(times), float(wind_dir_value))
    
    C1 = np.zeros((x.shape[0], x.shape[1], len(times)))
    print(f"[DEBUG] 叠加 {len(times)} 个时间步, 源点数量={stacks}")
    sys.stdout.flush()
    
    # 并行计算优化：仅在源点数量较多时启用（避免小任务的进程创建开销）
    use_parallel = stacks > 10  # 阈值：10个源点以上才使用并行
    
    if use_parallel:
        try:
            # 准备每个时间步的参数
            args_list = [
                (i, wind_dir_seq[i], Q, xs, ys, H, wind_speed_value, x, y, z, kp, L, Dy, Dz, STABILITY)
                for i in range(len(times))
            ]
            
            # 使用进程池并行计算
            num_workers = min(cpu_count(), len(times))
            print(f"[DEBUG] 使用并行计算: {num_workers} 个工作进程")
            sys.stdout.flush()
            
            with Pool(processes=num_workers) as pool:
                results = pool.map(compute_time_step_worker, args_list)
            
            # 收集结果
            for i, C_time in results:
                C1[:, :, i] = C_time
                
            print("[DEBUG] 并行计算完成")
            sys.stdout.flush()
            
        except Exception as e:
            print(f"[WARNING] 并行计算失败，回退到串行计算: {e}")
            sys.stdout.flush()
            use_parallel = False
    
    # 串行计算（回退或小任务）
    if not use_parallel:
        print("[DEBUG] 使用串行计算")
        sys.stdout.flush()
        for i in range(len(times)):
            for j in range(stacks):
                C = gauss_func(Q[j], wind_speed_value, wind_dir_seq[i],
                               x, y, z, xs[j], ys[j], H[j], kp, L, Dy, Dz, STABILITY)
                C1[:, :, i] += C

    # 累加得到C_donor
    C_donor = np.sum(C1, axis=2)
    mean_concentration_grain = C_donor

    # 保存矩形区域浓度矩阵到CSV（y反转）
    try:
        x_vals_1d = x[0, :]
        y_vals_1d = y[:, 0]
        half_w = width / 2.0
        half_h = height / 2.0
        x_mask = (x_vals_1d >= gm_offset_x - half_w) & (x_vals_1d <= gm_offset_x + half_w)
        y_mask = (y_vals_1d >= gm_offset_y - half_h) & (y_vals_1d <= gm_offset_y + half_h)
        C_local = mean_concentration_grain[np.ix_(y_mask, x_mask)]
        C_local_flipped = C_local[::-1, :]
        out_path = os.path.join(os.getcwd(), "C_donor.csv")
        np.savetxt(out_path, C_local_flipped, delimiter=',')
        print(f"[DEBUG] 成功保存C_donor到 {out_path}")
    except Exception as e:
        print(f"[WARN] 保存C_donor失败: {e}")

    # 绘制透明等高线覆盖层
    print("[DEBUG] 开始绘制填色等高线")
    sys.stdout.flush()
    px = py = 800
    dpi = 100
    fig = Figure(figsize=(px / dpi, py / dpi), dpi=dpi)
    ax = fig.add_axes([0, 0, 1, 1])

    # 对无扩散区域（≤0）不着色，保持透明
    data = np.ma.masked_less_equal(mean_concentration_grain, 0.0)
    positive = data.compressed()
    if positive.size > 0:
        vmax = float(np.percentile(positive, 99))
        if vmax <= 0 or not np.isfinite(vmax):
            vmax = float(np.max(positive)) if positive.size > 0 else 1.0
        vmin = float(np.percentile(positive, 5))
        if vmin <= 0 or not np.isfinite(vmin):
            vmin = float(np.min(positive)) if positive.size > 0 else 1e-6
        levels = np.linspace(vmin, vmax, 6)
    else:
        levels = [1, 2]

    cfs = ax.contourf(
        x, y, data,
        levels=levels,
        cmap='jet',
        alpha=0.40,
        antialiased=True,
        extend='max'
    )

    # 绘制面源矩形边界
    rect_x0 = gm_offset_x - width / 2.0
    rect_y0 = gm_offset_y - height / 2.0
    rectangle = plt.Rectangle((rect_x0, rect_y0), width, height, fill=False, color='red', linewidth=2, linestyle='--')
    ax.add_patch(rectangle)

    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.set_aspect('equal', adjustable='box')
    ax.axis('off')
    fig.patch.set_alpha(0.0)
    ax.set_facecolor("none")

    fd, overlay_png_path = tempfile.mkstemp(suffix=".png")
    os.close(fd)
    fig.savefig(overlay_png_path, transparent=True, dpi=dpi)

    print(f"[DEBUG] 等高线覆盖图已保存: {overlay_png_path}")
    sys.stdout.flush()

    return fig, x, y, mean_concentration_grain, overlay_png_path
