import torch
import numpy as np
import math
import time

def calc_sigmas_torch(category, x, device):
    """
    PyTorch vectorized version of calc_sigmas.
    category: int (1-6)
    x: torch.Tensor
    """
    abs_x = torch.abs(x)
    a = torch.zeros_like(abs_x)
    b = torch.zeros_like(abs_x)
    c_val = 0.0
    d_val = 0.0

    # Categorized constants (simplified port of calc_sigmas.py for core logic)
    # Focusing on common categories like 4 (Neutral)
    if category == 1:
        # very unstable - omitted full table for brevity but implementing some
        mask1 = (abs_x < 100) & (abs_x > 0)
        a[mask1], b[mask1] = 122.8, 0.9447
        mask2 = (abs_x >= 100)
        a[mask2], b[mask2] = 158.08, 1.0542 # Simplified for large x
        c_val, d_val = 24.167, 2.5334
    elif category == 4: # Neutral (Default in app)
        mask1 = (abs_x < 300) & (abs_x > 0)
        a[mask1], b[mask1] = 34.459, 0.86974
        mask2 = (abs_x >= 300) & (abs_x < 1000)
        a[mask2], b[mask2] = 32.093, 0.81066
        mask3 = (abs_x >= 1000)
        a[mask3], b[mask3] = 32.093, 0.64403
        c_val, d_val = 8.333, 0.72382
    else:
        # Generic fallback if not fully implemented
        a.fill_(34.0)
        b.fill_(0.8)
        c_val, d_val = 10.0, 1.0

    sig_z = a * (abs_x / 1000.0) ** b
    sig_z = torch.clamp(sig_z, max=5000.0)

    theta = 0.017453293 * (c_val - d_val * torch.log(abs_x + 1e-15 / 1000.0))
    sig_y = 465.11628 * abs_x / 1000.0 * torch.tan(theta)

    return sig_y, sig_z

def gauss_func_torch(Q, u, dir1, x, y, z, xs, ys, H, kp, L, Dy, Dz, STABILITY, device, v_d=0.25):
    """
    PyTorch vectorized gauss_func.
    Supports broadcasting over multiple source points.
    Q, xs, ys, H: Tensors of shape (N_sources,)
    x, y, z: Tensors of shape (Height, Width)
    """
    N = Q.shape[0]
    H_grid, W_grid = x.shape

    # Expand source dims: (N, 1, 1)
    Qs = Q.view(N, 1, 1)
    xs_v = xs.view(N, 1, 1)
    ys_v = ys.view(N, 1, 1)
    Hs = H.view(N, 1, 1)

    # Relative coordinates: (N, H, W)
    x1 = x - xs_v
    y1 = y - ys_v

    # Wind components
    rad_conv = math.pi / 180.0
    wx = u * math.sin((dir1 - 180.0) * rad_conv)
    wy = u * math.cos((dir1 - 180.0) * rad_conv)

    # Vectorized downwind/crosswind calculation
    dot_product = wx * x1 + wy * y1
    dist_sq = x1**2 + y1**2
    dist = torch.sqrt(dist_sq)
    
    # We can simplify downwind/crosswind using rotation
    # theta_wind = math.atan2(wx, wy)
    # and use geometry: downwind is projection on wind vector
    downwind = dot_product / (u + 1e-15)
    
    # crosswind magnitude
    # Using cross product formula in 2D or sqrt(dist^2 - downwind^2)
    crosswind_sq = torch.clamp(dist_sq - downwind**2, min=0.0)
    crosswind = torch.sqrt(crosswind_sq)

    # Mask for downwind only
    mask = downwind > 0.0
    C = torch.zeros((N, H_grid, W_grid), device=device)

    # Sigmas (N, H, W)
    sig_y, sig_z = calc_sigmas_torch(STABILITY, downwind, device)

    # Depletion correction
    H_prime = Hs - (v_d * downwind / u)

    # Gaussian formula
    # Factor: Q / (2pi * u * sig_y * sig_z)
    factor = Qs / (2.0 * math.pi * u * sig_y * sig_z + 1e-15)
    term_y = torch.exp(-crosswind**2 / (2.0 * sig_y**2 + 1e-15))
    term_z = torch.exp(-(z - H_prime)**2 / (2.0 * sig_z**2 + 1e-15)) + \
             torch.exp(-(z + H_prime)**2 / (2.0 * sig_z**2 + 1e-15))

    C = factor * term_y * term_z
    C = torch.where(mask, C, torch.tensor(0.0, device=device))

    return torch.sum(C, dim=0) # Sum over source points -> (H, W)

def run_gaussian_model_torch(wind_speed_value=5.0, wind_dir_value=10.0,
                             grid_km=1.0, dxy=1, source_radius=500.0,
                             gm_offset_x=0.0, gm_offset_y=0.0,
                             wind_mode='constant',
                             temperature_value=None, humidity_value=None,
                             pollen_factor=1.0, height_diff=3.0):
    """
    Optimized GPU implementation using PyTorch.
    """
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"[DEBUG] PyTorch Accelerator: Using device {device}")
    
    # Domain setup
    Lm = float(grid_km) * 1000.0
    half = Lm / 2.0
    dxy = float(dxy)
    
    # Create grid on device
    x_range = torch.arange(-half, half + dxy, dxy, device=device)
    y_range = torch.arange(-half, half + dxy, dxy, device=device)
    grid_y, grid_x = torch.meshgrid(y_range, x_range, indexing='ij')
    grid_z = torch.zeros_like(grid_x)

    # Source points
    side_length = float(source_radius) * 2.0
    grid_size = int(side_length / 10.0)
    if grid_size < 1: grid_size = 1
    
    spacing = 10.0
    width = (grid_size - 1) * spacing
    offsets = torch.linspace(-width/2, width/2, grid_size, device=device)
    os_y, os_x = torch.meshgrid(offsets, offsets, indexing='ij')
    
    xs = os_x.flatten() + gm_offset_x
    ys = os_y.flatten() + gm_offset_y
    stacks = len(xs)
    
    Qs = torch.full((stacks,), float(pollen_factor), device=device)
    Hs = torch.full((stacks,), float(height_diff), device=device)
    
    # Time steps
    times = 24
    if str(wind_mode).lower() in ['prevailing', '盛行风']:
        # Note: using numpy for rng consistency with original if needed, 
        # but torch.randn is faster.
        wind_dir_seq = torch.normal(mean=float(wind_dir_value), std=40.0, size=(times,), device=device)
        wind_dir_seq = torch.remainder(wind_dir_seq, 360.0)
    else:
        wind_dir_seq = torch.full((times,), float(wind_dir_value), device=device)

    # Accumulator
    total_conc = torch.zeros_like(grid_x)
    
    # Batch processing to fit in VRAM (8GB limits)
    # 801x801 is ~0.64M cells. (S, H, W) float32 is S * 2.5MB.
    # For 8GB, S=1000 consumes 2.5GB. S=2000 consumes 5GB.
    # We use batch_size = 500 to be safe on a laptop GPU.
    batch_size = 500
    
    start_time = time.time()
    for i in range(times):
        wdir = wind_dir_seq[i].item()
        
        # Process source points in batches
        for b_start in range(0, stacks, batch_size):
            b_end = min(b_start + batch_size, stacks)
            Q_b = Qs[b_start:b_end]
            xs_b = xs[b_start:b_end]
            ys_b = ys[b_start:b_end]
            H_b = Hs[b_start:b_end]
            
            total_conc += gauss_func_torch(Q_b, wind_speed_value, wdir, 
                                          grid_x, grid_y, grid_z, 
                                          xs_b, ys_b, H_b, 
                                          0.55, 3.0, 10.0, 10.0, 4, 
                                          device)
            
    torch.cuda.synchronize()
    end_time = time.time()
    print(f"[DEBUG] PyTorch GPU 计算耗时: {end_time - start_time:.4f}s")

    # Final result back to CPU NumPy
    conc_np = total_conc.cpu().numpy()
    x_np = grid_x.cpu().numpy()
    y_np = grid_y.cpu().numpy()
    
    return x_np, y_np, conc_np, side_length # Return same structure as needed
