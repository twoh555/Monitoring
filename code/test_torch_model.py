import sys
import os
# Add current dir to path
sys.path.append(os.getcwd())

try:
    from torch_gaussian_model import run_gaussian_model_torch
    import numpy as np
    
    print("Testing PyTorch Model...")
    x, y, conc, side = run_gaussian_model_torch(
        wind_speed_value=5.0, wind_dir_value=45.0,
        grid_km=1.0, dxy=10, source_radius=100.0,
        gm_offset_x=0.0, gm_offset_y=0.0,
        pollen_factor=1e6
    )
    
    print(f"Success! Result shape: {conc.shape}")
    print(f"Max concentration: {np.max(conc)}")
    print(f"Mean concentration: {np.mean(conc)}")
    
except Exception as e:
    print(f"Error during sanity check: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
