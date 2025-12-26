import numpy as np
from calc_sigmas import calc_sigmas

def gauss_func(Q, u, dir1, x, y, z, xs, ys, H, kp, L, Dy, Dz, STABILITY, v_d=0.25):
    u1 = u
    # 坐标转换到源点为中心
    x1 = x - xs
    y1 = y - ys

    # 风速在x/y方向分量
    wx = u1 * np.sin((dir1 - 180.0) * np.pi / 180.0)
    wy = u1 * np.cos((dir1 - 180.0) * np.pi / 180.0)

    # 与风向的夹角与下风/侧风距离
    dot_product = wx * x1 + wy * y1
    magnitudes = u1 * np.sqrt(x1 ** 2 + y1 ** 2)
    subtended = np.arccos(dot_product / (magnitudes + 1e-15))
    hypotenuse = np.sqrt(x1 ** 2 + y1 ** 2)
    downwind = np.cos(subtended) * hypotenuse
    crosswind = np.sin(subtended) * hypotenuse

    ind = np.where(downwind > 0.0)
    C = np.zeros((len(x), len(y)))

    # 湍流扩散尺度
    sig_y, sig_z = calc_sigmas(STABILITY, downwind)

    # 沉降修正后的等效源高
    H_prime = H - (v_d * downwind[ind] / u1)

    C[ind] = (
        Q / (2.0 * np.pi * u1 * sig_y[ind] * sig_z[ind])
        * np.exp(-crosswind[ind] ** 2 / (2.0 * sig_y[ind] ** 2))
        * (
            np.exp(-(z[ind] - H_prime) ** 2 / (2.0 * sig_z[ind] ** 2))
            + np.exp(-(z[ind] + H_prime) ** 2 / (2.0 * sig_z[ind] ** 2))
        )
    )

    return C

