import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# 读取 CSV 文件
folder = r"C:\Users\ASUS\PycharmProjects\pythonProject1\JiyinPiaoyi\Gauss\C_donor和C_recipient(修改Q后)\恒定风，间距=10m"
C_donor = pd.read_csv(f"{folder}\\C_donor，间距10m，风速=1.csv", header=None).values
C_recipient = pd.read_csv(f"{folder}\\C_recipient，风速=1.csv", header=None).values

# 定义参数
kp = 0.55  # 冠层截取花粉粒的系数
L = 3.0  # 穗高以上的累积叶面积指数
v_d = 0.25  # 沉降速度m/s
Q_recipient = 5e6 * 0.8418 # 受体花粉源强度
wind_speed = 1

# 冠层穿透率
trans = np.exp(-kp * L)
# 计算 D_donor
D_donor = trans * C_donor * v_d
# 计算 D_recipient
D_recipient = trans * Q_recipient + trans * C_recipient * v_d

# 计算比值
ratio = D_donor / (D_donor + D_recipient)
# 替换所有异常值为 0
ratio = np.nan_to_num(ratio, nan=0.0, posinf=0.0, neginf=0.0)

#热力图
plt.figure(figsize=(12, 6))
plt.imshow(ratio, cmap='Reds', aspect='auto',
           extent=[0, ratio.shape[1], ratio.shape[0], 0])
plt.clim(0,1)
plt.colorbar(label='D_donor / (D_donor + D_recipient)')
plt.xlabel('Column Position')
plt.ylabel('Donor Pollen Ratio')
plt.title('Pollen Deposition Ratio by Row\n' +
          f'Wind Speed: {wind_speed:.1f} m/s')
plt.tight_layout()
plt.show()
