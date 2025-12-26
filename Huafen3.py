import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import glob, os, re

os.chdir(r"C:\Users\ASUS\PycharmProjects\pythonProject1\JiyinPiaoyi\Gauss\C_donor和C_recipient(修改Q后)\平均值")
files = glob.glob('*间距*m.csv')

# 更简单的方法提取间距
def extract_spacing_simple(filename):
    # 直接查找数字
    numbers = re.findall(r'\d+', filename)
    # 取最后一个数字（通常是间距值）
    for num in reversed(numbers):
        num_int = int(num)
        if 10 <= num_int <= 100:  # 间距应该在10-100之间
            return num_int
    return 0

files_sorted = sorted(files, key=extract_spacing_simple)
spacing = [extract_spacing_simple(f) for f in files_sorted]

# 读取数据
ratios = []
for f in files_sorted:
    df = pd.read_csv(f, header=None)
    ratio_value = df.iloc[-1, 0]

    # 确保是数值
    ratio_value = float(ratio_value)
    ratios.append(ratio_value)

    # 使用传统字符串格式化
    spacing_val = extract_spacing_simple(f)
    print("文件 %s: 间距 = %dm, 比率 = %.6f" % (f, spacing_val, ratio_value))

# 绘制图表
# 最简单的垂直热力图版本
heatmap_data = np.array(ratios).reshape(-1, 1)
plt.figure(figsize=(3, 8))
im = plt.imshow(heatmap_data, cmap='RdYlBu_r', aspect='auto',
                extent=[0, 1, max(spacing), min(spacing)])
plt.ylabel('Spacing (m)')
plt.xlabel('wind_speed = 5 m/s')
plt.yticks(spacing)
plt.xticks([])
plt.title('Pollen Deposition Ratio')
plt.colorbar(im, label='D_donor / (D_donor + D_recipient)')
plt.tight_layout()
plt.show()