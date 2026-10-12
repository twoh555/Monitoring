# 转基因作物花粉漂移监测与风险评估系统

一个基于 Python / PyQt5 的桌面端监测与模拟系统，集成实时环境监测、百度地图交互、高斯羽模型扩散模拟与基因漂移风险评估。

> 当前代码中已内置一个百度地图 AK，克隆后可直接运行。  
> 但公开 AK 存在被限流或封禁的风险，长期使用建议替换为自己的 AK。

 功能特性
- 实时监测：风速、风向、温度、湿度、GPS 数据采集与显示
- 地图交互：百度地图定位、框选转基因 / 非转基因种植区域
- 扩散模拟：高斯羽模型计算花粉浓度空间分布
- 风险评估：计算非转基因区域内的基因漂移频率，输出安全距离
- 可视化：浓度热力图、漂移频率热力图、频率-距离曲线
- 数据导出：支持浓度矩阵、漂移频率矩阵导出为 CSV
- GPU 加速：检测到 PyTorch + CUDA 时自动启用 GPU，否则回退 CPU 多进程

 目录结构
```text
Monitoring/
├── code/                       # Python 源代码
│   ├── main.py                 # 程序入口
│   ├── monitor_app.py          # 主窗口与业务逻辑
│   ├── hardware_serial.py      # 硬件串口通信
│   ├── serial_thread.py        # 模拟串口线程
│   ├── gaussian_plume_model.py # 高斯烟羽模型
│   ├── gauss_func.py           # 高斯公式底层实现
│   ├── calc_sigmas.py          # 扩散参数计算
│   ├── drift_model.py          # 基因漂移概率模型
│   ├── model_thread.py         # 模型计算线程
│   ├── map_utils.py            # 地图工具
│   ├── torch_gaussian_model.py # 可选 PyTorch GPU 加速
│   ├── map.html                # 监测地图页面
│   ├── model_map.html          # 模型区域选择地图页面
│   └── temp_map.html           # 临时地图页面
├── temp_maps/                  # 运行过程中生成的临时地图文件
├── requirements.txt            # 依赖清单
└── README.md
```

> **注意**：`__pycache__/`、`temp_maps/`、`C_donor.csv`属于运行生成的缓存或输出文件，建议不要上传到 GitHub，可在 `.gitignore` 中忽略。


 环境要求
- 操作系统：Windows 10 / 11 64 位（推荐）
- Python：3.8 及以上，推荐 3.10 / 3.11
- 依赖：见 `requirements.txt`
- 硬件（可选）：CH340 USB 转串口、STM32 / Modbus 气象传感器、GPS 模块
- GPU（可选）：NVIDIA 显卡 + CUDA + PyTorch GPU 版

 快速启动
 1. 克隆项目
```bash
git clone https://github.com/twoh555/main.git
cd Monitoring
```

 2. 创建虚拟环境
Windows：
```bash
python -m venv venv
venv\Scripts\activate
```

Linux / macOS：
```bash
python3 -m venv venv
source venv/bin/activate
```

 3. 安装依赖
```bash
pip install -r requirements.txt
```

如果需要 GPU 加速，额外安装 PyTorch（根据你的 CUDA 版本选择）：
```bash
pip install torch --index-url https://download.pytorch.org/whl/cu121
```
不安装 PyTorch 也能运行，程序会自动回退到 CPU 多进程计算。

 4. 运行程序
必须进入 `code` 目录后运行，否则会报模块找不到的错误：
```bash
cd code
python main.py
```

 使用流程
1. 启动程序：在 `code` 目录下运行 `python main.py`。
2. 选择数据源：
   - 无硬件：选择“模拟数据”，点击“连接”。
   - 有硬件：选择“硬件设备”，刷新端口，选择 CH340 对应的 COM 口，波特率默认 115200，点击“连接”。
3. 定义种植区域：
   - 切换到“花粉浓度扩散”页。
   - 点击“框选转基因区域”，在地图上右键选择红色区域。
   - 点击“框选非转基因区域”，在地图上右键选择蓝色区域。
   - 可使用左侧直角坐标 / 极坐标面板精确调整受体位置。
4. 运行扩散模拟：
   - 确认风速、风向等参数（可手动输入，或使用传感器数据）。
   - 点击“运行模拟”，等待完成。
5. 评估基因漂移：
   - 切换到“基因漂移概率”页。
   - 设置生物学参数（可保持默认）。
   - 点击“生成热力图”，查看概率分布与安全距离。
6. 导出数据：
   - 各页面均有“导出 CSV”按钮，可保存浓度矩阵或概率矩阵。


 百度地图 AK 说明
当前代码中已内置一个公共 AK，克隆后可直接运行。  
但公开 AK 存在以下风险：
- 可能被百度地图限流或封禁
- 可能被他人盗用，导致配额耗尽
- 无法保证长期稳定可用

如需长期使用，建议前往 [百度地图开放平台](https://lbsyun.baidu.com/) 申请自己的 AK，并替换以下位置：
- `code/map_utils.py` 中的 `ak = "..."`
- `code/map.html` 中的 `<script src="...ak=...">`
- `code/model_map.html` 中的 `<script src="...ak=...">`
- `code/temp_map.html` 中的 `<script src="...ak=...">`
- `code/monitor_app.py` 中内嵌 HTML 的 AK 字符串


 GPU 加速
程序启动时会尝试导入 PyTorch 并检测 CUDA。  
如果可用，控制台会显示：
```text
[DEBUG] 使用 PyTorch GPU 加速计算
```
否则自动回退到 CPU 多进程计算。  
建议配置：NVIDIA 显卡（8GB 显存以上）、CUDA 11.8 / 12.1、对应版本的 PyTorch。


 构建分发包
如需将程序打包为 Windows 可执行文件，可使用 PyInstaller：

```bash
pip install pyinstaller
pyinstaller --noconfirm --clean --windowed --name PollenDriftMonitor ^
  --collect-all PyQt5 ^
  --collect-all PyQtWebEngine ^
  --hidden-import PyQt5.QtWebEngineWidgets ^
  --hidden-import PyQt5.QtWebEngineCore ^
  code/main.py
```

打包完成后，`dist/PollenDriftMonitor/` 文件夹内即为可执行程序，可压缩后上传至 GitHub Releases。


 常见问题
Q：地图不显示怎么办？  
A：检查网络连接，确认百度地图 AK 有效。如果 AK 被限流，可更换为自己的 AK。

Q：串口打不开？
A：检查 USB 是否插好，安装 CH340 驱动，确认 COM 口未被占用。

Q：模拟速度慢？
A：安装 PyTorch GPU 版可大幅加速；也可减小模拟范围或增大网格分辨率。

Q：没有 GPU 能用吗？  
A：可以，程序会自动使用 CPU 多进程计算。

Q：运行时报模块找不到？
A：请确保在 `code` 目录下运行 `python main.py`，并已正确安装 `requirements.txt` 中的依赖。
