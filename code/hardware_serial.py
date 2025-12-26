"""硬件串口通信模块
专门用于处理CH340连接的硬件设备数据
支持风速、风向、温度、湿度、GPS等多种传感器数据的实时读取和解析
"""
import re
import threading
import serial
import serial.tools.list_ports
import time
import struct
from datetime import datetime, timedelta
from PyQt5.QtCore import QObject, pyqtSignal
from dataclasses import dataclass
from typing import Optional, List


@dataclass
class HardwareSensorData:
    """硬件传感器数据结构"""
    timestamp: str = ""
    wind_speed: float = 0.0
    wind_direction: float = 0.0
    temperature: float = 0.0
    humidity: float = 0.0
    gps_latitude: float = 0.0
    gps_longitude: float = 0.0
    # 为兼容UI读取，增加别名字段
    latitude: float = 0.0
    longitude: float = 0.0
    gps_altitude: float = 0.0
    raw_data: str = ""


class HardwareDataUpdateSignal(QObject):
    """硬件数据更新信号"""
    update_signal = pyqtSignal(HardwareSensorData)
    error_signal = pyqtSignal(str)


class HardwareSerialThread(threading.Thread):
    """硬件串口读取线程"""
    
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.signal = HardwareDataUpdateSignal()
        self.sensor_data = HardwareSensorData()
        
    def convert_utc_to_beijing_time(self, utc_hour: int, utc_minute: int, utc_second: int) -> str:
        """将UTC时间转换为北京时间
        
        根据Keil代码中的时间转换逻辑：
        beijing_h = (Read_GPS_UTC_HOUR() + 8) % 24
        """
        try:
            # 创建UTC时间
            utc_time = datetime.now().replace(hour=utc_hour, minute=utc_minute, second=utc_second, microsecond=0)
            
            # 转换为北京时间 (UTC+8)
            beijing_time = utc_time + timedelta(hours=8)
            
            # 处理跨日情况
            beijing_hour = beijing_time.hour
            beijing_minute = beijing_time.minute
            beijing_second = beijing_time.second
            
            return f"{beijing_hour:02d}:{beijing_minute:02d}:{beijing_second:02d}"
            
        except Exception as e:
            print(f"时间转换错误: {e}")
            return f"{utc_hour:02d}:{utc_minute:02d}:{utc_second:02d}"
    
    def parse_modbus_response(self, data: bytes) -> Optional[dict]:
        """解析Modbus RTU响应数据
        
        根据Keil代码，风速风向传感器使用Modbus协议
        风速传感器地址: USART2
        风向传感器地址: USART3
        """
        try:
            if len(data) < 7:  # Modbus RTU最小响应长度
                return None
            
            # 解析Modbus RTU响应格式
            # [设备地址][功能码][数据长度][数据][CRC16]
            device_addr = data[0]
            function_code = data[1]
            data_length = data[2]
            
            if function_code == 0x03:  # 读保持寄存器
                if len(data) >= 5 + data_length:
                    # 提取数据部分
                    register_data = data[3:3+data_length]
                    
                    # 解析16位寄存器值
                    if len(register_data) >= 2:
                        value = struct.unpack('>H', register_data[:2])[0]  # 大端序
                        
                        # 根据设备地址判断传感器类型
                        if device_addr == 0x01:  # 风速传感器
                            return {"wind_speed": value / 10.0}  # 除以10转换为实际值
                        elif device_addr == 0x02:  # 风向传感器
                            return {"wind_direction": value / 10.0}  # 除以10转换为实际值
            
            return None
            
        except (struct.error, IndexError) as e:
            print(f"Modbus数据解析错误: {e}")
            return None
    
    def parse_hardware_data(self, line: str) -> Optional[HardwareSensorData]:
        """解析硬件数据行，匹配STM32输出格式
        
        根据Keil代码分析，硬件输出格式为：
        时间:HH:MM:SS\r\n
        LATN  LONE\r\n
        温度:TTC  湿度:HH%\r\n
        风速:V.V  风向:DDD\r\n\r\n
        
        Args:
            line: 从串口读取的数据行
            
        Returns:
            HardwareSensorData: 解析后的传感器数据，解析失败返回None
        """
        try:
            # 清理数据行
            line = line.strip()
            if not line:
                return None
            
            # 初始化数据缓冲区（如果不存在）
            if not hasattr(self, 'current_data_buffer'):
                self.current_data_buffer = {}
            
            # STM32输出格式解析
            # 格式: "时间:HH:MM:SS\r\nLATN  LONE\r\n温度:TTC  湿度:HH%\r\n风速:V.V  风向:DDD\r\n"
            
            # 如果包含"时间:"，开始新的数据包解析
            if "时间:" in line:
                # 允许负号与变长数字，随后做规范化
                time_match = re.search(r"时间:\s*(-?\d+):(-?\d+):(-?\d+)", line)
                if time_match:
                    h_raw, m_raw, s_raw = time_match.groups()
                    try:
                        h = int(h_raw)
                        m = int(m_raw)
                        s = int(s_raw)
                        # 规范化到合法范围（避免负值造成匹配失败）
                        h = ((h % 24) + 24) % 24
                        m = abs(m) % 60
                        s = abs(s) % 60
                        timestamp = f"{h:02d}:{m:02d}:{s:02d}"
                    except Exception:
                        timestamp = time.strftime("%H:%M:%S")
                    self.current_data_buffer = {"timestamp": timestamp}
                return None
            
            # 解析GPS坐标行 (格式: "39.9042N  116.4074E")
            gps_match = re.search(r'([\d.]+)N\s+([\d.]+)E', line)
            if gps_match:
                latitude = float(gps_match.group(1))
                longitude = float(gps_match.group(2))
                self.current_data_buffer.update({
                    "gps_latitude": latitude,
                    "gps_longitude": longitude,
                    # 兼容UI字段
                    "latitude": latitude,
                    "longitude": longitude,
                })
                return None
            
            # 解析温湿度行 (格式: "温度:25C  湿度:60%")
            temp_humi_match = re.search(r'温度:(\d+)C\s+湿度:(\d+)%', line)
            if temp_humi_match:
                temperature = float(temp_humi_match.group(1))
                humidity = float(temp_humi_match.group(2))
                self.current_data_buffer.update({
                    "temperature": temperature,
                    "humidity": humidity
                })
                return None
            
            # 解析风速风向行 (格式: "风速:5.2  风向:180")
            wind_match = re.search(r'风速:([\d.]+)\s+风向:(\d+)', line)
            if wind_match:
                wind_speed = float(wind_match.group(1))
                wind_direction = float(wind_match.group(2))
                self.current_data_buffer.update({
                    "wind_speed": wind_speed,
                    "wind_direction": wind_direction
                })
                
                # 收集到温湿度和风速风向即可组成一次完整数据（时间可选）
                if all(key in self.current_data_buffer for key in [
                    "temperature", "humidity", "wind_speed", "wind_direction"
                ]):
                    complete_data = HardwareSensorData(
                        timestamp=self.current_data_buffer.get("timestamp", time.strftime("%H:%M:%S")),
                        wind_speed=self.current_data_buffer.get("wind_speed", 0.0),
                        wind_direction=self.current_data_buffer.get("wind_direction", 0.0),
                        temperature=self.current_data_buffer.get("temperature", 0.0),
                        humidity=self.current_data_buffer.get("humidity", 0.0),
                        gps_latitude=self.current_data_buffer.get("gps_latitude", 0.0),
                        gps_longitude=self.current_data_buffer.get("gps_longitude", 0.0),
                        latitude=self.current_data_buffer.get("latitude", 0.0),
                        longitude=self.current_data_buffer.get("longitude", 0.0),
                        raw_data=line
                    )
                    self.current_data_buffer = {}
                    return complete_data
                return None
            
            # 兼容其他格式
            # 格式1: 逗号分隔 (时间戳,风速,风向,温度,湿度,纬度,经度)
            if ',' in line:
                parts = line.split(',')
                if len(parts) >= 6:
                    return HardwareSensorData(
                        timestamp=parts[0],
                        wind_speed=float(parts[1]),
                        wind_direction=float(parts[2]),
                        temperature=float(parts[3]),
                        humidity=float(parts[4]),
                        gps_latitude=float(parts[5]) if len(parts) > 5 else 0.0,
                        gps_longitude=float(parts[6]) if len(parts) > 6 else 0.0,
                        raw_data=line
                    )
            
            return None
            
        except (ValueError, IndexError) as e:
            print(f"数据解析错误: {e}, 原始数据: {line}")
            return None
    
    def run(self):
        """线程主循环"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            self.running = True
            print(f"硬件串口连接成功: {self.port}, 波特率: {self.baudrate}")
            
            buffer = ""
            
            while self.running:
                try:
                    if self.serial.in_waiting > 0:
                        # 读取数据
                        data = self.serial.read(self.serial.in_waiting)
                        
                        # 尝试不同的编码方式
                        try:
                            text = data.decode('utf-8')
                        except UnicodeDecodeError:
                            try:
                                text = data.decode('gbk')
                            except UnicodeDecodeError:
                                text = data.decode('ascii', errors='ignore')
                        
                        buffer += text
                        
                        # 按行处理数据
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            
                            if line:
                                # 解析数据
                                parsed_data = self.parse_hardware_data(line)
                                if parsed_data:
                                    self.signal.update_signal.emit(parsed_data)
                    
                    time.sleep(0.01)  # 短暂休眠，避免CPU占用过高
                    
                except serial.SerialException as e:
                    print(f"串口读取错误: {e}")
                    self.signal.error_signal.emit(f"串口读取错误: {e}")
                    break
                except Exception as e:
                    print(f"数据处理错误: {e}")
                    continue
                    
        except serial.SerialException as e:
            error_msg = f"无法打开串口 {self.port}: {e}"
            print(error_msg)
            self.signal.error_signal.emit(error_msg)
        except Exception as e:
            error_msg = f"硬件串口通信错误: {e}"
            print(error_msg)
            self.signal.error_signal.emit(error_msg)
        finally:
            self.close()
    
    def close(self):
        """关闭串口连接"""
        self.running = False
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
                print("硬件串口连接已关闭")
            except Exception as e:
                print(f"关闭串口时出错: {e}")
    
    def is_connected(self) -> bool:
        """检查串口是否连接"""
        return self.serial and self.serial.is_open and self.running


def get_available_ports():
    """获取可用的串口列表"""
    import serial.tools.list_ports
    
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append({
            'device': port.device,
            'description': port.description,
            'hwid': port.hwid
        })
    return ports


def find_ch340_ports():
    """查找CH340设备端口"""
    ch340_ports = []
    try:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # CH340设备的常见描述信息
            if any(keyword in port.description.upper() for keyword in ['CH340', 'USB-SERIAL', 'USB SERIAL']):
                ch340_ports.append(port.device)
            # 也检查硬件ID
            elif hasattr(port, 'hwid') and 'VID:PID=1A86:7523' in port.hwid:
                ch340_ports.append(port.device)
    except Exception as e:
        print(f"端口检测错误: {e}")
    
    return ch340_ports