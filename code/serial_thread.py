"""串口通信线程模块
包含串口数据读取线程和传感器数据结构的实现
"""
import struct
import threading
import serial
from PyQt5.QtCore import QObject, pyqtSignal


class SensorData:
    """传感器数据结构，用于存储和解析传感器数据包"""
    def __init__(self):
        self.temp = 0.0
        self.humi = 0
        self.wind_speed = 0.0
        self.wind_direction = 0.0

    def update_from_packet(self, packet):
        """从数据包更新传感器数据
        
        Args:
            packet: 33字节的数据包
            
        Returns:
            bool: 数据包解析是否成功
        """
        if len(packet) != 33:
            return False
        checksum = 0
        for i in range(32):
            checksum ^= packet[i]
        if checksum != packet[32]:
            return False
        self.temp = packet[1] + packet[2] / 10.0
        self.humi = packet[3]
        self.wind_speed = struct.unpack('>f', packet[4:8])[0]
        self.wind_direction = struct.unpack('>f', packet[8:12])[0]
        return True


class DataUpdateSignal(QObject):
    """数据更新信号，用于在线程间传递传感器数据"""
    update_signal = pyqtSignal(SensorData)


class SerialThread(threading.Thread):
    """串口读取线程，负责从串口读取传感器数据并发送信号"""
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.signal = DataUpdateSignal()
        self.sensor_data = SensorData()

    def run(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            buffer = bytearray()
            while self.running:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer.extend(data)
                    while len(buffer) >= 33:
                        header_index = buffer.find(0xA5)
                        if header_index == -1:
                            buffer.clear()
                            break
                        if header_index + 33 > len(buffer):
                            break
                        packet = buffer[header_index:header_index + 33]
                        if self.sensor_data.update_from_packet(packet):
                            self.signal.update_signal.emit(self.sensor_data)
                        buffer = buffer[header_index + 33:]
        except Exception as e:
            print(f"串口通信错误: {e}")
            self.signal.update_signal.emit(None)
        finally:
            self.close()

    def close(self):
        """关闭串口连接"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()