"""监测系统主入口模块
负责启动主应用程序
"""
import sys

from PyQt5.QtWidgets import QApplication

from monitor_app import MonitorApp

def main():
    """程序主入口函数"""
    app = QApplication(sys.argv)
    window = MonitorApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":  
    main()