import sys
import json
import os
from PyQt5.QtWidgets import (
    QApplication, QWidget, QMainWindow, QMessageBox, QListWidget, QListWidgetItem,
)
from PyQt5.QtCore import QProcess
from ui_main import Ui_mainWindow
from PyQt5.QtWidgets import QFileDialog


class ScriptRunner(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_mainWindow()
        self.ui.setupUi(self)
        # Initialization
        self.process = QProcess()
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.finished.connect(self.on_process_finished)

        # Autoware Tab
        self.ui.run_logging_simulator.clicked.connect(self.run_logging_simulator)
        self.ui.stop_logging_simulator.clicked.connect(self.stop_logging_simulator)
        self.ui.stop_logging_simulator.setEnabled(False)

        # 设置默认 map 路径
        home_path = os.path.expanduser("~/load_data/")
        self.ui.load_map.clear()
        self.ui.load_map.addItem(home_path)

        # 自动加载 home 目录下的所有文件夹作为选项
        for item in os.listdir(home_path):
            full_path = os.path.join(home_path, item)
            if os.path.isdir(full_path):
                self.ui.load_map.addItem(full_path)

        self.ui.load_map.setCurrentText(home_path)
        # self.ui.load_map.setEditable(True)
        # self.ui.load_map.addItem(home_path)
        # self.ui.load_map.setCurrentText(home_path)

        # Others Tab
        self.ui.startButton.clicked.connect(self.run_script)
        self.ui.stopButton.clicked.connect(self.stop_script)
        self.ui.scriptList.itemClicked.connect(self.select_script)
        self.ui.outputArea.setReadOnly(True)
        self.ui.stopButton.setEnabled(False)

        self.load_scripts()

    def run_logging_simulator(self):
        map_path = self.ui.load_map.currentText()

        # 判断选中的车型
        if self.ui.vehicleModel_gen1.isChecked():
            vehicle_model = "j6_gen1"
            sensor_model = "aip_x2"
        elif self.ui.vehicleModel_gen2.isChecked():
            vehicle_model = "j6_gen2"
            sensor_model = "aip_x2_gen2"
        else:
            vehicle_model = "unknown"
            sensor_model = "unknown"
        # CheckBox 转小写字符串 true/false
        sensing = str(self.ui.checkBox_sensing.isChecked()).lower()
        perception = str(self.ui.checkBox_perception.isChecked()).lower()
        vehicle = str(self.ui.checkBox_vehicle.isChecked()).lower()
        rviz = str(self.ui.checkBox_rviz.isChecked()).lower()
        localization = str(self.ui.checkBox_localizaiton.isChecked()).lower()
        planning = str(self.ui.checkBox_planning.isChecked()).lower()
        control = str(self.ui.checkBox_control.isChecked()).lower()
        system = str(self.ui.checkBox_system.isChecked()).lower()
        command = [
            # f"ros2 launch autoware_launch logging_simulator.launch.xml "
            "ros2", "launch", "autoware_launch", "logging_simulator.launch.xml",
            f"map_path:={map_path}",
            f"vehicle_model:={vehicle_model}",
            f"sensor_model:={sensor_model}",
            f"sensing:={sensing}",
            f"perception:={perception}",
            f"vehicle:={vehicle}",
            f"rviz:={rviz}",
            f"localization:={localization}",
            f"planning:={planning}",
            f"control:={control}",
            f"system:={system}"
        ]
        self.ui.outputArea.append("\n执行命令:\n" + " ".join(command))
        self.process.start("bash", ["-c", " ".join(command)])
        self.ui.run_logging_simulator.setEnabled(False)
        self.ui.stop_logging_simulator.setEnabled(True)

    def stop_logging_simulator(self):
        if self.process.state() != QProcess.NotRunning:
            self.process.kill()
            self.ui.outputArea.append("\n[已停止执行]\n")
        # 额外终止所有 ros 相关进程
        cleanup_cmd = "pgrep -a -f ros | grep -v Microsoft | grep -v ros2_daemon | awk '{ print \"kill -9\", $1 }' | sh"
        os.system(cleanup_cmd)
        self.ui.outputArea.append("[已强制关闭所有 ROS 相关进程]\n")
        self.ui.run_logging_simulator.setEnabled(True)
        self.ui.stop_logging_simulator.setEnabled(False)

    def select_map_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "选择地图文件夹", os.path.expanduser("~/load_data/"))
        if folder:
            if self.ui.comboBox_map.findText(folder) == -1:
                self.ui.comboBox_map.addItem(folder)
            self.ui.comboBox_map.setCurrentText(folder)

    def load_scripts(self):
        try:
            with open("config/scripts.json", "r", encoding="utf-8") as f:
                self.scripts = json.load(f)
                self.ui.scriptList.clear()
                for script in self.scripts:
                    item = QListWidgetItem(script["name"])
                    item.setData(1000, script)
                    self.ui.scriptList.addItem(item)
        except Exception as e:
            QMessageBox.critical(self, "加载失败", f"无法加载 scripts.json: {e}")

    def select_script(self, item):
        self.selected_script = item.data(1000)

    def run_script(self):
        if not hasattr(self, 'selected_script'):
            self.ui.outputArea.append("请先选择一个脚本。\n")
            return
        cmd = self.selected_script["path"]
        if self.selected_script.get("sudo"):
            cmd = f"sudo -S {cmd}"
        self.ui.outputArea.append(f"运行: {cmd}")
        self.process.start("/bin/bash", ["-c", cmd])
        self.ui.stopButton.setEnabled(True)

    def stop_script(self):
        if self.process.state() == QProcess.Running:
            self.process.kill()
            self.ui.outputArea.append("脚本已被终止。\n")
            self.ui.stopButton.setEnabled(False)

    def on_process_finished(self):
        self.ui.outputArea.append("脚本执行结束。\n")
        self.ui.stopButton.setEnabled(False)

    def read_output(self):
        output = self.process.readAllStandardOutput().data().decode()
        self.ui.outputArea.append(output)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ScriptRunner()
    window.show()
    sys.exit(app.exec_())
