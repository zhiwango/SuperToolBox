import sys
import json
import os
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QMainWindow, QMessageBox, QListWidget, QListWidgetItem,
)
from PyQt5.QtCore import QProcess
from ui_main import Ui_mainWindow
from PyQt5.QtWidgets import QFileDialog
from config.re_localizaiton import re_localization_topics


class ScriptRunner(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_mainWindow()
        self.ui.setupUi(self)
        # Initialization
        ## main process
        self.process_main = QProcess()
        self.process_main.setProcessChannelMode(QProcess.MergedChannels)
        self.process_main.readyReadStandardOutput.connect(self.read_output)
        self.process_main.finished.connect(self.on_process_finished)
        ## sub process
        self.process_sub = QProcess()
        self.process_sub.setProcessChannelMode(QProcess.MergedChannels)

        ##############################################################################################
        # Autoware Tab - Logging Simulator
        ##############################################################################################
        self.ui.pushButton_run_logging_simulator.clicked.connect(self.run_logging_simulator)
        self.ui.pushButton_stop_logging_simulator.clicked.connect(self.stop_logging_simulator)
        self.ui.pushButton_stop_logging_simulator.setEnabled(False)
        # 设置默认 map 路径
        map_path = os.path.expanduser("~/load_data/")
        self.ui.comboBox_map_path.clear()
        self.ui.comboBox_map_path.addItem(map_path)
        # 自动加载 load_data 目录下的所有文件夹作为选项
        for item in os.listdir(map_path):
            full_map_path = os.path.join(map_path, item)
            if os.path.isdir(full_map_path):
                self.ui.comboBox_map_path.addItem(full_map_path)
        self.ui.comboBox_map_path.setCurrentText(map_path)

        ##############################################################################################
        # Autoware Tab - ROS2 Bag Player
        ##############################################################################################
        self.ui.pushButton_play_rosbag.clicked.connect(self.play_rosbag)
        self.ui.pushButton_stop_rosbag.clicked.connect(self.stop_rosbag)
        # 设置默认 rosbag 路径
        rosbag_path = os.path.expanduser("~/rosbag/")
        self.ui.comboBox_rosbag_path.clear()
        self.ui.comboBox_rosbag_path.addItem(rosbag_path)
        # 自动加载 rosbag 目录下的所有 .db3 文件作为选项

        for root, dirs, files in os.walk(rosbag_path):
            for file in files:
                if file.endswith(".db3"):
                    full_path = os.path.join(root, file)
                    self.ui.comboBox_rosbag_path.addItem(full_path)
        self.ui.comboBox_rosbag_path.setCurrentText(rosbag_path)

        ##############################################################################################
        # Others Tab
        ##############################################################################################
        self.ui.startButton.clicked.connect(self.run_script)
        self.ui.stopButton.clicked.connect(self.stop_script)
        self.ui.scriptList.itemClicked.connect(self.select_script)
        self.ui.outputArea.setReadOnly(True)
        self.ui.stopButton.setEnabled(False)
        self.load_scripts()
        ##############################################################################################

    def run_logging_simulator(self):
        map_path = self.ui.comboBox_map_path.currentText()

        # 判断选中的车型
        if self.ui.radioButton_vehicle_model_gen1.isChecked():
            vehicle_model = "j6_gen1"
            sensor_model = "aip_x2"
        elif self.ui.radioButton_vehicle_model_gen2.isChecked():
            vehicle_model = "j6_gen2"
            sensor_model = "aip_x2_gen2"
        else:
            vehicle_model = "default"
            sensor_model = "default"
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
            "lanelet2_map_file:=lanelet2_map.osm", "pointcloud_map_file:=pointcloud_map.pcd",
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
        self.process_main.start("bash", ["-c", " ".join(command)])
        self.ui.pushButton_run_logging_simulator.setEnabled(False)
        self.ui.pushButton_stop_logging_simulator.setEnabled(True)

    def stop_logging_simulator(self):
        if self.process_main.state() != QProcess.NotRunning:
            self.process_main.kill()
            self.ui.outputArea.append("\n[已停止执行]\n")
        # 额外终止所有 ros 相关进程
        cleanup_cmd = "pgrep -a -f ros | grep -v Microsoft | grep -v ros2_daemon | awk '{ print \"kill -9\", $1 }' | sh"
        os.system(cleanup_cmd)
        self.ui.outputArea.append("[已强制关闭所有 ROS 相关进程]\n")
        self.ui.pushButton_run_logging_simulator.setEnabled(True)
        self.ui.pushButton_stop_logging_simulator.setEnabled(False)

    def play_rosbag(self):
        rosbag_path = self.ui.comboBox_rosbag_path.currentText()
        command = ["ros2", "bag", "play", rosbag_path, "--clock", "100", "-s", "sqlite3"]
        # remap_list = self.build_remap_args(rosbag_path)
        # command.extend(remap_list)
        topic_list = []
        if self.ui.radioButton_for_localization.isChecked():
            topic_list = re_localization_topics
        elif self.ui.radioButton_for_perception.isChecked():
            topic_list = []
        elif self.ui.radioButton_for_planning.isChecked():
            topic_list = []

        if topic_list:
            command.append("--topics")
            command.extend(topic_list)

        command_str = " ".join(command)
        self.ui.outputArea.append("\n执行命令:\n" + command_str)
        self.process_sub.start("bash", ["-c", command_str])

    def stop_rosbag(self):
        if self.process_sub.state() == QProcess.Running:
            self.process_sub.kill()
            self.ui.outputArea.append("脚本已被终止。\n")
            self.ui.stopButton.setEnabled(False)

    # TODO: pause rosbag play

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
        self.process_main.start("/bin/bash", ["-c", cmd])
        self.ui.stopButton.setEnabled(True)

    def stop_script(self):
        if self.process_main.state() == QProcess.Running:
            self.process_main.kill()
            self.ui.outputArea.append("脚本已被终止。\n")
            self.ui.stopButton.setEnabled(False)

    def on_process_finished(self):
        self.ui.outputArea.append("脚本执行结束。\n")
        self.ui.stopButton.setEnabled(False)

    def read_output(self):
        output = self.process_main.readAllStandardOutput().data().decode()
        self.ui.outputArea.append(output)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ScriptRunner()
    window.show()
    sys.exit(app.exec_())
