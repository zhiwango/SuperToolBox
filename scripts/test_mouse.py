import pyautogui
import time

while True:
    pyautogui.move(10, 0, duration=0.1)  # 右移 10 像素
    pyautogui.move(-10, 0, duration=0.1)  # 左移 10 像素
    time.sleep(30)  # 每 30 秒执行一次
