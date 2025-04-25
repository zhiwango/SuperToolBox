import socket
import can

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

print("开始接收 UDP 并注入到 vcan0")

while True:
    data, addr = sock.recvfrom(1024)
    if len(data) >= 4:
        can_id = int.from_bytes(data[:4], 'big')
        can_data = data[4:]
        msg = can.Message(arbitration_id=can_id, data=can_data, is_extended_id=False)
        try:
            bus.send(msg)
        except can.CanError as e:
            print(f"发送失败: {e}")
