import socket
import can

UDP_IP = "192.168.1.18"  # 接收端 IP
UDP_PORT = 5005

bus = can.interface.Bus(channel='pcan8', bustype='socketcan')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("开始监听 vcan0 并转发到 UDP", UDP_IP)

for msg in bus:
    data = msg.arbitration_id.to_bytes(4, 'big') + msg.data
    sock.sendto(data, (UDP_IP, UDP_PORT))
