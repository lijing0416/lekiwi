import serial

ser = serial.Serial("/dev/ttyACM0", 1000000, timeout=0.1)

def checksum(data):
    return (~sum(data)) & 0xFF

def write_data(servo_id, addr, value):
    packet = [
        0xFF, 0xFF,
        servo_id,
        4,
        3,
        addr,
        value
    ]
    packet.append(checksum(packet[2:]))
    ser.write(bytearray(packet))
    

# 把6号改成1号
write_data(6, 5, 1)

print("ID修改完成")