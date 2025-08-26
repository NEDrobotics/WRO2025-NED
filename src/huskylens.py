import serial
import time

def parse_block(data):
    x = int.from_bytes(data[0:2], 'little')
    y = int.from_bytes(data[2:4], 'little')
    w = int.from_bytes(data[4:6], 'little')
    h = int.from_bytes(data[6:8], 'little')
    _id = int.from_bytes(data[8:10], 'little')
    return {'id': _id, 'x': x, 'y': y, 'w': w, 'h': h}

def find_frame(buf):
    i = buf.find(b'\x55\xAA')
    if i == -1 or i+4 > len(buf):
        return None, buf
    length = buf[i+3]
    end = i + 4 + length + 1
    if end > len(buf):
        return None, buf
    frame = buf[i:end]
    return frame, buf[end:]

ser = serial.Serial("COM12", 9600, timeout=1)
time.sleep(3.5)

# Send "request blocks" command
ser.write(bytes([0x55, 0xAA, 0x11, 0x00, 0x21, 0x31]))

buf = ser.read(100)
print("Raw:", buf)

# Parse frames
while buf:
    frame, buf = find_frame(buf)
    if not frame:
        break
    cmd = frame[4]
    payload = frame[5:-1]  # exclude checksum
    if cmd == 0x29:
        print(f"Info frame: {int.from_bytes(payload[0:2], 'little')} block(s)")
    elif cmd == 0x2A:
        block = parse_block(payload)
        print("Detected block:", block)

ser.close()
