# show_color.py
# Reads lines like "123,45,67\n" from Arduino (baud 9600) and shows the color in a Tk window.

import sys
import tkinter as tk
from tkinter import simpledialog, messagebox
import serial
import serial.tools.list_ports
import threading
import time

BAUDRATE = 9600
READ_TIMEOUT = 1.0  # seconds

def choose_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    print("Available serial ports:")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")
    # If running headless, pick the first. Otherwise ask user.
    try:
        index = int(input("Choose port index (number): "))
        return ports[index].device
    except Exception:
        return ports[0].device

class SerialColorReader(threading.Thread):
    def __init__(self, port, baud=BAUDRATE):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.running = False
        self.ser = None
        self.latest = (0,0,0)

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=READ_TIMEOUT)
        except Exception as e:
            print("Cannot open serial port:", e)
            return
        self.running = True
        print("Serial opened:", self.port)
        # small startup delay
        time.sleep(0.2)
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                # Expect format: "r,g,b"
                parts = [s.strip() for s in line.split(',') if s.strip() != '']
                if len(parts) >= 3:
                    try:
                        r = int(float(parts[0]))
                        g = int(float(parts[1]))
                        b = int(float(parts[2]))
                        # constrain
                        r = max(0, min(255, r))
                        g = max(0, min(255, g))
                        b = max(0, min(255, b))
                        self.latest = (r, g, b)
                    except ValueError:
                        # ignore parse errors
                        pass
            except Exception:
                # on any serial read error, wait a bit
                time.sleep(0.1)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def stop(self):
        self.running = False

def rgb_to_hex(rgb):
    return "#{:02x}{:02x}{:02x}".format(*rgb)

def main():
    print("Make sure Arduino Serial Monitor is closed (only one program can use the port).")
    port = choose_port()
    if not port:
        messagebox.showerror("No port", "No serial ports found. Connect Arduino and try again.")
        return

    reader = SerialColorReader(port)
    reader.start()

    # Build GUI
    root = tk.Tk()
    root.title("TCS3200 Live Color Viewer")
    root.geometry("480x360")

    canvas = tk.Canvas(root, width=480, height=300)
    canvas.pack()

    info = tk.Label(root, text="Waiting for data...", font=("Helvetica", 14))
    info.pack(pady=8)

    hex_label = tk.Label(root, text="", font=("Helvetica", 12))
    hex_label.pack()

    def update_gui():
        r, g, b = reader.latest
        hexc = rgb_to_hex((r, g, b))
        canvas.configure(bg=hexc)
        info.configure(text=f"RGB: {r}, {g}, {b}")
        hex_label.configure(text=f"HEX: {hexc}")
        root.after(80, update_gui)  # refresh ~12.5 times per second

    def on_close():
        if messagebox.askokcancel("Quit", "Stop and exit?"):
            reader.stop()
            # allow thread to finish
            time.sleep(0.1)
            root.destroy()
            sys.exit(0)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(100, update_gui)
    root.mainloop()

if __name__ == "__main__":
    main()
