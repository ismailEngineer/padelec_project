import serial

# ⚠️ adapte le port (COM3 sous Windows / /dev/ttyUSB0 sous Linux)
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

print("Lecture FTDI...")

while True:
    data = ser.readline()   # lit jusqu'à \n
    if data:
        print(data.decode(errors="ignore").strip())