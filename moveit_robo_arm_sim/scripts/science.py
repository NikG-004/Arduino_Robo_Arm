import serial

def read_serial_data(port, baud_rate, timeout):
    try:
        ser = serial.Serial(port, baud_rate, timeout=timeout)
        print(f"Reading data from {port}...")
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode().strip()
                print("Received:", data)
    except serial.SerialException as e:
        print("Serial port error:", e)
    finally:
        if ser.is_open:
            ser.close()

if __name__ == "__main__":
    serial_port = "/dev/ttyUSB0"  # Adjust this based on your Arduino's port
    baud_rate = 115200  # Match this with your Arduino's baud rate
    timeout = 1  # Adjust timeout as needed
    
    read_serial_data(serial_port, baud_rate, timeout)
