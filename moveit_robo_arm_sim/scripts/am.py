import serial

# Replace 'COM3' with the appropriate port name
ser = serial.Serial('/dev/ttyACM0', 57600)

try:
    while True:
        try:
            # Read a line of serial data
            line = ser.readline().strip()
            
            # Display the received data
            print(line.decode('utf-8'))
        except UnicodeDecodeError as e:
            # Print the raw bytes if decoding fails
            print("Error decoding:", e)
            print("Raw bytes:", line)
except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
