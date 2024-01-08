import serial
import time

# Adjust the serial port and baud rate according to your setup
serial_port = '/dev/ttyACM0'  # Linux example, for Windows it might be 'COMx'
baud_rate = 9600

# Establish the serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)

try:
    while True:
        # Get an integer from the user (you can replace this with any source of integers)
        value_to_send = int(input("Enter an integer value: "))

        # Send the integer to the Arduino
        ser.write(str(value_to_send).encode())

        # Optionally, wait for a short time to avoid overwhelming the Arduino
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting program.")
    ser.close()
