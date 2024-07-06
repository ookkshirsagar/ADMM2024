import serial
import threading
import time

# Serial port settings
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Replace with your serial port
    baudrate=9600,
    timeout=1
)

# Function to send commands over serial
def send_serial_command(command):
    ser.write(command.encode())
    time.sleep(1)

# Function to receive data over serial
def receive_serial_data():
    while True:
        try:
            data = ser.readline().decode('utf-8').rstrip()
            if data:
                print(f"Received data: {data}")
        except serial.SerialException:
            print("Error reading serial data.")

def main():
    try:
        # Start a thread to receive serial data
        receive_thread = threading.Thread(target=receive_serial_data)
        receive_thread.daemon = True
        receive_thread.start()

        # Send initial commands over serial
        send_serial_command("AT\r\n")  # Example command

        # Main loop to send periodic commands
        while True:
            send_serial_command("AT+COMMAND\r\n")  # Example command
            time.sleep(5)

    except KeyboardInterrupt:
        print("\nExiting program.")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
