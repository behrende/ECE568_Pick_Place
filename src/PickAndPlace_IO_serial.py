import serial
import argparse
import threading
import json
import time

def read_serial():
    while True:
        data = ser.readline().decode('utf-8').strip()
        if data:
            print(f"Received: {data}")

def send_command(x, y, z, t, T=1041):
    """
    Build and send a JSON command with the format:
    {"T":1041,"x":<x>,"y":<y>,"z":<z>,"t":<t>}
    """
    command_dict = {"T": T, "x": x, "y": y, "z": z, "t": t}
    json_command = json.dumps(command_dict)
    ser.write(json_command.encode() + b'\n')
    print(f"Sent command: {json_command}")

def main():
    global ser
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')
    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
    ser.setRTS(False)
    ser.setDTR(False)

    # Start a thread to read incoming serial data.
    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

    try:
        while True:
            user_input = input("\nEnter target coordinates as x,y,z[,t] (or 'exit' to quit): ").strip()
            if user_input.lower() == 'exit':
                print("Exiting.")
                break

            parts = user_input.split(',')
            try:
                if len(parts) == 3:
                    x, y, z = map(float, parts)
                    t = 3.14  # Default: gripper open
                elif len(parts) == 4:
                    x, y, z, t = map(float, parts)
                else:
                    print("Invalid format. Enter as x,y,z or x,y,z,t.")
                    continue
            except ValueError:
                print("Invalid numbers. Ensure numeric values are entered.")
                continue

            # Move to user-defined position
            send_command(x, y, z, t)
            time.sleep(2)  # Adjust delay as needed

            # Close the gripper (t=0) while keeping position
            send_command(x, y, z, 0)
            time.sleep(2)  # Adjust delay as needed

            # Move to fixed placement coordinates
            fixed_x, fixed_y, fixed_z, fixed_t = 0.0, 0.0, 100.0, 3.14  # Update as needed
            send_command(fixed_x, fixed_y, fixed_z, fixed_t)
            time.sleep(2)  # Adjust delay as needed

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
