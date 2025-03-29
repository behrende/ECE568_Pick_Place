import requests
import argparse
import json
import time

def send_command(ip, x, y, z, t, T=1041):
    """
    Send a JSON command to the ESP32 via HTTP to move the robotic arm.
    """
    command_dict = {"T": T, "x": x, "y": y, "z": z, "t": t}
    url = f"http://{ip}/js"

    try:
        response = requests.post(url, json=command_dict, timeout=5)
        if response.status_code == 200:
            print(f"Command sent: {json.dumps(command_dict)}")
            print(f"Response: {response.text}")
        else:
            print(f"Failed to send command. HTTP {response.status_code}: {response.text}")
    except requests.exceptions.Timeout:
        print("Request timed out. Check your ESP32 connection.")
    except requests.exceptions.RequestException as e:
        print(f"Network error: {e}")

def main():
    parser = argparse.ArgumentParser(description='HTTP JSON Communication with ESP32')
    parser.add_argument('ip', type=str, help='ESP32 IP address (e.g., 192.168.10.104)')

    args = parser.parse_args()
    ip_addr = args.ip

    try:
        while True:
            user_input = input("\nEnter target coordinates as x,y,z[,t] (or 'exit' to quit): ").strip()
            if user_input.lower() == "exit":
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
            send_command(ip_addr, x, y, z, t)
            time.sleep(2)  # Adjust delay as needed

            # Close the gripper (t=0) while keeping position
            send_command(ip_addr, x, y, z, 0)
            time.sleep(2)  # Adjust delay as needed

            # Move to fixed placement coordinates
            fixed_x, fixed_y, fixed_z, fixed_t = 0.0, 0.0, 100.0, 3.14  # Update as needed
            send_command(ip_addr, fixed_x, fixed_y, fixed_z, fixed_t)
            time.sleep(2)  # Adjust delay as needed

    except KeyboardInterrupt:
        print("\nExiting.")

if __name__ == "__main__":
    main()
