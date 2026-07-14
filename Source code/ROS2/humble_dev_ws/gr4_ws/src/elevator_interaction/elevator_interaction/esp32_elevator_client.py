import sys
import requests


ESP32_IP = "192.168.8.226" 
BASE_URL = f"http://{ESP32_IP}"


def send_command(command: str):
    if command == "press":
        url = f"{BASE_URL}/press"
    elif command == "status":
        url = f"{BASE_URL}/status"
    elif command == "fsr":
        url = f"{BASE_URL}/fsr"
    else:
        print(f"Unknown command: {command}")
        print("Use: press, status, or fsr")
        return

    try:
        response = requests.get(url, timeout=10)
        print(response.text)
    except requests.exceptions.RequestException as e:
        print(f"Failed to contact ESP32: {e}")


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 esp32_elevator_client.py press")
        print("  python3 esp32_elevator_client.py status")
        print("  python3 esp32_elevator_client.py fsr")
        return

    command = sys.argv[1]
    send_command(command)


if __name__ == "__main__":
    main()
