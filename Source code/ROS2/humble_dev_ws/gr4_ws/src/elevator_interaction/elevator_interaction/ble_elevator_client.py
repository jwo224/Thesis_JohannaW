import argparse
import asyncio
from bleak import BleakClient, BleakScanner


DEVICE_NAME = "XIAO-FSR-SERVO"
DEFAULT_ADDRESS = "F6:75:48:70:C7:F7"

# Nordic UART Service UUIDs used by Adafruit BLEUart
UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Robot writes commands here
UART_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Microcontroller sends replies here


notification_buffer = ""


def handle_notification(sender, data):
    global notification_buffer

    try:
        chunk = data.decode("utf-8")
    except UnicodeDecodeError:
        print(data)
        return

    notification_buffer += chunk

    while "\n" in notification_buffer:
        line, notification_buffer = notification_buffer.split("\n", 1)
        line = line.strip()

        if line:
            print(line)


async def find_device_by_name(name: str, timeout: float = 10.0):
    print(f"Scanning for BLE device named '{name}'...")
    devices = await BleakScanner.discover(timeout=timeout)

    for device in devices:
        if device.name == name:
            print(f"Found {name}: {device.address}")
            return device.address

    print(f"Could not find BLE device named '{name}'.")
    return None


async def try_send_to_address(address: str, command: str, wait_time: float):
    global notification_buffer
    notification_buffer = ""
    
    try:
        print(f"Trying BLE address {address}...")

        async with BleakClient(address, timeout=8.0) as client:
            if not client.is_connected:
                print(f"Could not connect to {address}.")
                return False

            print(f"Connected to {address}.")

            await client.start_notify(UART_TX_CHAR_UUID, handle_notification)

            print(f"Sending command: {command}")
            await client.write_gatt_char(
                UART_RX_CHAR_UUID,
                f"{command}\n".encode("utf-8"),
                response=False,
            )

            await asyncio.sleep(wait_time)

            await client.stop_notify(UART_TX_CHAR_UUID)

        print("Disconnected.")
        return True

    except Exception as e:
        print(f"Direct BLE connection to {address} failed: {e}")
        return False


async def send_ble_command(command: str, address: str | None = None, wait_time: float = 3.0):
    command = command.strip().upper()

    first_address = address if address is not None else DEFAULT_ADDRESS

    success = await try_send_to_address(first_address, command, wait_time)

    if success:
        return

    print()
    print("Direct MAC connection failed. Falling back to scan by device name...")

    scanned_address = await find_device_by_name(DEVICE_NAME)

    if scanned_address is None:
        print("No BLE device found. Make sure the microcontroller is powered and advertising.")
        return

    if scanned_address == first_address:
        print("Scan found the same address that already failed.")
        return

    success = await try_send_to_address(scanned_address, command, wait_time)

    if not success:
        print("BLE command failed after both direct MAC and scan fallback.")


def main():
    parser = argparse.ArgumentParser(description="Send BLE UART commands to elevator microcontroller.")
    parser.add_argument(
        "command",
        nargs="+",
        help="Command to send, e.g. PRESS, STATUS, OPEN, CLOSE, GOTO 90, THRESH 200",
    )
    parser.add_argument(
        "--address",
        default=None,
        help=f"BLE MAC address of the device. Default: {DEFAULT_ADDRESS}",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=3.0,
        help="Seconds to wait for BLE responses.",
    )

    args = parser.parse_args()
    command = " ".join(args.command)

    asyncio.run(send_ble_command(command, address=args.address, wait_time=args.wait))


if __name__ == "__main__":
    main()