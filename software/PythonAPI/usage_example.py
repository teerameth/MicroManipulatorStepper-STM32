import argparse

from open_micro_stage_api import OpenMicroStageInterface


def list_available_ports():
    devices = OpenMicroStageInterface.enumerate_devices()
    if not devices:
        print('No serial devices detected.')
        return devices

    print('Available serial devices:')
    for device in devices:
        print(f"  {device['label']}")

    return devices


def resolve_port(port):
    if port:
        return port

    devices = OpenMicroStageInterface.enumerate_devices()
    pico_devices = [device for device in devices if 'pico' in device['label'].lower()]

    if len(pico_devices) == 1:
        print(f"Using detected Pico serial device: {pico_devices[0]['label']}")
        return pico_devices[0]['port']

    if len(pico_devices) > 1:
        print('Multiple Pico serial devices detected. Pass --port to choose one:')
        for device in pico_devices:
            print(f"  {device['label']}")
        raise SystemExit(1)

    if len(devices) == 1:
        print(f"Using detected serial device: {devices[0]['label']}")
        return devices[0]['port']

    if devices:
        print('Multiple serial devices detected. Pass --port to choose one:')
        for device in devices:
            print(f"  {device['label']}")
    else:
        print('No serial devices detected.')

    raise SystemExit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Basic OpenMicroManipulator serial API example.',
    )
    parser.add_argument(
        '--port',
        help='Serial port to use (for example /dev/ttyACM0 or COM3).',
    )
    parser.add_argument(
        '--list-ports',
        action='store_true',
        help='List detected serial devices and exit.',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.list_ports:
        list_available_ports()
        return

    port = resolve_port(args.port)
    oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
    if not oms.connect(port):
        raise SystemExit(f'Could not connect to {port}.')

    try:
        # run this once to calibrate joints
        # for i in range(3): oms.calibrate_joint(i, save_result=True)

        # home device
        oms.home()

        # move and wait
        oms.move_to(0, 0, 0, f=1)
        oms.wait_for_stop()

        # print some info
        oms.read_device_state_info()
    finally:
        oms.disconnect()


if __name__ == '__main__':
    main()
