import can
import time

# Candlelight firmware on Linux
#bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

# Stock slcan firmware on Linux
bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=250000)
# bus = can.interface.Bus(bustype='serial', channel='/dev/ttyACM0', bitrate=250000)
# bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=125000)

# Stock slcan firmware on Windows
# bus = can.interface.Bus(bustype='slcan', channel='COM0', bitrate=500000)

# msg = can.Message(arbitration_id=0xc0ffee,
#                   data=[0, 25, 0, 1, 3, 1, 4, 1],
#                   is_extended_id=True)





# msg = can.Message(arbitration_id=0x10e,
#                   data=[0, 25, 0, 1, 3, 1, 4, 1],
#                   is_extended_id=False)

# try:
#     bus.send(msg)
#     print("Message sent on {}".format(bus.channel_info))
# except can.CanError:
#     print("Message NOT sent")

""" USE NEXT COMMAND """
""" python -m can.viewer -c /dev/ttyACM0 -i slcan -b 250000 """

def main():

    # with can.Bus(receive_own_messages=True) as bus:

    print_listener = can.Printer()

    can.Notifier(bus, [print_listener])


    bus.send(can.Message(arbitration_id=1, is_extended_id=True))

    bus.send(can.Message(arbitration_id=2, is_extended_id=True))

    bus.send(can.Message(arbitration_id=1, is_extended_id=False))


    time.sleep(1.0)
    print("done")


    # https://stackoverflow.com/a/57476266



if __name__ == "__main__":

    main()
