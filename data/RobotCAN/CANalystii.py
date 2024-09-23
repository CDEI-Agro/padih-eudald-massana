import canalystii

# Connect to the Canalyst-II device
# Passing a bitrate to the constructor causes both channels to be initialized and started.
dev = canalystii.CanalystDevice(bitrate=5000
# Receive all pending messages on channel 0
for msg in dev.receive(0):
    print(msg)

# The canalystii.Message class is a ctypes Structure, to minimize overhead
new_message = canalystii.Message(can_id=0x300,
                                 remote=False,
                                 extended=False,
                                 data_len=8,
                                 data=(0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08))cd
# Send one copy to channel 1
dev.send(1, new_message)
# Send 3 copies to channel 0
# (argument can be an instance of canalystii.Message or a list of instances)
dev.send(0, [new_message] * 3)

# Stop both channels (need to call start() again to resume capturing or send any messages)
dev.stop(0)
dev.stop(1)

#  idVendor           0x04d8 Microchip Technology, Inc.
#  idProduct          0x0053
You can change the permissions of your usb device node by creating a udev rule. e.g. I added the following line to a file in /etc/udev/rules.d/

SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", MODE="0664", GROUP="usbusers"
