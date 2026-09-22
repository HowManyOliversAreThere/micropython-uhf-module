"""
Basic communication check for the UHF reader driver - works against any
module/reader in the Tenhanyun/RoyalRay protocol family (TY928,
RRUx2828M, ...).

Wire the module's RX/TX to a spare UART on your board (through a level
shifter if your board is 5V), power it up, and run this. It confirms
the module responds, prints its info/serial number, then repeatedly
inventories tags in range.
"""

from machine import UART
import time

from uhf_reader import UHFReader

UART_ID = 1
TX_PIN = 17
RX_PIN = 16

uart = UART(UART_ID, baudrate=115200, tx=TX_PIN, rx=RX_PIN, timeout=50, timeout_char=10)
reader = UHFReader(uart)

# Allow time for the module to finish its power-on initialisation.
time.sleep_ms(200)

if not reader.ping():
    raise RuntimeError("Reader did not respond - check connection")

info = reader.get_reader_info()
print("Reader info:", info)

serial = reader.get_serial_number()
print("Serial number:", serial.hex())

# Optionally set the frequency band and channel range for your jurisdiction
# Should only need to be done once
# reader.set_frequency("australia", min_channel=0, max_channel=9)

region = reader.read_region()
print("Frequency band:", region["band"], "min_channel:", region["min_channel"], "max_channel:", region["max_channel"])

print("Scanning for tags (Ctrl-C to stop)...")
while True:
    tags = reader.inventory()
    if not tags:
        print("  no tags in range")
    for tag in tags:
        print("  EPC %s  RSSI %3d  antenna 0x%02x" % (tag["epc"].hex(), tag["rssi"], tag["antenna"]))
    time.sleep_ms(300)
