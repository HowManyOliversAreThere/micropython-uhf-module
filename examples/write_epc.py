"""
Write a new EPC number to a tag - works against any module/reader in
the Tenhanyun/RoyalRay protocol family (TY928, RRUx2828M, ...).

Wire the module's RX/TX to a spare UART on your board (through a level
shifter if your board is 5V), power it up, place a SINGLE tag in the
antenna's field, and run this.

write_epc() has no way to target a specific tag - the protocol simply
applies it to whatever one tag is in range - so make sure only the tag
you want to reprogram is anywhere near the antenna before running this.
"""

from machine import UART
import time

from uhf_reader import UHFReader

UART_ID = 1
TX_PIN = 17
RX_PIN = 16

# The new EPC to write, as a hex string (even number of characters,
# 0-30 bytes). Pad/trim to match your tag population's EPC length.
NEW_EPC_HEX = "C0FFEECAFE2026"

uart = UART(UART_ID, baudrate=115200, tx=TX_PIN, rx=RX_PIN, timeout=50, timeout_char=10)
reader = UHFReader(uart)

# Allow time for the module to finish its power-on initialisation.
time.sleep_ms(200)

if not reader.ping():
    raise RuntimeError("reader did not respond - check wiring/baud rate")

tags = reader.inventory()
if not tags:
    raise RuntimeError("no tag found - place a single tag in range and retry")
if len(tags) > 1:
    raise RuntimeError(
        "more than one tag in range (%d found) - write_epc() targets "
        "whichever tag responds and can't be aimed at one of several"
        % len(tags)
    )

old_epc = tags[0]["epc"]
new_epc = bytes.fromhex(NEW_EPC_HEX)

print("Current EPC: %s" % old_epc.hex())
print("Writing new EPC: %s" % new_epc.hex())
reader.write_epc(new_epc)
print("Write complete.")

# Re-inventory to confirm the tag now reports the new EPC.
tags = reader.inventory()
if not tags:
    print("Warning: could not verify new EPC (tag not seen)")
elif tags[0]["epc"] != new_epc:
    print("Warning: could not verify new EPC (EPC didn't match)")
else:
    print("Verified: tag now reports EPC %s" % tags[0]["epc"].hex())
