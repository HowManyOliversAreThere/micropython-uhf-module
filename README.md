# UHF RFID Reader Modules

This is a MicroPython library for UHF RFID reader modules in the Tenhanyun/RoyalRay protocol families, controlled over UART. It implements the binary command/response protocol described in the "UHF RFID Reader Series User Manual" - a single protocol that, as far as can be determined, is shared across what's really one underlying OEM design sold under multiple brands and part numbers: Tenhanyun's [TY928](https://www.tenhanyun.com), and RoyalRay's [RRUx2828M family](https://www.rr-rfid.com/embedded/rrux2828m.html) (RRU72828M/RRU52828M/RRU32828M, for the Impinj E710/E510/E310 reader chips respectively), and likely other modules/readers in the same series with more antenna ports (e.g. RRU2881, RRU9816, RRUx881, RRUx199).

_This driver was written and tested against a TY928 specifically_ - it should work with little-to-no changes against other modules in the family (same command set, same framing), but expect treacherous untrodden paths (see `setup_antenna()` for a 4/8/16-port-only example). PRs welcome for extending support / functionality.

To use this library, copy `uhf_reader.py` into your MicroPython project and instantiate a `UHFReader` with a `machine.UART` object. The `examples` folder has a generic communication check and a tag EPC-write example.

Features:

- ✅ Communicate over UART using the shared frame protocol (framing + CRC16)
- ✅ Inventory (discover) EPC C1G2 tags in range, with per-tag RSSI
- ✅ Filter inventory to a single known tag - the basis of the warmer/colder use case
- ✅ Read / write tag memory (reserved, EPC, TID and user banks)
- ✅ Write a new EPC number to a tag
- ✅ Configure RF power, RF link profile, frequency band, inventory time, baud rate and device address
- ✅ Control the buzzer/LED output and GPIO pins
- ✅ Read reader info and serial number (handy as a communication self-check)
- ✅ Streaming "fast inventory" mode, extended reader parameters, per-antenna RF power and region readback
- 🚧 Everything else in the protocol - tag kill/protection, ISO18000-6B, the memory buffer commands, EM4325 sensor-tag support, and more - is present as a stub that raises `NotImplementedError`. See the bottom of [uhf_reader.py](uhf_reader.py) for the full list, with the relevant manual section/command code noted against each.

## Usage

```python
from machine import UART
from uhf_reader import UHFReader

uart = UART(1, baudrate=115200, tx=17, rx=16, timeout=50, timeout_char=10)
reader = UHFReader(uart)

print(reader.get_reader_info())

# Optionally set the frequency band and channel range for your jurisdiction
# Should only need to be done once
# reader.set_frequency("australia", min_channel=0, max_channel=9)

for tag in reader.inventory():
    print(tag["epc"].hex(), tag["rssi"])
```

See the `examples` folder for a basic communication check and an EPC-write example, and see `uhf_reader.py` for the class definition and all of the available methods.

## Notes

- **Set the correct frequency band and channel range for your jurisdiction** with `set_frequency()` before keying up RF at any meaningful power - see the docstring and `FREQUENCY_BANDS` in `uhf_reader.py`. This driver does not validate RF-legality for you.
- **Always have an antenna connected to the IPEX connector before transmitting** - running the PA into an open circuit for extended periods risks damaging the module.
- Factory-default UART settings are 57600bps, 8N1 - but `set_baud_rate()` persists across power-down, so a module may already be running at a different rate (115200bps for the module this repo's examples target; confirm yours with `ping()` before assuming the factory default). The protocol requires the gap between consecutive bytes to stay under 15ms - if your port's `UART` constructor supports a `timeout_char` argument, set it comfortably below that.
- Allow ~140ms after power-up/`EN` before the module will respond to commands.
