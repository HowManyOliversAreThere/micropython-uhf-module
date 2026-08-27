"""
MicroPython driver for Tenhanyun/RoyalRay-family UHF RFID reader
modules, communicating over UART.

This implements the binary command/response protocol described in the
"UHF RFID Reader Series User Manual" - a single protocol shared across
what appears to be one underlying OEM design sold under multiple
brands and part numbers: Tenhanyun's TY928, RoyalRay's RRUx2828M family
(RRU72828M/RRU52828M/RRU32828M, for the Impinj E710/E510/E310 reader
chips respectively), and likely other 1/4/8/16-port modules and
desktop/handheld readers in the same series (e.g. RRU2881, RRU9816).
This driver was written and tested against a TY928 specifically, so
expect the odd gap against other modules/port counts (see e.g.
setup_antenna()) - patches welcome.

Physical layer reminder (see your module's datasheet - values below are
the TY928's):
    - UART, 1 start bit, 8 data bits, 1 stop bit, no parity.
    - Default baud rate 57600bps.
    - Module needs ~140ms after power-up/EN before it will respond.
    - Gaps between consecutive bytes (in either direction) longer than
      15ms are treated by the module as a broken frame.

Only ONE command is in flight at a time: send a command and wait for
its response before sending the next one.
"""

import time


# ---------------------------------------------------------------------------
# EPC C1G2 (ISO18000-6C) command codes
# ---------------------------------------------------------------------------
_CMD_TAG_INVENTORY = const(0x01)
_CMD_READ_DATA = const(0x02)
_CMD_WRITE_DATA = const(0x03)
_CMD_WRITE_EPC = const(0x04)
_CMD_KILL_TAG = const(0x05)
_CMD_SET_PROTECTION = const(0x06)
_CMD_BLOCK_ERASE = const(0x07)
_CMD_READ_PROTECTION_CONFIG_EPC = const(0x08)
_CMD_READ_PROTECTION_CONFIG_BULK = const(0x09)
_CMD_UNLOCK_READ_PROTECTION = const(0x0A)
_CMD_READ_PROTECTION_STATUS = const(0x0B)
_CMD_EAS_CONFIG = const(0x0C)
_CMD_EAS_ALERT_DETECTION = const(0x0D)
_CMD_SINGLE_TAG_INVENTORY = const(0x0F)
_CMD_BLOCK_WRITE = const(0x10)
_CMD_GET_MONZA4QT_PARAMS = const(0x11)
_CMD_SET_MONZA4QT_PARAMS = const(0x12)
_CMD_EXTENDED_READ = const(0x15)
_CMD_EXTENDED_WRITE = const(0x16)
_CMD_INVENTORY_WITH_BUFFER = const(0x18)
_CMD_MIX_INVENTORY = const(0x19)
_CMD_INVENTORY_WITH_EPC = const(0x1A)
_CMD_QT_INVENTORY = const(0x1B)
_CMD_SELECT = const(0x9A)

# ISO18000-6B command codes
_CMD_6B_SINGLE_INVENTORY = const(0x50)
_CMD_6B_INVENTORY = const(0x51)
_CMD_6B_READ_DATA = const(0x52)
_CMD_6B_WRITE_DATA = const(0x53)
_CMD_6B_GET_LOCK_STATUS = const(0x54)
_CMD_6B_LOCK_BYTE = const(0x55)

# Reader customised command codes
_CMD_GET_READER_INFO = const(0x21)
_CMD_SET_FREQUENCY = const(0x22)
_CMD_SET_ADDRESS = const(0x24)
_CMD_SET_INVENTORY_TIME = const(0x25)
_CMD_SET_BAUD_RATE = const(0x28)
_CMD_SET_RF_POWER = const(0x2F)
_CMD_BUZZER_LED_CONTROL = const(0x33)
_CMD_SET_TAG_CUSTOM_FUNCTION = const(0x3A)
_CMD_SETUP_ANTENNA = const(0x3F)
_CMD_SET_BUZZER_ENABLED = const(0x40)
_CMD_GPIO_CONTROL = const(0x46)
_CMD_GET_GPIO_STATE = const(0x47)
_CMD_GET_SERIAL_NUMBER = const(0x4C)
_CMD_SET_ANTENNA_CHECK = const(0x66)
_CMD_SET_COMM_INTERFACE = const(0x6A)
_CMD_SET_ANTENNA_RETURN_LOSS = const(0x6E)
_CMD_SET_MAX_EPC_TID_LENGTH = const(0x70)
_CMD_GET_MAX_EPC_TID_LENGTH = const(0x71)
_CMD_GET_BUFFER_DATA = const(0x72)
_CMD_CLEAR_BUFFER = const(0x73)
_CMD_GET_BUFFER_TAG_COUNT = const(0x74)
_CMD_SET_REALTIME_INVENTORY_PARAMS = const(0x75)
_CMD_SET_WORKING_MODE = const(0x76)
_CMD_LOAD_REALTIME_INVENTORY_PARAMS = const(0x77)
_CMD_SET_HEARTBEAT_INTERVAL = const(0x78)
_CMD_SET_WRITE_RF_POWER = const(0x79)
_CMD_GET_WRITE_RF_POWER = const(0x7A)
_CMD_SET_MAX_WRITE_RETRY = const(0x7B)
_CMD_SET_TAG_CUSTOM_PASSWORD = const(0x7D)
_CMD_GET_TAG_CUSTOM_PASSWORD = const(0x7E)
_CMD_READER_PROFILE = const(0x7F)
_CMD_SYNC_EM4325_TIMESTAMP = const(0x85)
_CMD_GET_EM4325_TEMPERATURE = const(0x86)
_CMD_GET_EM4325_SPI_DATA = const(0x87)
_CMD_RESET_EM4325_ALERT = const(0x88)
_CMD_SET_DRM_CONFIG = const(0x90)
_CMD_MEASURE_ANTENNA_RETURN_LOSS = const(0x91)
_CMD_MEASURE_READER_TEMPERATURE = const(0x92)
_CMD_STOP_INVENTORY = const(0x93)
_CMD_GET_RF_POWER_BY_ANTENNA = const(0x94)
_CMD_READ_REGION = const(0x9E)
# NOTE: 0x50/0x51 "start/stop fast inventory" (table rows 39-40) alias
# the 18000-6B command codes above - they're disambiguated by context
# (which command you last sent), not by the code itself. The manual
# marks these, and the two extended-parameter commands below, as
# "Ex10 series only"; see the "Ex10 commands" note above
# start_fast_inventory() in the UHFReader class for why that most
# likely means "built on an E310/E510/E710 reader chip" (i.e. includes
# TY928/RRUx2828M), not a chip these modules lack.
_CMD_START_FAST_INVENTORY = const(0x50)
_CMD_STOP_FAST_INVENTORY = const(0x51)
_CMD_FAST_INVENTORY_REPORT = const(0xEE)  # unsolicited push during fast inventory
_CMD_SET_EXTENDED_PARAMS = const(0xEA)
_CMD_GET_EXTENDED_PARAMS = const(0xEB)


# ---------------------------------------------------------------------------
# Memory banks (used by read/write/erase/protect-style commands)
# ---------------------------------------------------------------------------
MEM_RESERVED = const(0x00)  # kill password (word0-1) + access password (word2-3)
MEM_EPC = const(0x01)  # word0 = CRC16, word1 = PC, word2.. = EPC
MEM_TID = const(0x02)  # read-only, inlay-manufacturer defined
MEM_USER = const(0x03)

# Bit offset of the EPC data itself within the EPC bank (after the
# 16-bit CRC and 16-bit PC words). Used by epc_mask().
_EPC_DATA_BIT_OFFSET = const(32)


# ---------------------------------------------------------------------------
# Frequency bands for set_frequency() - name -> FreBand code.
# Channel N ranges and the Fs = base + N * step (MHz) formula for each
# band are documented in the User Manual section 8.4.2 (Format-2 table);
# consult it before choosing max_channel/min_channel.
# ---------------------------------------------------------------------------
FREQUENCY_BANDS = {
    # "full": 0,  # 840 + N*2 MHz, N=0..60 (NOT region-legal on its own!)
    "china2": 1,  # 920.125 + N*0.25 MHz, N=0..19
    "us": 2,  # 902.75 + N*0.5 MHz, N=0..49
    "korea": 3,  # 917.1 + N*0.2 MHz, N=0..31
    "eu": 4,  # 865.1 + N*0.2 MHz, N=0..14
    "ukraine": 6,  # 868.0 + N*0.1 MHz, N=0..6
    "china1": 8,  # 840.125 + N*0.25 MHz, N=0..19
    "eu3": 9,  # 865.7 + N*0.6 MHz, N=0..3
    "us3": 12,  # 902.0 + N*0.5 MHz, N=0..52
    "hk": 16,  # 920.25 + N*0.5 MHz, N=0..9
    "taiwan": 17,  # 920.75 + N*0.5 MHz, N=0..13
    "etsi_upper": 18,  # 916.3 + N*1.2 MHz, N=0..2
    "malaysia": 19,  # 919.25 + N*0.5 MHz, N=0..7
    "brazil": 21,  # 902.75 + N*0.5 MHz (N=0..9) or 910.25 + N*0.5 (N=10..34)
    "thailand": 22,  # 920.25 + N*0.5 MHz, N=0..9
    "singapore": 23,  # 920.25 + N*0.5 MHz, N=0..9
    "australia": 24,  # 920.25 + N*0.5 MHz, N=0..9
    "india": 25,  # 865.1 + N*0.6 MHz, N=0..3
    "uruguay": 26,  # 916.25 + N*0.5 MHz, N=0..22
    "vietnam": 27,  # 918.75 + N*0.5 MHz, N=0..7
    "israel": 28,  # 916.25 MHz, N=0 only
    "indonesia": 29,  # 917.25 (N=0) or 919.75 + N*0.5 MHz, N=1..3
    "new_zealand": 30,  # 922.25 + N*0.5 MHz, N=0..9
    "japan2": 31,  # 916.8 + N*1.2 MHz, N=0..3
    "peru": 32,  # 916.25 + N*0.5 MHz, N=0..22
    "russia": 33,  # 916.2 + N*1.2 MHz, N=0..3
    "south_africa": 34,  # 915.6 + N*0.2 MHz, N=0..16
    "philippines": 35,  # 918.25 + N*0.5 MHz, N=0..3
}


# ---------------------------------------------------------------------------
# Response status codes (section 5, "List of the response status")
# ---------------------------------------------------------------------------
STATUS_OK = const(0x00)
STATUS_INVENTORY_OK = const(0x01)
STATUS_INVENTORY_TIMEOUT = const(0x02)
STATUS_MORE_DATA = const(0x03)
STATUS_BUFFER_FULL = const(0x04)
STATUS_ACCESS_PASSWORD_ERROR = const(0x05)
STATUS_TAG_KILL_FAILED = const(0x09)
STATUS_ALL_ZERO_KILL_PASSWORD = const(0x0A)
STATUS_COMMAND_NOT_SUPPORTED_BY_TAG = const(0x0B)
STATUS_ALL_ZERO_ACCESS_PASSWORD = const(0x0C)
STATUS_READ_PROTECTION_ENABLE_FAILED = const(0x0D)
STATUS_UNLOCK_FAILED = const(0x0E)
STATUS_6B_WRITE_LOCKED = const(0x10)
STATUS_6B_LOCK_FAILED = const(0x11)
STATUS_6B_ALREADY_LOCKED = const(0x12)
STATUS_PARAMETER_STORE_FAILED = const(0x13)
STATUS_MODIFICATION_FAILED = const(0x14)
STATUS_STATISTICS_PACKET = const(0x26)
STATUS_HEARTBEAT_PACKET = const(0x28)
STATUS_ANTENNA_CHECK_FAILURE = const(0xF8)
STATUS_COMMAND_EXECUTION_FAILED = const(0xF9)
STATUS_OPERATION_FAILED = const(0xFA)
STATUS_NO_OPERABLE_TAGS = const(0xFB)
STATUS_TAG_ERROR_CODE = const(0xFC)
STATUS_COMMAND_LENGTH_ERROR = const(0xFD)
STATUS_ILLEGAL_COMMAND = const(0xFE)
STATUS_PARAMETER_ERROR = const(0xFF)

STATUS_MESSAGES = {
    STATUS_OK: "operation succeeded",
    STATUS_INVENTORY_OK: "inventory succeeded",
    STATUS_INVENTORY_TIMEOUT: "inventory timed out",
    STATUS_MORE_DATA: "further data is waiting to be delivered",
    STATUS_BUFFER_FULL: "reader memory buffer is full",
    STATUS_ACCESS_PASSWORD_ERROR: "incorrect access password",
    STATUS_TAG_KILL_FAILED: "tag kill failed",
    STATUS_ALL_ZERO_KILL_PASSWORD: "cannot kill a tag with an all-zero kill password",
    STATUS_COMMAND_NOT_SUPPORTED_BY_TAG: "command not supported by this tag",
    STATUS_ALL_ZERO_ACCESS_PASSWORD: "tag has an all-zero access password / function unsupported",
    STATUS_READ_PROTECTION_ENABLE_FAILED: "failed to enable read protection",
    STATUS_UNLOCK_FAILED: "failed to unlock tag",
    STATUS_6B_WRITE_LOCKED: "some bytes on the 6B tag are locked",
    STATUS_6B_LOCK_FAILED: "failed to perform lock on 6B tag",
    STATUS_6B_ALREADY_LOCKED: "target 6B tag is locked",
    STATUS_PARAMETER_STORE_FAILED: "failed to store parameter (valid until power off)",
    STATUS_MODIFICATION_FAILED: "failed to adjust RF power",
    STATUS_ANTENNA_CHECK_FAILURE: "antenna connection error",
    STATUS_COMMAND_EXECUTION_FAILED: "command execution error",
    STATUS_OPERATION_FAILED: "tags detected but operation failed (poor communication)",
    STATUS_NO_OPERABLE_TAGS: "no operable tag detected in range",
    STATUS_TAG_ERROR_CODE: "tag reported an error code",
    STATUS_COMMAND_LENGTH_ERROR: "command frame length error",
    STATUS_ILLEGAL_COMMAND: "illegal command (unrecognised code or bad CRC)",
    STATUS_PARAMETER_ERROR: "unrecognised parameter in command frame",
}


def _crc16(data):
    """CRC16 as specified in the User Manual (preset 0xFFFF, poly
    0x8408, LSB-first/reflected). Covers Len..Data[] of a frame."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


class UHFReaderError(Exception):
    """Base class for all errors raised by this driver."""


class UHFReaderTimeoutError(UHFReaderError):
    """No (complete) response was received from the module in time."""


class UHFReaderProtocolError(UHFReaderError):
    """A response frame was received but was malformed (bad CRC etc)."""


class UHFReaderStatusError(UHFReaderError):
    """The module executed the command but reported a failure status.

    Attributes:
        status: the raw status byte (see the STATUS_* constants).
        err_code: for status == STATUS_TAG_ERROR_CODE, the extra
            "Err_code" byte reported by the tag itself, else None.
    """

    def __init__(self, status, err_code=None):
        self.status = status
        self.err_code = err_code
        message = STATUS_MESSAGES.get(status, "unrecognised status")
        if err_code is not None:
            message = "%s (tag error code 0x%02X)" % (message, err_code)
        super().__init__("status 0x%02X: %s" % (status, message))


class UHFReader:
    """Driver for a Tenhanyun/RoyalRay-family UHF RFID reader module
    (TY928, RRUx2828M, and likely other compatible modules/readers in
    the same protocol family), communicating over a UART.

    Example:
        from machine import UART
        from uhf_reader import UHFReader

        uart = UART(1, baudrate=57600, tx=17, rx=16, timeout=50, timeout_char=10)
        reader = UHFReader(uart)
        info = reader.get_reader_info()
    """

    BROADCAST_ADDRESS = const(0xFF)

    def __init__(self, uart, address=0x00, timeout_ms=1000):
        self._uart = uart
        self.address = address
        self.timeout_ms = timeout_ms

    # -- low level framing ---------------------------------------------

    def _send(self, cmd, data=b""):
        body = bytes([len(data) + 4, self.address, cmd]) + bytes(data)
        crc = _crc16(body)
        self._uart.write(body + bytes([crc & 0xFF, (crc >> 8) & 0xFF]))

    def _read_exact(self, n, timeout_ms):
        buf = bytearray()
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        while len(buf) < n:
            chunk = self._uart.read(n - len(buf)) if self._uart.any() else None
            if chunk:
                buf.extend(chunk)
                continue
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                raise UHFReaderTimeoutError(
                    "expected %d bytes, got %d" % (n, len(buf))
                )
        return bytes(buf)

    def _recv(self, timeout_ms=None):
        """Read and validate a single response frame. Returns
        (reCmd, status, data_bytes)."""
        timeout_ms = self.timeout_ms if timeout_ms is None else timeout_ms
        length = self._read_exact(1, timeout_ms)[0]
        rest = self._read_exact(length, timeout_ms)
        frame = bytes([length]) + rest
        recmd, status = frame[2], frame[3]
        data = frame[4:-2]
        crc_received = frame[-2] | (frame[-1] << 8)
        if _crc16(frame[:-2]) != crc_received:
            raise UHFReaderProtocolError("CRC mismatch in response frame")
        return recmd, status, data

    def _transact(self, cmd, data=b"", timeout_ms=None):
        self._send(cmd, data)
        return self._recv(timeout_ms)

    def _call(self, cmd, data=b"", timeout_ms=None):
        """Send a command and return its Data[] payload. Raises
        UHFReaderStatusError unless the module reports plain success
        (status 0x00) - suitable for the majority of commands that
        only ever have one successful outcome."""
        _recmd, status, resp = self._transact(cmd, data, timeout_ms)
        if status != STATUS_OK:
            err_code = resp[0] if status == STATUS_TAG_ERROR_CODE and resp else None
            raise UHFReaderStatusError(status, err_code)
        return resp

    def _not_implemented(self, name, code):
        raise NotImplementedError(
            "%s (command 0x%02X) is not implemented yet in this driver"
            % (name, code)
        )

    # -- tag targeting helpers -------------------------------------------
    #
    # Most EPC C1G2 commands that act on a tag's memory (read/write/erase/
    # protect/...) let you target either:
    #   - a single, fully-known EPC ("ENum" + "EPC" fields), or
    #   - an arbitrary bit-mask against EPC/TID/user memory ("MaskMem" +
    #     "MaskAdr" + "MaskLen" + "MaskData" fields), matching zero or
    #     more tags.
    # These two encodings are not adjacent in the frame for most of these
    # commands (fixed fields such as Mem/WordPtr/Num/Pwd sit between
    # them) - _tag_target() returns a (head, tail) pair to be spliced in
    # at the right places by each command's own method.
    #
    # Inventory-style commands (inventory(), etc) only support the mask
    # form (there is no ENum/EPC alternative), and simply omit the mask
    # fields entirely for "match everything" - see _optional_mask().

    @staticmethod
    def _tag_target(epc, mask):
        if (epc is None) == (mask is None):
            raise ValueError("provide exactly one of epc= or mask=")
        if epc is not None:
            if len(epc) % 2 or len(epc) // 2 > 15:
                raise ValueError("epc must be 0-30 bytes long (even length)")
            return bytes([len(epc) // 2]) + bytes(epc), b""
        mem, bit_addr, mask_data = mask
        head = bytes([0xFF])
        tail = (
            bytes([mem, (bit_addr >> 8) & 0xFF, bit_addr & 0xFF, len(mask_data) * 8])
            + bytes(mask_data)
        )
        return head, tail

    @staticmethod
    def _optional_mask(mask):
        if mask is None:
            return b""
        mem, bit_addr, mask_data = mask
        return (
            bytes([mem, (bit_addr >> 8) & 0xFF, bit_addr & 0xFF, len(mask_data) * 8])
            + bytes(mask_data)
        )

    @staticmethod
    def epc_mask(epc):
        """Build a mask tuple that matches tags whose EPC bank equals
        `epc` exactly, for use as the mask= argument of inventory-family
        methods (EPC data starts at bit offset 32 in the EPC bank, after
        the 16-bit CRC and PC words)."""
        return (MEM_EPC, _EPC_DATA_BIT_OFFSET, bytes(epc))

    @staticmethod
    def _parse_tag_record(data, offset):
        header = data[offset]
        has_tid = bool(header & 0x80)  # Impinj FastID: EPC+TID combined
        has_phase = bool(header & 0x40)
        n = header & 0x3F
        offset += 1
        payload = data[offset : offset + n]
        offset += n
        rssi = data[offset]
        offset += 1
        record = {"rssi": rssi}
        if has_phase:
            record["phase"] = data[offset : offset + 4]
            offset += 4
            record["freq_khz"] = int.from_bytes(data[offset : offset + 3], "big")
            offset += 3
        if has_tid:
            record["epc"] = payload[:-12]
            record["tid"] = payload[-12:]
        else:
            record["epc"] = payload
        return record, offset

    # -- basic / setup commands (manual section 8.4) -----------------------

    def get_reader_info(self, timeout_ms=None):
        """Obtain reader information (command 0x21): firmware version,
        model, supported protocols, frequency/power/inventory-time
        configuration, etc. Useful as a basic "is it there?" check."""
        data = self._call(_CMD_GET_READER_INFO, b"", timeout_ms)
        version = (data[0] << 8) | data[1]
        return {
            "firmware_version": "%d.%d" % (version >> 8, version & 0xFF),
            "model_code": data[2],
            "supports_iso18000_6c": bool(data[3] & 0x02),
            "supports_iso18000_6b": bool(data[3] & 0x01),
            "max_frequency_raw": data[4],
            "min_frequency_raw": data[5],
            "rf_power": data[6],
            "inventory_time_100ms": data[7],
            "antenna_config": data[8],
            "antenna_check_enabled": bool(data[11]),
        }

    def ping(self, timeout_ms=None):
        """Basic communication check. Returns True if the module
        responded to a get_reader_info() request, False on timeout."""
        try:
            self.get_reader_info(timeout_ms=timeout_ms)
            return True
        except UHFReaderTimeoutError:
            return False

    def get_serial_number(self, timeout_ms=None):
        """Obtain the reader's unique serial number (command 0x4c) as
        4 raw bytes."""
        return self._call(_CMD_GET_SERIAL_NUMBER, b"", timeout_ms)

    def set_address(self, new_address, timeout_ms=None):
        """Change the device address the module responds to (command
        0x24). Updates self.address on success (the module itself
        replies using the OLD address)."""
        if not (0x00 <= new_address <= 0xFE):
            raise ValueError("new_address must be 0x00-0xFE")
        self._call(_CMD_SET_ADDRESS, bytes([new_address]), timeout_ms)
        self.address = new_address

    def set_inventory_time(self, scan_time, timeout_ms=None):
        """Set the default inventory window (command 0x25), in units of
        100ms (0-255; 0 = unlimited, wait for all tags in range).
        Reader default is 20 (2s)."""
        if not (0 <= scan_time <= 255):
            raise ValueError("scan_time must be 0-255")
        return self._call(_CMD_SET_INVENTORY_TIME, bytes([scan_time]), timeout_ms)

    _BAUD_RATES = {9600: 0, 19200: 1, 38400: 2, 57600: 5, 115200: 6}

    def set_baud_rate(self, baud, timeout_ms=None):
        """Change the module's UART baud rate (command 0x28). The
        module replies using the OLD baud rate - you must reconfigure
        your machine.UART object to the new rate yourself afterwards,
        the new rate only takes effect on the NEXT communication."""
        if baud not in self._BAUD_RATES:
            raise ValueError("baud must be one of %s" % sorted(self._BAUD_RATES))
        return self._call(
            _CMD_SET_BAUD_RATE, bytes([self._BAUD_RATES[baud]]), timeout_ms
        )

    def set_rf_power(self, power, persist=True, timeout_ms=None):
        """Set RF output power (command 0x2f, format 1), 0-30 (30 is
        approximately 1W / 30dBm). persist=False makes the setting
        volatile (lost on power-off)."""
        if not (0 <= power <= 30):
            raise ValueError("power must be 0-30")
        value = power | (0x00 if persist else 0x80)
        return self._call(_CMD_SET_RF_POWER, bytes([value]), timeout_ms)

    def set_frequency(self, band, max_channel, min_channel, persist=True, timeout_ms=None):
        """Configure the RF frequency band and channel range (command
        0x22, format 2). `band` is a code from FREQUENCY_BANDS (or a
        name key into it); max_channel/min_channel are channel numbers
        N per that band's Fs = base + N*step formula (see
        FREQUENCY_BANDS' comments, or manual section 8.4.2).

        IMPORTANT: only configure a band and channel range that is
        legal for RFID operation in your jurisdiction and matches your
        antenna/hardware. This driver does not validate that for you.
        """
        if isinstance(band, str):
            band = FREQUENCY_BANDS.get(band)
            if band is None:
                raise ValueError("unknown frequency band name '%s'" % band)

        if max_channel < min_channel:
            raise ValueError("max_channel must be >= min_channel")

        flag = 0x00 if persist else 0x01
        field = bytes([flag, band, max_channel, min_channel])
        return self._call(_CMD_SET_FREQUENCY, field, timeout_ms)

    def buzzer_led_control(self, active_ms=100, silent_ms=100, times=1, timeout_ms=None):
        """Pulse the TAG LED / buzzer output, shared with the GPO1 pin
        (command 0x33). active_ms/silent_ms are rounded down to steps
        of 50ms (0-255 steps, i.e. 0-12750ms)."""
        active_t = min(255, max(0, active_ms // 50))
        silent_t = min(255, max(0, silent_ms // 50))
        times = min(255, max(0, times))
        return self._call(
            _CMD_BUZZER_LED_CONTROL, bytes([active_t, silent_t, times]), timeout_ms
        )

    def set_buzzer_enabled(self, enabled, timeout_ms=None):
        """Enable/disable the buzzer (command 0x40): when enabled, GPO1
        pulses low on every successful tag operation."""
        return self._call(
            _CMD_SET_BUZZER_ENABLED, bytes([0x01 if enabled else 0x00]), timeout_ms
        )

    def gpio_control(self, out1, out2, timeout_ms=None):
        """Set GPO1/GPO2 output level (command 0x46): True = high,
        False = low."""
        value = (0x01 if out1 else 0x00) | (0x02 if out2 else 0x00)
        return self._call(_CMD_GPIO_CONTROL, bytes([value]), timeout_ms)

    def get_gpio_state(self, timeout_ms=None):
        """Read GPIO input/output state (command 0x47). Returns a dict
        with 'in1', 'out1', 'out2' booleans."""
        data = self._call(_CMD_GET_GPIO_STATE, b"", timeout_ms)
        value = data[0]
        return {
            "in1": bool(value & 0x01),
            "out1": bool(value & 0x10),
            "out2": bool(value & 0x20),
        }

    # -- tag inventory & data access (manual section 8.2) -------------------

    def inventory(
        self,
        q=4,
        session=None,
        mask=None,
        epc=None,
        scan_time=None,
        want_stats=False,
        timeout_ms=None,
    ):
        """Inventory (discover) EPC C1G2 tags in range (command 0x01).

        q: initial Q-value (0-15), roughly log2(number of tags you
           expect to see at once).
        session: 0-3 for S0-S3, or None (default) to let the reader use
           its AUTO2 session strategy.
        mask: an epc_mask()-style (mem, bit_addr, data) tuple to filter
           results to tags matching a bit pattern, or None for no
           filtering.
        epc: shorthand for mask=self.epc_mask(epc), to filter to one
           specific, fully-known tag.
        scan_time: override the inventory window for this call only, in
           units of 100ms (3-255). None uses the reader's configured
           default (see set_inventory_time()).
        want_stats: if True, also request the trailing statistics
           packet (read rate / total count) - currently only consumed
           internally to terminate the read cleanly, not returned.

        Returns a list of dicts, each with at least 'epc' (bytes),
        'rssi' (0-255, raw reader units - a relative signal-strength
        indicator; higher generally means a stronger return signal, but
        Tenhanyun does not publish a dBm conversion) and 'antenna'.
        This RSSI is the basis for the "warmer/colder" disc-finding use
        case this driver was written for - see find_tag().
        """
        if not (0 <= q <= 15):
            raise ValueError("q must be 0-15")
        if epc is not None:
            if mask is not None:
                raise ValueError("provide at most one of epc= or mask=")
            mask = self.epc_mask(epc)
        qvalue = (0x80 if want_stats else 0x00) | q
        sess = 0xFE if session is None else session
        field = bytearray([qvalue, sess])
        field += self._optional_mask(mask)
        if scan_time is not None:
            if not (0 <= scan_time <= 255):
                raise ValueError("scan_time must be 0-255")
            field += bytes([0x00, 0x80, scan_time])  # Target=A, Ant=1, ScanTime

        tags = []
        recmd, status, data = self._transact(_CMD_TAG_INVENTORY, bytes(field), timeout_ms)
        while True:
            if status == STATUS_STATISTICS_PACKET:
                break
            if status not in (
                STATUS_INVENTORY_OK,
                STATUS_INVENTORY_TIMEOUT,
                STATUS_MORE_DATA,
                STATUS_BUFFER_FULL,
            ):
                if status == STATUS_NO_OPERABLE_TAGS:
                    break
                err_code = data[0] if status == STATUS_TAG_ERROR_CODE and data else None
                raise UHFReaderStatusError(status, err_code)
            antenna = data[0]
            num = data[1]
            offset = 2
            for _ in range(num):
                record, offset = self._parse_tag_record(data, offset)
                record["antenna"] = antenna
                tags.append(record)
            if status != STATUS_MORE_DATA:
                break
            recmd, status, data = self._recv(timeout_ms)
        return tags

    def single_tag_inventory(self, timeout_ms=None):
        """Inventory exactly one tag (command 0x0f) - a slightly
        cheaper/simpler call than inventory() when you only expect (or
        only care about) one tag. Returns a dict like inventory()'s
        entries, or None if no tag was found."""
        _recmd, status, data = self._transact(_CMD_SINGLE_TAG_INVENTORY, b"", timeout_ms)
        if status == STATUS_NO_OPERABLE_TAGS:
            return None
        if status != STATUS_INVENTORY_OK:
            err_code = data[0] if status == STATUS_TAG_ERROR_CODE and data else None
            raise UHFReaderStatusError(status, err_code)
        antenna = data[0]
        # data[1] (Num) is constant 0x01 for this command.
        record, _offset = self._parse_tag_record(data, 2)
        record["antenna"] = antenna
        return record

    def find_tag(self, epc, scan_time=None, timeout_ms=None):
        """Convenience wrapper for a "warmer/colder" proximity-tracking
        use case: inventory once, filtered to a single known EPC, and
        return its RSSI (int, 0-255) if seen in this scan, else None.

        Call this repeatedly (e.g. from a polling loop) and feed the
        returned RSSI into whatever feedback you're driving (LEDs,
        buzzer, ...) - a higher value / a tag that starts being found
        again means "warmer".
        """
        tags = self.inventory(epc=epc, scan_time=scan_time, timeout_ms=timeout_ms)
        return tags[0]["rssi"] if tags else None

    def read_data(
        self,
        mem,
        word_addr,
        num_words,
        epc=None,
        mask=None,
        password=b"\x00\x00\x00\x00",
        timeout_ms=None,
    ):
        """Read num_words 16-bit words starting at word_addr from a
        tag's memory bank (command 0x02). mem is one of MEM_RESERVED /
        MEM_EPC / MEM_TID / MEM_USER. Target a single known tag with
        epc=, or a bit-mask with mask= (see epc_mask()) - provide
        exactly one of the two if more than one tag may be in range.
        password is only checked when reading password-protected
        reserved memory. Returns the words read as raw bytes (length ==
        num_words * 2, most-significant byte first)."""
        if not (1 <= num_words <= 120):
            raise ValueError("num_words must be 1-120")
        head, tail = self._tag_target(epc, mask)
        field = head + bytes([mem, word_addr, num_words]) + bytes(password) + tail
        return self._call(_CMD_READ_DATA, field, timeout_ms)

    def write_data(
        self,
        mem,
        word_addr,
        words,
        epc=None,
        mask=None,
        password=b"\x00\x00\x00\x00",
        timeout_ms=None,
    ):
        """Write 1-32 16-bit words (`words`, as raw bytes, even length,
        most-significant byte first) to a tag's memory bank starting at
        word_addr (command 0x03). Target a single known tag with epc=,
        or a bit-mask with mask=. password must be the tag's correct
        access password if the target memory is password-protected,
        else it can be left as all-zero."""
        if len(words) % 2 or not (1 <= len(words) // 2 <= 32):
            raise ValueError("words must be 2-64 bytes long (1-32 words, even length)")
        head, tail = self._tag_target(epc, mask)
        wnum = len(words) // 2
        field = (
            bytes([wnum])
            + head
            + bytes([mem, word_addr])
            + bytes(words)
            + bytes(password)
            + tail
        )
        return self._call(_CMD_WRITE_DATA, field, timeout_ms)

    def write_epc(self, new_epc, password=b"\x00\x00\x00\x00", timeout_ms=None):
        """Write a new EPC number to a tag (command 0x04). new_epc is
        0-30 bytes (even length). Only ONE tag must be present in the
        antenna's field when calling this - the protocol has no way to
        target a specific tag for this command, unlike read_data() /
        write_data()."""
        if len(new_epc) % 2 or len(new_epc) // 2 > 15:
            raise ValueError("new_epc must be 0-30 bytes long (even length)")
        enum = len(new_epc) // 2
        field = bytes([enum]) + bytes(password) + bytes(new_epc)
        return self._call(_CMD_WRITE_EPC, field, timeout_ms)

    # -- not yet implemented ---------------------------------------------
    #
    # Everything below is a documented stub for a command defined in the
    # protocol manual that this driver doesn't implement yet. Each stub
    # raises NotImplementedError; the manual section / command code is
    # noted for whoever picks these up next. Grouped to match the
    # manual's own command tables (sections 4.1/4.2/4.3).

    # EPC C1G2 commands (manual section 8.2)
    def kill_tag(self, *args, **kwargs):
        """Kill tag - permanently and irreversibly disables a tag.
        Manual 8.2.5, command 0x05."""
        self._not_implemented("kill_tag", _CMD_KILL_TAG)

    def set_protection(self, *args, **kwargs):
        """Set read/write protection for a specific memory bank.
        Manual 8.2.6, command 0x06."""
        self._not_implemented("set_protection", _CMD_SET_PROTECTION)

    def block_erase(self, *args, **kwargs):
        """Erase multiple words in a tag's memory. Manual 8.2.7,
        command 0x07."""
        self._not_implemented("block_erase", _CMD_BLOCK_ERASE)

    def read_protection_config(self, *args, **kwargs):
        """Enable read protection for a tag by EPC number (NXP UCODE
        EPC G2X only). Manual 8.2.8, command 0x08."""
        self._not_implemented(
            "read_protection_config", _CMD_READ_PROTECTION_CONFIG_EPC
        )

    def read_protection_config_bulk(self, *args, **kwargs):
        """Enable read protection for all tags in range, without EPC
        targeting (NXP UCODE EPC G2X only). Manual 8.2.9, command
        0x09."""
        self._not_implemented(
            "read_protection_config_bulk", _CMD_READ_PROTECTION_CONFIG_BULK
        )

    def unlock_read_protection(self, *args, **kwargs):
        """Unlock read protection on a tag (NXP UCODE EPC G2X only).
        Manual 8.2.10, command 0x0a."""
        self._not_implemented("unlock_read_protection", _CMD_UNLOCK_READ_PROTECTION)

    def read_protection_status(self, *args, **kwargs):
        """Check whether read protection is enabled on a tag (NXP
        UCODE EPC G2X only). Manual 8.2.11, command 0x0b."""
        self._not_implemented("read_protection_status", _CMD_READ_PROTECTION_STATUS)

    def eas_configuration(self, *args, **kwargs):
        """Enable/disable EAS alert on a tag (NXP UCODE EPC G2 only).
        Manual 8.2.12, command 0x0c."""
        self._not_implemented("eas_configuration", _CMD_EAS_CONFIG)

    def eas_alert_detection(self, *args, **kwargs):
        """Detect EAS alert (NXP UCODE EPC G2 only). Manual 8.2.13,
        command 0x0d."""
        self._not_implemented("eas_alert_detection", _CMD_EAS_ALERT_DETECTION)

    def block_write(self, *args, **kwargs):
        """Write multiple words to a tag's memory in one cycle (similar
        to write_data() but without its 32-word cap). Manual 8.2.15,
        command 0x10."""
        self._not_implemented("block_write", _CMD_BLOCK_WRITE)

    def get_monza4qt_params(self, *args, **kwargs):
        """Obtain Impinj Monza 4QT working parameters. Manual 8.2.16,
        command 0x11."""
        self._not_implemented("get_monza4qt_params", _CMD_GET_MONZA4QT_PARAMS)

    def set_monza4qt_params(self, *args, **kwargs):
        """Modify Impinj Monza 4QT working parameters. Manual 8.2.17,
        command 0x12."""
        self._not_implemented("set_monza4qt_params", _CMD_SET_MONZA4QT_PARAMS)

    def extended_read(self, *args, **kwargs):
        """Read data with a 2-byte word address (extends read_data()'s
        1-byte WordPtr range). Manual 8.2.18, command 0x15."""
        self._not_implemented("extended_read", _CMD_EXTENDED_READ)

    def extended_write(self, *args, **kwargs):
        """Write data with a 2-byte word address (extends
        write_data()'s 1-byte WordPtr range). Manual 8.2.19, command
        0x16."""
        self._not_implemented("extended_write", _CMD_EXTENDED_WRITE)

    def inventory_with_buffer(self, *args, **kwargs):
        """Inventory into the reader's internal memory buffer, for
        retrieval via get_buffer_data() etc. Manual 8.2.20, command
        0x18."""
        self._not_implemented("inventory_with_buffer", _CMD_INVENTORY_WITH_BUFFER)

    def mix_inventory(self, *args, **kwargs):
        """Combined inventory + read/write in one command cycle.
        Manual 8.2.21, command 0x19."""
        self._not_implemented("mix_inventory", _CMD_MIX_INVENTORY)

    def inventory_with_epc(self, *args, **kwargs):
        """Inventory constrained to (a) specific EPC number(s). Manual
        8.2.22, command 0x1a."""
        self._not_implemented("inventory_with_epc", _CMD_INVENTORY_WITH_EPC)

    def qt_inventory(self, *args, **kwargs):
        """Inventory tags via Impinj QT private/public mode switching.
        Manual 8.2.23, command 0x1b."""
        self._not_implemented("qt_inventory", _CMD_QT_INVENTORY)

    def select(self, *args, **kwargs):
        """EPC Gen2 Select command (pre-filter the tag population before
        the next inventory). Manual 8.2.24, command 0x9a."""
        self._not_implemented("select", _CMD_SELECT)

    # ISO18000-6B commands (manual section 8.3) - the modules this
    # driver was developed and tested against (TY928/RRU32828M) are
    # 18000-6C (EPC C1G2) only; these are stubbed for completeness in
    # case a 6B-capable module/reader in the wider family is used.
    def single_tag_inventory_6b(self, *args, **kwargs):
        """18000-6B single tag inventory, no filtering. Manual 8.3.1,
        command 0x50."""
        self._not_implemented("single_tag_inventory_6b", _CMD_6B_SINGLE_INVENTORY)

    def inventory_6b(self, *args, **kwargs):
        """18000-6B multi-tag inventory with a filter condition. Manual
        8.3.2, command 0x51."""
        self._not_implemented("inventory_6b", _CMD_6B_INVENTORY)

    def read_data_6b(self, *args, **kwargs):
        """18000-6B read data (up to 32 bytes). Manual 8.3.3, command
        0x52."""
        self._not_implemented("read_data_6b", _CMD_6B_READ_DATA)

    def write_data_6b(self, *args, **kwargs):
        """18000-6B write data (up to 32 bytes). Manual 8.3.4, command
        0x53."""
        self._not_implemented("write_data_6b", _CMD_6B_WRITE_DATA)

    def get_lock_status_6b(self, *args, **kwargs):
        """18000-6B obtain lock status of a memory unit. Manual 8.3.5,
        command 0x54."""
        self._not_implemented("get_lock_status_6b", _CMD_6B_GET_LOCK_STATUS)

    def lock_byte_6b(self, *args, **kwargs):
        """18000-6B lock a single (unlocked) byte. Manual 8.3.6, command
        0x55."""
        self._not_implemented("lock_byte_6b", _CMD_6B_LOCK_BYTE)

    # Reader customised commands (manual section 8.4) not yet implemented
    def set_tag_custom_function(self, *args, **kwargs):
        """Launch a tag-specific customised utility (e.g. Monza4QT
        Peek). Manual 8.4.13, command 0x3a."""
        self._not_implemented("set_tag_custom_function", _CMD_SET_TAG_CUSTOM_FUNCTION)

    def setup_antenna(self, *args, **kwargs):
        """Configure which antenna port(s) are enabled - relevant to
        the 4/8/16-port modules and readers elsewhere in this family
        (e.g. RRUx881/RRUx199); the single-port TY928/RRUx2828M this
        driver was developed against only has one antenna to enable.
        Manual 8.4.8, command 0x3f."""
        self._not_implemented("setup_antenna", _CMD_SETUP_ANTENNA)

    def set_antenna_check(self, *args, **kwargs):
        """Enable/disable antenna connection checking before tag
        operations. Manual 8.4.14, command 0x66."""
        self._not_implemented("set_antenna_check", _CMD_SET_ANTENNA_CHECK)

    def set_comm_interface(self, *args, **kwargs):
        """Switch between USB and UART communication (needs a
        power-cycle to take effect). Manual 8.4.15, command 0x6a."""
        self._not_implemented("set_comm_interface", _CMD_SET_COMM_INTERFACE)

    def set_antenna_return_loss(self, *args, **kwargs):
        """Modify/load the antenna return-loss threshold used by the
        antenna check. Manual 8.4.16, command 0x6e."""
        self._not_implemented(
            "set_antenna_return_loss", _CMD_SET_ANTENNA_RETURN_LOSS
        )

    def set_max_epc_tid_length(self, *args, **kwargs):
        """Configure the max EPC/TID length used by the memory buffer.
        Manual 8.4.17, command 0x70."""
        self._not_implemented(
            "set_max_epc_tid_length", _CMD_SET_MAX_EPC_TID_LENGTH
        )

    def get_max_epc_tid_length(self, *args, **kwargs):
        """Read back the max EPC/TID length configuration. Manual
        8.4.18, command 0x71."""
        self._not_implemented(
            "get_max_epc_tid_length", _CMD_GET_MAX_EPC_TID_LENGTH
        )

    def get_buffer_data(self, *args, **kwargs):
        """Retrieve tag data from the reader's memory buffer (see
        inventory_with_buffer()). Manual 8.4.19, command 0x72."""
        self._not_implemented("get_buffer_data", _CMD_GET_BUFFER_DATA)

    def clear_buffer(self, *args, **kwargs):
        """Clear the reader's memory buffer. Manual 8.4.20, command
        0x73."""
        self._not_implemented("clear_buffer", _CMD_CLEAR_BUFFER)

    def get_buffer_tag_count(self, *args, **kwargs):
        """Get the total tag count currently in the memory buffer.
        Manual 8.4.21, command 0x74."""
        self._not_implemented("get_buffer_tag_count", _CMD_GET_BUFFER_TAG_COUNT)

    def set_realtime_inventory_params(self, *args, **kwargs):
        """Configure real-time (streaming) inventory mode parameters.
        Manual 8.4.22, command 0x75."""
        self._not_implemented(
            "set_realtime_inventory_params", _CMD_SET_REALTIME_INVENTORY_PARAMS
        )

    def set_working_mode(self, *args, **kwargs):
        """Switch the reader's overall working mode (e.g. S-PRO/S-RPT/
        AUTO-2/real-time streaming). Manual 8.4.23, command 0x76."""
        self._not_implemented("set_working_mode", _CMD_SET_WORKING_MODE)

    def load_realtime_inventory_params(self, *args, **kwargs):
        """Load the current real-time inventory mode parameters back
        from the reader. Manual 8.4.24, command 0x77."""
        self._not_implemented(
            "load_realtime_inventory_params", _CMD_LOAD_REALTIME_INVENTORY_PARAMS
        )

    def set_heartbeat_interval(self, *args, **kwargs):
        """Configure the heartbeat packet interval for real-time
        inventory mode. Manual 8.4.25, command 0x78."""
        self._not_implemented("set_heartbeat_interval", _CMD_SET_HEARTBEAT_INTERVAL)

    def set_write_rf_power(self, *args, **kwargs):
        """Configure a separate RF power level used only for write
        operations. Manual 8.4.26, command 0x79."""
        self._not_implemented("set_write_rf_power", _CMD_SET_WRITE_RF_POWER)

    def get_write_rf_power(self, *args, **kwargs):
        """Read back the write-operation RF power configuration.
        Manual 8.4.27, command 0x7a."""
        self._not_implemented("get_write_rf_power", _CMD_GET_WRITE_RF_POWER)

    def set_max_write_retry(self, *args, **kwargs):
        """Configure the maximum retry count for write operations.
        Manual 8.4.28, command 0x7b."""
        self._not_implemented("set_max_write_retry", _CMD_SET_MAX_WRITE_RETRY)

    def set_tag_custom_password(self, *args, **kwargs):
        """Set the password gating tag customised functions. Manual
        8.4.29, command 0x7d."""
        self._not_implemented(
            "set_tag_custom_password", _CMD_SET_TAG_CUSTOM_PASSWORD
        )

    def get_tag_custom_password(self, *args, **kwargs):
        """Read back the tag customised functions password. Manual
        8.4.30, command 0x7e."""
        self._not_implemented(
            "get_tag_custom_password", _CMD_GET_TAG_CUSTOM_PASSWORD
        )

    def reader_profile(self, *args, **kwargs):
        """Load/modify the reader's air-interface profile (see the
        Profile table in the Application Manual). Manual 8.4.31,
        command 0x7f."""
        self._not_implemented("reader_profile", _CMD_READER_PROFILE)

    def sync_em4325_timestamp(self, *args, **kwargs):
        """Synchronise the RTC of an EM4325 sensor tag. Manual 8.4.32,
        command 0x85."""
        self._not_implemented("sync_em4325_timestamp", _CMD_SYNC_EM4325_TIMESTAMP)

    def get_em4325_temperature(self, *args, **kwargs):
        """Read temperature log data from an EM4325 sensor tag. Manual
        8.4.33, command 0x86."""
        self._not_implemented(
            "get_em4325_temperature", _CMD_GET_EM4325_TEMPERATURE
        )

    def get_em4325_spi_data(self, *args, **kwargs):
        """Read external sensor data via an EM4325 tag's SPI bus.
        Manual 8.4.34, command 0x87."""
        self._not_implemented("get_em4325_spi_data", _CMD_GET_EM4325_SPI_DATA)

    def reset_em4325_alert(self, *args, **kwargs):
        """Reset the alert flag on an EM4325 sensor tag. Manual 8.4.35,
        command 0x88."""
        self._not_implemented("reset_em4325_alert", _CMD_RESET_EM4325_ALERT)

    def set_drm_config(self, *args, **kwargs):
        """Modify/load Dense Reader Mode configuration. Manual 8.4.36,
        command 0x90."""
        self._not_implemented("set_drm_config", _CMD_SET_DRM_CONFIG)

    def measure_antenna_return_loss(self, *args, **kwargs):
        """Measure the current antenna return loss. Manual 8.4.37,
        command 0x91."""
        self._not_implemented(
            "measure_antenna_return_loss", _CMD_MEASURE_ANTENNA_RETURN_LOSS
        )

    def measure_reader_temperature(self, *args, **kwargs):
        """Measure the reader module's own internal temperature.
        Manual 8.4.38, command 0x92."""
        self._not_implemented(
            "measure_reader_temperature", _CMD_MEASURE_READER_TEMPERATURE
        )

    def stop_inventory(self):
        """Immediately stop an in-progress tag inventory (command 0x01)
        and make the reader return its current (partial) result -
        useful to bail out early once find_tag()/inventory() has seen
        what you need. Manual 8.4.44, command 0x93.

        NOTE: per the manual this command has NO response frame of its
        own - don't wrap it in _call()/_recv(). The reader ends the
        in-flight command within ~1 second, and its (probably partial)
        reply for the *original* command arrives as normal after that.
        """
        self._send(_CMD_STOP_INVENTORY, b"")

    def get_rf_power_by_antenna(self, timeout_ms=None):
        """Read back the per-antenna RF power configuration (command
        0x94). Returns a tuple of raw power values, one per antenna
        port (a single-port module like the TY928/RRUx2828M will
        return a single-element tuple). Manual 8.4.43."""
        data = self._call(_CMD_GET_RF_POWER_BY_ANTENNA, b"", timeout_ms)
        return tuple(data)

    def read_region(self, timeout_ms=None):
        """Read back the reader's configured regulatory region (command
        0x9e). Returns a dict with 'band' (a FREQUENCY_BANDS code),
        'max_channel' and 'min_channel' - the same shape set_frequency()
        takes. Manual 8.4.45."""
        data = self._call(_CMD_READ_REGION, b"", timeout_ms)
        return {"band": data[0], "max_channel": data[1], "min_channel": data[2]}

    # -- "Ex10" commands ---------------------------------------------------
    #
    # The manual marks these four as "currently only supports Ex10
    # series". Impinj's E310/E510/E710 reader-chip family (which the
    # whole TY928/RRUx2828M module family is built on - see the
    # RRU72828M/RRU52828M/RRU32828M naming) is what "Ex10" most plausibly
    # denotes here: RoyalRay (who appear to be the OEM behind this
    # module and this manual) sell a distinct "Ex10 Fixed Reader"/"Ex10
    # ... Module" product line explicitly built on E710/E510/E310 chips.
    # So "Ex10 series" most likely means "built on an Ex10 (E-x10)
    # reader chip", not a chip the E310-based TY928 lacks. That reading
    # is not confirmed against a physical TY928 though - if this
    # firmware build doesn't actually implement them, expect a
    # UHFReaderStatusError with STATUS_ILLEGAL_COMMAND or
    # STATUS_COMMAND_EXECUTION_FAILED.

    def set_extended_params(self, cfg_no, cfg_data, persist=True, timeout_ms=None):
        """Set an extended reader parameter (command 0xea). cfg_no
        selects which parameter (see the manual's Reader Parameter
        Configuration table, section 8.5); cfg_data is its raw byte
        value. See the "Ex10" commands note above. Manual 8.4.39."""
        flag = 0x00 if persist else 0x01
        field = bytes([flag, cfg_no]) + bytes(cfg_data)
        return self._call(_CMD_SET_EXTENDED_PARAMS, field, timeout_ms)

    def get_extended_params(self, cfg_no, timeout_ms=None):
        """Read back an extended reader parameter (command 0xeb). See
        set_extended_params() and the "Ex10" commands note above.
        Manual 8.4.40."""
        return self._call(_CMD_GET_EXTENDED_PARAMS, bytes([cfg_no]), timeout_ms)

    def start_fast_inventory(self, target=0, timeout_ms=None):
        """Start streaming ("fast") inventory (command 0x50): the
        reader acknowledges immediately, then pushes one UNSOLICITED
        frame per matching tag found (reCmd 0xee) until
        stop_fast_inventory() is called - call
        read_fast_inventory_report() in a loop to consume them. See the
        "Ex10" commands note above. Manual 8.4.41.

        target: 0 for Session Target A, 1 for Target B.
        """
        if target not in (0, 1):
            raise ValueError("target must be 0 (A) or 1 (B)")
        return self._call(_CMD_START_FAST_INVENTORY, bytes([target]), timeout_ms)

    def stop_fast_inventory(self, timeout_ms=None):
        """Stop streaming ("fast") inventory started by
        start_fast_inventory() (command 0x51). See the "Ex10" commands
        note above. Manual 8.4.42."""
        return self._call(_CMD_STOP_FAST_INVENTORY, b"", timeout_ms)

    def read_fast_inventory_report(self, timeout_ms=None):
        """Read one unsolicited tag report pushed by the reader after
        start_fast_inventory(). Returns a dict with 'antenna' (a port
        bitmask, not a port index - e.g. 0x05 means antennas 1 and 3),
        'epc' (bytes) and 'rssi', or None if the frame received wasn't
        a tag report (e.g. it was the response to some other command
        that happened to still be in flight)."""
        recmd, status, data = self._recv(timeout_ms)
        if recmd != _CMD_FAST_INVENTORY_REPORT or status != STATUS_OK:
            return None
        antenna = data[0]
        length = data[1]
        epc = data[2 : 2 + length]
        rssi = data[2 + length]
        return {"antenna": antenna, "epc": epc, "rssi": rssi}
