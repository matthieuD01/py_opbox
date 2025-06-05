# Author: Matthieu Dominici
# Date Started: 12/20/2024
# Notes: Register addresses and direct order commands for OpBox 2.1

# Vendor and product IDs for OPBOX
VENDOR_ID = 0x0547
PRODUCT_ID = 0x1003

# Control transfer commands bmRequest Type
USB_OUT = 0x40
USB_IN = 0xC0

USB_IN_ENDPOINT = 0x86  # endpoint to read measurement data

# Direct Orders Description Table

OPBOX_SN_ORDER = 0xD0  # Retrieves the device serial number.
# Value=0, Index=0, Length=2, IN
# Data[0]: SN_YEAR (0–255)
# Data[1]: SN_NO (0–255)

RESET_ORDER = 0xD1  # Resets all device registers to their default values.
# Value=0, Index=0, Length=1, OUT

FIFO_RESET_ORDER = 0xD2  # Clears all stored frames in memory and resets FIFO.
# Value=0, Index=0, Length=0, OUT

# A faster and simpler way to initiate measurement via software.
DIRECT_SW_TRIG_ORDER = 0xD3
# Value=0, Index=0, Length=0, OUT

RESERVED_ORDER = 0xD4  # Reserved for future commands or functionality.
# Value=0, Index=0, Length=0, OUT

DIRECT_FRAME_READY_ORDER = 0xD5  # Indicates measurement status.
# Value=0, Index=0, Length=1, IN
# Data[0]:
#   0x00 - Memory empty or less frames than specified in PACKET_LENGTH register.
#   0x01 - New packet ready (specified number of frames are stored and ready).

# Configures pulse amplitude (0...63 corresponds to 0...360V).
PULSE_AMPLITUDE_ORDER = 0xD6
# Value=Pulse Amplitude (0...63), Index=1, Length=1, OUT
# NOTE: Requires initialization after reset.

USB_MODE_ORDER = 0xD7  # Checks USB speed mode status.
# Value=0, Index=0, Length=1, IN
# Data[0]:
#   0x00 - Device enumerated in Full-Speed mode (12 Mbps).
#   0x01 - Device enumerated in High-Speed mode (480 Mbps).


READ_REGISTER = 0xE1
WRITE_REGISTER = 0xE0

DEV_REV_REG = 0x00  # Register for Device Revision Information
POWER_CTRL_REG = 0x02       # Register for power control
PACKET_LEN_REG = 0x04
FRAME_IDX_REG = 0x06
FRAME_CNT_REG = 0x08       # Register for number of frames in memory
CAPT_REG_REG = 0x0A
TRIGGER_REG = 0x10     # Register for trigger settings
TRG_OVERRUN_REG = 0x12
XY_DIVIDER_REG = 0x14
TIMER_REG = 0x16
TIMER_CAPT_REG = 0x18
ANALOG_CTRL_REG = 0x1A
PULSER_TIME_REG = 0x1C
BURST_REG = 0x1E
MEASURE_REG = 0x20
DELAY_REG = 0x22            # Register for delay setting
# Depth low word register (2 least significant bytes)
DEPTH_L_REG = 0x24
# Depth high word register (15 most significant bytes)
DEPTH_H_REG = 0x26
CONST_GAIN_REG = 0x28       # Register for DAC Gain setting
# Skipping everything linked with peak detectors and encoders.


# Bit definitions
# 0000 0001 in binary: when writing this to power control register, enables power.
POWER_ENABLE_BIT = 0x01
POWER_OK_BIT = 0x10     # 0001 0000 in binary: this is a control bit. The 5th bit when reading the power control register is 0 if no power and 1 if power


def bytes2bits(bytes):
    # Takes 2 bytes inputs and outputs a 16-len array containing its bits. array[0] is least-significant bit.
    bits = []
    for byte in bytes[::-1]:
        bits += [int(x) for x in format(byte, '8b').replace(' ', '0')]
    return bits[::-1]


def bytes2dec(bytes, i0, i1):
    # Takes 2 bytes inputs and outputs an int containing the value of the bits between bytes[i0] add bytes[i1] (included)
    bits = bytes2bits(bytes)
    relevant_bits = bits[i0:i1+1]
    return sum([2**i if relevant_bits[i] == 1 else 0 for i in range(len(relevant_bits))])


def get_device_parameters(version='OpBox 2.1'):
    # Returns a dictionnary with all hardware parameter limits as described in the manual. Allows to easily adapt the software for any variants we might have without rewriting everything.
    if version != 'OpBox 2.1':
        raise Exception('Current software implements parameters only for the OpBox 2.1 version. Or something. Maybe it\'s 2.2. Docs are unclear, I have to ping them and make sure. self.version should be \'OpBox 2.x\'')
    dic = {}
    dic['sampling_rate'] = 100e6  # 100MHz max sampling rate
    dic['voltage_steps'] = 64  # number of voltage steps (0-->63 = 64)
    dic['max_voltage'] = 360  # V
    # Minimum voltage above zero. According to docs, setting voltage at step 0 corresponds to 0V, so step 1 is min_voltage.
    dic['min_voltage'] = 1 * dic['max_voltage'] / (dic['voltage_steps'] - 1)
    dic['min_gain'] = -28  # dB
    dic['max_gain'] = 68  # dB
    dic['min_delay'] = 0  # us
    dic['max_delay'] = 65535  # in sampling cycles
    dic['buffer_size'] = 262144  # bytes
    dic['header_size'] = 54  # bytes
    dic['max_packet_length'] = 4854
    dic['pulse_step_time'] = 0.1  # us per step of charging time
    dic['max_pulse_steps'] = 63  # 63
    dic['receiver_input_voltage'] = 0.275  # V
    dic['analog_filters'] = {
        0: "0.5 - 6 MHz",
        1: "1 - 6 MHz",
        2: "2 - 6 MHz",
        3: "4 - 6 MHz",
        4: "0.5 - 10 MHz",
        5: "1 - 10 MHz",
        6: "2 - 10 MHz",
        7: "4 - 10 MHz",
        8: "0.5 - 15 MHz",
        9: "1 - 15 MHz",
        10: "2 - 15 MHz",
        11: "4 - 15 MHz",
        12: "0.5 - 25 MHz",
        13: "1 - 25 MHz",
        14: "2 - 25 MHz",
        15: "4 - 25 MHz"
    }
    return dic
