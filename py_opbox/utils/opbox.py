# Author:
# Date Started:
# Notes:

import usb.core
import usb.util
import struct

# import pyvisa

import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

import binascii

from .opbox_visa_class import *
from .OpBox_21_Registers import *

# from pithy3 import showme
showme = plt.show


class Opbox_v21():
    """
        OPBOX ver 2.1

        u'www.OPTEL.pl'
        u'OPBOX-2.1 Ultrasonic Box'
        u'SN_13.34'

        use help() function, example:
        opcard = v22()
        help(opcard)
        help(opcard.SetTimePulse)
    """

    trigger = Trigger()
    measure = Measure()
    analogCtrl = AnalogCtrl()
    PulserTime = PulserTime()

    def __init__(self, verbose=False):

        self.header_size = 54
        self.depth = 5000
        self.verbose = verbose
        self.vendor_id = VENDOR_ID
        self.product_id = PRODUCT_ID
        # Initialize USB device
        self.device = usb.core.find(
            idVendor=self.vendor_id, idProduct=self.product_id)
        if self.device is None:
            raise ValueError("Device not found. Check connection.")
        # Essential step of the USB config process.
        self.device.set_configuration()
        if self.verbose:
            print("Device successfully connected")

        self.__opbox = self.device
        self.dev_params = get_device_parameters(version='OpBox 2.1')
        self.SetDefaultConfig()

    def power_on_device(self):
        # See Control Registers description tables
        self.ctrl_out(POWER_CTRL_REG, struct.pack("B", 0x01))

        # Wait until the Power OK bit is set
        while True:
            power_status = self.ctrl_in(POWER_CTRL_REG, 2)
            if power_status[0] & 0x10:
                if self.verbose:
                    print('Device powered on.')
                break
            time.sleep(0.1)

    def __del__(self):
        print("Device off.")
        usb.util.release_interface(self.device, interface=0)
        usb.util.dispose_resources(self.device)

    def ctrl_in(self, register_addr, length):
        """Helper for control transfer IN."""
        return self.device.ctrl_transfer(USB_IN, READ_REGISTER, 0, register_addr, length)

    def ctrl_out(self, register_addr, data):
        """Helper for control transfer OUT."""
        return self.device.ctrl_transfer(USB_OUT, WRITE_REGISTER, 0, register_addr, data)

    def direct_order_command(self, direction, order, value, index, length_or_data):
        if direction.lower() == 'in':
            usb_dir = USB_IN
        elif direction.lower() == 'out':
            usb_dir = USB_OUT
        else:
            raise ValueError(
                'direction should be either \'in\' (tp read from device or \'out\' te write to device.')
        return self.device.ctrl_transfer(usb_dir, order, value, index, length_or_data)

    def SetDefaultConfig(self):
        # Number of samples collected locally before transfer to PC over USB. All these waveforms are averaged.
        self._sample_averaging = 64
        # Max sampling rate of 100MHz
        self._sampling_freq = self.dev_params['sampling_rate']
        # Hard reset of all registers to default values.
        self.Reset()
        time.sleep(0.1)
        # Tells the OpBox to power on and checks voltage levels
        self.power_on_device()
        # Sets the pulse voltage (from step=0 to step = 63 ===> 0V - 360V)
        self.SetPulseVoltage(self.dev_params['voltage_steps'] - 1)
        # Sets the hardware bandpass filters.
        self.SetAnalogFilters(12)
        # Range of the data collected, in us
        self.SetRange(20)
        # Deactivates input attenuator and pre-amplifier (can be turned if gain > 65dB is needed)
        self.SetAttenHilo(0)
        self.SetGain(0)                     # Sets the gain in dB
        self.SetDelay(0)                    # Sets the delay in us
        # Sets transmission mode. Call SetAnalogInput_PE() for pulse-echo mode.
        self.SetAnalogInput_TT()
        # Sets the sampling frequency to its max value (100MHz)
        self.SetSamplingFreq(1)
        # Sets the voltage pulse time, in us.
        self.SetTimePulse_stop_run(0.2)
        # Sets the number of packets (=waveform) to collect before sending over to the computer. Defaults to self._sample_averaging.
        self.SetPacketLength()
        # Enables the triggering of the OpBox
        self.TrigEnable(1)
        if self.verbose:
            print('Default configuration set.')

    def SetParams(self, params):
        self.TrigEnable(0)
        # Set amplifier gain
        min_gain, max_gain = self.dev_params['min_gain'], self.dev_params['max_gain']
        if 'gain' not in params.keys():
            raise Exception(
                f'Please include a \'gain\' key in your parameter dictionnary. Gain should be an int between {min_gain}dB and {max_gain}dB.')
        self._gain = params['gain']
        self.SetGain(self._gain)

    def GetInfo(self):
        """
            Info1 - SN_YEAR
            Info2 - SN_NO
        """
        opbox_info = self.direct_order_command('in', OPBOX_SN_ORDER, 0, 0, 2)
        return opbox_info

    def Reset(self):
        return self.direct_order_command('out', RESET_ORDER, 0, 0, chr(1))

    def PowerOnOff(self, on_off):
        if on_off:
            return self.ctrl_out(POWER_CTRL_REG, chr(1))
        else:
            usb.util_release_interface(self.device, 0)
            usb.util.dispose_resources(self.device)
            return self.ctrl_out(POWER_CTRL_REG, chr(0))

    def SetGain(self, gain):
        self._gain = gain
        min_gain, max_gain = self.dev_params['min_gain'], self.dev_params['max_gain']
        if gain > max_gain:
            raise Exception(
                f'Gain parameter is too high. Maximum accepted gain is {max_gain}dB. You tried setting gain at {gain}.')
        if gain < min_gain:
            raise Exception(
                f'Gain parameter is too low. Minimum accepted gain is {min_gain}dB. You tried setting gain at {gain}.')
        # Example gain setting. 8->200 gain DAC, -28dB to 68dB. Gain[dB] = (DAC/2) - 32
        CONST_GAIN = int((gain + 32) * 2)
        if self.ctrl_out(CONST_GAIN_REG, struct.pack('<H', CONST_GAIN)):
            if self.verbose:
                print(
                    f"Amplifier gain set to step {CONST_GAIN} = {CONST_GAIN/2 - 32}dB (0.5dB steps)")
            return 1

    def SetExpParameters(self, params):
        self.TrigEnable(0)
        # Set amplifier gain
        min_gain, max_gain = self.dev_params['min_gain'], self.dev_params['max_gain']
        if 'gain' not in params.keys():
            raise Exception(
                f'Please include a \'gain\' key in your parameter dictionnary. Gain should be an int between {min_gain}dB and {max_gain}dB.')
        self._gain = params['gain']
        self.SetGain(self._gain)

        # Set delay
        max_delay_cycles = self.dev_params['max_delay']
        max_delay_us = 1e6 * max_delay_cycles / self._sampling_freq
        if 'delay' not in params.keys():
            raise Exception(
                f'Please include a \'delay\' key in your parameter dictionnary with units of us. At the default sampling frequency of 100MHz, delay should be a float between 0 and {max_delay_us} us.')
        self._delay = params['delay']
        self.SetDelay(self._delay)

        # Set range
        max_range_cycles = 262090 / self._sample_averaging
        max_range_us = 1e6 * max_range_cycles / self._sampling_freq
        min_range_us = 1e6 * 1 / self._sampling_freq
        if 'range' not in params.keys():
            raise Exception(
                f'Please include a \'range\' key in your parameter dictionnary with units of us. At the default sampling frequency of 100MHz, range should be a float between {min_range_us}us and {max_range_us}us.')
        self._meas_range = params['range']
        self.SetRange(self._meas_range)

        # Reset FIFO (internal buffer for samples) to clear any previous frames
        self.direct_order_command('out', FIFO_RESET_ORDER, 0, 0, 0)
        if self.verbose:
            print("FIFO buffer reset.\nAll experiment parameters have been set.")
        self.exp_params = params
        self.TrigEnable(True)

    def Instr_RestetFIFO(self):
        return self.direct_order_command('out', FIFO_RESET_ORDER, 0, 0, chr(1))

    def StartSW(self):
        """Trigger from aplication"""
        return self.direct_order_command('out', DIRECT_SW_TRIG_ORDER, 0, 0, 0)

    def Check_Packet_Ready(self):
        """Check that packet is ready to read"""
        return self.direct_order_command('in', DIRECT_FRAME_READY_ORDER, 0, 0, 1)[0]

    def Check_Frame_Count(self):
        """ Returns the number of frames stored in FIFO buffer"""
        return self.ctrl_in(FRAME_CNT_REG, 2)[0]

    def setTrigger(self):
        return self.ctrl_out(TRIGGER_REG, self.trigger.value)

    def setMeasure(self):
        return self.ctrl_out(MEASURE_REG, self.measure.value)

    def setAnalogCtrl(self):
        return self.ctrl_out(ANALOG_CTRL_REG, self.analogCtrl.value)

    def TrigEnable(self, value=1):
        self.trigger.TriggerEnable = value
        self.trigger.setValue()
        return self.setTrigger()

    def ReadData(self):
        assert self.Check_Packet_Ready() == 1
        assert self._sample_averaging == self.Check_Frame_Count()

        meas_range = self._meas_range
        DEPTH = int(meas_range * 1e-6 * self._sampling_freq)
        if self.verbose:
            print(f'Reading data with measurement depth of {DEPTH}.')
        BUFFER_SIZE = (54 + DEPTH) * self._sample_averaging
        data = self.device.read(USB_IN_ENDPOINT, 2*BUFFER_SIZE)
        assert self.Check_Frame_Count() == 0
        assert self.Check_Packet_Ready() == 0

        if len(data) != BUFFER_SIZE:
            raise Exception(
                f'Data length is incorrect. Expected {BUFFER_SIZE} bytes, got {len(data)} bytes.')
        return data

    def SetDelay(self, delay=0):
        # Configure delay
        self._delay = delay
        max_delay_cycles = self.dev_params['max_delay']
        max_delay_us = 1e6 * max_delay_cycles / self._sampling_freq

        DELAY = int(delay * 1e-6 * self._sampling_freq)
        if DELAY > max_delay_cycles:
            raise Exception(
                f'Delay parameter is too high. Maximum accepted delay is {max_delay_cycles} sampling cycles, or {max_delay_us}us at the default sampling rate of {self._sampling_freq*1e-6}MHz. You tried setting delay at {delay}us = {DELAY} cycles.')
        if DELAY < 0:
            raise Exception(
                f'Delay cannot be negative. Minimum accepted delay is 0. You tried setting delay at {delay}us.')
        if self.ctrl_out(DELAY_REG, struct.pack("<H", DELAY)):
            if self.verbose:
                print(f"Delay set to {DELAY} sample periods, or {delay}us")
            return 1

    def SetRange(self, meas_range):
        # sample size s = header_size + depth = 54 + depth
        # sample averaging N = number of measurements to average to get 1 measurement
        # We have  N*s <= buffer_size --> Depth <= (buffer_size/N) - header_size
        self._meas_range = meas_range
        buffer_size = self.dev_params['buffer_size']
        header_size = self.dev_params['header_size']
        max_range_cycles = int(
            (buffer_size / self._sample_averaging) - header_size)
        max_range_us = 1e6 * max_range_cycles / self._sampling_freq
        min_range_us = 1e6 * 1 / self._sampling_freq
        # Configure measurement depth
        # number of data points that one measurement contains. The higher the depth, the more extended in time the signal is.
        DEPTH = int(meas_range * 1e-6 * self._sampling_freq)
        if DEPTH > max_range_cycles:
            raise Exception(
                f'Depth is too high. The measurement depth (or range) (number of sample points per measurement) cannot exceed {max_range_cycles} samples = {max_range_us} us with the default sampling rate of {1e-6*self._sampling_freq}MHz and sample averaging over {self._sample_averaging} cycles. You tried setting depth at {DEPTH} cycles = {DEPTH*1e6/self._sampling_freq}us.')
        if DEPTH < 1:
            raise Exception(
                f'Depth cannot be below 1. The minimum measurement depth (or range) (number of sample points per measurement) is 1 cycle = {min_range_us}us with the default sampling rate of {1e-6*self.default_sampling_rate}MHz. You tried setting depth at {DEPTH} cycles = {DEPTH*1e6/self._sampling_freq}us.')
        d = binascii.unhexlify((DEPTH).to_bytes(4, byteorder='little').hex())
        if self.ctrl_out(DEPTH_L_REG, d):
            if self.verbose:
                print(f"Measurement depth set to {DEPTH} samples")
            return 1
        return 0

    def SetAttenHilo(self, attenHilo=0):
        """
            This function set the input Attenuator (-20dB) or PostAmplifier (+24dB)
            attenHilo = 0 – 0dB – Attenuator and PostAmplifier are turned off
            attenHilo = 1 – +24dB – PostAmplifier turned on
            attenHilo = 2 – - 20dB – Attenuator turned on
        """
        if attenHilo < 0:
            attenHilo = 0
        if attenHilo > 2:
            attenHilo = 1

        if attenHilo == 0:
            self.analogCtrl.InputAttenuator = 0b0
            self.analogCtrl.PostAmplifier = 0b0
            self.analogCtrl.setValue()
        elif attenHilo == 1:
            self.analogCtrl.InputAttenuator = 0b0
            self.analogCtrl.PostAmplifier = 0b1
            self.analogCtrl.setValue()
        elif attenHilo == 2:
            self.analogCtrl.InputAttenuator = 0b1
            self.analogCtrl.PostAmplifier = 0b0
            self.analogCtrl.setValue()

        return self.setAnalogCtrl()

    def SetAnalogFilters(self, analogFilters=1):
        """
            This function sets the analog filters like specified below:
            analogFilters 	Settings
            0 	0.5 – 6 MHz,
            1 	1 – 6 MHz,
            2 	2 – 6 MHz,
            3 	4 – 6 MHz,
            4 	0.5 – 10 MHz,
            5 	1 – 10 MHz,
            6 	2 – 10 MHz,
            7 	4 – 10 MHz,
            8 	0.5 – 15 MHz,
            9 	1 – 15 MHz,
            10 	2 – 15 MHz,
            11 	4 – 15 MHz,
            12 	0.5 – 25 MHz,
            13 	1 – 25 MHz,
            14 	2 – 25 MHz,
            15 	4 – 25 MHz
        """
        self.analogCtrl.AnalogFilter = analogFilters
        self.analogCtrl.setValue()
        return self.setAnalogCtrl()

    def SetAnalogInput_TT(self):
        self.analogCtrl.AnalogInput = 0b1
        self.analogCtrl.setValue()
        self.setAnalogCtrl()
        return 0

    def SetAnalogInput_PE(self):
        self.analogCtrl.AnalogInput = 0b0
        self.analogCtrl.setValue()
        self.setAnalogCtrl()
        return 0

    def SetPulseVoltage(self, pulseVoltage=63):
        self._voltage = pulseVoltage
        min_voltage, max_voltage = self.dev_params['min_voltage'], self.dev_params['max_voltage']
        if not type(self._voltage) is int:
            raise Exception(
                f'Voltage parameter should be an int. You tried submitting a type {type(self._voltage)}.')
        if self._voltage > max_voltage:
            raise Exception(
                f'Voltage parameter is too high. Maximum accepted voltage is {max_voltage}V. You tried setting voltage at {self._voltage}.')
        if self._voltage < min_voltage:
            raise Exception(
                f'Voltage parameter is too low. Minimum accepted voltage is {min_voltage}V. You tried setting voltage at {self._voltage}.')
        steps = self.dev_params['voltage_steps'] - 1  # -1 for if step 0 = 0V
        # Signal amplitude. 0 to 63 (0V to 360V on PE)
        PULSE_AMPLITUDE = int(self._voltage * steps / max_voltage)
        if self.direct_order_command('out', PULSE_AMPLITUDE_ORDER, int(pulseVoltage), 0, 0):
            if self.verbose:
                print(
                    f"Pulse amplitude set to step {PULSE_AMPLITUDE} = {max_voltage/steps*PULSE_AMPLITUDE:.1f}V (steps of {max_voltage/steps:.2f}V)")
            return 1

    def SetTimePulse_stop_run(self, timeP=3.1):
        self.PulserTime.PulseTime = int(timeP*10)
        self.PulserTime.setValue()
        if self.ctrl_out(PULSER_TIME_REG, self.PulserTime.value):
            return 1
        return 0

    def SetPulserDriver_Enable(self):
        self.PulserTime.DriverEnable = 0b0
        self.PulserTime.setValue()
        if self.ctrl_out(PULSER_TIME_REG, self.PulserTime.value):
            return 1
        return 0

    def SetPulserDriver_Disable(self):
        self.PulserTime.DriverEnable = 0b1
        self.PulserTime.setValue()
        if self.ctrl_out(PULSER_TIME_REG, self.PulserTime.value):
            return 1
        return 0

    def SetGainMode(self, mode):
        self.measure.GainMode = mode
        self.measure.setValue()
        self.setMeasure()
        return 0

    def SetSamplingFreq(self, samplingFreq=0):
        """
            Function sets the sampling frequency. 
            SamplingFreq	Frequency
            0	100 MHz
            1	100 MHz
            2	50 MHz
            3	33.3 MHz
            4	25 MHz
            5	20 MHz
            6	16.7 MHz
            7	14.3 MHz
            8	12.5 MHz
            9	11.1 MHz
            10	10 MHz
            11	9.1 MHz
            12	8.3 MHz
            13	7.7 MHz
            14	7.1 MHz
            15	6.7 MHz
        """
        if samplingFreq < 0:
            samplingFreq = 0
        if samplingFreq > 15:
            samplingFreq = 15
        self.measure.SamplingFreq = samplingFreq
        self.measure.setValue()
        self.setMeasure()
        return 0

    def SetTimerPeriod(self, period=10000):
        """
        PRF (pulse repetition frequency) setting
        100..65535 [us] – period setting for internal Timer
        (max 10kHz)
        """
        if period < 100:
            period = 100  # 10000Hz
        elif period > 65535:
            period = 65535  # 15Hz
        period = binascii.unhexlify(
            (period).to_bytes(2, byteorder='little').hex())
        if self.ctrl_out(TIMER_REG, period):
            return 1

    def SetSampleAveraging(self, value: int | None = None):
        self._sample_averaging = value
        if value is None:
            self._sample_averaging = 1
        self.SetPacketLength()
        if self._meas_range is not None:        # If the range has already been defined, check it is compatible with the new sample_averaging value
            try:
                params = {'range': self._meas_range,
                          'delay': self._delay, 'gain': self._gain}
                self.set_settings(params)
            except Exception as e:
                raise Exception(
                    f'While setting a new value of self.sample_averaging, the following exception occurred: {e}.\nCheck that the new sample averaging value is compatible with the range that was previously set, or change the range parameter.')

    def SetPacketLength(self):
        ''' Defines the packet length, aka the number of measurements acquired by the OpBox before it raises the flag PACKET_READY, allowing the USB device to read data from it. Here, we capture as many measurement as sample averaging, which also means one measurement from a user/analytics standpoint corresponds to sample_averaging measurements for the OpBox.'''
        max_packet_length = self.dev_params['max_packet_length']
        if self._sample_averaging > max_packet_length:
            raise Exception(
                f'The maximum packet length has been exceeded. Max packet length is {max_packet_length}, and you tried to set it to {self.sample_averaging}. Packet length corresponds to the sample_averaging attribute.')
        self.ctrl_out(PACKET_LEN_REG, struct.pack("H", self._sample_averaging))
        if self.verbose:
            print(f'Packet length set to {self._sample_averaging}')

    def Trigger_And_Read(self):
        if self.verbose:
            print('Starting a measurement...')
        self.TrigEnable(True)
        i = 0
        while self.Check_Packet_Ready() == 0:
            self.StartSW()
            # print(f'Triggered. Trigger status = {self.get_trigger_status()}, {self.get_frame_count()} frames are ready, packet ready flag = {self.Check_Packet_Ready()}, trigger overruns = {self.get_trigger_overruns()}.')
            # device_registers = read_all_parameters(self.dev)
            # for key, value in device_registers.items():
            #     if key == key.upper():
            #         print('')
            #     print(f'{key}: {value}')
            # print(self.dev.ctrl_transfer(USB_IN, READ_REGISTER, 0, TRIGGER_REG, 2))
            # while self.get_trigger_status() == 1:
            #     print(self.get_trigger_status())
            #     #time.sleep(.05)
            time.sleep(0.001)
            i += 1

        try:
            assert i == self._sample_averaging
        except:
            raise Exception(
                f'The packet ready flag is raised, but {i} measurements have supposedly been made instead of {self._sample_averaging} measurements (see self._sample_averaging).')
        if self.verbose:
            print('All measurement made, data packet is ready.')

        # N_reads = int(BUFFER_SIZE / 512) + 1
        # data = b""
        # for i in range(N_reads):
        #     print(f'Reading no {i}...')
        #     try:
        #         chunk = dev.read(USB_IN_ENDPOINT, 512)
        #         data += chunk
        #     except usb.core.USBTimeoutError:
        #         print(f'Finished reading with USBTimeoutError.')
        #     nb_frames = dev.ctrl_transfer(USB_IN, READ_REGISTER, 0, FRAME_CNT_REG, 2)
        #     packet_ready = dev.ctrl_transfer(USB_IN, DIRECT_FRAME_READY_ORDER, 0, 0, 2)[0]
        #     print(f'{nb_frames} frames stored in memory and PACKET_READY = {packet_ready}.')

        # print(f'Len of data: {len(data)} ')
        raw_data = self.ReadData()
        self.raw_data = raw_data
        # Reset other data variables before recalculating them.
        self.data = None
        self.all_samples = None
        self.header = None
        # Parse raw_data and calculates self.data, self.all_samples, self.header
        self.ParseData(self.raw_data)
        if self.verbose:
            print('Data packet was successfully read.')
        return self.data

    def ParseHeader(self, header_bytes):
        if len(header_bytes) != 54:
            raise Exception(
                'Header length is incorrect. Expected 54 bytes, got {len(header_bytes} bytes.')
        data = header_bytes
        header = {}
        header['Start of Frame'] = chr(data[0])  # Byte 1
        header['FrameIdx'] = struct.unpack("<H", data[1:3])[0]  # Bytes 2-3
        header['TimeStamp'] = struct.unpack("<H", data[4:6])[0]  # Bytes 4-5
        header['TriggerOverrun'] = struct.unpack("<H", data[6:8])[
            0]  # Bytes 6-7
        header['TriggerOverrunSource'] = data[8]  # Byte 8
        header['GPI Captured'] = data[9]  # Byte 9
        header['Encoder 1 Position'] = struct.unpack("<I", data[10:14])[
            0]  # Bytes 10-13
        header['Encoder 2 Position'] = struct.unpack("<I", data[14:18])[
            0]  # Bytes 14-17
        header['Peak Detectors Status'] = data[18]  # Byte 18

        # For 3-byte fields, manually construct a 4-byte integer using bitwise operations
        header['PDA RefPos'] = data[20] | (
            data[21] << 8) | (data[22] << 16)  # Bytes 20-22
        header['PDA MaxVal'] = data[24]  # Byte 24
        header['PDA MaxPos'] = data[26] | (
            data[27] << 8) | (data[28] << 16)  # Bytes 26-28
        header['PDB RefPos'] = data[30] | (
            data[31] << 8) | (data[32] << 16)  # Bytes 30-32
        header['PDB MaxVal'] = data[34]  # Byte 34
        header['PDB MaxPos'] = data[36] | (
            data[37] << 8) | (data[38] << 16)  # Bytes 36-38
        header['PDC RefPos'] = data[40] | (
            data[41] << 8) | (data[42] << 16)  # Bytes 40-42
        header['PDC MaxVal'] = data[44]  # Byte 44
        header['PDC MaxPos'] = data[46] | (
            data[47] << 8) | (data[48] << 16)  # Bytes 46-48

        # DataCount as a 3-byte field
        header['DataCount'] = data[50] | (
            data[51] << 8) | (data[52] << 16)  # Bytes 50-52
        header['End of Header'] = chr(data[53])  # Byte 54
        return header

    def ParseData(self, raw_data):
        # Ensure the data is the expected length: 54 bytes for header + 1000 samples
        # Parse header according to the structure in the manual
        depth = int(self._meas_range * 1e-6 * self._sampling_freq)
        all_samples = np.zeros(
            shape=(self._sample_averaging, depth), dtype=np.uint8)
        meas_size = depth + 54
        for i in range(self._sample_averaging):
            meas_data = raw_data[i*meas_size:(i+1)*meas_size]
            header_bytes, sample_bytes = meas_data[:54], meas_data[54:]
            if i == 0:
                header = self.ParseHeader(header_bytes)
            # Parse the samples ()
            samples = np.array([ord(chr(i))
                               for i in sample_bytes]).astype(np.uint8)
            all_samples[i] = samples
            if i == self._sample_averaging - 1:
                assert (i+1)*meas_size == len(raw_data)

        averaged_data = ((np.mean(all_samples, axis=0) -
                         127) / 128).astype(np.float16)
        self.data = averaged_data
        self.all_samples = all_samples
        self.header = header
        return header, averaged_data, all_samples

    def read_all_parameters(self, verbose=True):
        params = {}
        OPBOX_SN = self.device.ctrl_transfer(USB_IN, OPBOX_SN_ORDER, 0, 0, 2)
        params['SN_YEAR'] = OPBOX_SN[1]
        params['SN_NO'] = OPBOX_SN[0]

        FRAME_READY = self.device.ctrl_transfer(
            USB_IN, DIRECT_FRAME_READY_ORDER, 0, 0, 1)[0]
        params['DIRECT_FRAME_READY'] = FRAME_READY

        USB_MODE = self.device.ctrl_transfer(
            USB_IN, USB_MODE_ORDER, 0, 0, 1)[0]
        params['USB_MODE'] = 'High Speed (480Mbps)' if USB_MODE else 'Full Speed (12Mbps)'

        separator = '-----'
        DEV_REV = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, DEV_REV_REG, 2)
        params['DEVICE REVISION INFORMATION REGISTER'] = separator
        params['Firmware Revision'] = bytes2dec(DEV_REV, 0, 7)
        params['Hardware Subversion'] = bytes2dec(DEV_REV, 8, 11)
        params['Hardware Major Version'] = bytes2dec(DEV_REV, 12, 15)
        params['Hard+Firm Version'] = '.'.join([str(params['Hardware Major Version']), str(
            params['Hardware Subversion']), str(params['Firmware Revision'])])

        POWER_CTRL = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, POWER_CTRL_REG, 2)
        params['POWER SUPPLY CONTROL REGISTER'] = separator
        params['Power Enable'] = bytes2dec(POWER_CTRL, 0, 0)
        params['Power OK'] = bytes2dec(POWER_CTRL, 4, 4)
        params['ANALOG PWR Status'] = bytes2dec(POWER_CTRL, 5, 5)
        params['DC12V Status'] = bytes2dec(POWER_CTRL, 6, 6)
        params['VREG Status'] = bytes2dec(POWER_CTRL, 7, 7)

        PACKET_LEN = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, PACKET_LEN_REG, 2)
        params['PACKET LENGTH REGISTER'] = separator
        params['PacketLength'] = bytes2dec(PACKET_LEN, 0, 12)

        FRAME_IDX = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, FRAME_IDX_REG, 2)
        params['FRAME INDEX REGISTER'] = separator
        params['FrameIdx'] = bytes2dec(FRAME_IDX, 0, 15)

        FRAME_CNT = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, FRAME_CNT_REG, 2)
        params['FRAME COUTNER REGISTER'] = separator
        params['FrameCnt'] = bytes2dec(FRAME_CNT, 0, 12)

        CAPT_REG = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, CAPT_REG_REG, 2)
        params['Captured GPI and Trigger Overrun Source Register'.upper()
               ] = separator
        params['TrgOvrSrc_A'] = bytes2dec(CAPT_REG, 0, 0)
        params['TrgOvrSrc_H'] = bytes2dec(CAPT_REG, 1, 1)
        params['TrgOvrSrc_F'] = bytes2dec(CAPT_REG, 2, 2)
        params['TrgOvrSrc_P'] = bytes2dec(CAPT_REG, 3, 3)
        params['GPIcaptured'] = bytes2dec(CAPT_REG, 12, 8)

        TRIGGER = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, TRIGGER_REG, 2)
        params['TRIGGER Control Register '.upper()] = separator
        params['Trigger Source'] = bytes2dec(TRIGGER, 0, 3)
        params['Trigger Enable'] = bytes2dec(TRIGGER, 4, 4)
        params['Trigger Reset'] = bytes2dec(TRIGGER, 5, 5)
        params['Trigger Sw'] = bytes2dec(TRIGGER, 6, 6)
        params['XY Divider Enable'] = bytes2dec(TRIGGER, 8, 8)
        params['XY Divider Reset'] = bytes2dec(TRIGGER, 9, 9)
        params['Timer Enable'] = bytes2dec(TRIGGER, 10, 10)
        params['Trigger Status'] = bytes2dec(TRIGGER, 12, 12)
        params['Trigger OVerrun Status'] = bytes2dec(TRIGGER, 14, 14)

        TRG_OVERRUN = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, TRG_OVERRUN_REG, 2)
        params['Trigger Overrun Counter Register'.upper()] = separator
        params['Trigger Overrun Counter'] = bytes2dec(TRG_OVERRUN, 0, 15)

        XY_DIVIDER = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, XY_DIVIDER_REG, 2)
        params['Divider Register for trigger X and Y'.upper()] = separator
        params['DividerXY TopValue'] = bytes2dec(XY_DIVIDER, 0, 15)

        TIMER = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, TIMER_REG, 2)
        params['Internal Timer Register'.upper()] = separator
        params['Timer Period'] = bytes2dec(TIMER, 0, 15)

        TIMER_CAPT = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, TIMER_CAPT_REG, 2)
        params['Internal Timer Capture Register'.upper()] = separator
        params['Timer Capture'] = bytes2dec(TIMER, 0, 15)

        ANALOG_CTRL = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, ANALOG_CTRL_REG, 2)
        params['Analog Control Register'.upper()] = separator
        params['Analog Filter'] = bytes2dec(ANALOG_CTRL, 0, 3)
        params['Input Attenuator'] = bytes2dec(ANALOG_CTRL, 4, 4)
        params['Post Amplifier'] = bytes2dec(ANALOG_CTRL, 5, 5)
        params['Analog Input'] = bytes2dec(ANALOG_CTRL, 6, 6)

        PULSER_TIME = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, PULSER_TIME_REG, 2)
        params['Pulser Control Register'.upper()] = separator
        params['Pulse Time'] = bytes2dec(PULSER_TIME, 0, 5)
        params['Pulser Select'] = bytes2dec(PULSER_TIME, 6, 6)
        params['Driver Enable'] = bytes2dec(PULSER_TIME, 7, 7)

        MEASURE = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, MEASURE_REG, 2)
        params['Measurement Control Register'.upper()] = separator
        params['Sampling Freq'] = bytes2dec(MEASURE, 0, 3)
        params['Gain Mode'] = bytes2dec(MEASURE, 4, 5)
        params['Data Processing Mode'] = bytes2dec(MEASURE, 7, 7)
        params['Store Disable'] = bytes2dec(MEASURE, 9, 9)

        DELAY = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, DELAY_REG, 2)
        params['Delay (post trigger) Control Register'.upper()] = separator
        params['Delay'] = bytes2dec(DELAY, 0, 15)

        DEPTH_L = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, DEPTH_L_REG, 2)
        params['Depth of Measurement Sample Buffer Register'.upper()
               ] = separator
        params['Depth_L'] = bytes2dec(DEPTH_L, 0, 15)

        DEPTH_H = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, DEPTH_L_REG, 4)
        params['Depth'] = bytes2dec(DEPTH_H, 0, 65)

        CONST_GAIN = self.device.ctrl_transfer(
            USB_IN, READ_REGISTER, 0, CONST_GAIN_REG, 2)
        params['Constants Gain Register (Require initialization after reset)'.upper(
        )] = separator
        params['Constant Gain'] = bytes2dec(CONST_GAIN, 0, 7)
        if verbose:
            for key, value in params.items():
                if key == key.upper():
                    print('')
                print(f'{key}: {value}')
        return params


if __name__ == '__main__':
    params = {
        'delay': 0,  # us
        'range': 20,  # In
        'gain': 20  # dB
    }

    opbox = Opbox_v21(verbose=True)
    opbox.SetExpParameters(params)

    # 2 ways to get the averaged data: as the object returned by trigger_and_read, or with opbox.data.
    avg_data = opbox.Trigger_And_Read()
    header, avg_data, all_samples = opbox.header, opbox.data, opbox.all_samples
    print(
        f'{len(all_samples)} acquisitions per measurement, each acquisition has length of {len(all_samples[0])} bytes.')
    fig, ax = plt.subplots()
    timesteps = np.linspace(
        params['delay'], params['delay'] + params['range'], len(opbox.data))
    ax.plot(timesteps, opbox.data, ls='', marker='.', ms=1)
    ax.set_ylim(-1, 1)
    ax.set_xlabel('Time (us)')
    ax.set_ylabel(r'Amplitude $\in$ [0, 255]')
    showme()
