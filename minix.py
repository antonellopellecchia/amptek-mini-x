#!/usr/bin/env python3
from pyftdi.ftdi import Ftdi, UsbTools
from functools import wraps


Ftdi.add_custom_product(Ftdi.DEFAULT_VENDOR, 0xD058)


class Request:
    OUTPUT_MODE = 0x7B
    INPUT_MODE = 0x7B
    OUTPUT_MODE_H = 0x08

    def __init__(self, state):
        self.__state = state
        self.__commands = []

    def add_bytes(self, *byte):
        for b in byte:
            self.__commands.append(b)
        return self

    def set_clock_divisor(self):
        return self.add_bytes(0x86, 0x03, 0x00)

    def set_input(self):
        self.__commands.append(0x80)
        self.__commands.append(self.__state.low_byte_state)
        self.__commands.append(Request.INPUT_MODE)
        return self

    def set_clear_low(self, set_registers, clear_registers):
        self.__commands.append(0x80)
        if type(set_registers) is not list:
            set_registers = [set_registers]
        if type(clear_registers) is not list:
            clear_registers = [clear_registers]
        for register in set_registers:
            self.__state.low_byte_state |= register
        for register in clear_registers:
            self.__state.low_byte_state &= ~register
        self.__commands.append(self.__state.low_byte_state)
        self.__commands.append(Request.OUTPUT_MODE)
        return self

    def set_low(self, *registers):
        return self.set_clear_low(list(registers), [])

    def clear_low(self, *registers):
        return self.set_clear_low([], list(registers))

    def set_clear_high(self, set_registers, clear_registers):
        self.__commands.append(0x82)
        if type(set_registers) is not list:
            set_registers = [set_registers]
        if type(clear_registers) is not list:
            clear_registers = [clear_registers]
        for register in set_registers:
            self.__state.high_byte_state |= register
        for register in clear_registers:
            self.__state.high_byte_state &= ~register
        self.__commands.append(self.__state.high_byte_state)
        self.__commands.append(Request.OUTPUT_MODE_H)
        return self

    def set_high(self, *registers):
        return self.set_clear_high(list(registers), [])

    def clear_high(self, *registers):
        return self.set_clear_high([], list(registers))

    def get_command(self):
        return bytes(self.__commands)


def check_connected(func):
    @wraps(func)
    def wrap(self, *args, **kwargs):
        if not self._ftdi or not self._ftdi.is_mpsse:
            raise NotConnected()
        return func(self, *args, **kwargs)

    return wrap


class RegisterState:
    def __init__(self, low_byte_state, high_byte_state):
        self.low_byte_state = low_byte_state
        self.high_byte_state = high_byte_state


class NotConnected(Exception):
    pass


class ReadError(Exception):
    pass


class ValueOutOfRange(Exception):
    pass


class PowerNotSafe(Exception):
    pass


class MiniX:
    READ_DELAY = 0.5
    HIGH_VOLTAGE_DEFAULT = 10.0
    HIGH_VOLTAGE_RANGE = (9.999999, 40.0)
    HIGH_VOLTAGE_CONVERSION_FACTOR = 10.0
    CURRENT_DEFAULT = 5.0
    CURRENT_RANGE = (4.999999, 200.0)
    CURRENT_CONVERSION_FACTOR = 50.0
    V_REF = 4.096  # Reference voltage for the DAC
    DAC_ADC_SCALE = 4096  # Scale of the ADC/DAC on the MiniX
    POWER_MAX = 4.00  # Absolute maximum power rating of the MiniX
    SAFETY_MARGIN = 0.050  # Safety margin below the max power
    POWER_SAFE_MW = (POWER_MAX - SAFETY_MARGIN) * 1000.0  # Max power in mW

    # Chip selects
    CLK_FN_NEG = 0x10
    CLK_FN = 0x10
    TSCS = 0x08
    ADCS = 0x08
    DACS = 0x10
    CTRL_HV_EN_A = 0x20
    CTRL_HV_EN_B = 0x40
    DATASTATE = 0x02
    CLKSTATE = 0x01
    DACA = 0x18
    DACB = 0x19
    AD0 = 0xD0
    AD1 = 0xF0
    GPIO_LOW = 0x00
    GPIO_HIGH = 0x02
    TSCMD = 0xE0
    TSSTATUS = 0x00
    TSLSB = 0x01
    TSMSB = 0x02
    TSCONFIG = 0x80

    VID = 0x403
    PID = 0xD058

    def __init__(self, serial_number=None):
        self._is_open = False
        self._is_hv_on = False
        self._voltage = MiniX.HIGH_VOLTAGE_DEFAULT
        self._current = MiniX.CURRENT_DEFAULT
        self._register_state = RegisterState(0xFB, 0x08)

        self._ftdi = None
        self.__connect(serial_number)

    @staticmethod
    def find_devices():
        devices = []
        for descriptor, _ in UsbTools.find_all([(MiniX.VID, MiniX.PID)]):
            devices.append(
                {
                    "vid": descriptor.vid,
                    "pid": descriptor.pid,
                    "serial_number": descriptor.sn,
                    "description": descriptor.description,
                }
            )
        return devices

    def __is_safe_power(self, voltage, current):
        return voltage * current < MiniX.POWER_SAFE_MW

    def __read_data(self, nbytes):
        res = b""
        while len(res) < nbytes:
            res += self._ftdi.read_data(1)
        return res

    @check_connected
    def __init_temperatur_sensor(self):
        r = (
            Request(self._register_state)
            .set_clock_divisor()
            .clear_low(MiniX.CLKSTATE)
            .set_high(MiniX.TSCS)
            .clear_low(MiniX.CLKSTATE)
            .clear_low(MiniX.CLKSTATE)
            .set_low(MiniX.CLKSTATE)
            .add_bytes(0x10, 0x01, 0x00, MiniX.TSCONFIG, MiniX.TSCMD + 0x08)
            .clear_high(MiniX.TSCS)
        )
        self._ftdi.write_data(r.get_command())

    def __connect(self, serial_number=None):
        self._ftdi = Ftdi()

        if serial_number is None:
            devices = MiniX.find_devices()
            if len(devices) == 0:
                raise IOError("No devices found!")
            serial_number = devices[0]["serial_number"]

        self._ftdi.open_mpsse(
            vendor=MiniX.VID,
            product=MiniX.PID,
            serial=serial_number,
            interface=1,
            latency=12,
        )
        self._serial_number = serial_number

        # Initialize IO lines
        self._register_state.high_byte_state = 0x08
        self._register_state.low_byte_state = 0xFB
        r = (
            Request(self._register_state)
            .clear_high(MiniX.TSCS)
            .clear_low(MiniX.CTRL_HV_EN_A, MiniX.CTRL_HV_EN_B)
        )
        self._ftdi.write_data(r.get_command())

        # Initialize other features
        self.__init_temperatur_sensor()
        r = Request(self._register_state).set_clock_divisor()
        self._ftdi.write_data(r.get_command())
        self.voltage = MiniX.HIGH_VOLTAGE_DEFAULT
        self.current = MiniX.CURRENT_DEFAULT

    @property
    def serial_number(self):
        return self._serial_number

    @property
    @check_connected
    def config(self):
        r = Request(self._register_state).add_bytes(0x81, 0x83)
        self._ftdi.write_data(r.get_command())
        return self.__read_data(2)

    @property
    @check_connected
    def interlock_state(self):
        return bool(self.config[1] & 0x01)

    @property
    @check_connected
    def monitor_ready(self):
        return not bool(self.config[1] & 0x80)

    @property
    @check_connected
    def enabled(self):
        state = bool(self.config[0] & (0x20 | 0x40))  # HV_CHANNEL_A | HV_CHANNEL_B
        self._is_hv_on = state
        return state

    @check_connected
    def power_on(self):
        if not self.HIGH_VOLTAGE_RANGE[0] < self._voltage < self.HIGH_VOLTAGE_RANGE[1]:
            raise ValueOutOfRange("High voltage is out of range!")
        if not self.CURRENT_RANGE[0] < self._current < self.CURRENT_RANGE[1]:
            raise ValueOutOfRange("Current is out of range!")
        if not self.__is_safe_power(self._voltage, self._current):
            raise PowerNotSafe("Too much power, lower voltage or current!")
        r = Request(self._register_state).set_low(
            MiniX.CTRL_HV_EN_A, MiniX.CTRL_HV_EN_B
        )
        self._ftdi.write_data(r.get_command())
        self._is_hv_on = True

    @check_connected
    def power_off(self):
        r = Request(self._register_state).clear_low(
            MiniX.CTRL_HV_EN_A, MiniX.CTRL_HV_EN_B
        )
        self._ftdi.write_data(r.get_command())
        self._is_hv_on = False

    @property
    @check_connected
    def measured_voltage(self):
        r = (
            Request(self._register_state)
            .set_clock_divisor()
            .clear_low(MiniX.ADCS, MiniX.CLKSTATE)
            .add_bytes(0x13, 0x03, MiniX.AD0)
            .set_input()
            .add_bytes(0x20, 0x01, 0x00)
            .set_low(MiniX.ADCS)
        )
        self._ftdi.write_data(r.get_command())
        result = self.__read_data(2)

        msb = result[0] >> 3
        lsb = ((result[0] & 0x07) << 5) | (result[1] >> 3)
        i_volts = (msb << 8) | lsb
        return (
            i_volts
            / MiniX.DAC_ADC_SCALE
            * MiniX.V_REF
            * MiniX.HIGH_VOLTAGE_CONVERSION_FACTOR
        )

    @property
    @check_connected
    def voltage(self):
        return self._voltage

    @voltage.setter
    @check_connected
    def voltage(self, voltage):
        if self._is_hv_on:
            if not self.HIGH_VOLTAGE_RANGE[0] < voltage < self.HIGH_VOLTAGE_RANGE[1]:
                raise ValueOutOfRange()

        # Check if power is within safe range before setting the value
        if not self.__is_safe_power(voltage, self._current):
            raise PowerNotSafe()

        self._voltage = voltage

        # Convert HV voltage to ADC voltage-value
        adc_voltage = (voltage + 0.00001) / MiniX.HIGH_VOLTAGE_CONVERSION_FACTOR

        i_volts = int(adc_voltage / MiniX.V_REF * MiniX.DAC_ADC_SCALE)

        byte_high = (i_volts & 0x0FF0) >> 4
        byte_low = (i_volts & 0x0F) << 4

        r = (
            Request(self._register_state)
            .set_clear_low(MiniX.CLKSTATE, MiniX.DACS)
            .add_bytes(MiniX.CLK_FN, 0x02, 0x00, MiniX.DACA, byte_high, byte_low)
            .set_low(MiniX.DACS)
        )
        self._ftdi.write_data(r.get_command())

    @property
    @check_connected
    def measured_current(self):
        r = (
            Request(self._register_state)
            .set_clock_divisor()
            .clear_low(MiniX.ADCS, MiniX.CLKSTATE)
            .add_bytes(0x13, 0x03, MiniX.AD1)
            .set_input()
            .add_bytes(0x20, 0x01, 0x00)
            .set_low(MiniX.ADCS)
        )
        self._ftdi.write_data(r.get_command())
        result = self.__read_data(2)

        msb = result[0] >> 3
        lsb = ((result[0] & 0x07) << 5) | (result[1] >> 3)

        i_volts = (msb << 8) | lsb
        return (
            i_volts
            / MiniX.DAC_ADC_SCALE
            * MiniX.V_REF
            * MiniX.CURRENT_CONVERSION_FACTOR
        )

    @property
    @check_connected
    def current(self):
        return self._current

    @current.setter
    @check_connected
    def current(self, new_current):
        if self._is_hv_on:
            if not MiniX.CURRENT_RANGE[0] < new_current < MiniX.CURRENT_RANGE[1]:
                raise ValueOutOfRange(
                    "The current has to be within {} < current < {}".format(
                        MiniX.CURRENT_RANGE[0], MiniX.CURRENT_RANGE[1]
                    )
                )

        if not self.__is_safe_power(self._voltage, new_current):
            raise PowerNotSafe()

        self._current = new_current

        # Convert to DAC value
        new_current_dac = (new_current + 0.00001) / MiniX.CURRENT_CONVERSION_FACTOR
        i_volts = int(new_current_dac / MiniX.V_REF * MiniX.DAC_ADC_SCALE)

        byte_high = (i_volts & 0x0FF0) >> 4
        byte_low = (i_volts & 0x0F) << 4

        r = (
            Request(self._register_state)
            .set_clear_low(MiniX.CLKSTATE, MiniX.DACS)
            .add_bytes(MiniX.CLK_FN, 0x02, 0x00, MiniX.DACB, byte_high, byte_low)
            .set_low(MiniX.DACS)
        )
        self._ftdi.write_data(r.get_command())

    @property
    @check_connected
    def temperature(self):
        r = (
            Request(self._register_state)
            .add_bytes(0x86, 0x03, 0x00)  # TCK_DIVISOR=0x86
            .clear_low(MiniX.CLKSTATE, MiniX.DATASTATE)
            .set_high(MiniX.TSCS)
            .clear_low(MiniX.CLKSTATE, MiniX.DATASTATE)
            .clear_low(MiniX.CLKSTATE, MiniX.DATASTATE)
            .set_clear_low(MiniX.CLKSTATE, MiniX.DATASTATE)
            .add_bytes(0x12, 0x07, 0x01)
            .set_input()
            .add_bytes(0x20, 0x01, 0x00)
            .clear_high(MiniX.TSCS)
        )
        self._ftdi.write_data(r.get_command())
        result = self.__read_data(2)

        # Check for sign bit
        lsb, msb = result
        i = (msb << 4) + (lsb >> 4)

        # Do sign conversion
        if msb & (1 << 8):
            i -= 4096
        return i * 0.0625
