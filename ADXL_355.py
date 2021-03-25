from machine import Pin, SPI
from micropython import const
import time
import gc
import math


# =====================
# WARNING
# =====================
# libraries for ADXL355 and 357 are perfectly the same except for g_range
# set accelerometer model according to the accelerometer mounted on the device
accelerometer_model = 'ADXL355'  # 'ADXL355', 'ADXL357'
# ==============================


class Accelerometer:

    def __init__(self, cs_pin=5, sclk_pin=18, mosi_pin=23, miso_pin=19, spi_freq=10000000):
        """
        Class for fast SPI comunications between an ESP32 flashed with MicroPython and an Analog Devices ADXL355Z
          accelerometer
        :param cs_pin: MCU pin number at which accelerometer's CS wire is connected
        :param sclk_pin: MCU pin number at which accelerometer's SCL wire is connected (SCK)
        :param mosi_pin: MCU pin number at which accelerometer's SDA wire is connected (MOSI)
        :param miso_pin: MCU pin number at which accelerometer's SDO wire is connected (MISO)
        :param spi_freq: frequency of SPI comunications, max 10MHz
        """

        # valid inputs
        if spi_freq > 10000000:
            spi_freq = 10000000
            print('max spi clock frequency for adxl355 is 10Mhz')

        # constants
        self.standard_g         = 9.80665       # m/s2
        self.read_mask          = const(0x01)
        self.nmaxvalues_infifo  = 32  # xyz counts 1
        self.bytes_per_3axes    = 9  # 3bytes * 3axes
        self.device_id          = 0xAD
        self.twentybits         = 1048576  # max int for 20 bits
        self.scale_factor       = 1.95E-6  # convert raw int to g units

        # default values
        self.power_mode = 'standby'
        self.fifo_mode = 'stream'
        self.g_range            = 2
        self.sampling_rate      = 4000
        self.highpass_cutoff    = 0
        self.watermark          = 96

        # addresses
        self.regaddr_devid          = const(0x00)
        self.regaddr_status         = const(0x04)
        self.regaddr_fifoentries    = const(0x05)
        self.regaddr_acc            = const(0x08)
        self.regaddr_fifodata       = const(0x11)
        self.regaddr_filter         = const(0x28)
        self.regaddr_fifosamples    = const(0x29)
        self.regaddr_range          = const(0x2c)
        self.regaddr_pwrctl         = const(0x2d)
        self.regaddr_reset          = const(0x2F)
        self.regaddr_shadow         = const(0x50)

        # SPI pins
        self.cs_pin = cs_pin
        self.scl_pin = sclk_pin
        self.miso_pin = miso_pin
        self.mosi_pin = mosi_pin
        self.spi_freq = spi_freq

        # init
        self.init_spi()
        self.reset()  # otherwise acc data are stuck to lower boundary of range (device bug)"""

    # == general purpose ==
    def init_spi(self):
        time.sleep(.1)
        self.cs = Pin(self.cs_pin, Pin.OUT, value=1)
        time.sleep(.1)
        self.spi = SPI(
            2, sck=Pin(self.scl_pin, Pin.OUT), mosi=Pin(self.mosi_pin, Pin.OUT), miso=Pin(self.miso_pin),
            baudrate=self.spi_freq, polarity=0, phase=0, bits=8, firstbit=SPI.MSB
        )
        time.sleep(.1)
        # dunno why but necessary to init twice to make it work
        self.cs = Pin(self.cs_pin, Pin.OUT, value=1)
        time.sleep(.1)
        self.spi.deinit()
        time.sleep(.1)
        self.spi = SPI(
            2, sck=Pin(self.scl_pin, Pin.OUT), mosi=Pin(self.mosi_pin, Pin.OUT), miso=Pin(self.miso_pin),
            baudrate=self.spi_freq, polarity=0, phase=0, bits=8, firstbit=SPI.MSB
        )
        time.sleep(.1)
        # check if it's working
        if not self.is_spi_communication_working():
            time.sleep(.1)
            if not self.is_spi_communication_working():
                print(
                    'SPI communication is not working: '
                    '\n\t* wrong wiring?'
                    '\n\t* reinitialised SPI?'
                    '\n\t* broken sensor'
                )
        return self

    def deinit_spi(self):
        self.spi.deinit()
        return self

    @micropython.native
    def write(self, regaddr:int, the_byte:int):
        """
        write byte into register address
        :param regaddr: register address to write
        :param the_byte: byte to write
        """
        self.cs.value(0)
        self.spi.write(bytearray((regaddr << 1, the_byte)))
        self.cs.value(1)
        return self

    @micropython.native
    def read(self, regaddr:int, nbytes:int) -> bytearray or int:
        """
        read bytes from register
        :param regaddr: register address to read
        :param nbytes: number of bytes to read
        :return: byte or bytes read
        """
        self.cs.value(0)
        value = self.spi.read(nbytes + 1, (regaddr << 1) | self.read_mask)[1:]
        self.cs.value(1)
        return value

    @micropython.native
    def read_into(self, rbuf:bytearray, regaddr:int) -> bytearray:
        """
        read bytes from register into an existing bytearray, generally faster than normal read
        :param rbuf: bytearray where read values will be assigned to
        :param regaddr: register address to read
        :return: modified input bytearray
        """
        wbuf = (regaddr << 1) | self.read_mask
        self.cs.value(0)
        self.spi.readinto(rbuf, wbuf)
        self.cs.value(1)
        return rbuf

    @micropython.native
    def remove_first_bytes_from_bytearray_of_many_transactions(self, buf: bytearray) -> bytearray:
        """
        remove first byte of SPI transaction (which is irrelevant) from a buffer read through spi.readinto
        :param buf: bytearray of size multiple of (self.bytes_per_3axes + 1)
        :return: bytearray of size multiple of self.bytes_per_3axes
        """
        bytes_per_3axes = self.bytes_per_3axes
        return bytearray([b for i, b in enumerate(buf) if i % (bytes_per_3axes + 1) != 0])

    @micropython.native
    def remove_first_byte_in_trasmissions(self, buf:bytearray, nvalues_trasms:list) -> bytearray:
        """
        :param buf: bytearray of uncorrected data obtained from multiple multi-values spi.readinto from fifo
        :param nvalues_trasms: array where each entry contains the values read from fifo by the trasmission
        :return: input bytearray where first byte of each trasmission has been removed
        """
        cursor = 0
        for nvalues_trasm in nvalues_trasms:
            buf = buf[:cursor] + buf[cursor + 1:]
            cursor += nvalues_trasm
        return buf

    # == settings ==================================================
    def reset(self):
        """
        reset without checking shadow registers
        """
        self.write(self.regaddr_reset, 0x52)
        print('device reset')
        return self

    def reset2(self):
        """
        reset and check shadow register as indicated in the docs: gets stucked because they are never correct
        """
        print('resetting device')  # <== debug
        shadow_registers = self.read(self.regaddr_shadow, 5)
        while True:
            self.write(self.regaddr_reset, 0x52)
            time.sleep(1)
            if shadow_registers == self.read(self.regaddr_shadow, 5):
                break
        print('device reset')
        return self

    @micropython.native
    def set_power_mode(self, mode:str='standby'):
        """
        set the power mode of the accelerometer
        :param mode: {'measure', 'standby'}
        """
        mode2byte = {'measure': 2, 'measureT': 0, 'standby': 1}
        if mode not in mode2byte.keys():
            print(str(mode) + " not in allowed power modes " + str(list(mode2byte.keys())))
            mode = 'standby'
        current_value = self.read(self.regaddr_pwrctl, 1)[0]
        correct_value = current_value & 0b11111100  # clears last two bits
        correct_value = correct_value | mode2byte[mode]  # sets last two bits
        self.write(self.regaddr_pwrctl, correct_value)
        self.pwr_mode = mode
        time.sleep(.5)
        print('set power mode to %s' % (mode))
        return self

    def set_g_range(self, grange:int=2):
        """
        set the scale of output acceleration data
        :param grange: allowed values depend on accelerometer model
        """
        if accelerometer_model == 'ADXL355':
            range2byte = {2: 1, 4: 2, 8: 3}
        elif accelerometer_model == 'ADXL357':
            range2byte = {10: 1, 20: 2, 40: 3}
        else:
            raise ValueError('supported models are ADXL355 and ADXL357')
        if grange not in range2byte.keys():
            print(str(grange) + " not in allowed ranges " + str(list(range2byte.keys())))
            grange = list(range2byte.keys())[0]
        current_value = self.read(self.regaddr_range, 1)[0]
        correct_value = current_value & 0b11111100  # clears last two bits
        correct_value = correct_value | range2byte[grange]  # sets last two bits
        self.write(self.regaddr_range, correct_value)
        self.g_range = grange
        time.sleep(.5)
        print('set range to pm %s' % (grange))
        return self

    def set_sampling_rate(self, sr:float=2000):
        """
        :param sr: sampling rate of the accelerometer can be {4000, 2000, 1000, 500, 250, 125, 62.5, 31.25, 15.625, 7.813, 3.906}
        """
        sr_2_byte = {4000: 0, 2000: 1, 1000: 2, 500:3, 250:4, 125:5, 62.5:6, 31.25:7, 15.625:8, 7.813:9, 3.906:10}
        if sr not in sr_2_byte.keys():
            print(str(sr) + " not in allowed frequencies " + str(list(sr_2_byte.keys())))
            sr = 2000
        current_value = self.read(self.regaddr_filter, 1)[0]
        correct_value = current_value & 0b11110000  # clears last 4 bits
        correct_value = correct_value | sr_2_byte[sr]  # sets last 4 bits
        self.write(self.regaddr_filter, correct_value)
        self.sampling_rate = sr
        time.sleep(.5)
        print('set sampling_rate to ' + str(sr) + ', lpf cutoff at ' + str(sr / 4))
        return self

    def set_highpass_cutoff(self, hpf_corner:float=0):
        """
        :param hpf_corner:
        :return:
        """
        hpf2byte = {0: 0, .0238: 96, .0954: 80, .3862: 64, 1.5545: 48, 6.2084: 32, 24.7: 16}
        if hpf_corner not in hpf2byte.keys():
            print(str(hpf_corner) + " not in allowed frequencies " + str(list(hpf2byte.keys())))
            hpf_corner = 0
        current_value = self.read(self.regaddr_filter, 1)[0]
        correct_value = current_value & 0b10001111  # clears three bits of hpf
        correct_value = correct_value | hpf2byte[hpf_corner]  # sets three bits of hpf
        self.write(self.regaddr_filter, correct_value)
        self.hpf_corner = hpf_corner
        time.sleep(.5)
        print('set high-pass filter corner to ' + str(hpf_corner) + ' 10^-4 odr')
        return self

    def set_fifo_mode(self, mode='stream'):
        """
        ADXL355 is always ins stream mode, leave this method for compatibility with other accelerometers
        """
        print('set fifo mode to stream')
        return self

    def set_watermark(self, nrows:int=16):
        """
        set the number of new measures (xyz counts 1) after which the watermark is triggered
        """
        if not 0 < nrows < 3*self.nmaxvalues_infifo+1:
            nrows = 48
        current_value = self.read(self.regaddr_fifosamples, 1)[0]
        correct_value = current_value & 0b10000000  # clears last 7 bits
        correct_value = correct_value | nrows  # sets last 7 bits
        self.write(self.regaddr_fifosamples, correct_value)
        self.watermark = nrows
        print('set watermark to %s rows' % (nrows))
        return self

    # == readings ==================================================
    def is_spi_communication_working(self) -> bool:
        """
        :return: True if SPI test is succefull
        """
        if self.read(self.regaddr_devid, 1)[0] == self.device_id:
            return True
        else:
            return False

    @micropython.native
    def clear_fifo(self):
        """
        Clears all values in fifo: usefull to start reading FIFO when expected, otherwise the first 96 values were
        recorded before actually starting the measure
        """
        _ = self.read_many_xyz_fromfifo(self.nmaxvalues_infifo)
        del _

    @micropython.native
    def is_data_ready(self) -> bool:
        """
        :return: 1 if a new measure has arrived since last reading, 0 otherwise
        """
        return self.read(self.regaddr_status, 1)[0] >> 0 & 1

    @micropython.native
    def were_measures_lost(self) -> bool:
        """
        :return: 1 if a sample in fifo was overwritten without measuring it since last measure, 0 otherwise
        """
        return self.read(self.regaddr_status, 1)[0] >> 2 & 1

    @micropython.native
    def is_dataready_measurelost(self):
        """
        :return: two booleans: the first is True if there are new data available,
            the second is True if data were overwritten without reading them
        """
        byte = self.read(self.regaddr_status, 1)[0]
        is_dataready = byte >> 1 & 1
        were_measurelost = byte >> 2 & 1
        return is_dataready, were_measurelost

    @micropython.native
    def is_watermark_reached(self) -> bool:
        """
        :return: 1 if watermark level of measures was reached since last reading, 0 otherwise
        """
        return self.read(self.regaddr_status, 1)[0] >> 2 & 1

    @micropython.native
    def get_nvalues_in_fifo(self) -> int:
        """
        :return: number of measures (xyz counts 1) in the fifo since last reading
        """
        naxes_infifo = self.read(self.regaddr_fifoentries, 1)[0] & 0x7f
        nxyz_infifo = math.floor(naxes_infifo / 3)
        return nxyz_infifo

    @micropython.native
    def read_many_xyz_fromfifo(self, n:int=1) -> bytearray:
        """
        read many measures of accaleration on the 3 axes from the fifo register
        :param n: number of measures to read (xyz counts 1)
        :return: bytearray containing 3 bytes for each of the 3 axes, for nrows (9 * nrows bytes)
        """
        return self.read(self.regaddr_fifodata, self.bytes_per_3axes * n)

    @micropython.native
    def read_continuos_xyz_fromfifo(self, acquisition_time:int=1) -> tuple:
        """
        reads accelerations values continuously for the given number of seconds
        :param acquisition_time: seconds
        :return: buffer of measures along with the time of each measure
        """
        print("Measuring for %s seconds at %s Hz, range %sg" % (acquisition_time, self.sampling_rate, self.g_range))
        # definitions
        n_exp_meas = int(acquisition_time * self.sampling_rate)
        n_exp_bytes = self.bytes_per_3axes * n_exp_meas
        buf = bytearray(int(n_exp_bytes * 1.5))
        m = memoryview(buf)
        # set up device
        self.set_fifo_mode('stream')
        gc.collect()
        # measure
        n_act_meas = 0
        self.set_power_mode('measure')
        self.clear_fifo()
        self.were_measures_lost()  # clear data overflow register
        while n_act_meas < n_exp_meas:
            if self.were_measures_lost():
                raise OverflowError('Data overflow: try lowering sampling rate and/or acquisition time')
            nvalues_infifo = self.get_nvalues_in_fifo()
            if nvalues_infifo:
                start_index = n_act_meas * self.bytes_per_3axes
                stop_index  = n_act_meas * self.bytes_per_3axes + (nvalues_infifo * self.bytes_per_3axes)
                m[start_index : stop_index] = self.read_many_xyz_fromfifo(nvalues_infifo)
                n_act_meas += nvalues_infifo
        self.set_power_mode('standby')
        # final corrections
        buf = buf[:n_exp_meas * self.bytes_per_3axes]  # remove exceeding values
        T = [(i+1) / self.sampling_rate for i in range(n_exp_meas)]
        gc.collect()
        return buf, T

    # == conversions ==================================================
    def oneaxisbits2g(self, buf:bytearray, g_range:int=0) -> float:
        """
        convert 3 bytes corresponding to one measure on one axis in units of gravity on the sealevel (g)
        :param buf:
        :param g_range:
        :return:
        """
        if not g_range: g_range=self.g_range
        x = (buf[0] << 16 | buf[1] << 8 | buf[2]) >> 4
        if x >= (self.twentybits / 2): x = x - self.twentybits
        x = x * g_range * self.scale_factor
        return x

    def listofmeasuresonthreeaxes2g(self, buf:bytearray, g_range:int=0) -> tuple:
        """
        convert a bytearray of measures on the three axes xyz in three lists where the acceleration is in units of
            gravity on the sealevel (g)
        :param buf: bytearray of 3bytes * 3 axes * nvalues
        :return: 3 lists of ints corresponding to x, y, z values of acceleration in units of g
        """
        if not g_range: g_range=self.g_range
        x, y, z = [], [], []
        bytes_per_axis = int(self.bytes_per_3axes / 3)
        for i in range(len(buf)):
            if i % self.bytes_per_3axes == 0:
                x.append(self.oneaxisbits2g(buf[i                        : i + bytes_per_axis      ], g_range))
                y.append(self.oneaxisbits2g(buf[i + bytes_per_axis       : i + (bytes_per_axis * 2)], g_range))
                z.append(self.oneaxisbits2g(buf[i + (bytes_per_axis * 2) : i + self.bytes_per_3axes], g_range))
        return x, y, z
