# Device register info
class Ad7124_Register:
    '''
    Class representation of a register on the AD7124. This is a simple struct
    in the C++ library.
    '''
    def __init__(self, addr: int, value: int, size: int, rw: int):
        self.addr = addr
        self.value = value
        self.size = size
        self.rw = rw

class Ad7124SetupVals:
    def __init__(self):
        self.ref = AD7124_Ref_ExtRef1
        self.gain = AD7124_Gain_1
        self.bipolar = True
        self.burnout = AD7124_Burnout_Off
        self.filter = AD7124_Filter_SINC4
        self.fs = 0
        self.post_filter = AD7124_PostFilter_NoPost
        self.rej60 = False
        self.single_cycle = False
        self.offset_coeff = 0
        self.gain_coeff = 0
        self.refV = 2.500

class Ad7124Setup:
    '''Subclass to manage AD7124 "setups"'''

    def __init__(self, driver, index):
        self._setup_number = index
        self._driver = driver
        self.setup_values = Ad7124SetupVals()

    def set_config(self, ref_source, gain, bipolar: bool, 
                   burnout = AD7124_Burnout_Off, 
                   exRefV: float = 2.50):
        '''Sets configuration register values'''

        self.setup_values.ref = ref_source
        self.setup_values.gain = gain
        self.setup_values.bipolar = bipolar
        self.setup_values.burnout #not yet supported
        self.setup_values.refV = exRefV

        #Offset to config reg group
        reg = self._setup_number + AD7124_CFG0_REG

        self._driver.regs[reg].value = AD7124_CFG_REG_REF_SEL(ref_source) | \
                                       AD7124_CFG_REG_PGA(gain) | \
                                       (AD7124_CFG_REG_BIPOLAR if bipolar else 0) | \
                                       AD7124_CFG_REG_BURNOUT(burnout) | \
                                       AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_REF_BUFM | \
                                       AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM

        #print(f"Writing {hex(self._driver.regs[reg].value)} to adc config register")
        return self._driver.write_register(self._driver.regs[reg])

    # Not sure what to do about type hints here?
    def set_filter(self, filter, fs, 
                   post_filter = AD7124_PostFilter_NoPost,
                   rej60: bool = False, single_cycle: bool = False):
        '''Sets the filter type and output word rate for a setup'''

        self.setup_values.filter = filter
        self.setup_values.fs = fs
        self.setup_values.post_filter = post_filter
        self.setup_values.rej60 = rej60
        self.setup_values.single_cycle = single_cycle

        #Offset to filter reg group
        reg = self._setup_number + AD7124_FILT0_REG

        self._driver.regs[reg].value = AD7124_FILT_REG_FILTER(filter) | \
                                       AD7124_FILT_REG_POST_FILTER(post_filter) | \
                                       AD7124_FILT_REG_FS(fs) | \
                                       (AD7124_FILT_REG_REJ60 if rej60 else 0) | \
                                       (AD7124_FILT_REG_SINGLE_CYCLE if single_cycle else 0)

        #print(f"Writing {hex(self._driver.regs[reg].value)} to the {hex(reg)} filter register")
        return self._driver.write_register(self._driver.regs[reg])

    def set_offset_cal(self, value: int):
        '''
        Sets the offset calibration value for a setup
        NOT YET IMPLEMENTED
        '''
        pass

    def set_gain_cal(self, value: int):
        '''
        Sets the gain calibration value for a setup
        NOT YET IMPLEMENTED
        '''
        pass

class Ad7124:
    def __init__(self, csPin, spi):
        '''
        Initializes the AD7124 and sets up the SPI interface.
        '''
        self.spi = spi
        self._cs = csPin  # Chip-select pin
        self._crc_enabled = False
        self.opmode = AD7124_OpMode_SingleConv

        self.setup = []

        for i in range(8):
            self.setup.append(Ad7124Setup(self, i))

        # SPI buffer
        self.spi_buffer = bytearray(8)
        self.spi_buf_mv = memoryview(self.spi_buffer)
        self.spi_buf_2 = self.spi_buf_mv[:2]
        self.spi_buf_3 = self.spi_buf_mv[:3]
        self.spi_buf_4 = self.spi_buf_mv[:4]
        self.spi_buf_5 = self.spi_buf_mv[:5]
        self.spi_buffs = (self.spi_buf_2, self.spi_buf_3, self.spi_buf_4, self.spi_buf_5)

        # Temporary reg struct for data with extra byte to hold status bits
        self.reg_data_and_status = Ad7124_Register(0x02, 0x0000, 4, 2)

        # Initialize list of register values. Initially set to POR values
        self.regs = [
            Ad7124_Register(0x00, 0x00, 1, 2),     # Status
            Ad7124_Register(0x01, 0x0000, 2, 1),   # ADC_Control
            Ad7124_Register(0x02, 0x0000, 3, 2),   # Data
            Ad7124_Register(0x03, 0x0000, 3, 1),   # IOCon1
            Ad7124_Register(0x04, 0x0000, 2, 1),   # IOCon2
            Ad7124_Register(0x05, 0x02, 1, 2),     # ID
            Ad7124_Register(0x06, 0x0000, 3, 2),   # Error
            Ad7124_Register(0x07, 0x0044, 3, 1),   # Error_En
            Ad7124_Register(0x08, 0x00, 1, 2),     # Mclk_Count

            Ad7124_Register(0x09, 0x8001, 2, 1),   # Channel_0
            Ad7124_Register(0x0A, 0x0001, 2, 1),   # Channel_1
            Ad7124_Register(0x0B, 0x0001, 2, 1),   # Channel_2 
            Ad7124_Register(0x0C, 0x0001, 2, 1),   # Channel_3 
            Ad7124_Register(0x0D, 0x0001, 2, 1),   # Channel_4 
            Ad7124_Register(0x0E, 0x0001, 2, 1),   # Channel_5 
            Ad7124_Register(0x0F, 0x0001, 2, 1),   # Channel_6 
            Ad7124_Register(0x10, 0x0001, 2, 1),   # Channel_7 
            Ad7124_Register(0x11, 0x0001, 2, 1),   # Channel_8 
            Ad7124_Register(0x12, 0x0001, 2, 1),   # Channel_9 
            Ad7124_Register(0x13, 0x0001, 2, 1),   # Channel_10
            Ad7124_Register(0x14, 0x0001, 2, 1),   # Channel_11
            Ad7124_Register(0x15, 0x0001, 2, 1),   # Channel_12
            Ad7124_Register(0x16, 0x0001, 2, 1),   # Channel_13
            Ad7124_Register(0x17, 0x0001, 2, 1),   # Channel_14
            Ad7124_Register(0x18, 0x0001, 2, 1),   # Channel_15

            Ad7124_Register(0x19, 0x0860, 2, 1),   # Config_0 
            Ad7124_Register(0x1A, 0x0860, 2, 1),   # Config_1 
            Ad7124_Register(0x1B, 0x0860, 2, 1),   # Config_2 
            Ad7124_Register(0x1C, 0x0860, 2, 1),   # Config_3 
            Ad7124_Register(0x1D, 0x0860, 2, 1),   # Config_4 
            Ad7124_Register(0x1E, 0x0860, 2, 1),   # Config_5 
            Ad7124_Register(0x1F, 0x0860, 2, 1),   # Config_6 
            Ad7124_Register(0x20, 0x0860, 2, 1),   # Config_7 

            Ad7124_Register(0x21, 0x060180, 3, 1), # Filter_0 
            Ad7124_Register(0x22, 0x060180, 3, 1), # Filter_1 
            Ad7124_Register(0x23, 0x060180, 3, 1), # Filter_2 
            Ad7124_Register(0x24, 0x060180, 3, 1), # Filter_3 
            Ad7124_Register(0x25, 0x060180, 3, 1), # Filter_4 
            Ad7124_Register(0x26, 0x060180, 3, 1), # Filter_5 
            Ad7124_Register(0x27, 0x060180, 3, 1), # Filter_6 
            Ad7124_Register(0x28, 0x060180, 3, 1), # Filter_7 

            Ad7124_Register(0x29, 0x800000, 3, 1), # Offset_0 
            Ad7124_Register(0x2A, 0x800000, 3, 1), # Offset_1 
            Ad7124_Register(0x2B, 0x800000, 3, 1), # Offset_2 
            Ad7124_Register(0x2C, 0x800000, 3, 1), # Offset_3 
            Ad7124_Register(0x2D, 0x800000, 3, 1), # Offset_4 
            Ad7124_Register(0x2E, 0x800000, 3, 1), # Offset_5 
            Ad7124_Register(0x2F, 0x800000, 3, 1), # Offset_6 
            Ad7124_Register(0x30, 0x800000, 3, 1), # Offset_7 

            Ad7124_Register(0x31, 0x500000, 3, 1), # Gain_0 
            Ad7124_Register(0x32, 0x500000, 3, 1), # Gain_1 
            Ad7124_Register(0x33, 0x500000, 3, 1), # Gain_2 
            Ad7124_Register(0x34, 0x500000, 3, 1), # Gain_3 
            Ad7124_Register(0x35, 0x500000, 3, 1), # Gain_4 
            Ad7124_Register(0x36, 0x500000, 3, 1), # Gain_5 
            Ad7124_Register(0x37, 0x500000, 3, 1), # Gain_6 
            Ad7124_Register(0x38, 0x500000, 3, 1), # Gain_7
        ]

        self.reset()
        time.sleep(0.1)

    def reset(self):
        '''
        Write 64 1s to reset the chip.
        '''
        print("Attempt to reset by writing 64 1s")

        buff = bytearray([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
        self.spi_write_and_read(buff) #NOTE: No CS    

        print("Waiting for power on")
        return self.wait_for_power_on(100)

    def get_ID(self):
        self.read_register(self.regs[AD7124_ID_REG])
        return self.regs[AD7124_ID_REG].value

    def setPWRSW(self, enabled):
        self.regs[AD7124_IO_CTRL1_REG].value &= ~AD7124_IO_CTRL1_REG_PDSW

        if enabled:
            self.regs[AD7124_IO_CTRL1_REG].value |= AD7124_IO_CTRL1_REG_PDSW

        return self.write_register(self.regs[AD7124_IO_CTRL1_REG])

    def wait_for_power_on(self, timeout=None):
        """
        Waits until the AD7124 completes its power-on reset (POR).

        Args:
            timeout: Optional timeout in milliseconds. Defaults to waiting indefinitely.

        Returns:
            True if POR completed successfully, False if timeout occurred or an error
            encountered.
        """
        powered_on = False
        start_time = time.monotonic() * 1000

        while True:
            self.read_register(self.regs[AD7124_STATUS_REG])
            powered_on = not (self.regs[AD7124_STATUS_REG].value & AD7124_STATUS_REG_POR_FLAG)

            if powered_on:
                return True

            if timeout is None:
                continue

            if (time.monotonic() * 1000 - start_time) >= timeout:
                return AD7124_TIMEOUT

            time.sleep(0.001)

        raise RuntimeError("Unexpected error!")

    def set_vbias(self, vBiasPin, enabled):
        '''
        Sets enables the bias voltage generator on the given pin. A bias voltage
        is necessary to read truly bipolar output from thermocouples 
        '''
        self.regs[AD7124_IO_CTRL2_REG].value &= ~(1 << vBiasPin)

        if enabled:
            self.regs[AD7124_IO_CTRL2_REG].value |= (1 << vBiasPin)

        return self.write_register(self.regs[AD7124_IO_CTRL2_REG])

    def read_raw(self, ch: int):
        '''
        Get a reading in raw counts from a single channel
        '''
        cur_ch = self.current_channel()

        if ch != cur_ch:
            # Disable previous channel if different
            ret = self.enable_channel(cur_ch, False)
            if ret < 0:
                return ret

            # Moved here so only called if channel changed
            if self.opmode == AD7124_OpMode_SingleConv:
                ret = self.enable_channel(ch, True)
                if ret < 0:
                    return ret

                # write the mode register again to start the conversion.
                ret = self.set_mode(AD7124_OpMode_SingleConv)
                if ret < 0:
                    return ret

            # If in continuous mode, just enable the channel we want to read now
            else:
                ret = self.enable_channel(ch, True)
                if ret < 0:
                    return ret

        # If no channel change, just call setMode
        else:
            # If we are in single conversion mode, we need to write the mode  
            # register again to start the conversion.
            if self.opmode == AD7124_OpMode_SingleConv:
                ret = self.set_mode(AD7124_OpMode_SingleConv)
                if ret < 0:
                    return ret

        ret = self.wait_for_conv_ready(DEFAULT_TIMEOUT_MS)
        if ret < 0:
            return ret

        return self.get_data()

    def read_volts(self, ch: int):
        '''Get a reading in voltage from a single channel.'''
        return self.to_volts(self.read_raw(ch), ch)

    def read_fb(self, ch: int, vEx: float, scale_factor: float = 1.00):
        '''
        Read a 4 wire full bridge sensor. Return value can be scaled with
        optional scaleFactor arg. Returns mV/V if scale factor is one (default)
        '''
        return ((self.read_volts(ch) * 1000.0) / vEx) * scale_factor
    
def set_adc_control(self, mode: int, power_mode: int, ref_en: bool, 
                    clk_sel: int = AD7124_Clk_Internal):
    '''
    Sets up the ADC control register
    '''
    #NOTE: We always uses Data + Status mode
    self.regs[AD7124_ADC_CTRL_REG].value = AD7124_ADC_CTRL_REG_MODE(mode) | \
                                           AD7124_ADC_CTRL_REG_POWER_MODE(power_mode) | \
                                           AD7124_ADC_CTRL_REG_CLK_SEL(clk_sel) | \
                                           (AD7124_ADC_CTRL_REG_REF_EN if ref_en else 0) | \
                                           AD7124_ADC_CTRL_REG_DATA_STATUS | \
                                           AD7124_ADC_CTRL_REG_CS_EN

    print(f"Writing {hex(self.regs[AD7124_ADC_CTRL_REG].value)} to ADC control register")
    return self.write_register(self.regs[AD7124_ADC_CTRL_REG])



    def read_ic_temp(self, ch: int):
        '''
        Read the on chip temp sensor. 
        NOTE: The channel must first be setup properly
        for reading thermocouples. 
        '''
        return self.scale_ic_temp(self.read_raw(ch))

    def set_mode(self, mode: int):
        '''Control the mode of operation for ADC'''

        self.opmode = mode

        self.regs[AD7124_ADC_CTRL_REG].value &= ~AD7124_ADC_CTRL_REG_MODE(0x3C) #clear mode
        self.regs[AD7124_ADC_CTRL_REG].value |= AD7124_ADC_CTRL_REG_MODE(mode)

        return self.write_register(self.regs[AD7124_ADC_CTRL_REG])

    def set_channel(self, ch: int, setup: int, aiPos: int, aiNeg: int, enable: bool):
        '''
        Configure a channel

        Args:
            ch (int):                Channel to configure
            setup (Ad7124Setup):     Setup to use for channel
            aiPos (AD7124_InputSel): Physical pin, or internal source for AIN +
            aiNeg (AD7124_InputSel): Physical pin, or internal source for AIN -
            enable (bool):           enable/disable channel
        '''
        if ((ch < 16) and (setup < 8)):
            # Offset to channel regs
            ch += AD7124_CH0_MAP_REG

            self.regs[ch].value = AD7124_CH_MAP_REG_SETUP(setup) | \
                                  AD7124_CH_MAP_REG_AINP(aiPos) | \
                                  AD7124_CH_MAP_REG_AINM(aiNeg) | \
                                  (AD7124_CH_MAP_REG_CH_ENABLE if enable else 0)

            return self.write_register(self.regs[ch])

        return -1

    def enable_channel(self, ch: int, enable: bool):
        '''
        Enable/Disable a channel

        Args:
            ch (int):
            enabled (bool):  Enabled (True) or disabled (False)
        '''
        if (ch < 16):
            # Offset to channel regs
            ch += AD7124_CH0_MAP_REG

            ret = self.read_register(self.regs[ch])
            if ret < 0:
                return ret

            if enable:
                self.regs[ch].value |= AD7124_CH_MAP_REG_CH_ENABLE
            else:
                self.regs[ch].value &= ~AD7124_CH_MAP_REG_CH_ENABLE

            return self.write_register(self.regs[ch])

        return -1

    def enabled(self, ch: int) -> bool:
        ''' Simply returns if a channel is enabled or not'''
        ch += AD7124_CH0_MAP_REG
        return (self.regs[ch].value & AD7124_CH_MAP_REG_CH_ENABLE) >> 15

    def channel_setup(self, ch: int):
        ''' Returns the setup number used by the channel '''
        if (ch < AD7124_MAX_CHANNELS):
            ch += AD7124_CH0_MAP_REG
            setup = (self.regs[ch].value >> 12) & 0x07
            return setup
        return -1

    def current_channel(self):
        '''Returns the currently active channel'''
        return self.regs[AD7124_STATUS_REG].value & 0x0F

    def get_data(self):
        '''
        Returns positive raw ADC counts, or negative error code
        '''
        ret = self.no_check_read_register(self.reg_data_and_status)
        if ret < 0:
            return ret

        self.regs[AD7124_STATUS_REG].value = self.reg_data_and_status.value & 0xFF
        self.regs[AD7124_DATA_REG].value = (self.reg_data_and_status.value >> 8) & 0x00FFFFFF

        return self.regs[AD7124_DATA_REG].value

    def to_volts(self, value: int, ch: int):
        '''Convert raw ADC data to volts'''
        idx = self.channel_setup(ch)
        chReg = ch + AD7124_CH0_MAP_REG

        # Special case, if reading internal temp sensor just 
        # return the original value unchanged so that the output
        # can be easily converted with formula from datasheet  
        ainP = (self.regs[chReg].value >> 5) & 0x1F
        ainN = self.regs[chReg].value & 0x1F
        if (ainP == AD7124_Input_TEMP) or (ainN == AD7124_Input_TEMP):
            return value

        if self.setup[idx].setup_values.bipolar:
            voltage = value / 0x7FFFFF - 1
        else:
            voltage = value / 0xFFFFFF

        # .setup_values.gain holds 0 to 7 value that sets the actual gain in the register
        voltage = (voltage * self.setup[idx].setup_values.refV) / (1 << self.setup[idx].setup_values.gain)
        return voltage

    def scale_ic_temp(self, value):
        '''
        Convert raw value from IC temperature sensor to degrees C
        '''
        return ((value - 0x800000) / 13548.00) - 272.5

    def wait_for_conv_ready(self, timeout):
        '''Waits until a new conversion result is available.'''
        start_time = time.monotonic() * 1000

        while True:
            ret = self.no_check_read_register(self.regs[AD7124_STATUS_REG])
            if ret < 0:
                return ret

            ready = (self.regs[AD7124_STATUS_REG].value & AD7124_STATUS_REG_RDY) == 0
            if ready:
                return ready

            if (time.monotonic() * 1000 - start_time) >= timeout:
                return AD7124_TIMEOUT

            time.sleep(0.001)

    def no_check_read_register(self, reg: Ad7124_Register):
        '''
        Reads the value of the specified register without checking if the
        device is ready. Updates reg.value in place
        '''
        if (reg is None) or (reg.rw == AD7124_W):
            return AD7124_INVALID_VAL

        buffer = self.spi_buffs[reg.size - 1]

        buffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg.addr)

        self.spi_write_and_read(buffer)

        reg.value = 0
        for i in range(1, reg.size + 1):
            reg.value <<= 8
            reg.value += buffer[i]

        return 0

    def read_register(self, reg: Ad7124_Register):
        '''
        Reads the value of the specified register after checking if the
        device is ready. Updates reg.value in place
        '''
        if reg.addr != AD7124_ERR_REG:
            ret = self.wait_for_spi_ready()
            if ret < 0:
                return ret

        return self.no_check_read_register(reg)

    def no_check_write_register(self, reg: Ad7124_Register):
        '''
        Writes the value of the specified register without checking if the
        device is ready. 
        '''
        if (reg is None) or (reg.rw == AD7124_R):
            return AD7124_INVALID_VAL

        buffer = self.spi_buffs[reg.size - 1]

        buffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA(reg.addr)

        value = reg.value

        for i in range(reg.size):
            buffer[reg.size - i] = value & 0xFF
            value >>= 8

        self.spi_write_and_read(buffer)

        return 0

    def write_register(self, reg: Ad7124_Register):
        '''
        Writes the value of the specified register after checking if the
        device is ready. 
        '''
        ret = self.wait_for_spi_ready()
        if ret < 0:
            return ret

        return self.no_check_write_register(reg)

    def spi_write_and_read(self, buff: bytearray):
        '''Writes and reads data via SPI'''
        try:
            self._cs.value = 0
            self.spi.xfer2(buff)
        finally:
            self._cs.value = 1

    def wait_for_spi_ready(self):
        '''
        Waits for the SPI interface to be ready for read/write.
        '''
        reg = self.regs[AD7124_ERR_REG]

        while True:
            ret = self.no_check_read_register(reg)

            ready = not (self.regs[AD7124_ERR_REG].value & AD7124_ERR_REG_SPI_IGNORE_ERR)
            if ready:
                return ready

    def computeCRC8(self, buffer, size):
        '''
        Computes the CRC checksum for a data buffer.
        NOT USED YET
        '''
        crc = 0

        while size:
            for i in range(80, 0, -1):
                if ((crc & 0x80) != 0) != ((buffer[0] & i) != 0):
                    crc <<= 1
                    crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION
                else:
                    crc <<= 1
            buffer = buffer[1:]
            size -= 1

        return crc


# Example usage

# Assuming you have the Ad7124 class already defined in ad7124_cp3.py
#from ad7124_cp3 import Ad7124, AD7124_Input_AIN0, AD7124_Gain_1, AD7124_Ref_Internal, AD7124_OpMode_Continuous

# SPI configuration
spi = spidev.SpiDev()
spi.open(0, 1)  # Open SPI bus 0, device (CS) 0
spi.max_speed_hz = 500000  # Set SPI speed (500 kHz)
spi.mode = 0  # SPI mode 0

# Chip-select pin (replace with your actual CS pin configuration)
cs_pin = 7

# Initialize AD7124
adc = Ad7124(cs_pin, spi)

# Configure ADC control
adc.set_adc_control(mode=AD7124_OpMode_Continuous, power_mode=2, ref_en=True)

# Configure channel A0
adc.set_channel(ch=0, setup=0, aiPos=AD7124_Input_AIN0, aiNeg=AD7124_Input_AIN0, enable=True)

# Configure setup 0
adc.setup[0].set_config(ref_source=AD7124_Ref_Internal, gain=AD7124_Gain_1, bipolar=True)

# Wait for a moment to let the ADC settle
time.sleep(0.1)

# Read the voltage on channel A0
voltage = adc.read_volts(0)

print(f"Measured voltage on channel A0: {voltage:.6f} V")
