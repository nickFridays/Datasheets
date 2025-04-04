#!/usr/bin/env python3
#
# Mikroe ADC 8 Click   
#########################################################################################

import time

I2C_ADDR    = 0x48  # Default i2c address for ADC 8 Click    
READ_REG    = 0x00
WRT_REG     = 0x01
LOW_THRSH   = 0x02
HIGH_THRSH  = 0x03
PWR_MODE    = 0x8000
MUX_OFFSET  = 12
DATA_RATE_DFLT = 128  # Sample rate DR bit default status in config reg.
CONTINUOUS_MODE  = 0x0000
SINGLE_MODE      = 0x0100
COMPR_DISABLE    = 0x0003
GAIN_CONFIG = {2/3:0x0000,1:0x0200,2:0x0400,4:0x0600,8:0x0800,16:0x0A00}
DATA_RATE = {8:0x0000,16:0x0020,32:0x0040,64:0x0060,128:0x0080,250:0x00A0,475:0x00C0,860:0x00E0}

class ADS1115():
    def __init__(self,bus,address=I2C_ADDR):
        self._address = address
        self._bus = bus
    def close(self):
        self._bus.close()

    def _conversion_value(self, low, high)->int:
        # Convert to signed int_16.
        value = ((high & 0xFF) << 8) | (low & 0xFF)
        # Return negative value if the sign bit is set.
        if value & 0x8000 != 0:
            value -= 1 << 16
        return value
    def _read(self, mux, gain, data_rate, mode):
        """Read ADC, returns a signed int."""
        config = PWR_MODE  # set bit 15 for power-up to start conversion.
        # Specify mux value.
        config |= (mux & 0x07) << MUX_OFFSET
        # Validate the passed in gain and then set it in the config.
        if gain not in GAIN_CONFIG:
            raise ValueError('Gain must be one of: 2/3, 1, 2, 4, 8, 16')
        config |= GAIN_CONFIG[gain]
        # Set the mode (continuous or single mode).
        config |= mode
        if data_rate is None or data_rate not in DATA_RATE:
            data_rate = DATA_RATE_DFLT
        # Set a data rate in config reg.
        config |= DATA_RATE[data_rate]
        config |= COMPR_DISABLE  # disalble comparator
        # Start the ADC conversion based on config.
        # Convert 16-bit value into a big endian.
        self.writeList(WRT_REG, [(config >> 8) & 0xFF, config & 0xFF])
        # Wait for adc conversion time = data rate plus 0.1 mlSec to get ADC sample.
        time.sleep(1.0 / data_rate + 0.0001)
        result = self.readList(READ_REG, 2)
        return self._conversion_value(result[1], result[0])
        #
    def read_adc(self, channel, gain=1, data_rate=None,avrg=3):
        """Reads a single ADC channel 0-3 and returns a signed int_16.
        The Config Reg. DR bit sets conversion speed samples/sec. 8,16,32,64,128(dflt),250,475,860      
        Gain vs Ref. voltage:        
             2/3 = +/-6.144V, 1 = +/-4.096V, 2 = +/-2.048V, 4 = +/-1.024V, 8 = +/-0.512V, 16 = +/-0.256V
        """
        assert 0 <= channel <= 3, 'Channel number is 0-3'
        # Single read with the mux value set to the channel plus bit 3 set to 1.
        volt=[]
        for i in range(avrg):
            volt.append(self._read(channel + 0x04, gain, data_rate, SINGLE_MODE))
        return int(sum(volt)/avrg)
        #
    def read_adc_diff(self, differential, gain=1, data_rate=None):
        """Read the difference between two ADC channels and return the ADC value
        as a signed integer result.  differential par can be:
          0 = Channel 0 minus channel 1
          1 = Channel 0 minus channel 3
          2 = Channel 1 minus channel 3
          3 = Channel 2 minus channel 3
        """
        assert 0 <= differential <= 3, 'Differential value is 0-3'
        # Single mode differential read. The mux value enables differential mode.
        return self._read(differential, gain, data_rate, SINGLE_MODE)
        #
    def start_adc(self, channel, gain=1, data_rate=None):
        """Start continuous ADC conversions for a channel 0-3. 
        Return an initial result, then calls get_last_result()
        to read the most recent result. Call stop_adc() to stop conversions.
        """
        assert 0 <= channel <= 3, 'Channel must be 0-3'
        # Start continuous reads and set the mux value to the channel plus
        # the highest bit (bit 3) set.
        return self._read(channel + 0x04, gain, data_rate, CONTINUOUS_MODE)
        #
    def start_adc_diff(self, differential, gain=1, data_rate=None):
        """Start continuous readings between two ADC channels. Differential par:
          - 0 = Channel 0 minus channel 1
          - 1 = Channel 0 minus channel 3
          - 2 = Channel 1 minus channel 3
          - 3 = Channel 2 minus channel 3
        Returns first result and calls get_last_result() continuously for 
        the most recent data. Call stop_adc() to stop ADC.
        """
        assert 0 <= differential <= 3, 'Differential must be 0-3!'
        # Perform a single shot read using the provided differential value
        # as the mux value (which will enable differential mode).
        return self._read(differential, gain, data_rate, CONTINUOUS_MODE)
    def stop_adc(self):
        """Stops continuous ADC"""
        # Set the config register to default value of 0x8583 to stop continuous conversions.
        config = 0x8583
        self.writeList(WRT_REG, [(config >> 8) & 0xFF, config & 0xFF])
    def get_last_result(self):
        """Returns a signed int_16 for the last ADC result in continuous mode."""
        # Get conversion register data and return it as big endian signed int_16.
        result = self.readList(READ_REG, 2)
        return self._conversion_value(result[1], result[0])
    def readList(self, register, length):
        """Read a length number of bytes from the specified register.Returs a bytearray."""
        results = self._bus.read_i2c_block_data(self._address, register, length)
        return results    
    def writeList(self, register, data):
        """Write bytes to the register."""
        self._bus.write_i2c_block_data(self._address, register, data)



import smbus
i2c_bus= smbus.SMBus(1)
adc = ADS1115(i2c_bus)  # dflt i2c addr 0x48

#adc = ADS1015(address=0x49, busnum=1)

# gain = 1 for reading voltages from 0 to 4.09V.
GAIN = 1

# Read ADC channel between 0-3 with specified gain 1.  round(adc.read_adc(2)*125e-6, 2)
print( round(adc.read_adc(0, gain=1)*125e-3, 4))
print( round(adc.read_adc(0, gain=2)*125e-3/2, 4))
print( round(adc.read_adc(0, gain=4)*125e-3/4, 4))
print( round(adc.read_adc(0, gain=8)*125e-3/8, 4))
print( round(adc.read_adc(0, gain=16)*125e-3/16, 4))
print( round(adc.read_adc(0, gain=16)*125e-3/16, 4))
print( round(adc.read_adc(0, gain=16)*125e-3/16, 4))
print( round(adc.read_adc(0, gain=1,data_rate=DATA_RATE[32])*125e-3, 4))
print( round(adc.read_adc(0, gain=2,data_rate=DATA_RATE[32])*125e-3/2, 4))
print( round(adc.read_adc(0, gain=4,data_rate=DATA_RATE[32])*125e-3/4, 4))
print( round(adc.read_adc(0, gain=8,data_rate=DATA_RATE[32])*125e-3/8, 4))
print( round(adc.read_adc(0, gain=16,data_rate=DATA_RATE[32])*125e-3/16, 4))
print( round(adc.read_adc(0, gain=16,data_rate=DATA_RATE[32])*125e-3/16, 4))
print( round(adc.read_adc(0, gain=16,data_rate=DATA_RATE[32])*125e-3/16, 4))

adc.close()

