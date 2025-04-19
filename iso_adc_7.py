
# This code reads adc data from Texas Instrument AMC130M03

import gpiod
from gpiod.line import Direction,Value,Bias  # "Value", "Direction", "Bias", "Drive", "Edge", "Clock"
import gpiod.line as line 
import time
import spidev

class GPIOD:
    def __init__(self,pi_chip=0):
        self.chip_str='/dev/gpiochip0' if pi_chip==0 else '/dev/gpiochip4'
        self.chip = gpiod.Chip(self.chip_str)  
        self.outputs=None
        self.inputs=None
    def close(self):
        self.chip.close()
    def config_outputs(self,ioList:list,statusList:list,biasIoList=[],biasType=[]):  # biasType=[Bias.PULL_DOWN ,Bias.AS_IS]
        config={}
        if len(biasIoList)!=0 and len(ioList)!=len(biasIoList):
            raise ValueError('pi5gpio.gpiod.config_outputs(). if par biasIoList[] and biasType[] not empty, they must be as long as ioList[] ' )
        if len(biasIoList)==0: 
            config={ioList[i]:gpiod.LineSettings(direction=Direction.OUTPUT,
                    output_value=Value.ACTIVE if statusList[i] else Value.INACTIVE,)
                                                    for i in range(len(ioList))
                                                    } # dict dynamic        
        else:
            config={ioList[i]:gpiod.LineSettings(direction=Direction.OUTPUT,
                    bias=biasType[i] if biasIoList[i] else Bias.AS_IS,
                    output_value=Value.ACTIVE if statusList[i] else Value.INACTIVE,)
                                                    for i in range(len(ioList))
                                                    } 
        self.outputs = gpiod.request_lines(self.chip_str,config)
        return
    def config_inputs(self,ioList:list,pullList:Bias): # list of Bias.PULL settings
        config={}
        config = {ioList[i]:gpiod.LineSettings(direction=Direction.INPUT,bias=pullList[i]) 
                                               for i in range(len(ioList))
                                               }        
        self.inputs=gpiod.request_lines(self.chip_str,config)
    def config_pull(self,input:int,pull:Bias):
        config = {input:gpiod.LineSettings(bias=pull)} 
    def set_outputs(self,ioList,onOffList):
        self.outputs.set_values({ioList[i]:Value.ACTIVE if onOffList[i] else Value.INACTIVE
                                        for i in range(len(ioList))
                                        })      
    def set_output(self,io,onOff):
        self.outputs.set_value(io,Value.ACTIVE if onOff else Value.INACTIVE)
    def get_line(self,gpio):
        #chip =gpiod.Chip("/dev/gpiochip4") 
        lineInfo=self.chip.get_line_info(gpio)
        lineInfo=self.chip.get_info()
        lineInfo=line.Value.INACTIVE
        lineInfo=line.Value

    def read_input(self, gpio_num):
        """Reads the value of the specified GPIO pin configured as an input."""
        if self.inputs is None:
            raise RuntimeError("Inputs have not been configured. Call 'config_inputs' first.")
        value = self.inputs.get_value(gpio_num)
        return Value.ACTIVE if value else Value.INACTIVE



gpio_d = GPIOD()
gpio_d.config_outputs([17],[0])   # Reset pin sync_reset = 22
gpio_d.config_inputs([26],[Bias.AS_IS])  # config_inputs([13],[Bias.PULL_DOWN])

# The imports
import spidev
import time
from array import array

class AMC130M03:
    def __init__(self, spi_bus=0, spi_device=1,spi_mode=1):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)  # Use SPI0
        self.spi.max_speed_hz = 100000
        self.spi.mode = spi_mode  # SPI mode 
        self.reset()

    def reset(self,rst_gpio=17):
        # Trigger a reset
        time.sleep(0.001)
        gpio_d.set_outputs([rst_gpio],[0])
        time.sleep(0.001)
        gpio_d.set_outputs([rst_gpio],[1])
        time.sleep(0.001)

    # Helper functions for binary formatting
    def bin_8(self,in_val):
        # Format the 8-bit binary
        return f"0b{(in_val >> 7) & 0x1}{(in_val >> 6) & 0x1}{(in_val >> 5) & 0x1}" \
            f"{(in_val >> 4) & 0x1}{(in_val >> 3) & 0x1}{(in_val >> 2) & 0x1}" \
            f"{(in_val >> 1) & 0x1}{(in_val >> 0) & 0x1}"

    def bin_16(self,in_val):
        # Format the 16-bit binary value, with a little space for ease of reading
        out_str = "0b"
        for ii in range(15, 7, -1):
            out_str += str((in_val >> ii) & 0x1)
        out_str += " "
        for ii in range(7, -1, -1):
            out_str += str((in_val >> ii) & 0x1)
        return out_str

    def read_AMC130M03(self ):
        #This function will read the ADC values.
        # Read register
        # At reset, default is 24-bit words, 16-bit value plus one byte of 0s. figured out 
        txdata = [0] * 15
        rxdata = self.spi.xfer2(txdata)
        # The RAW ADC data, return in a list
        raw_adc = [(rxdata[3] << 8) + rxdata[4], (rxdata[6] << 8) + rxdata[7], (rxdata[9] << 8) + rxdata[10]]
        # Return the ADC values
        return raw_adc

    def read_reg_AMC130M03(self, reg_addr):
        #Reads a register
        # reg_addr - expect an address from 0 to 63
        # Checks on the input address
        assert 0 <= reg_addr <= 63
        # Read register
        txdata = [0b10100000 | (reg_addr >> 1), (reg_addr & 1) << 7, 0] + [0] * 12
        rxdata = self.spi.xfer2(txdata)
        # Send blank data next
        txdata2 = [0] * 15
        rxdata2 = self.spi.xfer2(txdata2) 
        return (rxdata2[0] << 8) | rxdata2[1]

    def write_reg_AMC130M03(self, reg_addr, reg_value):
        #Write a register
        # reg_addr - expect an address from 0 to 63
        # reg_value - the new value, expect 16-bit
        # Checks on the input address
        assert 0 <= reg_addr <= 63
        assert 0 <= reg_value <= 65535
        # Write register
        txdata = [0b01100000 | (reg_addr >> 1), (reg_addr & 1) << 7, 0, (reg_value >> 8), (reg_value & 255)] + [0] * 10
        self.spi.xfer2(txdata)



# Usage 

iso7 = AMC130M03()
# Data ready pin
print("DRDY should be 1: " + str(gpio_d.read_input(26)))

# Read the ID - don't be surprised if it is different than what is reported in the datasheet
id_reg = iso7.read_reg_AMC130M03(0x0)
status_reg = iso7.read_reg_AMC130M03(0x1)
mode_reg = iso7.read_reg_AMC130M03( 0x2)
clock_reg = iso7.read_reg_AMC130M03(0x3)
gain_reg = iso7.read_reg_AMC130M03( 0x4)
cfg_reg = iso7.read_reg_AMC130M03(0x6)

# Turn on the bit
iso7.write_reg_AMC130M03(0x31, 1)
DCDC_ctrl_reg = iso7.read_reg_AMC130M03(0x31)
print("DCDC_CTRL_REG " + iso7.bin_16(DCDC_ctrl_reg))

time.sleep(0.001)

# Grab some samples
NUM_SAMPLES = 10
wave0 = array("I", [0] * NUM_SAMPLES)
wave1 = array("I", [0] * NUM_SAMPLES)

for i in range(NUM_SAMPLES):
    temp = iso7.read_AMC130M03()
    wave0[i] = temp[0]
    wave1[i] = temp[1]
    time.sleep(0.001)

# Print them out after scaling
for i in range(NUM_SAMPLES):
    temp0 = wave0[i]
    if temp0 > 32767:
        temp0 -= 65536
    temp0 = temp0 / 32768.0 * 1.2

    temp1 = wave1[i]
    if temp1 > 32767:
        temp1 -= 65536
    temp1 = temp1 / 32768.0 * 1.2

    print(f"{temp0}, {temp1}")

