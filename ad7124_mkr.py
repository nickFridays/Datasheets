# Mikroe ISO ADC 6
import spidev
import time

# Constants for the ISO ADC 6 Click Driver
REG_COMM_STATUS = 0x00
REG_CTRL = 0x01
REG_DATA = 0x02
REG_IO_CTRL_1 = 0x03
REG_IO_CTRL_2 = 0x04
REG_ID = 0x05
CMD_SPI_READ = 0x40
NEW_DATA_BIT_MASK = 0x80
NEW_DATA_NOT_READY = 0x01
DEVICE_ID = 0x01
CTRL_DATA_STATUS_EN = 0x00000400
CTRL_DOUT_PIN_EN = 0x00000200
CTRL_INTRNL_REF_EN = 0x00000100
CTRL_FULL_PWR_MODE = 0x00000080
CTRL_CONT_CONV_MODE = 0x00000800  # Continuous conversion mode
CFG_BIP_OP_EN = 0x0800
CFG_BUFF_ON_AINP_EN = 0x0040
CFG_BUFF_ON_AINM_EN = 0x0020
CH_ENABLE = 0x8000
CH_NEG_AN_IN_AIN1 = 0x0001
VTG_REF_2_65_V = 2.65
CALIB_DEFAULT = 1.245  # Updated calibration factor
GAIN_COEFF = 0x00400000
# Channels A0 - A15
CH_ = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
# Gain settings
GAIN_1   = 0x00
GAIN_2   = 0x01
GAIN_4   = 0x02
GAIN_8   = 0x03
GAIN_16  = 0x04
GAIN_32  = 0x05
GAIN_64  = 0x06
GAIN_128 = 0x07

class ISOADC6:
    def __init__(self, spi_bus=0, spi_device=1):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 100000
        self.spi.mode = 0b00
        self.vref = VTG_REF_2_65_V
        self.calib = CALIB_DEFAULT

    def spi_write(self, reg, data):
        self.spi.xfer2([reg] + data)
        time.sleep(0.00001)

    def spi_read(self, reg, length):
        return self.spi.xfer2([reg | CMD_SPI_READ] + [0x00] * length)[1:]

    def reset(self):
        # Send 64 consecutive 1s to reset the ADC
        self.spi.xfer2([0xFF] * 8)
        time.sleep(0.1)  # Wait for the reset to complete

    def check_data_ready(self):
        status = self.spi_read(REG_COMM_STATUS, 1)[0]
        #print(f"Status Register: {status:08b}")
        return (status & NEW_DATA_BIT_MASK) == 0

    def get_device_id(self):
        device_id = self.spi_read(REG_ID, 1)[0]
        print(f"Device ID Raw: {device_id:08b}")
        return (device_id >> 4), (device_id & 0x0F)

    def config_setup(self, configs):
        self.reset()  # Reset the ADC before configuration
        device_id, sil_rev = self.get_device_id()
        print(f"Device ID: {device_id}, Silicon Revision: {sil_rev}")
        if device_id == DEVICE_ID:
            for (channel, gain, config) in configs:
                self.set_ch_config(config)
                self.set_ch(channel, CH_ENABLE | CH_NEG_AN_IN_AIN1)
                self.set_adc_ctrl(CTRL_DATA_STATUS_EN | CTRL_DOUT_PIN_EN |
                                  CTRL_INTRNL_REF_EN | CTRL_FULL_PWR_MODE |
                                  CTRL_CONT_CONV_MODE)  # Continuous conversion mode
                self.set_gain(gain)  # Set gain
                print(f"Configuration setup completed for channel {channel} with gain: {gain} and config: {config:016b}")
            return True
        else:
            print("Unsupported device ID")
            return False

    def set_ch_config(self, config):
        self.spi_write(REG_IO_CTRL_1, [(config >> 8) & 0xFF, config & 0xFF])
        print(f"Channel Config set to: {config:016b}")

    def set_ch(self, sel_ch, config):
        if sel_ch <= 15:
            self.spi_write(REG_IO_CTRL_1 + sel_ch, [(config >> 8) & 0xFF, config & 0xFF])
            print(f"Channel {sel_ch} set with config: {config:016b}")

    def set_adc_ctrl(self, ctrl):
        self.spi_write(REG_CTRL, [(ctrl >> 8) & 0xFF, ctrl & 0xFF])
        print(f"ADC Control set to: {ctrl:016b}")

    def set_gain(self, gain):
        self.spi_write(REG_CTRL, [gain])
        print(f"Gain set to: {gain}")

    def get_adc_data(self):
        while not self.check_data_ready():
            print("Data not ready, waiting...")
            time.sleep(0.01)
        data = self.spi_read(REG_DATA, 3)
        adc_data = (data[0] << 16) | (data[1] << 8) | data[2]
        #print(f"ADC Data: {adc_data}")
        return adc_data

    def get_voltage(self):
        adc_data = self.get_adc_data()
        voltage = self.calib * self.vref * adc_data / GAIN_COEFF
        return voltage

    def get_voltage_mv(self):
        return  self.get_voltage() * 1000

    def get_average_voltage(self, num_samples=10):
        voltages = [self.get_voltage() for _ in range(num_samples)]
        avg_voltage = sum(voltages) / num_samples
        return avg_voltage

    def get_cal_vtg(self, avdd):
        # Calculate the new calibration factor for a provided AVDD
        self.calib = avdd / self.vref
        return self.calib

def main():
    isoadc6 = ISOADC6()
    # Configure Channel 0 for 500 mV and Channel 1 for 50 mV
    configs=[(CH_[0],GAIN_2,  CFG_BIP_OP_EN | CFG_BUFF_ON_AINP_EN | CFG_BUFF_ON_AINM_EN),
             (CH_[2],GAIN_128,CFG_BIP_OP_EN | CFG_BUFF_ON_AINP_EN | CFG_BUFF_ON_AINM_EN)]
    configs2=[(CH_[2],GAIN_2,CFG_BIP_OP_EN | CFG_BUFF_ON_AINP_EN | CFG_BUFF_ON_AINM_EN)]
    if not isoadc6.config_setup(configs):
        print("Configuration setup failed.")
        return
    # Calibration calculation for a specific AVDD
    isoadc6.get_cal_vtg(3.3)  # Update the calibration factor based on AVDD of 3.3V

    while True:
        # Measure voltage on Channel 0
        voltage_mv_ch0 = isoadc6.get_voltage_mv()
        print(f"*** on CH_0:   {voltage_mv_ch0:.3f} [mV]")

        # Measure voltage on Channel 1
        avg_voltage_ch1 = isoadc6.get_average_voltage()
        print(f"^^^ on CH 1:   {avg_voltage_ch1:.3f} [V]")

        time.sleep(1)

if __name__ == "__main__":
    main()
