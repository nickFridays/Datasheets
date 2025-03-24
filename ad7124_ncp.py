# part1 
import spidev
import time

# Constants
DEFAULT_TIMEOUT_MS = 200

AD7124_MAX_CHANNELS = 16

AD7124_INVALID_VAL = -1
AD7124_COMM_ERR = -2
AD7124_TIMEOUT = -3

AD7124_RW = 1  # Read and Write
AD7124_R = 2   # Read only
AD7124_W = 3   # Write only

# AD7124 Register Map
AD7124_COMM_REG = 0x00
AD7124_STATUS_REG = 0x00
AD7124_ADC_CTRL_REG = 0x01
AD7124_DATA_REG = 0x02
AD7124_IO_CTRL1_REG = 0x03
AD7124_IO_CTRL2_REG = 0x04
AD7124_ID_REG = 0x05
AD7124_ERR_REG = 0x06
AD7124_ERREN_REG = 0x07
AD7124_CH0_MAP_REG = 0x09
AD7124_CH1_MAP_REG = 0x0A
AD7124_CH2_MAP_REG = 0x0B
AD7124_CH3_MAP_REG = 0x0C
AD7124_CH4_MAP_REG = 0x0D
AD7124_CH5_MAP_REG = 0x0E
AD7124_CH6_MAP_REG = 0x0F
AD7124_CH7_MAP_REG = 0x10
AD7124_CH8_MAP_REG = 0x11
AD7124_CH9_MAP_REG = 0x12
AD7124_CH10_MAP_REG = 0x13
AD7124_CH11_MAP_REG = 0x14
AD7124_CH12_MAP_REG = 0x15
AD7124_CH13_MAP_REG = 0x16
AD7124_CH14_MAP_REG = 0x17
AD7124_CH15_MAP_REG = 0x18
AD7124_CFG0_REG = 0x19
AD7124_CFG1_REG = 0x1A
AD7124_CFG2_REG = 0x1B
AD7124_CFG3_REG = 0x1C
AD7124_CFG4_REG = 0x1D
AD7124_CFG5_REG = 0x1E
AD7124_CFG6_REG = 0x1F
AD7124_CFG7_REG = 0x20
AD7124_FILT0_REG = 0x21
AD7124_FILT1_REG = 0x22
AD7124_FILT2_REG = 0x23
AD7124_FILT3_REG = 0x24
AD7124_FILT4_REG = 0x25
AD7124_FILT5_REG = 0x26
AD7124_FILT6_REG = 0x27
AD7124_FILT7_REG = 0x28
AD7124_OFFS0_REG = 0x29
AD7124_OFFS1_REG = 0x2A
AD7124_OFFS2_REG = 0x2B
AD7124_OFFS3_REG = 0x2C
AD7124_OFFS4_REG = 0x2D
AD7124_OFFS5_REG = 0x2E
AD7124_OFFS6_REG = 0x2F
AD7124_OFFS7_REG = 0x30
AD7124_GAIN0_REG = 0x31
AD7124_GAIN1_REG = 0x32
AD7124_GAIN2_REG = 0x33
AD7124_GAIN3_REG = 0x34
AD7124_GAIN4_REG = 0x35
AD7124_GAIN5_REG = 0x36
AD7124_GAIN6_REG = 0x37
AD7124_GAIN7_REG = 0x38

# Communication Register bits 
AD7124_COMM_REG_WEN = 0 << 7
AD7124_COMM_REG_WR = 0 << 6
AD7124_COMM_REG_RD = 1 << 6
AD7124_COMM_REG_RA = lambda x: (x & 0x3F)

# Status Register bits 
AD7124_STATUS_REG_RDY = 1 << 7
AD7124_STATUS_REG_ERROR_FLAG = 1 << 6
AD7124_STATUS_REG_POR_FLAG = 1 << 4
AD7124_STATUS_REG_CH_ACTIVE = lambda x: (x & 0xF)

# ADC_Control Register bits
AD7124_ADC_CTRL_REG_DOUT_RDY_DEL = 1 << 12
AD7124_ADC_CTRL_REG_CONT_READ = 1 << 11
AD7124_ADC_CTRL_REG_DATA_STATUS = 1 << 10
AD7124_ADC_CTRL_REG_CS_EN = 1 << 9
AD7124_ADC_CTRL_REG_REF_EN = 1 << 8
AD7124_ADC_CTRL_REG_POWER_MODE = lambda x: ((x & 0x3) << 6)
AD7124_ADC_CTRL_REG_MODE = lambda x: ((x & 0xF) << 2)
AD7124_ADC_CTRL_REG_CLK_SEL = lambda x: ((x & 0x3) << 0)

# IO_Control_1 Register bits
AD7124_IO_CTRL1_REG_GPIO_DAT2 = 1 << 23
AD7124_IO_CTRL1_REG_GPIO_DAT1 = 1 << 22
AD7124_IO_CTRL1_REG_GPIO_CTRL2 = 1 << 19
AD7124_IO_CTRL1_REG_GPIO_CTRL1 = 1 << 18
AD7124_IO_CTRL1_REG_PDSW = 1 << 15
AD7124_IO_CTRL1_REG_IOUT1 = lambda x: ((x & 0x7) << 11)
AD7124_IO_CTRL1_REG_IOUT0 = lambda x: ((x & 0x7) << 8)
AD7124_IO_CTRL1_REG_IOUT_CH1 = lambda x: ((x & 0xF) << 4)
AD7124_IO_CTRL1_REG_IOUT_CH0 = lambda x: ((x & 0xF) << 0)

# IO_Control_2 Register bits
AD7124_IO_CTRL2_REG_GPIO_VBIAS7 = 1 << 15
AD7124_IO_CTRL2_REG_GPIO_VBIAS6 = 1 << 14
AD7124_IO_CTRL2_REG_GPIO_VBIAS5 = 1 << 11
AD7124_IO_CTRL2_REG_GPIO_VBIAS4 = 1 << 10
AD7124_IO_CTRL2_REG_GPIO_VBIAS3 = 1 << 5
AD7124_IO_CTRL2_REG_GPIO_VBIAS2 = 1 << 4
AD7124_IO_CTRL2_REG_GPIO_VBIAS1 = 1 << 1
AD7124_IO_CTRL2_REG_GPIO_VBIAS0 = 1 << 0

# ID Register bits
AD7124_ID_REG_DEVICE_ID = lambda x: ((x & 0xF) << 4)
AD7124_ID_REG_SILICON_REV = lambda x: ((x & 0xF) << 0)

# Error Register bits
AD7124_ERR_REG_LDO_CAP_ERR = 1 << 19
AD7124_ERR_REG_ADC_CAL_ERR = 1 << 18
AD7124_ERR_REG_ADC_CONV_ERR = 1 << 17
AD7124_ERR_REG_ADC_SAT_ERR = 1 << 16
AD7124_ERR_REG_AINP_OV_ERR = 1 << 15
AD7124_ERR_REG_AINP_UV_ERR = 1 << 14
AD7124_ERR_REG_AINM_OV_ERR = 1 << 13
AD7124_ERR_REG_AINM_UV_ERR = 1 << 12
AD7124_ERR_REG_REF_DET_ERR = 1 << 11
AD7124_ERR_REG_DLDO_PSM_ERR = 1 << 9
AD7124_ERR_REG_ALDO_PSM_ERR = 1 << 7
AD7124_ERR_REG_SPI_IGNORE_ERR = 1 << 6
AD7124_ERR_REG_SPI_SLCK_CNT_ERR = 1 << 5
AD7124_ERR_REG_SPI_READ_ERR = 1 << 4
AD7124_ERR_REG_SPI_WRITE_ERR = 1 << 3
AD7124_ERR_REG_SPI_CRC_ERR = 1 << 2
AD7124_ERR_REG_MM_CRC_ERR = 1 << 1
AD7124_ERR_REG_ROM_CRC_ERR = 1 << 0

# Error_En Register bits
AD7124_ERREN_REG_MCLK_CNT_EN = 1 << 22
AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN = 1 << 21
AD7124_ERREN_REG_LDO_CAP_CHK = lambda x: ((x & 0x3) << 19)
AD7124_ERREN_REG_ADC_CAL_ERR_EN = 1 << 18
AD7124_ERREN_REG_ADC_CONV_ERR_EN = 1 << 17
AD7124_ERREN_REG_ADC_SAT_ERR_EN = 1 << 16
AD7124_ERREN_REG_AINP_OV_ERR_EN = 1 << 15
AD7124_ERREN_REG_AINP_UV_ERR_EN = 1 << 14
AD7124_ERREN_REG_AINM_OV_ERR_EN = 1 << 13
AD7124_ERREN_REG_AINM_UV_ERR_EN = 1 << 12
AD7124_ERREN_REG_REF_DET_ERR_EN = 1 << 11
AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN = 1 << 10
AD7124_ERREN_REG_DLDO_PSM_ERR_ERR = 1 << 9
AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN = 1 << 8
AD7124_ERREN_REG_ALDO_PSM_ERR_EN = 1 << 7
AD7124_ERREN_REG_SPI_IGNORE_ERR_EN = 1 << 6
AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN = 1 << 5
AD7124_ERREN_REG_SPI_READ_ERR_EN = 1 << 4
AD7124_ERREN_REG_SPI_WRITE_ERR_EN = 1 << 3
AD7124_ERREN_REG_SPI_CRC_ERR_EN = 1 << 2
AD7124_ERREN_REG_MM_CRC_ERR_EN = 1 << 1
AD7124_ERREN_REG_ROM_CRC_ERR_EN = 1 << 0

# Channel Registers 0-15 bits
AD7124_CH_MAP_REG_CH_ENABLE = 1 << 15
AD7124_CH_MAP_REG_SETUP = lambda x: ((x & 0x7) << 12)
AD7124_CH_MAP_REG_AINP = lambda x: ((x & 0x1F) << 5)
AD7124_CH_MAP_REG_AINM = lambda x: ((x & 0x1F) << 0)

# Configuration Registers 0-7 bits
AD7124_CFG_REG_BIPOLAR = 1 << 11
AD7124_CFG_REG_BURNOUT = lambda x: ((x & 0x3) << 9)
AD7124_CFG_REG_REF_BUFP = 1 << 8
AD7124_CFG_REG_REF_BUFM = 1 << 7
AD7124_CFG_REG_AIN_BUFP = 1 << 6
AD7124_CFG_REG_AINN_BUFM = 1 << 5
AD7124_CFG_REG_REF_SEL = lambda x: ((x & 0x3) << 3)
AD7124_CFG_REG_PGA = lambda x: ((x & 0x7) << 0)

# Filter Register 0-7 bits
AD7124_FILT_REG_FILTER = lambda x: ((x & 0x7) << 21)
AD7124_FILT_REG_REJ60 = 1 << 20
AD7124_FILT_REG_POST_FILTER = lambda x: ((x & 0x7) << 17)
AD7124_FILT_REG_SINGLE_CYCLE = 1 << 16
AD7124_FILT_REG_FS = lambda x: ((x & 0x7FF) << 0)

# AD7124 Constants
AD7124_CRC8_POLYNOMIAL_REPRESENTATION = 0x07  # x8 + x2 + x + 1 
AD7124_DISABLE_CRC = 0
AD7124_USE_CRC = 1

# Operating Modes
AD7124_OpMode_Continuous = 0  # Continuous conversion mode (default).
AD7124_OpMode_SingleConv = 1  # Single conversion mode.
AD7124_OpMode_Standby = 2  # Standby mode.
AD7124_OpMode_PowerDown = 3  # Power-down mode.
AD7124_OpMode_Idle = 4  # Idle mode.
AD7124_OpMode_InternalOffsetCalibration = 5  # Internal zero-scale (offset) calibration.
AD7124_OpMode_InternalGainCalibration = 6  # Internal full-scale (gain) calibration.
AD7124_OpMode_SystemOffsetCalibration = 7  # System zero-scale (offset) calibration.
AD7124_OpMode_SystemGainCalibration = 8  # System full-scale (gain) calibration.

# Power Modes    
AD7124_LowPower = 0
AD7124_MidPower = 1
AD7124_FullPower = 2

# Clock Sources
AD7124_Clk_Internal = 0  # internal 614.4 kHz clock.
AD7124_Clk_InternalWithOutput = 1  # internal 614.4 kHz clock.
AD7124_Clk_External = 2  # external 614.4 kHz clock.
AD7124_Clk_ExternalDiv4 = 3  # external clock.

# Input Selection
AD7124_Input_AIN0 = 0
AD7124_Input_AIN1 = 1
AD7124_Input_AIN2 = 2
AD7124_Input_AIN3 = 3
AD7124_Input_AIN4 = 4
AD7124_Input_AIN5 = 5
AD7124_Input_AIN6 = 6
AD7124_Input_AIN7 = 7
AD7124_Input_AIN8 = 8
AD7124_Input_AIN9 = 9
AD7124_Input_AIN10 = 10
AD7124_Input_AIN11 = 11
AD7124_Input_AIN12 = 12
AD7124_Input_AIN13 = 13
AD7124_Input_AIN14 = 14
AD7124_Input_AIN15 = 15
AD7124_Input_TEMP = 16  # Temperature sensor (internal)
AD7124_Input_AVSS = 17  # Connect to AVss
AD7124_Input_REF = 18  # Connect to Internal reference
AD7124_Input_DGND = 19  # Connect to DGND.
AD7124_Input_AVDD6P = 20  # (AVdd − AVss)/6+.
AD7124_Input_AVDD6M = 21  # (AVdd − AVss)/6−.
AD7124_Input_IOVDD6P = 22  # (IOVdd − DGND)/6+.
AD7124_Input_IOVDD6M = 23  # (IOVdd − DGND)/6−.
AD7124_Input_ALDO6P = 24  # (ALDO − AVss)/6+.
AD7124_Input_ALDO6M = 25  # (ALDO − AVss)/6−.
AD7124_Input_DLDO6P = 26  # (DLDO − DGND)/6+.
AD7124_Input_DLDO6M = 27  # (DLDO − DGND)/6−.
AD7124_Input_V20mVP = 28  # V_20MV_P.
AD7124_Input_V20mVM = 29  # V_20MV_M.

# Gain Selection
AD7124_Gain_1 = 0  # Gain 1, Input Range When VREF = 2.5 V: ±2.5 V
AD7124_Gain_2 = 1  # Gain 2, Input Range When VREF = 2.5 V: ±1.25 V
AD7124_Gain_4 = 2  # Gain 4, Input Range When VREF = 2.5 V: ± 625 mV
AD7124_Gain_8 = 3  # Gain 8, Input Range When VREF = 2.5 V: ±312.5 mV
AD7124_Gain_16 = 4  # Gain 16, Input Range When VREF = 2.5 V: ±156.25 mV
AD7124_Gain_32 = 5  # Gain 32, Input Range When VREF = 2.5 V: ±78.125 mV
AD7124_Gain_64 = 6  # Gain 64, Input Range When VREF = 2.5 V: ±39.06 mV
AD7124_Gain_128 = 7  # Gain 128, Input Range When VREF = 2.5 V: ±19.53 mV

# Reference Sources    
AD7124_Ref_ExtRef1 = 0x00
AD7124_Ref_ExtRef2 = 0x01
AD7124_Ref_Internal = 0x02
AD7124_Ref_Avdd = 0x03

# Filter Options
AD7124_Filter_SINC4 = 0x00  # SINC4 Filter - Default after reset.
AD7124_Filter_SINC3 = 0x02  # SINC3 Filter.
AD7124_Filter_FAST4 = 0x04  # Fast settling + Sinc4
AD7124_Filter_FAST3 = 0x05  # Fast settling + Sinc3
AD7124_Filter_POST = 0x07  # Post filter enable

# Post Filter Options
AD7124_PostFilter_NoPost = 0  # No Post Filter (Default value)
AD7124_PostFilter_dB47 = 2  # Rejection at 50 Hz and 60 Hz ± 1 Hz: 47 dB
AD7124_PostFilter_dB62 = 3  # Rejection at 50 Hz and 60 Hz ± 1 Hz: 62 dB
AD7124_PostFilter_dB86 = 5  # Rejection at 50 Hz and 60 Hz ± 1 Hz: 86 dB
AD7124_PostFilter_dB92 = 6  # Rejection at 50 Hz and 60 Hz ± 1 Hz: 92 dB

# Burnout Current Options
AD7124_Burnout_Off = 0  # burnout current source off (default).
AD7124_Burnout_500nA = 1  # burnout current source on, 0.5 μA.
AD7124_Burnout_2uA = 2  # burnout current source on, 2 μA.
AD7124_Burnout_4uA = 3  # burnout current source on, 4 μA.

# Excitation Currents Options - Not used yet.
AD7124_ExCurrent_Off = 0x00
AD7124_ExCurrent_50uA = 0x01
AD7124_ExCurrent_100uA = 0x02
AD7124_ExCurrent_250uA = 0x03
AD7124_ExCurrent_500uA = 0x04
AD7124_ExCurrent_750uA = 0x05
AD7124_ExCurrent_1mA = 0x06

