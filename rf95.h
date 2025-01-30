#ifndef RH_RF95_h
#define RH_RF95_h

#include <stdint.h>

// The crystal oscillator frequency of the module
#define RF95_FXOSC 32000000.0
#define RF95_FSTEP  (RF95_FXOSC / 524288)
#define DATA_LEN 4

void rf95_setup();
void rf95_send(uint8_t * data, uint8_t len);
void rf95_receive(uint8_t* data);

// The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19


#ifdef SENDER
#define RF95_RX_BASE_ADDR 0x80
#elif RECIEVER
#define RF95_RX_BASE_ADDR 0x08
#else
#error NOT SENDER NOR RECIEVER
#endif

// Register names (LoRa Mode, from table 85)
#define RF95_00_FIFO                                0x00
#define RF95_01_OP_MODE                             0x01
#define RF95_02_RESERVED                            0x02
#define RF95_03_RESERVED                            0x03
#define RF95_04_RESERVED                            0x04
#define RF95_05_RESERVED                            0x05
#define RF95_06_FRF_MSB                             0x06
#define RF95_07_FRF_MID                             0x07
#define RF95_08_FRF_LSB                             0x08
#define RF95_09_PA_CONFIG                           0x09
#define RF95_0A_PA_RAMP                             0x0a
#define RF95_0B_OCP                                 0x0b
#define RF95_0C_LNA                                 0x0c
#define RF95_0D_FIFO_ADDR_PTR                       0x0d
#define RF95_0E_FIFO_TX_BASE_ADDR                   0x0e
#define RF95_0F_FIFO_RX_BASE_ADDR                   0x0f
#define RF95_10_FIFO_RX_CURRENT_ADDR                0x10
#define RF95_11_IRQ_FLAGS_MASK                      0x11
#define RF95_12_IRQ_FLAGS                           0x12
#define RF95_13_RX_NB_BYTES                         0x13
#define RF95_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define RF95_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define RF95_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define RF95_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define RF95_18_MODEM_STAT                          0x18
#define RF95_19_PKT_SNR_VALUE                       0x19
#define RF95_1A_PKT_RSSI_VALUE                      0x1a
#define RF95_1B_RSSI_VALUE                          0x1b
#define RF95_1C_HOP_CHANNEL                         0x1c
#define RF95_1D_MODEM_CONFIG1                       0x1d
#define RF95_1E_MODEM_CONFIG2                       0x1e
#define RF95_1F_SYMB_TIMEOUT_LSB                    0x1f
#define RF95_20_PREAMBLE_MSB                        0x20
#define RF95_21_PREAMBLE_LSB                        0x21
#define RF95_22_PAYLOAD_LENGTH                      0x22
#define RF95_23_MAX_PAYLOAD_LENGTH                  0x23
#define RF95_24_HOP_PERIOD                          0x24
#define RF95_25_FIFO_RX_BYTE_ADDR                   0x25
#define RF95_26_MODEM_CONFIG3                       0x26
#define RF95_27_PPM_CORRECTION                      0x27
#define RF95_28_FEI_MSB                             0x28
#define RF95_29_FEI_MID                             0x29
#define RF95_2A_FEI_LSB                             0x2a
#define RF95_2C_RSSI_WIDEBAND                       0x2c
#define RF95_31_DETECT_OPTIMIZE                     0x31
#define RF95_33_INVERT_IQ                           0x33
#define RF95_37_DETECTION_THRESHOLD                 0x37
#define RF95_39_SYNC_WORD                           0x39
#define RF95_40_DIO_MAPPING1                        0x40
#define RF95_41_DIO_MAPPING2                        0x41
#define RF95_42_VERSION                             0x42
#define RF95_4B_TCXO                                0x4b
#define RF95_4D_PA_DAC                              0x4d
#define RF95_5B_FORMER_TEMP                         0x5b
#define RF95_61_AGC_REF                             0x61
#define RF95_62_AGC_THRESH1                         0x62
#define RF95_63_AGC_THRESH2                         0x63
#define RF95_64_AGC_THRESH3                         0x64

// RH_RF95_REG_01_OP_MODE                             0x01
#define RF95_LONG_RANGE_MODE                       0x80
#define RF95_ACCESS_SHARED_REG                     0x40
#define RF95_LOW_FREQUENCY_MODE                    0x08
#define RF95_MODE                                  0x07
#define RF95_MODE_SLEEP                            0x00
#define RF95_MODE_STDBY                            0x01
#define RF95_MODE_FSTX                             0x02
#define RF95_MODE_TX                               0x03
#define RF95_MODE_FSRX                             0x04
#define RF95_MODE_RXCONTINUOUS                     0x05
#define RF95_MODE_RXSINGLE                         0x06
#define RF95_MODE_CAD                              0x07

// RH_RF95_REG_09_PA_CONFIG                           0x09
#define RF95_PA_SELECT                             0x80
#define RF95_MAX_POWER                             0x70
#define RF95_OUTPUT_POWER                          0x0f

// RH_RF95_REG_0A_PA_RAMP                             0x0a
#define RF95_LOW_PN_TX_PLL_OFF                     0x10
#define RF95_PA_RAMP                               0x0f
#define RF95_PA_RAMP_3_4MS                         0x00
#define RF95_PA_RAMP_2MS                           0x01
#define RF95_PA_RAMP_1MS                           0x02
#define RF95_PA_RAMP_500US                         0x03
#define RF95_PA_RAMP_250US                         0x04
#define RF95_PA_RAMP_125US                         0x05
#define RF95_PA_RAMP_100US                         0x06
#define RF95_PA_RAMP_62US                          0x07
#define RF95_PA_RAMP_50US                          0x08
#define RF95_PA_RAMP_40US                          0x09
#define RF95_PA_RAMP_31US                          0x0a
#define RF95_PA_RAMP_25US                          0x0b
#define RF95_PA_RAMP_20US                          0x0c
#define RF95_PA_RAMP_15US                          0x0d
#define RF95_PA_RAMP_12US                          0x0e
#define RF95_PA_RAMP_10US                          0x0f

// RH_RF95_REG_0B_OCP                                 0x0b
#define RF95_OCP_ON                                0x20
#define RF95_OCP_TRIM                              0x1f

// RH_RF95_REG_0C_LNA                                 0x0c
#define RF95_LNA_GAIN                              0xe0
#define RF95_LNA_GAIN_G1                           0x20
#define RF95_LNA_GAIN_G2                           0x40
#define RF95_LNA_GAIN_G3                           0x60
#define RF95_LNA_GAIN_G4                           0x80
#define RF95_LNA_GAIN_G5                           0xa0
#define RF95_LNA_GAIN_G6                           0xc0
#define RF95_LNA_BOOST_LF                          0x18
#define RF95_LNA_BOOST_LF_DEFAULT                  0x00
#define RF95_LNA_BOOST_HF                          0x03
#define RF95_LNA_BOOST_HF_DEFAULT                  0x00
#define RF95_LNA_BOOST_HF_150PC                    0x03

// RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RF95_RX_TIMEOUT_MASK                       0x80
#define RF95_RX_DONE_MASK                          0x40
#define RF95_PAYLOAD_CRC_ERROR_MASK                0x20
#define RF95_VALID_HEADER_MASK                     0x10
#define RF95_TX_DONE_MASK                          0x08
#define RF95_CAD_DONE_MASK                         0x04
#define RF95_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RF95_CAD_DETECTED_MASK                     0x01

// RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RF95_RX_TIMEOUT                            0x80
#define RF95_RX_DONE                               0x40
#define RF95_PAYLOAD_CRC_ERROR                     0x20
#define RF95_VALID_HEADER                          0x10
#define RF95_TX_DONE                               0x08
#define RF95_CAD_DONE                              0x04
#define RF95_FHSS_CHANGE_CHANNEL                   0x02
#define RF95_CAD_DETECTED                          0x01

// RH_RF95_REG_18_MODEM_STAT                          0x18
#define RF95_RX_CODING_RATE                        0xe0
#define RF95_MODEM_STATUS_CLEAR                    0x10
#define RF95_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RF95_MODEM_STATUS_RX_ONGOING               0x04
#define RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RF95_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RF95_PLL_TIMEOUT                           0x80
#define RF95_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RF95_FHSS_PRESENT_CHANNEL                  0x3f

// RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RF95_BW                                    0xf0

#define RF95_BW_7_8KHZ                             0x00
#define RF95_BW_10_4KHZ                            0x10
#define RF95_BW_15_6KHZ                            0x20
#define RF95_BW_20_8KHZ                            0x30
#define RF95_BW_31_25KHZ                           0x40
#define RF95_BW_41_7KHZ                            0x50
#define RF95_BW_62_5KHZ                            0x60
#define RF95_BW_125KHZ                             0x70
#define RF95_BW_250KHZ                             0x80
#define RF95_BW_500KHZ                             0x90
#define RF95_CODING_RATE                           0x0e
#define RF95_CODING_RATE_4_5                       0x02
#define RF95_CODING_RATE_4_6                       0x04
#define RF95_CODING_RATE_4_7                       0x06
#define RF95_CODING_RATE_4_8                       0x08
#define RF95_IMPLICIT_HEADER_MODE_ON               0x01

// RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RF95_SPREADING_FACTOR                      0xf0
#define RF95_SPREADING_FACTOR_64CPS                0x60
#define RF95_SPREADING_FACTOR_128CPS               0x70
#define RF95_SPREADING_FACTOR_256CPS               0x80
#define RF95_SPREADING_FACTOR_512CPS               0x90
#define RF95_SPREADING_FACTOR_1024CPS              0xa0
#define RF95_SPREADING_FACTOR_2048CPS              0xb0
#define RF95_SPREADING_FACTOR_4096CPS              0xc0
#define RF95_TX_CONTINUOUS_MODE                    0x08

#define RF95_PAYLOAD_CRC_ON                        0x04
#define RF95_SYM_TIMEOUT_MSB                       0x03

// RH_RF95_REG_26_MODEM_CONFIG3
#define RF95_MOBILE_NODE                           0x08 // HopeRF term
#define RF95_LOW_DATA_RATE_OPTIMIZE                0x08 // Semtechs term
#define RF95_AGC_AUTO_ON                           0x04

// RH_RF95_REG_4B_TCXO                                0x4b
#define RF95_TCXO_TCXO_INPUT_ON                    0x10

// RH_RF95_REG_4D_PA_DAC                              0x4d
#define RF95_PA_DAC_DISABLE                        0x04
#define RF95_PA_DAC_ENABLE                         0x07

#define RF95_REG_VERSION                           0x12

#define RF95_00_FIFO                                0x00
#define RF95_01_OP_MODE                             0x01
#define RF95_02_BITRATE_MSB							0x02
#define RF95_03_BITRATE_LSB							0x03
#define RF95_04_FDEV_MSB							0x04
#define RF95_05_FDEV_LSB							0x05
#define RF95_06_FRF_MSB                             0x06
#define RF95_07_FRF_MID                             0x07
#define RF95_08_FRF_LSB                             0x08
#define RF95_09_PA_CONFIG                           0x09
#define RF95_0A_PA_RAMP                             0x0a
#define RF95_0B_OCP                                 0x0b
#define RF95_0C_LNA                                 0x0c
#define RF95_0D_RX_CONFIG							0x0d
#define RF95_0E_RSSI_CONFIG							0x0e
#define RF95_0F_RSSI_COLLISION						0x0f
#define RF95_10_RSSI_THRESH							0x10
#define RF95_11_RSSI_VALUE							0x11
#define RF95_12_RX_BW								0x12
#define RF95_13_AFC_BW								0x13
#define RF95_14_OOK_PEAK							0x14
#define RF95_15_OOK_FIX								0x15
#define RF95_16_OOK_AVG								0x16
#define RF95_17_RESERVED							0x17
#define RF95_18_RESERVED							0x18
#define RF95_19_RESERVED							0x19
#define RF95_1A_AFC_FEI								0x1a
#define RF95_1B_AFC_MSB								0x1b
#define RF95_1C_AFC_LSB								0x1c
#define RF95_1D_FEI_MSB								0x1d
#define RF95_1E_FEI_LSB								0x1e
#define RF95_1F_PREAMBLE_DETECT						0x1f
#define RF95_20_RX_TIMEOUT1							0x20
#define RF95_21_RX_TIMEOUT2							0x21
#define RF95_22_RX_TIMEOUT3							0x22
#define RF95_23_RX_DELAY							0x23
#define RF95_24_OSC									0x24
#define RF95_25_PREAMBLE_MSB						0x25
#define RF95_26_PREAMBLE_LSB						0x26
#define RF95_27_RESERVED							0x27
#define RF95_28_SYNC_VALUE1							0x28
#define RF95_29_SYNC_VALUE2							0x28
#define RF95_2A_SYNC_VALUE3							0x2a
#define RF95_2B_SYNC_VALUE4							0x2b
#define RF95_2C_SYNC_VALUE5							0x2c
#define RF95_2D_SYNC_VALUE6							0x2d
#define RF95_2E_SYNC_VALUE7							0x2e
#define RF95_2F_SYNC_VALUE8							0x2f
#define RF95_30_PACKET_CONFIG1						0x30
#define RF95_31_PACKET_CONFIG2						0x31
#define RF95_32_PAYLOAD_LENGTH						0x32
#define RF95_33_NODE_ADRS							0x33
#define RF95_34_BROADCAST_ADRS						0x34
#define RF95_35_FIFO_THRESH							0x35
#define RF95_36_SEQ_CONFIG1							0x36
#define RF95_37_SEQ_CONFIG2							0x37
#define RF95_38_TIMER_RESOL							0x38
#define RF95_39_TIMER1_COEF							0x39
#define RF95_3A_TIMER2_COEF							0x3a
#define RF95_3B_IMAGE_CAL							0x3b
#define RF95_3C_TEMP								0x3c
#define RF95_3D_LOW_BAT								0x3d
#define RF95_3E_IRQ_FLAGS1							0x3e
#define RF95_3F_IRQ_FLAGS2							0x3f
#define RF95_40_DIO_MAPPING1                        0x40
#define RF95_41_DIO_MAPPING2                        0x41
#define RF95_42_VERSION                             0x42
#define RF95_44_PLL_HOP								0x44
#define RF95_4B_TCXO								0x4b
#define RF95_4D_PA_DAC								0x4d
#define RF95_5B_FORMER_TEMP                         0x5b
#define RF95_61_AGC_REF                             0x61
#define RF95_62_AGC_THRESH1                         0x62
#define RF95_63_AGC_THRESH2                         0x63
#define RF95_64_AGC_THRESH3                         0x64
#define RF95_70_PLL									0x70

// RF95_01_OP_MODE                             0x01
#define RF95_MODULATION_FSK							0x00
#define RF95_MODULATION_OOK							0x20
#define RF95_LOW_FREQUENCY_MODE						0x08
#define RF95_MODE_SLEEP								0x00
#define RF95_MODE_STDBY								0x01
#define RF95_MODE_FSTX								0x02
#define RF95_MODE_TX								0x03
#define RF95_MODE_FSRX								0x04
#define RF95_MODE_RX								0x05

// RF95_09_PA_CONFIG                           0x09
#define RF95_PA_SELECT								0x80

// RF95_0A_PA_RAMP                             0x0a
#define RF95_FSK_MOD_SHAPE_NONE						0x00
#define RF95_FSK_MOD_SHAPE_GAUS1					0x20
#define RF95_FSK_MOD_SHAPE_GAUS0_5					0x40
#define RF95_FSK_MOD_SHAPE_GAUS0_3					0x60
#define RF95_OOK_MOD_SHAPE_NONE						0x00
#define RF95_OOK_MOD_SHAPE_FCUT						0x20
#define RF95_OOK_MOD_SHAPE_FCUT2					0x40
#define RF95_PA_RAMP_3_4MS							0x00
#define RF95_PA_RAMP_2MS							0x01
#define RF95_PA_RAMP_1MS							0x02
#define RF95_PA_RAMP_500US							0x03
#define RF95_PA_RAMP_250US							0x04
#define RF95_PA_RAMP_125US							0x05
#define RF95_PA_RAMP_100US							0x06
#define RF95_PA_RAMP_62US							0x07
#define RF95_PA_RAMP_50US							0x08
#define RF95_PA_RAMP_40US							0x09
#define RF95_PA_RAMP_31US							0x0a
#define RF95_PA_RAMP_25US							0x0b
#define RF95_PA_RAMP_20US							0x0c
#define RF95_PA_RAMP_15US							0x0d
#define RF95_PA_RAMP_12US							0x0e
#define RF95_PA_RAMP_10US							0x0f


// RF95_0D_RX_CONFIG							0x0d

#define RF95_RX_RESTART_COLLISION					0x80
#define RF95_RX_RESTART_WITHOUT_PLL					0x40
#define RF95_RX_RESTART_WITH_PLL					0x20
#define RF95_RX_AFC_AUTO_ON							0x10
#define RF95_RX_AGC_AUTO_ON							0x08

// RF95_0E_RSSI_CONFIG							0x0e

#define RF95_RSSI_SAMPLES_2							0x00
#define RF95_RSSI_SAMPLES_4							0x01
#define RF95_RSSI_SAMPLES_8							0x02
#define RF95_RSSI_SAMPLES_16						0x03
#define RF95_RSSI_SAMPLES_32						0x04
#define RF95_RSSI_SAMPLES_64						0x05
#define RF95_RSSI_SAMPLES_128						0x06
#define RF95_RSSI_SAMPLES_256						0x07

#endif // RH_RF95_H
