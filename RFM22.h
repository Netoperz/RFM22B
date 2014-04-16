#ifndef RFM22_h
#define RFM22_h
#include <Arduino.h>
#include <SPI.h>
#include "stdint.h"

#define RFM22B_RFM_INT_FFERR           (1 << 16)
#define RFM22B_RFM_INT_TXFFAFULL       (1 << 14)
#define RFM22B_RFM_INT_XTFFAEM         (1 << 13)
#define RFM22B_RFM_INT_RXFFAFULL       (1 << 12)
#define RFM22B_RFM_INT_EXT             (1 << 11)
#define RFM22B_RFM_INT_PKSENT          (1 << 10)
#define RFM22B_RFM_INT_PKVALID         (1 << 9)
#define RFM22B_RFM_INT_CRERROR         (1 << 8)

#define RFM22B_RFM_INT_SWDET           (1 << 7)
#define RFM22B_RFM_INT_PREAVAL         (1 << 6)
#define RFM22B_RFM_INT_PREAINVAL       (1 << 5)
#define RFM22B_RFM_INT_RSSI            (1 << 4)
#define RFM22B_RFM_INT_WUT             (1 << 3)
#define RFM22B_RFM_INT_LBD             (1 << 2)
#define RFM22B_RFM_INT_CHIPRDY         (1 << 1)
#define RFM22B_RFM_INT_POR             (1)

//register definitions
#define RFM22B_TYPE_REGISTER           0x00
#define RFM22B_VERSION_REGISTER        0x01
#define RFM22B_STATUS_REGISTER         0x02
#define RFM22B_INTERUPT_STATUS1        0x03
#define RFM22B_INTERUPT_STATUS2        0x04
#define RFM22B_INTERUPT_ENABLE1        0x05
#define RFM22B_INTERUPT_ENABLE2        0x06
#define RFM22B_OPERATING_FUNC_CTRL1    0x07
#define RFM22B_OPERATING_FUNC_CTRL2    0x08
#define RFM22B_CRYSTAL_CAPACITANCE     0x09
#define RFM22B_UC_OUTPUT_CLOCK         0x0A
#define RFM22B_GPIO_CONFIG0            0x0B
#define RFM22B_GPIO_CONFIG1            0x0C
#define RFM22B_GPIO_CONFIG2            0x0D
#define RFM22B_IO_PORT_CONFIG          0x0E
#define RFM22B_ADC_CONFIG              0x0F

#define RFM22B_ADC_SENSOR_AMP_OFFSET   0x10
#define RFM22B_ADC_VALUE               0x11
#define RFM22B_TEMP_SENSOR_CAL         0x12
#define RFM22B_TEMP_SENSOR_OFFSET      0x13
#define RFM22B_WAKE_UP_TIME_PERIOD1    0x14
#define RFM22B_WAKE_UP_TIME_PERIOD2    0x15
#define RFM22B_WAKE_UP_TIME_PERIOD3    0x16
#define RFM22B_WAKE_UP_TIME_VALUE1     0x17
#define RFM22B_WAKE_UP_TIME_VALUE2     0x18
//#define RFM22B_ 0x19
//#define RFM22B_ 0x1A
#define RFM22B_BATTERY_VOLTAGE         0x1B
#define RFM22B_IF_FILTER_BANDWIDTH     0x1C
#define RFM22B_AFC_LOOP_GEARSHIFT      0x1D
#define RFM22B_AFC_TIMING_CONTROL      0x1E
//#define RFM22B_ 0x1F

#define RFM22B_CLK_RCV_OVERSAMP_RATIO  0x20
#define RFM22B_CLOCK_RECOVERY_OFFSET1  0x21
#define RFM22B_CLOCK_RECOVERY_OFFSET2  0x22
#define RFM22B_CLOCK_RECOVERY_OFFSET3  0x23
#define RFM22B_CLOCK_RECOV_TIME_GAIN1  0x24
#define RFM22B_CLOCK_RECOV_TIME_GAIN0  0x25
//#define RFM22B_ 0x26
//#define RFM22B_ 0x27
//#define RFM22B_ 0x28
//#define RFM22B_ 0x29

#define RFM22B_DATA_ACCESS_CONTROL          0x30
#define RFM22B_EZMAC_STATUS                 0x31
#define RFM22B_HEADER_CONTROL1              0x32
#define RFM22B_HEADER_CONTROL2              0x33
#define RFM22B_PREAMBLE_LENGTH              0x34
#define RFM22B_PREAMBLE_DETECTION_CONTROL   0x35
#define RFM22B_SYNC_WORD3                   0x36
#define RFM22B_SYNC_WORD2                   0x37
#define RFM22B_SYNC_WORD1                   0x38
#define RFM22B_SYNC_WORD0                   0x39
#define RFM22B_TRANSMIT_HEADER3             0x3A
#define RFM22B_TRANSMIT_HEADER2             0x3B
#define RFM22B_TRANSMIT_HEADER1             0x3C
#define RFM22B_TRANSMIT_HEADER0             0x3D
#define RFM22B_TRANSMIT_PACKET_LENGTH       0x3E
#define RFM22B_CHECK_HEADER3                0x3F

#define RFM22B_CHECK_HEADER2                0x40
#define RFM22B_CHECK_HEADER1                0x41
#define RFM22B_CHECK_HEADER0                0x42
#define RFM22B_HEADER_ENABLE3               0x43
#define RFM22B_HEADER_ENABLE2               0x44
#define RFM22B_HEADER_ENABLE1               0x45
#define RFM22B_HEADER_ENABLE0               0x46
#define RFM22B_RECIEVE_HEADER3              0x47
#define RFM22B_RECIEVE_HEADER2              0x48
#define RFM22B_RECIEVE_HEADER1              0x49
#define RFM22B_RFM22B_RECIEVE_HEADER0       0x4A
#define RFM22B_RECIEVED_PACKET_LENGTH       0x4B

#define RFM22B_ANALOG_TEST_BUS         0x50
#define RFM22B_DIGITAL_TEST_BUS        0x51
#define RFM22B_TX_RAMP_CONTROL         0x52
#define RFM22B_PLL_TUNE_TIME           0x53

#define RFM22B_CALIBRATION_CONTROL     0x55
#define RFM22B_MODEM_TEST              0x56
#define RFM22B_CHARGEPUMP_TEST         0x57
#define RFM22B_CHARGEPUMP_CURRENT_TRIM 0x58
#define RFM22B_DIVIDER_CURRENT_TRIMM   0x59
#define RFM22B_VCO_CURRENT_TRIMM       0x5A
#define RFM22B_VCO_CALIBRATION_OVER    0x5B
#define RFM22B_SYNTHESIZER_TEST        0x5C
#define RFM22B_BLOCK_ENABLE_OVERRIDE1  0x5D
#define RFM22B_BLOCK_ENABLE_OVERRIDE2  0x5E
#define RFM22B_BLOCK_ENABLE_OVERRIDE3  0x5F

#define RFM22B_DELTASIGMA_ADC_TUNING1  0x67
#define RFM22B_DELTASIGMA_ADC_TUNING2  0x68
#define RFM22B_AGC_OVERRIDE1           0x69
#define RFM22B_AGC_OVERRIDE2           0x6A
#define RFM22B_GFSK_FIR_FILTER_ADDR    0x6B
#define RFM22B_GFSK_FIR_FILTER_VALUE   0x6C
#define RFM22B_TX_POWER                0x6D
#define RFM22B_TX_DATA_RATE1           0x6E
#define RFM22B_TX_DATA_RATE0           0x6F

#define RFM22B_MOD_MODE_CONTROL1       0x70
#define RFM22B_MOD_MODE_CONTROL2       0x71
#define RFM22B_FREQ_DEVIATION          0x72

#define RFM22B_FREQUENCY_OFFSET1       0x73
#define RFM22B_FREQUENCY_OFFSET2       0x74
#define RFM22B_FREQUENCY_BAND_SELECT   0x75
#define RFM22B_NOMINAL_CARRIER_FREQ1   0x76
#define RFM22B_NOMINAL_CARRIER_FREQ2   0x77

#define RFM22B_FREQ_HOP_CHANNEL_SEL    0x79
#define RFM22B_FREQ_CHANNEL_STEP_SIZE  0x7A

#define RFM22B_TX_FIFO_CONTROL1        0x7C
#define RFM22B_TX_FIFO_CONTROL2        0x7D
#define RFM22B_RX_FIFO_CONTROL         0x7E
#define RFM22B_FIFO_ACCESS             0x7F

//device type values
#define TYPE_TRANSMIT           0x08
#define TYPE_RECIEVE            0x07
#define TYPE_UNKNOWN            0x00

//device version values
#define UNKNOWN_VERSION         0x00
#define VERSION_X4              0x01
#define VERSION_V2              0x02
#define VERSION_A0              0x03

class rfm22 {
    public:
        RFM22B();
        RFM22B(uint8_t address);
        void init();

        uint16_t transmitData(uint8_t data);
        uint8_t type(void);
        uint8_t version(void);
        uint8_t status(void);
        float voltageLevel(void);
        void disableInterupts(void);
        void temperature(void);

        void setInterrupt(uint16_t interrupt, uint16_t is_on);
        uint16_t readAndClearInterrupts(void);
        void resetFIFO(void);
    private:
    
        uint8_t read(uint8_t addr) const;
        void read(uint8_t start_addr, uint8_t buf[], uint8_t len);

        void write(uint8_t addr, uint8_t data) const;
        void write(uint8_t start_addr, uint8_t data[], uint8_t len);

        uint8_t _select_pin;
        uint8_t _interupt_pin;
        uint8_t _device_type;
        uint8_t _device_version;
        int8_t _power_consumption;
};

#endif