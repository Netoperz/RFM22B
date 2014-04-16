#include "RFM22.h"

uint8_t rfm22::read(uint8_t addr) const {
    //write ss low to start
    digitalWrite(_select_pin, LOW);
    
    // make sure the msb is 0 so we do a read, not a write
    addr &= 0x7F;
    SPI.transfer(addr);
    uint8_t val = SPI.transfer(0x00);
    
    //write ss high to end
    digitalWrite(_select_pin, HIGH);
    
    return val;
}


void rfm22::read(uint8_t start_addr, uint8_t buf[], uint8_t len) {
    //write ss low to start
    digitalWrite(_select_pin, LOW);

    // make sure the msb is 0 so we do a read, not a write
    start_addr &= 0x7F;
    SPI.transfer(start_addr);
    for (int i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }

    //write ss high to end
    digitalWrite(_select_pin, HIGH);
}

void rfm22::write(uint8_t start_addr, uint8_t data[], uint8_t len) {
    //write ss low to start
    digitalWrite(_select_pin, LOW);

    // make sure the msb is 1 so we do a write
    start_addr |= 0x80;
    SPI.transfer(start_addr);
    for (int i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }

    //write ss high to end
    digitalWrite(_select_pin, HIGH);
}

void rfm22::write(uint8_t addr, uint8_t data) const {
    //write ss low to start
    digitalWrite(_select_pin, LOW);
    
    // make sure the msb is 1 so we do a write
    addr |= 0x80;
    SPI.transfer(addr);
    SPI.transfer(data);
    
    //write ss high to end
    digitalWrite(_select_pin, HIGH);
}


uint8_t rfm22::type(void) {
    uint8_t type_register_val = (read(TYPE_REGISTER) & 0x1F);
    if(type_register_val == TYPE_TRANSMIT){
        _device_type = TYPE_TRANSMIT;
        return true;
    } else if(type_register_val == TYPE_RECIEVE){
        _device_type = TYPE_RECIEVE;
        return true;
    }
    _device_type = TYPE_UNKNOWN;
    return false;
}

uint8_t rfm22::version(void) {
    uint8_t version_register_val = (read(VERSION_REGISTER) & 0x1F);
    if(version_register_val == VERSION_X4){
        _device_version = VERSION_X4;
    } else if(version_register_val == VERSION_V2){
        _device_version = VERSION_V2;
    } else if(version_register_val == VERSION_A0){
        _device_version = VERSION_A0;
    } else {
        _device_version = UNKNOWN_VERSION;
        return false;
    }
    return true;
}

uint8_t rfm22::status(void) {
    return read(STATUS_REGISTER);
}

float rfm22::voltageLevel(void) {
    float battery_voltage;
    uint8_t raw_adc;

    raw_adc = read(BATTERY_VOLTAGE);
    battery_voltage = 1.7 + (0.05*raw_adc);
    return battery_voltage;
}

uint8_t rfm22::temperature(void) {
    write(RFM22B_ADC_CONFIG, 0x70);
    write(RFM22B_TEMP_SENSOR_CAL, 0x20);

    uint8_t raw_temperature_value = read(ADC_VALUE);

    return raw_temperature_value;
}

void rfm22::disableInterupts(void) {
    // disable all interrupts
    write(RFM22B_INTERUPT_ENABLE2, 0x00);
}

void rfm22::resetFIFO(void) {
    write(RFM22B_OPERATING_FUNC_CTRL2, 0x03);
    write(RFM22B_OPERATING_FUNC_CTRL2, 0x00);
}

void rfm22::setInterrupt(uint16_t interrupt, uint16_t is_on) {
    //high bits of interrupt are 0x03/0x05, low are 0x04/0x06
    uint8_t high = (interrupt >> 8) & 0xff;//  0x03/0x05
    uint8_t high_is_on = (is_on >> 8) & 0xff;
    uint8_t low = interrupt & 0xff;//      0x04/0x06
    uint8_t low_is_on = is_on & 0xff;
  
    // read out a set of regs
    //uint8_t regs = read(0x05);
    // mask out the values we will leave along
    //uint8_t ignore = regs & (~high);
    // zero out the values we don't care about (probably improperly set)
    //uint8_t important = high_is_on & high;
    // combine the two sets and write back out
    write(RFM22B_INTERUPT_ENABLE1, (read(INTERUPT_ENABLE1) & (~high)) | (high_is_on & high));
    
    write(RFM22B_INTERUPT_ENABLE2, (read(INTERUPT_ENABLE2) & (~low))  | (low_is_on  & low));
}

uint16_t rfm22::readAndClearInterrupts(void) {
    //high bits are 0x03, low are 0x04
    return (read(INTERUPT_STATUS1) << 8) | read(INTERUPT_STATUS2);
}

void rfm22::init() {
    _select_pin = SS_PIN;
    _interupt_pin = NRIQ_PIN;

    pinMode(_select_pin, OUTPUT);
    digitalWrite(_select_pin, HIGH);

    disable_interupts();


    SPI.begin();
    // RFM22 seems to speak spi mode 0
    SPI.setDataMode(SPI_MODE0);
    // Setting clock speed to 8mhz, as 10 is the max for the rfm22
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    // MSB first
    //SPI.setBitOrder(MSBFIRST);
    
    // move to ready mode
    write(RFM22B_OPERATING_FUNC_CTRL1, 0x01);
    
    set_xtal_cap(0x7F);
    
    // GPIO setup - not using any, like the example from sfi
    // Set GPIO clock output to 2MHz - this is probably not needed, since I am ignoring GPIO...
    write(RFM22B_UC_OUTPUT_CLOCK, 0x05);//default is 1MHz
    
    // GPIO 0-2 are ignored, leaving them at default
    write(RFM22B_GPIO_CONFIG0, 0x00);
    write(RFM22B_GPIO_CONFIG1, 0x00);
    write(RFM22B_GPIO_CONFIG2, 0x00);

    // no reading/writing to GPIO
    write(RFM22B_IO_PORT_CONFIG, 0x00);
    
    // ADC and temp are off
    write(RFM22B_ADC_CONFIG, 0x70);
    write(RFM22B_ADC_SENSOR_AMP_OFFSET, 0x00);
    write(RFM22B_TEMP_SENSOR_CAL, 0x00);
    write(RFM22B_TEMP_SENSOR_OFFSET, 0x00);
    
    // no whiting, no manchester encoding, data rate will be under 30kbps
    // subject to change - don't I want these features turned on?
    write(RFM22B_MOD_MODE_CONTROL1, 0x20);
    
    // RX Modem settings (not, apparently, IF Filter?)
    // filset= 0b0100 or 0b1101
    // fuck it, going with 3e-club.ru's settings
    write(RFM22B_IF_FILTER_BANDWIDTH, 0x04);
    write(RFM22B_AFC_LOOP_GEARSHIFT, 0x40);//"battery voltage" my ass
    write(RFM22B_AFC_TIMING_CONTROL, 0x08);//apparently my device's default
    
    // Clock recovery - straight from 3e-club.ru with no understanding
    write(RFM22B_CLK_RCV_OVERSAMP_RATIO, 0x41);
    write(RFM22B_CLOCK_RECOVERY_OFFSET1, 0x60);
    write(RFM22B_CLOCK_RECOVERY_OFFSET2, 0x27);
    write(RFM22B_CLOCK_RECOVERY_OFFSET3, 0x52);

    // Clock recovery timing
    write(RFM22B_CLOCK_RECOV_TIME_GAIN1, 0x00);
    write(RFM22B_CLOCK_RECOV_TIME_GAIN0, 0x06);
    
    // Tx power to max
    set_tx_power(0x07);
    
    // Tx data rate (1, 0) - these are the same in both examples
    write(RFM22B_TX_DATA_RATE1, 0x27);
    write(RFM22B_TX_DATA_RATE0, 0x52);
    
    // "Data Access Control"
    // Enable CRC
    // Enable "Packet TX Handling" (wrap up data in packets for bigger chunks, but more reliable delivery)
    // Enable "Packet RX Handling"
    write(RFM22B_DATA_ACCESS_CONTROL, 0x8C);
    
    // "Header Control" - appears to be a sort of 'Who did i mean this message for'
    // we are opting for broadcast
    write(RFM22B_HEADER_CONTROL1, 0xFF);
    
    // "Header 3, 2, 1, 0 used for head length, fixed packet length, synchronize word length 3, 2,"
    // Fixed packet length is off, meaning packet length is part of the data stream
    write(RFM22B_HEADER_CONTROL2, 0x42);
    
    // "64 nibble = 32 byte preamble" - write this many sets of 1010 before starting real data. NOTE THE LACK OF '0x'
    write(RFM22B_PREAMBLE_LENGTH, 64);
    // "0x35 need to detect 20bit preamble" - not sure why, but this needs to match the preceeding register
    write(RFM22B_0x35, 0x20);
    
    // synchronize word - apparently we only set this once?
    write(RFM22B_SYNC_WORD3, 0x2D);
    write(RFM22B_SYNC_WORD2, 0xD4);
    write(RFM22B_SYNC_WORD1, 0x00);
    write(RFM22B_SYNC_WORD0, 0x00);
    
    // 4 bytes in header to send (note that these appear to go out backward?)
    write(RFM22B_TRANSMIT_HEADER3, 's');
    write(RFM22B_TRANSMIT_HEADER2, 'o');
    write(RFM22B_TRANSMIT_HEADER1, 'n');
    write(RFM22B_TRANSMIT_HEADER0, 'g');
    
    // Packets will have 1 bytes of real data
    write(RFM22B_TRANSMIT_PACKET_LENGTH, 1);
    
    // 4 bytes in header to recieve and check
    write(RFM22B_CHECK_HEADER3, 's');
    write(RFM22B_CHECK_HEADER2, 'o');
    write(RFM22B_CHECK_HEADER1, 'n');
    write(RFM22B_CHECK_HEADER0, 'g');
    
    // Check all bits of all 4 bytes of the check header
    write(RFM22B_HEADER_ENABLE3, 0xFF);
    write(RFM22B_HEADER_ENABLE2, 0xFF);
    write(RFM22B_HEADER_ENABLE1, 0xFF);
    write(RFM22B_HEADER_ENABLE0, 0xFF);
    
    //No channel hopping enabled
    write(RFM22B_FREQ_HOP_CHANNEL_SEL, 0x00);
    write(RFM22B_FREQ_CHANNEL_STEP_SIZE, 0x00);
    
    // FSK, fd[8]=0, no invert for TX/RX data, FIFO mode, no clock
    write(RFM22B_MOD_MODE_CONTROL2, 0x22);
    
    // "Frequency deviation setting to 45K=72*625"
    write(RFM22B_FREQ_DEVIATION, 0x48);
    
    // "No Frequency Offet" - channels?
    write(RFM22B_FREQUENCY_OFFSET1, 0x00);
    write(RFM22B_FREQUENCY_OFFSET2, 0x00);
    
    
    set_frequency();
    resetFIFO();
}

void set_frequency(float frequency) {
    // "frequency set to 434MHz" board default
    float limited_freq = fmax(24, fmin(96, (floor(frequency)/10)));
    if(limited_freq < 48) {
        write(RFM22B_FREQUENCY_BAND_SELECT, (limited_freq + 40)); //40 = - 24 + 64
    } else {
        write(RFM22B_FREQUENCY_BAND_SELECT, (limited_freq + 72)); //72 = - 24 + 32 + 64
    }
  
    //these still need to be calculated
    write(RFM22B_NOMINAL_CARRIER_FREQ1, 0x64);
    write(RFM22B_NOMINAL_CARRIER_FREQ2, 0x00);

}

void rfm22::set_tx_power(uint8_t value) {

    write(RFM22B_TX_POWER, value);// 0x07 //or is it 0x03?
}

void rfm22::set_xtal_cap(uint8_t value) {
    // set crystal oscillator cap to 12.5pf (but I don't know what this means)
    write(RFM22B_CRYSTAL_CAPACITANCE, 0x7F);
}

void rfm22::shutdown(void) {

}

void rfm22::sleep(void) {

}

void rfm22::standby(void) {

}

void rfm22::shutdown(void) {

}