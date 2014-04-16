replace: radio.write(0x10,0b10011000);
with radio.write(ADC_SENSOR_AMP_OFFSET, 0x98)
then radio.adc_sensor_amp_offset(0x98)
etc.

Provide methods that can be called for burst reads/writes for use in the FIFO buffers
Simple messaging protocols that make use of these low level commands.

automatically select correct code for different chip versions