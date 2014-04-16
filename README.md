RFM22B
======

A c++ library for the arduino to talk to the HopeRF RFM22 wireless radio (http://www.hoperf.com/rf_fsk/rfm22.htm) from HopeRF for two-way communication.

Has been used as part of a pocketqube mission, creating flight heritage.

The library does some basic SPI setup to talk to the device, and provides some low level commands to get and set the values of various registers on the device. Assuming the Arduino has one of its interrupt lines connected to the device, setting and clearing interrupts that can be set is also exposed from a few simple methods.

init()
//goes through and sets the various values required for basic wireless operation for the 434MHz models.

set_frequency()
//takes a floating variable (1.0 == 1MHz)