NUcr
====

## Motivation
The bare OpenCR 1.0 board did not have enough headers nor the right headers for the then existing cabling that was used for the CM740. For example, the OpenCR only has three JST B4B-EH-A headers for the RS485 Dynamixel bus.

## Description
This Arduino-like shield effectively emulated the headers found on the CM740 as well as a few other conveniences like a connection for a standalone RS485 serial converter and headers for 12 V fans.

It has:
* nine Molex Spox 22-03-5045 headers -- three of which connect to the OpenCR's three JST headers --,
* one Molex Picoblade 53047-0810 header and one 53047-0910 header for the button panel,
* four JST B2B-PH-K-S headers for the fans -- one of which connects to the OpenCR's 12 V header --,
* an Arduino footprint -- this only serves to connect 3.3 V and to provide structural connection --,
* and lastly one three-pin header for a standalone RS485 serial converter.
