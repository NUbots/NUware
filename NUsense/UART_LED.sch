EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ADM6713RAKS
LIBS:Header_Connectors
LIBS:MPU6000
LIBS:NUbots
LIBS:STM32F746ZGT6
LIBS:NUsense-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 8 9
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R R?
U 1 1 5A6E5498
P 2500 1800
F 0 "R?" V 2580 1800 50  0000 C CNN
F 1 "R" V 2500 1800 50  0000 C CNN
F 2 "" V 2430 1800 50  0001 C CNN
F 3 "" H 2500 1800 50  0001 C CNN
	1    2500 1800
	0    1    1    0   
$EndComp
$Comp
L SM712 U2?
U 1 1 5A6E54FE
P 3100 2000
F 0 "U2?" H 3450 2350 60  0000 C CNN
F 1 "SM712" H 3400 1950 60  0000 C CNN
F 2 "" H 3100 2000 60  0001 C CNN
F 3 "" H 3100 2000 60  0001 C CNN
	1    3100 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1800 2900 1800
Wire Wire Line
	2100 1800 2350 1800
Wire Wire Line
	2100 1450 4200 1450
Wire Wire Line
	2100 1550 4200 1550
Wire Wire Line
	3800 1550 3800 1800
Wire Wire Line
	2800 1450 2800 1800
Connection ~ 2800 1800
$Comp
L GND #PWR?
U 1 1 5A6E5595
P 3900 2050
F 0 "#PWR?" H 3900 1800 50  0001 C CNN
F 1 "GND" H 3900 1900 50  0000 C CNN
F 2 "" H 3900 2050 50  0001 C CNN
F 3 "" H 3900 2050 50  0001 C CNN
	1    3900 2050
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A6E55AD
P 4350 1550
F 0 "R?" V 4430 1550 50  0000 C CNN
F 1 "R" V 4350 1550 50  0000 C CNN
F 2 "" V 4280 1550 50  0001 C CNN
F 3 "" H 4350 1550 50  0001 C CNN
	1    4350 1550
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5A6E55F8
P 4350 1450
F 0 "R?" V 4430 1450 50  0000 C CNN
F 1 "R" V 4350 1450 50  0000 C CNN
F 2 "" V 4280 1450 50  0001 C CNN
F 3 "" H 4350 1450 50  0001 C CNN
	1    4350 1450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5A6E561E
P 4800 2250
F 0 "#PWR?" H 4800 2000 50  0001 C CNN
F 1 "GND" H 4800 2100 50  0000 C CNN
F 2 "" H 4800 2250 50  0001 C CNN
F 3 "" H 4800 2250 50  0001 C CNN
	1    4800 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A6E56E1
P 5200 2250
F 0 "#PWR?" H 5200 2000 50  0001 C CNN
F 1 "GND" H 5200 2100 50  0000 C CNN
F 2 "" H 5200 2250 50  0001 C CNN
F 3 "" H 5200 2250 50  0001 C CNN
	1    5200 2250
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A6E56F6
P 4800 1900
F 0 "C?" H 4825 2000 50  0000 L CNN
F 1 "C" H 4825 1800 50  0000 L CNN
F 2 "" H 4838 1750 50  0001 C CNN
F 3 "" H 4800 1900 50  0001 C CNN
	1    4800 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1900 3900 1900
Wire Wire Line
	3900 1900 3900 2050
Connection ~ 3800 1550
Wire Wire Line
	4500 1550 5350 1550
Wire Wire Line
	5200 2250 5200 1250
Wire Wire Line
	5200 1250 5350 1250
Wire Wire Line
	4500 1450 5350 1450
Connection ~ 2800 1450
Wire Wire Line
	4800 1250 4800 1750
Wire Wire Line
	4800 1350 5350 1350
Connection ~ 4800 1350
Wire Wire Line
	4800 2050 4800 2250
$Comp
L +3.3V #PWR?
U 1 1 5A6FD7BD
P 4800 1250
F 0 "#PWR?" H 4800 1100 50  0001 C CNN
F 1 "+3.3V" H 4800 1390 50  0000 C CNN
F 2 "" H 4800 1250 50  0001 C CNN
F 3 "" H 4800 1250 50  0001 C CNN
	1    4800 1250
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5A6FD880
P 2100 1800
F 0 "#PWR?" H 2100 1650 50  0001 C CNN
F 1 "+3.3V" H 2100 1940 50  0000 C CNN
F 2 "" H 2100 1800 50  0001 C CNN
F 3 "" H 2100 1800 50  0001 C CNN
	1    2100 1800
	0    -1   -1   0   
$EndComp
Text HLabel 2100 1450 0    60   Input ~ 0
USER_UART_RX
Text HLabel 2100 1550 0    60   Output ~ 0
USER_UART_TX
$EndSCHEMATC
