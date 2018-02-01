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
$Descr A3 16535 11693
encoding utf-8
Sheet 5 9
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
L MAX3443ECSA+ U21?
U 1 1 5A698B5D
P 4500 3700
F 0 "U21?" H 4750 4350 60  0000 C CNN
F 1 "MAX3443ECSA+" H 4750 3650 60  0000 C CNN
F 2 "" H 4500 3700 60  0001 C CNN
F 3 "" H 4500 3700 60  0001 C CNN
	1    4500 3700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A698BD3
P 3750 2800
F 0 "R?" V 3830 2800 50  0000 C CNN
F 1 "R" V 3750 2800 50  0000 C CNN
F 2 "" V 3680 2800 50  0001 C CNN
F 3 "" H 3750 2800 50  0001 C CNN
	1    3750 2800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A698C04
P 5500 3800
F 0 "#PWR?" H 5500 3550 50  0001 C CNN
F 1 "GND" H 5500 3650 50  0000 C CNN
F 2 "" H 5500 3800 50  0001 C CNN
F 3 "" H 5500 3800 50  0001 C CNN
	1    5500 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A698C36
P 5800 4050
F 0 "#PWR?" H 5800 3800 50  0001 C CNN
F 1 "GND" H 5800 3900 50  0000 C CNN
F 2 "" H 5800 4050 50  0001 C CNN
F 3 "" H 5800 4050 50  0001 C CNN
	1    5800 4050
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A698C47
P 5800 3750
F 0 "C?" H 5825 3850 50  0000 L CNN
F 1 "0.1uF" H 5825 3650 50  0000 L CNN
F 2 "" H 5838 3600 50  0001 C CNN
F 3 "" H 5800 3750 50  0001 C CNN
	1    5800 3750
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A698C8E
P 6700 3300
F 0 "R?" V 6780 3300 50  0000 C CNN
F 1 "10" V 6700 3300 50  0000 C CNN
F 2 "" V 6630 3300 50  0001 C CNN
F 3 "" H 6700 3300 50  0001 C CNN
	1    6700 3300
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5A698D2E
P 6700 3200
F 0 "R?" V 6780 3200 50  0000 C CNN
F 1 "10" V 6700 3200 50  0000 C CNN
F 2 "" V 6630 3200 50  0001 C CNN
F 3 "" H 6700 3200 50  0001 C CNN
	1    6700 3200
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5A6992FB
P 4200 5050
F 0 "#PWR?" H 4200 4800 50  0001 C CNN
F 1 "GND" H 4200 4900 50  0000 C CNN
F 2 "" H 4200 5050 50  0001 C CNN
F 3 "" H 4200 5050 50  0001 C CNN
	1    4200 5050
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A699329
P 3850 5250
F 0 "C?" H 3875 5350 50  0000 L CNN
F 1 "0.1uF" H 3875 5150 50  0000 L CNN
F 2 "" H 3888 5100 50  0001 C CNN
F 3 "" H 3850 5250 50  0001 C CNN
	1    3850 5250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A69938B
P 3850 5500
F 0 "#PWR?" H 3850 5250 50  0001 C CNN
F 1 "GND" H 3850 5350 50  0000 C CNN
F 2 "" H 3850 5500 50  0001 C CNN
F 3 "" H 3850 5500 50  0001 C CNN
	1    3850 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 3600 4150 3600
Wire Wire Line
	4150 3600 4300 3600
Wire Wire Line
	4300 3500 3550 3500
Wire Wire Line
	3550 3200 3750 3200
Wire Wire Line
	3750 3200 4300 3200
Wire Wire Line
	4300 3300 4150 3300
Wire Wire Line
	4150 3300 4150 3600
Connection ~ 4150 3600
Wire Wire Line
	3750 2950 3750 3200
Connection ~ 3750 3200
Wire Wire Line
	3750 2550 3750 2650
Wire Wire Line
	6550 3200 5200 3200
Wire Wire Line
	5200 3300 6550 3300
Wire Wire Line
	5200 3500 5800 3500
Wire Wire Line
	5800 3500 6100 3500
Wire Wire Line
	5800 3600 5800 3500
Connection ~ 5800 3500
Wire Wire Line
	5800 4050 5800 3900
Wire Wire Line
	5500 3800 5500 3600
Wire Wire Line
	5500 3600 5200 3600
Wire Wire Line
	7200 3300 6850 3300
Wire Wire Line
	6850 3200 7200 3200
Wire Wire Line
	4500 4850 3550 4850
Wire Wire Line
	4500 4950 3550 4950
Wire Wire Line
	3850 5500 3850 5400
Wire Wire Line
	3850 4450 3850 4750
Wire Wire Line
	3850 4750 3850 5100
Wire Wire Line
	3850 4750 4500 4750
Connection ~ 3850 4750
Wire Wire Line
	4200 5050 4200 4650
Wire Wire Line
	4200 4650 4500 4650
$Comp
L MOLEX22-03-5045 J18?
U 1 1 5A6A7E14
P 4700 4800
F 0 "J18?" H 4950 5100 60  0000 C CNN
F 1 "MOLEX22-03-5045" H 5000 4500 60  0000 C CNN
F 2 "" H 4700 4800 60  0001 C CNN
F 3 "" H 4700 4800 60  0001 C CNN
	1    4700 4800
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5A6FDCF8
P 3750 2550
F 0 "#PWR?" H 3750 2400 50  0001 C CNN
F 1 "+5V" H 3750 2690 50  0000 C CNN
F 2 "" H 3750 2550 50  0001 C CNN
F 3 "" H 3750 2550 50  0001 C CNN
	1    3750 2550
	1    0    0    -1  
$EndComp
Text HLabel 1950 3300 0    60   BiDi ~ 0
DXL_RXD
Text HLabel 1950 3400 0    60   BiDi ~ 0
DXL_TXD
Text HLabel 3550 3600 0    60   Input ~ 0
DXL_DIR
$Comp
L +5V #PWR?
U 1 1 5A6FED8C
P 6100 3500
F 0 "#PWR?" H 6100 3350 50  0001 C CNN
F 1 "+5V" H 6100 3640 50  0000 C CNN
F 2 "" H 6100 3500 50  0001 C CNN
F 3 "" H 6100 3500 50  0001 C CNN
	1    6100 3500
	0    1    1    0   
$EndComp
Text Label 7200 3200 0    60   ~ 0
DXL_485+
Text Label 7200 3300 0    60   ~ 0
DXL_485-
Text Label 3550 4850 2    60   ~ 0
DXL_485+
Text Label 3550 4950 2    60   ~ 0
DXL_485-
Text HLabel 3850 4450 0    60   Input ~ 0
DXL_PWR
$Comp
L AVRC5S_05D_050_050R U?
U 1 1 5A72D5C4
P 2150 3600
F 0 "U?" H 2400 4050 60  0000 C CNN
F 1 "AVRC5S_05D_050_050R" H 2450 3550 60  0000 C CNN
F 2 "" H 2150 3600 60  0001 C CNN
F 3 "" H 2150 3600 60  0001 C CNN
	1    2150 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3300 3550 3300
Wire Wire Line
	3550 3300 3550 3200
Wire Wire Line
	3550 3500 3550 3400
Wire Wire Line
	3550 3400 2850 3400
Wire Wire Line
	1950 3500 1850 3500
Wire Wire Line
	1850 3500 1850 3800
$Comp
L GND #PWR?
U 1 1 5A72D76F
P 1850 3800
F 0 "#PWR?" H 1850 3550 50  0001 C CNN
F 1 "GND" H 1850 3650 50  0000 C CNN
F 2 "" H 1850 3800 50  0001 C CNN
F 3 "" H 1850 3800 50  0001 C CNN
	1    1850 3800
	1    0    0    -1  
$EndComp
$EndSCHEMATC
