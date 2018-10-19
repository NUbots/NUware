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
Sheet 9 9
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
L ACS724 U11?
U 1 1 5A6D62FA
P 2100 2000
F 0 "U11?" H 2400 2550 60  0000 C CNN
F 1 "ACS724" H 2400 1950 60  0000 C CNN
F 2 "" H 2100 2000 60  0001 C CNN
F 3 "" H 2100 2000 60  0001 C CNN
	1    2100 2000
	1    0    0    -1  
$EndComp
$Comp
L C 1nF
U 1 1 5A6D635A
P 3150 2050
F 0 "1nF" H 3175 2150 50  0000 L CNN
F 1 "C" H 3175 1950 50  0000 L CNN
F 2 "" H 3188 1900 50  0001 C CNN
F 3 "" H 3150 2050 50  0001 C CNN
	1    3150 2050
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A6D63B5
P 3450 2050
F 0 "C?" H 3475 2150 50  0000 L CNN
F 1 "C" H 3475 1950 50  0000 L CNN
F 2 "" H 3488 1900 50  0001 C CNN
F 3 "" H 3450 2050 50  0001 C CNN
	1    3450 2050
	1    0    0    -1  
$EndComp
$Comp
L C 0.1uF
U 1 1 5A6D63DA
P 3750 2050
F 0 "0.1uF" H 3775 2150 50  0000 L CNN
F 1 "C" H 3775 1950 50  0000 L CNN
F 2 "" H 3788 1900 50  0001 C CNN
F 3 "" H 3750 2050 50  0001 C CNN
	1    3750 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A6D644B
P 3150 2400
F 0 "#PWR?" H 3150 2150 50  0001 C CNN
F 1 "GND" H 3150 2250 50  0000 C CNN
F 2 "" H 3150 2400 50  0001 C CNN
F 3 "" H 3150 2400 50  0001 C CNN
	1    3150 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A6D6469
P 3450 2400
F 0 "#PWR?" H 3450 2150 50  0001 C CNN
F 1 "GND" H 3450 2250 50  0000 C CNN
F 2 "" H 3450 2400 50  0001 C CNN
F 3 "" H 3450 2400 50  0001 C CNN
	1    3450 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A6D6480
P 3750 2400
F 0 "#PWR?" H 3750 2150 50  0001 C CNN
F 1 "GND" H 3750 2250 50  0000 C CNN
F 2 "" H 3750 2400 50  0001 C CNN
F 3 "" H 3750 2400 50  0001 C CNN
	1    3750 2400
	1    0    0    -1  
$EndComp
Text GLabel 1600 1600 0    60   Input ~ 0
I+
Text GLabel 1600 1800 0    60   Input ~ 0
I-
Text HLabel 4150 1700 2    60   Output ~ 0
I_OUT
$Comp
L +5V #PWR?
U 1 1 5A71C782
P 3750 1450
F 0 "#PWR?" H 3750 1300 50  0001 C CNN
F 1 "+5V" H 3750 1590 50  0000 C CNN
F 2 "" H 3750 1450 50  0001 C CNN
F 3 "" H 3750 1450 50  0001 C CNN
	1    3750 1450
	1    0    0    -1  
$EndComp
$Comp
L bq76920 U?
U 1 1 5A7144DD
P 2900 4450
F 0 "U?" H 3450 5600 60  0000 C CNN
F 1 "bq76920" H 3450 4400 60  0000 C CNN
F 2 "" H 2900 4450 60  0001 C CNN
F 3 "" H 2900 4450 60  0001 C CNN
	1    2900 4450
	1    0    0    -1  
$EndComp
$Comp
L 5_PIN_JST U?
U 1 1 5A71501A
P 6350 3550
F 0 "U?" H 6500 4200 60  0000 C CNN
F 1 "5_PIN_JST" H 6500 3500 60  0000 C CNN
F 2 "" H 6350 3550 60  0001 C CNN
F 3 "" H 6350 3550 60  0001 C CNN
	1    6350 3550
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5A7150F8
P 5100 4350
F 0 "R?" V 5180 4350 50  0000 C CNN
F 1 "R" V 5100 4350 50  0000 C CNN
F 2 "" V 5030 4350 50  0001 C CNN
F 3 "" H 5100 4350 50  0001 C CNN
	1    5100 4350
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5A71513F
P 5100 3150
F 0 "R?" V 5180 3150 50  0000 C CNN
F 1 "R" V 5100 3150 50  0000 C CNN
F 2 "" V 5030 3150 50  0001 C CNN
F 3 "" H 5100 3150 50  0001 C CNN
	1    5100 3150
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 5A71518D
P 5100 3950
F 0 "R?" V 5180 3950 50  0000 C CNN
F 1 "R" V 5100 3950 50  0000 C CNN
F 2 "" V 5030 3950 50  0001 C CNN
F 3 "" H 5100 3950 50  0001 C CNN
	1    5100 3950
	0    -1   1    0   
$EndComp
$Comp
L C C?
U 1 1 5A71527F
P 4800 3750
F 0 "C?" H 4825 3850 50  0000 L CNN
F 1 "C" H 4825 3650 50  0000 L CNN
F 2 "" H 4838 3600 50  0001 C CNN
F 3 "" H 4800 3750 50  0001 C CNN
	1    4800 3750
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A715314
P 4800 4150
F 0 "C?" H 4825 4250 50  0000 L CNN
F 1 "C" H 4825 4050 50  0000 L CNN
F 2 "" H 4838 4000 50  0001 C CNN
F 3 "" H 4800 4150 50  0001 C CNN
	1    4800 4150
	-1   0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A7153F1
P 5100 4750
F 0 "R?" V 5180 4750 50  0000 C CNN
F 1 "R" V 5100 4750 50  0000 C CNN
F 2 "" V 5030 4750 50  0001 C CNN
F 3 "" H 5100 4750 50  0001 C CNN
	1    5100 4750
	0    -1   1    0   
$EndComp
$Comp
L C C?
U 1 1 5A7153F7
P 4800 4550
F 0 "C?" H 4825 4650 50  0000 L CNN
F 1 "C" H 4825 4450 50  0000 L CNN
F 2 "" H 4838 4400 50  0001 C CNN
F 3 "" H 4800 4550 50  0001 C CNN
	1    4800 4550
	-1   0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A715441
P 5100 5150
F 0 "R?" V 5180 5150 50  0000 C CNN
F 1 "R" V 5100 5150 50  0000 C CNN
F 2 "" V 5030 5150 50  0001 C CNN
F 3 "" H 5100 5150 50  0001 C CNN
	1    5100 5150
	0    -1   1    0   
$EndComp
$Comp
L C C?
U 1 1 5A715447
P 4800 4950
F 0 "C?" H 4825 5050 50  0000 L CNN
F 1 "C" H 4825 4850 50  0000 L CNN
F 2 "" H 4838 4800 50  0001 C CNN
F 3 "" H 4800 4950 50  0001 C CNN
	1    4800 4950
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A715642
P 4800 3350
F 0 "C?" H 4825 3450 50  0000 L CNN
F 1 "C" H 4825 3250 50  0000 L CNN
F 2 "" H 4838 3200 50  0001 C CNN
F 3 "" H 4800 3350 50  0001 C CNN
	1    4800 3350
	-1   0    0    -1  
$EndComp
Text HLabel 2700 3850 0    60   Input ~ 0
VS_SCL
Text HLabel 2700 3750 0    60   Input ~ 0
VS_SDA
$Comp
L R R?
U 1 1 5A717567
P 2300 4300
F 0 "R?" V 2380 4300 50  0000 C CNN
F 1 "R" V 2300 4300 50  0000 C CNN
F 2 "" V 2230 4300 50  0001 C CNN
F 3 "" H 2300 4300 50  0001 C CNN
	1    2300 4300
	-1   0    0    1   
$EndComp
$Comp
L R_Variable R?
U 1 1 5A7175C4
P 1950 3950
F 0 "R?" V 2050 3850 50  0000 L CNN
F 1 "R_Variable" V 1850 3900 50  0000 L CNN
F 2 "" V 1880 3950 50  0001 C CNN
F 3 "" H 1950 3950 50  0001 C CNN
	1    1950 3950
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5A7178EF
P 1550 4150
F 0 "#PWR?" H 1550 3900 50  0001 C CNN
F 1 "GND" H 1550 4000 50  0000 C CNN
F 2 "" H 1550 4150 50  0001 C CNN
F 3 "" H 1550 4150 50  0001 C CNN
	1    1550 4150
	1    0    0    -1  
$EndComp
Text HLabel 3900 3450 2    60   Output ~ 0
VS_ALERT
$Comp
L C C?
U 1 1 5A718214
P 2450 5400
F 0 "C?" H 2475 5500 50  0000 L CNN
F 1 "C" H 2475 5300 50  0000 L CNN
F 2 "" H 2488 5250 50  0001 C CNN
F 3 "" H 2450 5400 50  0001 C CNN
	1    2450 5400
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A7183EF
P 2450 5650
F 0 "#PWR?" H 2450 5400 50  0001 C CNN
F 1 "GND" H 2450 5500 50  0000 C CNN
F 2 "" H 2450 5650 50  0001 C CNN
F 3 "" H 2450 5650 50  0001 C CNN
	1    2450 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1600 3750 1600
Wire Wire Line
	3750 1450 3750 1900
Wire Wire Line
	2850 1700 4150 1700
Wire Wire Line
	3450 1700 3450 1900
Wire Wire Line
	2850 1800 3150 1800
Wire Wire Line
	3150 1800 3150 1900
Wire Wire Line
	2850 1900 2850 2300
Wire Wire Line
	2850 2300 3150 2300
Wire Wire Line
	3150 2200 3150 2400
Wire Wire Line
	3750 2400 3750 2200
Wire Wire Line
	3450 2400 3450 2200
Connection ~ 3150 2300
Wire Wire Line
	1600 1800 1900 1800
Wire Wire Line
	1900 1900 1800 1900
Wire Wire Line
	1800 1900 1800 1800
Connection ~ 1800 1800
Wire Wire Line
	1900 1700 1800 1700
Wire Wire Line
	1800 1700 1800 1600
Wire Wire Line
	1600 1600 1900 1600
Connection ~ 1800 1600
Connection ~ 3750 1600
Connection ~ 3450 1700
Wire Wire Line
	4600 3950 4950 3950
Wire Wire Line
	4800 3900 4800 4000
Connection ~ 4800 3950
Wire Wire Line
	4500 4350 4950 4350
Wire Wire Line
	4800 4300 4800 4400
Connection ~ 4800 4350
Wire Wire Line
	4800 4700 4800 4800
Wire Wire Line
	4800 5100 4800 5150
Wire Wire Line
	2600 5150 4950 5150
Wire Wire Line
	4400 4750 4950 4750
Connection ~ 4800 4750
Wire Wire Line
	4600 3150 4950 3150
Wire Wire Line
	4800 3150 4800 3200
Wire Wire Line
	4800 3500 4800 3600
Wire Wire Line
	4600 3150 4600 3750
Wire Wire Line
	4600 3750 3800 3750
Connection ~ 4800 3150
Wire Wire Line
	3800 3850 4600 3850
Wire Wire Line
	4600 3850 4600 3950
Wire Wire Line
	3800 3950 4500 3950
Wire Wire Line
	4500 3950 4500 4350
Wire Wire Line
	3800 4050 4400 4050
Wire Wire Line
	4400 4050 4400 4750
Connection ~ 4800 5150
Wire Wire Line
	5250 3150 5400 3150
Wire Wire Line
	5400 3150 5400 3650
Wire Wire Line
	5400 3650 5900 3650
Wire Wire Line
	5400 3750 5900 3750
Wire Wire Line
	5400 3750 5400 3950
Wire Wire Line
	5400 3950 5250 3950
Wire Wire Line
	5250 4350 5500 4350
Wire Wire Line
	5500 4350 5500 3850
Wire Wire Line
	5500 3850 5900 3850
Wire Wire Line
	5250 4750 5600 4750
Wire Wire Line
	5600 4750 5600 3950
Wire Wire Line
	5600 3950 5900 3950
Wire Wire Line
	5250 5150 5700 5150
Wire Wire Line
	5700 5150 5700 4050
Wire Wire Line
	5700 4050 5900 4050
Wire Wire Line
	2700 4250 2600 4250
Wire Wire Line
	2600 4250 2600 5150
Connection ~ 4300 5150
Wire Wire Line
	4800 3550 5400 3550
Connection ~ 5400 3550
Connection ~ 4800 3550
Wire Wire Line
	3900 4600 3900 3850
Wire Wire Line
	2300 4600 3900 4600
Connection ~ 3900 3850
Wire Wire Line
	2100 3950 2700 3950
Wire Wire Line
	2300 4150 2300 3950
Connection ~ 2300 3950
Wire Wire Line
	2300 4450 2300 4600
Wire Wire Line
	1800 3950 1550 3950
Wire Wire Line
	1550 3650 1550 4150
Wire Wire Line
	3800 4250 4300 4250
Wire Wire Line
	4300 4250 4300 5150
Wire Wire Line
	3800 4150 3800 4050
Wire Wire Line
	2700 4150 2450 4150
Wire Wire Line
	2450 4150 2450 5250
Wire Wire Line
	2450 5550 2450 5650
Wire Wire Line
	2450 5150 2200 5150
Wire Wire Line
	2200 5150 2200 5600
Connection ~ 2450 5150
$Comp
L R R?
U 1 1 5A7187DD
P 2700 4850
F 0 "R?" V 2780 4850 50  0000 C CNN
F 1 "R" V 2700 4850 50  0000 C CNN
F 2 "" V 2630 4850 50  0001 C CNN
F 3 "" H 2700 4850 50  0001 C CNN
	1    2700 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4350 2700 4700
Wire Wire Line
	2700 5000 2700 5150
Connection ~ 2700 5150
$Comp
L GND #PWR?
U 1 1 5A7189F4
P 2950 4950
F 0 "#PWR?" H 2950 4700 50  0001 C CNN
F 1 "GND" H 2950 4800 50  0000 C CNN
F 2 "" H 2950 4950 50  0001 C CNN
F 3 "" H 2950 4950 50  0001 C CNN
	1    2950 4950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A718B0F
P 2950 4800
F 0 "C?" H 2975 4900 50  0000 L CNN
F 1 "C" H 2975 4700 50  0000 L CNN
F 2 "" H 2988 4650 50  0001 C CNN
F 3 "" H 2950 4800 50  0001 C CNN
	1    2950 4800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2950 4650 2950 4500
Wire Wire Line
	2950 4500 2700 4500
Connection ~ 2700 4500
Wire Wire Line
	2700 4050 2150 4050
Wire Wire Line
	2150 4050 2150 4650
$Comp
L C C?
U 1 1 5A718C78
P 2150 4800
F 0 "C?" H 2175 4900 50  0000 L CNN
F 1 "C" H 2175 4700 50  0000 L CNN
F 2 "" H 2188 4650 50  0001 C CNN
F 3 "" H 2150 4800 50  0001 C CNN
	1    2150 4800
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A718CD0
P 2150 4950
F 0 "#PWR?" H 2150 4700 50  0001 C CNN
F 1 "GND" H 2150 4800 50  0000 C CNN
F 2 "" H 2150 4950 50  0001 C CNN
F 3 "" H 2150 4950 50  0001 C CNN
	1    2150 4950
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A719665
P 2700 5300
F 0 "C?" H 2725 5400 50  0000 L CNN
F 1 "C" H 2725 5200 50  0000 L CNN
F 2 "" H 2738 5150 50  0001 C CNN
F 3 "" H 2700 5300 50  0001 C CNN
	1    2700 5300
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A7196C4
P 2700 5450
F 0 "#PWR?" H 2700 5200 50  0001 C CNN
F 1 "GND" H 2700 5300 50  0000 C CNN
F 2 "" H 2700 5450 50  0001 C CNN
F 3 "" H 2700 5450 50  0001 C CNN
	1    2700 5450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5A719D8E
P 2200 5600
F 0 "#PWR?" H 2200 5450 50  0001 C CNN
F 1 "+3.3V" H 2200 5740 50  0000 C CNN
F 2 "" H 2200 5600 50  0001 C CNN
F 3 "" H 2200 5600 50  0001 C CNN
	1    2200 5600
	-1   0    0    1   
$EndComp
Wire Wire Line
	2700 3650 1550 3650
Connection ~ 1550 3950
Wire Wire Line
	3900 3450 3800 3450
Wire Wire Line
	3850 3450 3850 3200
Wire Wire Line
	2550 3200 2550 3650
Connection ~ 2550 3650
Connection ~ 3850 3450
$Comp
L R R?
U 1 1 5A71C3E9
P 2850 3200
F 0 "R?" V 2930 3200 50  0000 C CNN
F 1 "1M" V 2850 3200 50  0000 C CNN
F 2 "" V 2780 3200 50  0001 C CNN
F 3 "" H 2850 3200 50  0001 C CNN
	1    2850 3200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2550 3200 2700 3200
Wire Wire Line
	3850 3200 3000 3200
$EndSCHEMATC