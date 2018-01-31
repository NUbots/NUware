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
Sheet 2 9
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
L MPU6000 U9?
U 1 1 5A6989D1
P 5200 3850
F 0 "U9?" H 5650 5500 60  0000 C CNN
F 1 "MPU6000" H 5650 3800 60  0000 C CNN
F 2 "" H 5200 3650 60  0001 C CNN
F 3 "" H 5200 3650 60  0001 C CNN
	1    5200 3850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A698A78
P 6500 2500
F 0 "#PWR?" H 6500 2250 50  0001 C CNN
F 1 "GND" H 6500 2350 50  0000 C CNN
F 2 "" H 6500 2500 50  0001 C CNN
F 3 "" H 6500 2500 50  0001 C CNN
	1    6500 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A698AEC
P 4600 3800
F 0 "#PWR?" H 4600 3550 50  0001 C CNN
F 1 "GND" H 4600 3650 50  0000 C CNN
F 2 "" H 4600 3800 50  0001 C CNN
F 3 "" H 4600 3800 50  0001 C CNN
	1    4600 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 2350 6500 2350
Wire Wire Line
	6250 2850 6850 2850
$Comp
L C C
U 1 1 5A6D29D1
P 7000 2850
F 0 "C" H 7025 2950 50  0000 L CNN
F 1 "0.1uF" H 7025 2750 50  0000 L CNN
F 2 "" H 7038 2700 50  0001 C CNN
F 3 "" H 7000 2850 50  0001 C CNN
	1    7000 2850
	0    -1   -1   0   
$EndComp
Connection ~ 6750 2850
Wire Wire Line
	6750 2100 6750 2850
$Comp
L GND #PWR?
U 1 1 5A6D2AC0
P 7250 2950
F 0 "#PWR?" H 7250 2700 50  0001 C CNN
F 1 "GND" H 7250 2800 50  0000 C CNN
F 2 "" H 7250 2950 50  0001 C CNN
F 3 "" H 7250 2950 50  0001 C CNN
	1    7250 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 2850 7250 2850
Wire Wire Line
	7250 2850 7250 2950
Wire Wire Line
	6500 2350 6500 2500
$Comp
L GND #PWR?
U 1 1 5A6D2B2A
P 7150 3500
F 0 "#PWR?" H 7150 3250 50  0001 C CNN
F 1 "GND" H 7150 3350 50  0000 C CNN
F 2 "" H 7150 3500 50  0001 C CNN
F 3 "" H 7150 3500 50  0001 C CNN
	1    7150 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 3450 6850 3500
$Comp
L GND #PWR?
U 1 1 5A6D2BA4
P 6850 3850
F 0 "#PWR?" H 6850 3600 50  0001 C CNN
F 1 "GND" H 6850 3700 50  0000 C CNN
F 2 "" H 6850 3850 50  0001 C CNN
F 3 "" H 6850 3850 50  0001 C CNN
	1    6850 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 3800 6850 3850
$Comp
L C C
U 1 1 5A6D2BCA
P 4600 3400
F 0 "C" H 4625 3500 50  0000 L CNN
F 1 "2.2nF" H 4625 3300 50  0000 L CNN
F 2 "" H 4638 3250 50  0001 C CNN
F 3 "" H 4600 3400 50  0001 C CNN
	1    4600 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2450 5000 2450
Wire Wire Line
	6250 3250 7750 3250
Wire Wire Line
	6250 3550 6350 3550
Wire Wire Line
	6350 3550 6350 2850
Connection ~ 6350 2850
Wire Wire Line
	6850 3450 6250 3450
Wire Wire Line
	6250 3350 7150 3350
Wire Wire Line
	7150 3350 7150 3500
Wire Wire Line
	5000 3250 4850 3250
Wire Wire Line
	4850 3250 4850 3650
Wire Wire Line
	4850 3650 4600 3650
Wire Wire Line
	4600 3550 4600 3800
Connection ~ 4600 3650
Wire Wire Line
	4600 2450 4600 3250
$Comp
L +3.3V #PWR?
U 1 1 5A6FD48D
P 6750 2100
F 0 "#PWR?" H 6750 1950 50  0001 C CNN
F 1 "+3.3V" H 6750 2240 50  0000 C CNN
F 2 "" H 6750 2100 50  0001 C CNN
F 3 "" H 6750 2100 50  0001 C CNN
	1    6750 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 2850 5000 2850
Wire Wire Line
	5000 2750 4200 2750
Text HLabel 4200 2750 0    60   Output ~ 0
MPU_SCL
Text HLabel 4200 2850 0    60   Output ~ 0
MPU_SDA/SDI
Text HLabel 7750 3250 2    60   Output ~ 0
MPU_INT
$Comp
L R R
U 1 1 5A700977
P 4350 2200
F 0 "R" V 4430 2200 50  0000 C CNN
F 1 "2K" V 4350 2200 50  0000 C CNN
F 2 "" V 4280 2200 50  0001 C CNN
F 3 "" H 4350 2200 50  0001 C CNN
	1    4350 2200
	-1   0    0    1   
$EndComp
Wire Wire Line
	4350 2350 4350 2750
Connection ~ 4350 2750
Wire Wire Line
	4500 2350 4500 2850
Connection ~ 4500 2850
Wire Wire Line
	4500 1850 4500 2050
Wire Wire Line
	4500 1950 4350 1950
Wire Wire Line
	4350 1950 4350 2050
Connection ~ 4500 1950
$Comp
L +3.3V #PWR?
U 1 1 5A700A52
P 4500 1850
F 0 "#PWR?" H 4500 1700 50  0001 C CNN
F 1 "+3.3V" H 4500 1990 50  0000 C CNN
F 2 "" H 4500 1850 50  0001 C CNN
F 3 "" H 4500 1850 50  0001 C CNN
	1    4500 1850
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A704E41
P 4500 2200
F 0 "R?" V 4580 2200 50  0000 C CNN
F 1 "2K" V 4500 2200 50  0000 C CNN
F 2 "" V 4430 2200 50  0001 C CNN
F 3 "" H 4500 2200 50  0001 C CNN
	1    4500 2200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5A704EA0
P 6850 3650
F 0 "C?" H 6875 3750 50  0000 L CNN
F 1 "0.1uF" H 6875 3550 50  0000 L CNN
F 2 "" H 6888 3500 50  0001 C CNN
F 3 "" H 6850 3650 50  0001 C CNN
	1    6850 3650
	1    0    0    -1  
$EndComp
$EndSCHEMATC
