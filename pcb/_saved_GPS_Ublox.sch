EESchema Schematic File Version 2
LIBS:power
LIBS:device
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
LIBS:a2235-h
LIBS:adp3335
LIBS:adxl345
LIBS:alpha_trx433s
LIBS:buzzer
LIBS:FDN304P
LIBS:FG6943010R
LIBS:hmc5883l
LIBS:irf7910
LIBS:l3g4200d
LIBS:max-7q
LIBS:ms5611-01ba03
LIBS:resistor
LIBS:rfm69w
LIBS:stm32f405vgt
LIBS:swd
LIBS:tvsd
LIBS:u-blox_cam-m8q
LIBS:uSD_holder
LIBS:radiometrix_mtx2
LIBS:stm32f072cbt6
LIBS:jsta
LIBS:sma
LIBS:cga0402mlc-12g
LIBS:woodchuck-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 6
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
L u-blox_CAM-M8Q IC?
U 1 1 565D0659
P 5350 2900
F 0 "IC?" H 4900 3450 60  0000 C CNN
F 1 "u-blox_CAM-M8Q" H 5250 1400 60  0000 C CNN
F 2 "" H 5550 2000 60  0000 C CNN
F 3 "" H 5550 2000 60  0000 C CNN
	1    5350 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 2900 4550 4300
Wire Wire Line
	4650 2900 4550 2900
Wire Wire Line
	4650 3000 4550 3000
Connection ~ 4550 3000
Wire Wire Line
	4650 3100 4550 3100
Connection ~ 4550 3100
Wire Wire Line
	4650 3200 4550 3200
Connection ~ 4550 3200
Wire Wire Line
	4650 3300 4550 3300
Connection ~ 4550 3300
Wire Wire Line
	4650 3400 4550 3400
Connection ~ 4550 3400
Wire Wire Line
	4650 3600 4550 3600
Connection ~ 4550 3600
Wire Wire Line
	4650 3500 4550 3500
Connection ~ 4550 3500
Wire Wire Line
	4650 3700 4550 3700
Connection ~ 4550 3700
Wire Wire Line
	4650 3800 4550 3800
Connection ~ 4550 3800
Wire Wire Line
	4650 4000 4550 4000
Connection ~ 4550 4000
Wire Wire Line
	4550 3900 4650 3900
Connection ~ 4550 3900
Wire Wire Line
	4650 4100 4550 4100
Connection ~ 4550 4100
Wire Wire Line
	4650 4200 4550 4200
Connection ~ 4550 4200
$Comp
L GND #PWR?
U 1 1 565F67FA
P 4550 4300
F 0 "#PWR?" H 4550 4050 50  0001 C CNN
F 1 "GND" H 4550 4150 50  0000 C CNN
F 2 "" H 4550 4300 60  0000 C CNN
F 3 "" H 4550 4300 60  0000 C CNN
	1    4550 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2500 4550 2500
Wire Wire Line
	4550 2500 4550 2400
$Comp
L +3.3V #PWR?
U 1 1 565F7A37
P 4550 2400
F 0 "#PWR?" H 4550 2250 50  0001 C CNN
F 1 "+3.3V" H 4550 2540 50  0000 C CNN
F 2 "" H 4550 2400 60  0000 C CNN
F 3 "" H 4550 2400 60  0000 C CNN
	1    4550 2400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 565F7A4D
P 4000 2450
F 0 "C?" H 4025 2550 50  0000 L CNN
F 1 "100n" H 4025 2350 50  0000 L CNN
F 2 "" H 4038 2300 30  0000 C CNN
F 3 "" H 4000 2450 60  0000 C CNN
F 4 "Value" H 4000 2450 60  0001 C CNN "Fieldname"
	1    4000 2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 565F7B5B
P 4000 2700
F 0 "#PWR?" H 4000 2450 50  0001 C CNN
F 1 "GND" H 4000 2550 50  0000 C CNN
F 2 "" H 4000 2700 60  0000 C CNN
F 3 "" H 4000 2700 60  0000 C CNN
	1    4000 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2200 4000 2300
Wire Wire Line
	4000 2600 4000 2700
$Comp
L +3V3 #PWR?
U 1 1 565F7CC7
P 4000 2200
F 0 "#PWR?" H 4000 2050 50  0001 C CNN
F 1 "+3V3" H 4000 2340 50  0000 C CNN
F 2 "" H 4000 2200 60  0000 C CNN
F 3 "" H 4000 2200 60  0000 C CNN
	1    4000 2200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
