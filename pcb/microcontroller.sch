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
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 5
Title ""
Date "8 may 2016"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L GND #PWR016
U 1 1 565B23CF
P 5900 5600
F 0 "#PWR016" H 5900 5350 50  0001 C CNN
F 1 "GND" H 5900 5450 50  0000 C CNN
F 2 "" H 5900 5600 60  0000 C CNN
F 3 "" H 5900 5600 60  0000 C CNN
	1    5900 5600
	1    0    0    -1  
$EndComp
$Comp
L C C403
U 1 1 565B2484
P 3000 2000
F 0 "C403" H 3025 2100 50  0000 L CNN
F 1 "0.1uF" H 3025 1900 50  0000 L CNN
F 2 "Woodchuck:C0603" H 3050 1800 30  0001 C CNN
F 3 "" H 3000 2000 60  0000 C CNN
F 4 "Value" H 3000 2000 60  0001 C CNN "Digikey"
F 5 "1759037" H 3000 2000 60  0001 C CNN "Farnell"
F 6 "Value" H 3000 2000 60  0001 C CNN "Fieldname"
	1    3000 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 565B2D06
P 3000 2350
F 0 "#PWR017" H 3000 2100 50  0001 C CNN
F 1 "GND" H 3000 2200 50  0000 C CNN
F 2 "" H 3000 2350 60  0000 C CNN
F 3 "" H 3000 2350 60  0000 C CNN
	1    3000 2350
	1    0    0    -1  
$EndComp
$Comp
L C C402
U 1 1 565B2D3B
P 2750 2000
F 0 "C402" H 2775 2100 50  0000 L CNN
F 1 "0.1uF" H 2775 1900 50  0000 L CNN
F 2 "Woodchuck:C0603" H 2788 1850 30  0001 C CNN
F 3 "" H 2750 2000 60  0000 C CNN
F 4 "Value" H 2750 2000 60  0001 C CNN "Digikey"
F 5 "1759037" H 2750 2000 60  0001 C CNN "Farnell"
F 6 "Value" H 2750 2000 60  0001 C CNN "Fieldname"
	1    2750 2000
	1    0    0    -1  
$EndComp
Text Notes 3500 1450 2    60   ~ 0
NOTE: Place one near each\nVDD pin (24, 48) and one\nnear VDDIO2 (36)
$Comp
L C C401
U 1 1 565B339D
P 1650 2000
F 0 "C401" H 1675 2100 50  0000 L CNN
F 1 "10nF" H 1675 1900 50  0000 L CNN
F 2 "Woodchuck:C0603" H 1688 1850 30  0001 C CNN
F 3 "" H 1650 2000 60  0000 C CNN
F 4 "Value" H 1650 2000 60  0001 C CNN "Digikey"
F 5 "1907314" H 1650 2000 60  0001 C CNN "Farnell"
F 6 "Value" H 1650 2000 60  0001 C CNN "Fieldname"
	1    1650 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 565B33AB
P 1650 2250
F 0 "#PWR018" H 1650 2000 50  0001 C CNN
F 1 "GND" H 1650 2100 50  0000 C CNN
F 2 "" H 1650 2250 60  0000 C CNN
F 3 "" H 1650 2250 60  0000 C CNN
	1    1650 2250
	1    0    0    -1  
$EndComp
Text Notes 1450 2500 1    60   ~ 0
NOTE: Place near VDDA (9)
$Comp
L C C404
U 1 1 565B3429
P 3250 2000
F 0 "C404" H 3275 2100 50  0000 L CNN
F 1 "0.1uF" H 3275 1900 50  0000 L CNN
F 2 "Woodchuck:C0603" H 3288 1850 30  0001 C CNN
F 3 "" H 3250 2000 60  0000 C CNN
F 4 "Value" H 3250 2000 60  0001 C CNN "Digikey"
F 5 "1759037" H 3250 2000 60  0001 C CNN "Farnell"
F 6 "Value" H 3250 2000 60  0001 C CNN "Fieldname"
	1    3250 2000
	1    0    0    -1  
$EndComp
Text Notes 6250 2500 0    60   ~ 0
NOTE: VDD supply needs 4.7uF across it
$Comp
L SWD P401
U 1 1 565B3F04
P 2300 3450
F 0 "P401" H 2550 3750 60  0000 C CNN
F 1 "SWD" H 2000 3750 60  0000 C CNN
F 2 "Woodchuck:FTSH-105-01-F-D-K" H 2650 3250 60  0001 C CNN
F 3 "" H 2650 3250 60  0000 C CNN
F 4 "Value" H 2300 3450 60  0001 C CNN "Digikey"
F 5 "1865285" H 2300 3450 60  0001 C CNN "Farnell"
F 6 "Value" H 2300 3450 60  0001 C CNN "Fieldname"
	1    2300 3450
	1    0    0    -1  
$EndComp
Text Label 6900 3350 0    60   ~ 0
nRST
Text Label 2900 3650 0    60   ~ 0
nRST
NoConn ~ 2750 3450
NoConn ~ 2750 3550
Text Label 2900 3350 0    60   ~ 0
SWCLK
Text Label 2900 3250 0    60   ~ 0
SWDIO
Text Label 4650 4900 2    60   ~ 0
SWDIO
Text Label 4650 5000 2    60   ~ 0
SWCLK
$Comp
L GND #PWR019
U 1 1 565B4620
P 1700 3750
F 0 "#PWR019" H 1700 3500 50  0001 C CNN
F 1 "GND" H 1700 3600 50  0000 C CNN
F 2 "" H 1700 3750 60  0000 C CNN
F 3 "" H 1700 3750 60  0000 C CNN
	1    1700 3750
	1    0    0    -1  
$EndComp
$Comp
L JSTA-4 J401
U 1 1 565B5A63
P 2500 4550
F 0 "J401" H 2550 4100 60  0000 C CNN
F 1 "JSTA-4" H 2500 4600 60  0000 C CNN
F 2 "Woodchuck:S04B-PASK-2" H 2500 4650 60  0001 C CNN
F 3 "" H 2500 4650 60  0000 C CNN
F 4 "Value" H 2500 4550 60  0001 C CNN "Digikey"
F 5 "1830749" H 2500 4550 60  0001 C CNN "Farnell"
F 6 "Value" H 2500 4550 60  0001 C CNN "Fieldname"
	1    2500 4550
	1    0    0    -1  
$EndComp
Text Notes 3250 4000 2    60   ~ 0
SWD PORT
Text Notes 3600 2700 2    60   ~ 0
SUPPLY CAPS
Text Label 2200 4700 2    60   ~ 0
EXT_TX
Text Label 2200 4800 2    60   ~ 0
EXT_RX
$Comp
L GND #PWR020
U 1 1 565B6C28
P 2200 5000
F 0 "#PWR020" H 2200 4750 50  0001 C CNN
F 1 "GND" H 2200 4850 50  0000 C CNN
F 2 "" H 2200 5000 60  0000 C CNN
F 3 "" H 2200 5000 60  0000 C CNN
	1    2200 5000
	1    0    0    -1  
$EndComp
Text Notes 2750 5250 2    60   ~ 0
EXT PORT
$Comp
L GND #PWR021
U 1 1 565B80EF
P 2400 6100
F 0 "#PWR021" H 2400 5850 50  0001 C CNN
F 1 "GND" H 2400 5950 50  0000 C CNN
F 2 "" H 2400 6100 60  0000 C CNN
F 3 "" H 2400 6100 60  0000 C CNN
	1    2400 6100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 565B8112
P 2800 6300
F 0 "#PWR022" H 2800 6050 50  0001 C CNN
F 1 "GND" H 2800 6150 50  0000 C CNN
F 2 "" H 2800 6300 60  0000 C CNN
F 3 "" H 2800 6300 60  0000 C CNN
	1    2800 6300
	1    0    0    -1  
$EndComp
Text Label 4650 4700 2    60   ~ 0
USB_D-
Text Label 4650 4800 2    60   ~ 0
USB_D+
Text Label 2200 5750 2    60   ~ 0
USB_D-
Text Label 2200 5850 2    60   ~ 0
USB_D+
Text Notes 3000 6550 2    60   ~ 0
USB PORT
Text Label 6900 3100 0    60   ~ 0
BOOT0
$Comp
L SPST SW401
U 1 1 565B9D59
P 8550 4750
F 0 "SW401" H 8550 4850 50  0000 C CNN
F 1 "SPST" H 8550 4650 50  0000 C CNN
F 2 "Woodchuck:B3U-1000P_switch" H 8550 4950 60  0001 C CNN
F 3 "http://www.farnell.com/datasheets/1685550.pdf" H 8550 4750 60  0001 C CNN
F 4 "1333652" H 8550 4750 60  0001 C CNN "Farnell"
	1    8550 4750
	1    0    0    -1  
$EndComp
Text Label 7850 4750 2    60   ~ 0
BOOT0
$Comp
L RESISTOR R404
U 1 1 565BA71B
P 7950 5100
F 0 "R404" H 8070 5190 60  0000 C CNN
F 1 "10K" H 7950 4950 60  0000 C CNN
F 2 "Woodchuck:0603" H 7950 4850 60  0001 C CNN
F 3 "" H 7950 5100 60  0000 C CNN
F 4 "Value" H 7950 5100 60  0001 C CNN "Digikey"
F 5 "9331700" H 7950 5100 60  0001 C CNN "Farnell"
F 6 "Value" H 7950 5100 60  0001 C CNN "Fieldname"
	1    7950 5100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR023
U 1 1 565BAC22
P 7950 5450
F 0 "#PWR023" H 7950 5200 50  0001 C CNN
F 1 "GND" H 7950 5300 50  0000 C CNN
F 2 "" H 7950 5450 60  0000 C CNN
F 3 "" H 7950 5450 60  0000 C CNN
	1    7950 5450
	1    0    0    -1  
$EndComp
Text Notes 9250 5650 2    60   ~ 0
BOOTLOADER SELECT
$Comp
L +3.3V #PWR024
U 1 1 565F343F
P 9150 4650
F 0 "#PWR024" H 9150 4500 50  0001 C CNN
F 1 "+3.3V" H 9150 4790 50  0000 C CNN
F 2 "" H 9150 4650 50  0000 C CNN
F 3 "" H 9150 4650 50  0000 C CNN
	1    9150 4650
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR025
U 1 1 565F3C95
P 2200 4500
F 0 "#PWR025" H 2200 4350 50  0001 C CNN
F 1 "+3.3V" H 2200 4640 50  0000 C CNN
F 2 "" H 2200 4500 50  0000 C CNN
F 3 "" H 2200 4500 50  0000 C CNN
	1    2200 4500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR026
U 1 1 565F3F3F
P 1700 3150
F 0 "#PWR026" H 1700 3000 50  0001 C CNN
F 1 "+3.3V" H 1700 3290 50  0000 C CNN
F 2 "" H 1700 3150 50  0000 C CNN
F 3 "" H 1700 3150 50  0000 C CNN
	1    1700 3150
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR027
U 1 1 565F40A9
P 3000 1650
F 0 "#PWR027" H 3000 1500 50  0001 C CNN
F 1 "+3.3V" H 3000 1790 50  0000 C CNN
F 2 "" H 3000 1650 50  0000 C CNN
F 3 "" H 3000 1650 50  0000 C CNN
	1    3000 1650
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR028
U 1 1 565F40DC
P 1650 1750
F 0 "#PWR028" H 1650 1600 50  0001 C CNN
F 1 "+3.3V" H 1650 1890 50  0000 C CNN
F 2 "" H 1650 1750 50  0000 C CNN
F 3 "" H 1650 1750 50  0000 C CNN
	1    1650 1750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR029
U 1 1 565F4A36
P 5800 2400
F 0 "#PWR029" H 5800 2250 50  0001 C CNN
F 1 "+3.3V" H 5800 2540 50  0000 C CNN
F 2 "" H 5800 2400 50  0000 C CNN
F 3 "" H 5800 2400 50  0000 C CNN
	1    5800 2400
	1    0    0    -1  
$EndComp
Text Label 6900 4600 0    60   ~ 0
EXT_TX
Text Label 6900 4700 0    60   ~ 0
EXT_RX
Text HLabel 4650 3800 0    60   Output ~ 0
GPS_FROM_MCU
Text HLabel 4650 3900 0    60   Input ~ 0
GPS_TO_MCU
Text HLabel 4650 5100 0    60   Output ~ 0
BARO_CS_N
Text HLabel 6900 3900 2    60   Output ~ 0
BARO_CLK
Text HLabel 6900 4000 2    60   Input ~ 0
BARO_MISO
Text HLabel 6900 4100 2    60   Output ~ 0
BARO_MOSI
Text HLabel 4650 4500 0    60   Output ~ 0
RADIO_EN
Text HLabel 4650 4000 0    60   Output ~ 0
RADIO_TX
NoConn ~ 6700 3600
NoConn ~ 6700 3700
NoConn ~ 6700 3800
NoConn ~ 6700 4400
NoConn ~ 6700 4500
NoConn ~ 6700 4800
NoConn ~ 4850 4600
NoConn ~ 4850 4400
NoConn ~ 4850 4300
NoConn ~ 4850 3600
NoConn ~ 4850 3400
NoConn ~ 4850 3300
NoConn ~ 4850 3200
NoConn ~ 4850 3100
NoConn ~ 4850 2950
NoConn ~ 5500 2600
$Comp
L RESISTOR R401
U 1 1 569D2BC0
P 3750 3700
F 0 "R401" H 3650 3800 60  0000 C CNN
F 1 "510" H 3750 3550 60  0001 C CNN
F 2 "Woodchuck:0603" H 3750 3700 60  0001 C CNN
F 3 "" H 3750 3700 60  0000 C CNN
F 4 "Value" H 3750 3700 60  0001 C CNN "Digikey"
F 5 "1469826" H 3750 3700 60  0001 C CNN "Farnell"
F 6 "Value" H 3750 3700 60  0001 C CNN "Fieldname"
	1    3750 3700
	1    0    0    -1  
$EndComp
$Comp
L RESISTOR R402
U 1 1 569D2C37
P 4150 4100
F 0 "R402" H 3950 4050 60  0000 C CNN
F 1 "510" H 4150 3950 60  0001 C CNN
F 2 "Woodchuck:0603" H 4150 4100 60  0001 C CNN
F 3 "" H 4150 4100 60  0000 C CNN
F 4 "Value" H 4150 4100 60  0001 C CNN "Digikey"
F 5 "1469826" H 4150 4100 60  0001 C CNN "Farnell"
F 6 "Value" H 4150 4100 60  0001 C CNN "Fieldname"
	1    4150 4100
	1    0    0    1   
$EndComp
$Comp
L RESISTOR R403
U 1 1 569D2C6E
P 4500 4200
F 0 "R403" H 4600 4100 60  0000 C CNN
F 1 "510" H 4500 4050 60  0001 C CNN
F 2 "Woodchuck:0603" H 4500 4200 60  0001 C CNN
F 3 "" H 4500 4200 60  0000 C CNN
F 4 "Value" H 4500 4200 60  0001 C CNN "Digikey"
F 5 "1469826" H 4500 4200 60  0001 C CNN "Farnell"
F 6 "Value" H 4500 4200 60  0001 C CNN "Fieldname"
	1    4500 4200
	1    0    0    -1  
$EndComp
$Comp
L STM32F072CBT6 U401
U 1 1 565B237E
P 5050 2800
F 0 "U401" H 5100 2850 60  0000 C CNN
F 1 "STM32F072CBT6" H 6300 2850 60  0000 C CNN
F 2 "Woodchuck:LQFP48" H 5400 2900 60  0001 C CNN
F 3 "http://www.farnell.com/datasheets/2006534.pdf" H 5400 2900 60  0001 C CNN
F 4 "Value" H 5050 2800 60  0001 C CNN "Digikey"
F 5 "2432094" H 5050 2800 60  0001 C CNN "Farnell"
F 6 "Value" H 5050 2800 60  0001 C CNN "Fieldname"
	1    5050 2800
	1    0    0    -1  
$EndComp
$Comp
L USB_OTG P402
U 1 1 569D56C3
P 2700 5850
F 0 "P402" V 3025 5725 50  0000 C CNN
F 1 "USB_OTG" H 2700 6050 50  0000 C CNN
F 2 "Woodchuck:MOLEX_67503_1020_MINI_USB" V 2650 5750 60  0001 C CNN
F 3 "" V 2650 5750 60  0000 C CNN
F 4 "Value" H 2700 5850 60  0001 C CNN "Digikey"
F 5 "2426381" H 2700 5850 60  0001 C CNN "Farnell"
F 6 "Value" H 2700 5850 60  0001 C CNN "Fieldname"
	1    2700 5850
	0    1    1    0   
$EndComp
NoConn ~ 2400 5650
NoConn ~ 2400 5950
Text HLabel 6900 5000 2    60   Output ~ 0
MTX2_P1
Text HLabel 6900 4900 2    60   Output ~ 0
MTX2_P0
$Comp
L 24AA01 IC1
U 1 1 56BF54E5
P 8200 3500
F 0 "IC1" H 8000 3700 50  0000 L CNN
F 1 "24AA01" H 8000 3300 50  0000 L CNN
F 2 "agg:SOT-23-5" H 8000 3200 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21711c.pdf" H 8000 3100 50  0001 L CNN
F 4 "1331269" H 8000 3000 50  0001 L CNN "Farnell"
	1    8200 3500
	-1   0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 56BF5C75
P 8850 3500
F 0 "C1" H 8875 3600 50  0000 L CNN
F 1 "0.1uF" H 8875 3400 50  0000 L CNN
F 2 "Woodchuck:C0603" H 8888 3350 30  0001 C CNN
F 3 "" H 8850 3500 60  0001 C CNN
F 4 "Value" H 8850 3500 60  0001 C CNN "Digikey"
F 5 "1759037" H 8850 3500 60  0001 C CNN "Farnell"
F 6 "Value" H 8850 3500 60  0001 C CNN "Fieldname"
	1    8850 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR030
U 1 1 56BF61FD
P 8850 3800
F 0 "#PWR030" H 8850 3550 50  0001 C CNN
F 1 "GND" H 8850 3650 50  0000 C CNN
F 2 "" H 8850 3800 60  0000 C CNN
F 3 "" H 8850 3800 60  0000 C CNN
	1    8850 3800
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR031
U 1 1 56BF6515
P 8850 3250
F 0 "#PWR031" H 8850 3100 50  0001 C CNN
F 1 "+3.3V" H 8850 3390 50  0000 C CNN
F 2 "" H 8850 3250 50  0000 C CNN
F 3 "" H 8850 3250 50  0000 C CNN
	1    8850 3250
	1    0    0    -1  
$EndComp
Text Notes 9200 4100 2    60   ~ 0
EEPROM
$Comp
L RESISTOR R2
U 1 1 56BF98C6
P 7700 3100
F 0 "R2" H 7550 3200 60  0000 C CNN
F 1 "10K" H 7700 2950 60  0001 C CNN
F 2 "Woodchuck:0603" H 7700 3100 60  0001 C CNN
F 3 "" H 7700 3100 60  0000 C CNN
F 4 "Value" H 7700 3100 60  0001 C CNN "Digikey"
F 5 "9331700" H 7700 3100 60  0001 C CNN "Farnell"
F 6 "Value" H 7700 3100 60  0001 C CNN "Fieldname"
	1    7700 3100
	0    1    1    0   
$EndComp
$Comp
L RESISTOR R1
U 1 1 56BF9EA2
P 7600 3100
F 0 "R1" H 7450 3000 60  0000 C CNN
F 1 "10K" H 7600 2950 60  0001 C CNN
F 2 "Woodchuck:0603" H 7600 3100 60  0001 C CNN
F 3 "" H 7600 3100 60  0000 C CNN
F 4 "Value" H 7600 3100 60  0001 C CNN "Digikey"
F 5 "9331700" H 7600 3100 60  0001 C CNN "Farnell"
F 6 "Value" H 7600 3100 60  0001 C CNN "Fieldname"
	1    7600 3100
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR032
U 1 1 56BFA35F
P 7650 2750
F 0 "#PWR032" H 7650 2600 50  0001 C CNN
F 1 "+3.3V" H 7650 2890 50  0000 C CNN
F 2 "" H 7650 2750 50  0000 C CNN
F 3 "" H 7650 2750 50  0000 C CNN
	1    7650 2750
	1    0    0    -1  
$EndComp
NoConn ~ 6700 5100
$Comp
L C C406
U 1 1 56C45ED5
P 3500 2000
F 0 "C406" H 3525 2100 50  0000 L CNN
F 1 "4u7" H 3525 1900 50  0000 L CNN
F 2 "Woodchuck:C0603" H 3500 1700 50  0001 C CNN
F 3 "" H 3500 2000 50  0000 C CNN
F 4 "1535567" H 3500 2000 60  0001 C CNN "Farnell"
	1    3500 2000
	1    0    0    -1  
$EndComp
$Comp
L C C405
U 1 1 56C46154
P 2050 2000
F 0 "C405" H 2075 2100 50  0000 L CNN
F 1 "1u" H 2075 1900 50  0000 L CNN
F 2 "Woodchuck:C0603" H 2050 1750 50  0001 C CNN
F 3 "" H 2050 2000 50  0000 C CNN
F 4 "2496935" H 2050 2000 60  0001 C CNN "Farnell"
	1    2050 2000
	1    0    0    -1  
$EndComp
$Comp
L PART X402
U 1 1 56C48D82
P 5250 6500
F 0 "X402" H 5300 6600 50  0000 L CNN
F 1 "Mounting hole" H 5300 6500 50  0000 L CNN
F 2 "Woodchuck:M3_MOUNT" H 5250 6500 50  0001 C CNN
F 3 "" H 5250 6500 50  0001 C CNN
	1    5250 6500
	1    0    0    -1  
$EndComp
$Comp
L PART X401
U 1 1 56C48F80
P 5250 6150
F 0 "X401" H 5300 6250 50  0000 L CNN
F 1 "Mounting hole" H 5300 6150 50  0000 L CNN
F 2 "Woodchuck:M3_MOUNT" H 5250 6150 50  0001 C CNN
F 3 "" H 5250 6150 50  0001 C CNN
	1    5250 6150
	1    0    0    -1  
$EndComp
$Comp
L PART X403
U 1 1 56C4AB59
P 6000 6150
F 0 "X403" H 6050 6250 50  0000 L CNN
F 1 "CUSF LOGO" H 6050 6150 50  0000 L CNN
F 2 "Woodchuck:cusf_logo" H 6000 6150 50  0001 C CNN
F 3 "" H 6000 6150 50  0001 C CNN
	1    6000 6150
	1    0    0    -1  
$EndComp
$Comp
L PART X404
U 1 1 56C4ABE4
P 6000 6500
F 0 "X404" H 6050 6600 50  0000 L CNN
F 1 "CUSF LOGO BACK" H 6050 6500 50  0000 L CNN
F 2 "Woodchuck:cusf_logo_full" H 6000 6500 50  0001 C CNN
F 3 "" H 6000 6500 50  0001 C CNN
	1    6000 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 5400 5800 5500
Wire Wire Line
	5800 5500 6100 5500
Wire Wire Line
	5900 5400 5900 5600
Connection ~ 5900 5500
Wire Wire Line
	6000 5500 6000 5400
Wire Wire Line
	5800 2400 5800 2600
Wire Wire Line
	5700 2600 5700 2500
Wire Wire Line
	5600 2500 5900 2500
Wire Wire Line
	3000 1650 3000 1850
Wire Wire Line
	3000 2150 3000 2350
Connection ~ 3000 2250
Wire Wire Line
	2750 2150 2750 2250
Wire Wire Line
	2750 1850 2750 1750
Connection ~ 3000 1750
Wire Wire Line
	2750 2250 3500 2250
Wire Wire Line
	2750 1750 3500 1750
Wire Wire Line
	1650 1750 1650 1850
Wire Wire Line
	3250 1750 3250 1850
Wire Wire Line
	3250 2250 3250 2150
Wire Wire Line
	5900 2500 5900 2600
Wire Wire Line
	4650 4700 4850 4700
Wire Wire Line
	4850 4800 4650 4800
Wire Wire Line
	2750 3650 2900 3650
Wire Wire Line
	2750 3350 2900 3350
Wire Wire Line
	2750 3250 2900 3250
Wire Wire Line
	4850 4900 4650 4900
Wire Wire Line
	4850 5000 4650 5000
Wire Wire Line
	1800 3250 1700 3250
Wire Wire Line
	1700 3250 1700 3150
Wire Wire Line
	1700 3650 1800 3650
Wire Wire Line
	1700 3350 1700 3750
Wire Wire Line
	1800 3450 1700 3450
Connection ~ 1700 3650
Wire Wire Line
	1800 3350 1700 3350
Connection ~ 1700 3450
Wire Notes Line
	1550 2900 1550 4000
Wire Notes Line
	1550 4000 3250 4000
Wire Notes Line
	3250 4000 3250 2900
Wire Notes Line
	3250 2900 1550 2900
Wire Notes Line
	1300 2700 3600 2700
Wire Notes Line
	3600 2700 3600 1200
Wire Notes Line
	3600 1200 1300 1200
Wire Notes Line
	1300 1200 1300 2700
Wire Wire Line
	2300 4700 2200 4700
Wire Wire Line
	2300 4800 2200 4800
Wire Wire Line
	2300 4900 2200 4900
Wire Wire Line
	2200 4900 2200 5000
Wire Wire Line
	2300 4600 2200 4600
Wire Wire Line
	2200 4600 2200 4500
Wire Notes Line
	1850 4300 2750 4300
Wire Notes Line
	2750 4300 2750 5250
Wire Notes Line
	2750 5250 1850 5250
Wire Notes Line
	1850 5250 1850 4300
Wire Notes Line
	1800 5550 3000 5550
Wire Notes Line
	3000 5550 3000 6550
Wire Notes Line
	3000 6550 1800 6550
Wire Notes Line
	1800 6550 1800 5550
Wire Wire Line
	7850 4750 8050 4750
Wire Wire Line
	9050 4750 9150 4750
Wire Wire Line
	9150 4750 9150 4650
Wire Wire Line
	7950 5450 7950 5350
Wire Wire Line
	7950 4850 7950 4750
Connection ~ 7950 4750
Wire Notes Line
	9250 4400 7500 4400
Wire Notes Line
	7500 4400 7500 5650
Wire Notes Line
	7500 5650 9250 5650
Wire Notes Line
	9250 5650 9250 4400
Wire Wire Line
	1650 2150 1650 2250
Wire Wire Line
	6700 3350 6900 3350
Wire Wire Line
	6900 3100 6700 3100
Wire Wire Line
	6700 4700 6900 4700
Wire Wire Line
	6900 4600 6700 4600
Wire Wire Line
	5600 2500 5600 2600
Connection ~ 5800 2500
Connection ~ 5700 2500
Wire Wire Line
	4850 3800 4650 3800
Wire Wire Line
	4850 3900 4650 3900
Wire Wire Line
	6700 3900 6900 3900
Wire Wire Line
	6700 4000 6900 4000
Wire Wire Line
	6900 4100 6700 4100
Wire Wire Line
	4850 5100 4650 5100
Wire Wire Line
	4850 4500 4650 4500
Connection ~ 6000 5500
Wire Wire Line
	2400 5750 2200 5750
Wire Wire Line
	2200 5850 2400 5850
Wire Wire Line
	2400 6100 2400 6050
Wire Wire Line
	2800 6250 2800 6300
Wire Wire Line
	6700 4900 6900 4900
Wire Wire Line
	6700 5000 6900 5000
Wire Wire Line
	8500 3400 8700 3400
Wire Wire Line
	8700 3400 8700 3350
Wire Wire Line
	8700 3350 8850 3350
Wire Wire Line
	8500 3600 8650 3600
Wire Wire Line
	8650 3600 8650 3650
Wire Wire Line
	8650 3650 8850 3650
Wire Wire Line
	8850 3650 8850 3800
Wire Wire Line
	8850 3350 8850 3250
Wire Wire Line
	8850 3750 7900 3750
Wire Wire Line
	7900 3750 7900 3600
Connection ~ 8850 3750
Wire Wire Line
	7600 4200 6700 4200
Wire Wire Line
	7600 3350 7600 4200
Wire Wire Line
	7600 3500 7900 3500
Wire Wire Line
	7900 3400 7700 3400
Wire Wire Line
	7700 3350 7700 4300
Wire Wire Line
	7700 4300 6700 4300
Wire Notes Line
	7800 3050 9200 3050
Wire Notes Line
	9200 3050 9200 4100
Wire Notes Line
	9200 4100 7800 4100
Wire Notes Line
	7800 4100 7800 3050
Connection ~ 7700 3400
Connection ~ 7600 3500
Wire Wire Line
	7600 2850 7600 2800
Wire Wire Line
	7600 2800 7700 2800
Wire Wire Line
	7700 2800 7700 2850
Wire Wire Line
	7650 2750 7650 2800
Connection ~ 7650 2800
Wire Wire Line
	4650 4000 4850 4000
Connection ~ 1650 1800
Connection ~ 1650 2200
Wire Wire Line
	3500 1750 3500 1850
Connection ~ 3250 1750
Wire Wire Line
	3500 2250 3500 2150
Connection ~ 3250 2250
Wire Wire Line
	1650 1800 2050 1800
Wire Wire Line
	2050 1800 2050 1850
Wire Wire Line
	2050 2150 2050 2200
Wire Wire Line
	2050 2200 1650 2200
$Comp
L +3.3V #PWR033
U 1 1 56B44BB2
P 3150 4250
F 0 "#PWR033" H 3150 4100 50  0001 C CNN
F 1 "+3.3V" H 3150 4390 50  0000 C CNN
F 2 "" H 3150 4250 60  0000 C CNN
F 3 "" H 3150 4250 60  0000 C CNN
	1    3150 4250
	1    0    0    -1  
$EndComp
$Comp
L LED_RABG D401
U 1 1 569D292A
P 3600 4550
F 0 "D401" H 3600 4900 50  0000 C CNN
F 1 "LED_RABG" H 3600 4200 50  0001 C CNN
F 2 "Woodchuck:CREE_RGB_LED" H 3550 5050 50  0001 C CNN
F 3 "http://www.farnell.com/datasheets/1965387.pdf" H 3500 4950 50  0001 C CNN
F 4 "Value" H 3600 4550 60  0001 C CNN "Digikey"
F 5 "2093940" H 3600 4550 60  0001 C CNN "Farnell"
F 6 "Value" H 3600 4550 60  0001 C CNN "Fieldname"
	1    3600 4550
	0    -1   1    0   
$EndComp
Wire Wire Line
	3150 4250 3150 4900
Wire Wire Line
	3150 4900 3600 4900
Wire Wire Line
	3600 4900 3600 4850
Wire Wire Line
	4850 4200 4750 4200
Wire Wire Line
	4250 4200 3800 4200
Wire Wire Line
	3800 4200 3800 4250
Wire Wire Line
	4850 4100 4400 4100
Wire Wire Line
	3900 4100 3600 4100
Wire Wire Line
	3600 4100 3600 4250
Wire Wire Line
	4000 3700 4850 3700
Wire Wire Line
	3500 3700 3400 3700
Wire Wire Line
	3400 3700 3400 4250
Wire Wire Line
	6100 5500 6100 5400
$EndSCHEMATC
