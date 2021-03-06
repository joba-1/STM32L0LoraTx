EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "LoraTx"
Date "2021-02-19"
Rev "1"
Comp "Joachim Banzhaf"
Comment1 "STM32L011 TSSOP20"
Comment2 "RFM95"
Comment3 "external SYS_SW, TX, I2C, SPI"
Comment4 "2 voltage dividers"
$EndDescr
$Comp
L RF_Module:RFM95W-868S2 U2
U 1 1 602FC82F
P 9550 2750
F 0 "U2" H 9550 3431 50  0000 C CNN
F 1 "RFM95W-868S2" H 9550 3340 50  0000 C CNN
F 2 "RF_Modules:Hopref_RFM9XW_SMD" H 6250 4400 50  0001 C CNN
F 3 "https://www.hoperf.com/data/upload/portal/20181127/5bfcbea20e9ef.pdf" H 6250 4400 50  0001 C CNN
	1    9550 2750
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32L0:STM32L011F4Px U1
U 1 1 603006A3
P 3050 2750
F 0 "U1" H 3000 1861 50  0000 C CNN
F 1 "STM32L011F4Px" H 3000 1770 50  0000 C CNN
F 2 "Housings_SSOP:TSSOP-20_4.4x6.5mm_Pitch0.65mm" H 2650 2050 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00206508.pdf" H 3050 2750 50  0001 C CNN
	1    3050 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 6030BE99
P 2400 3600
F 0 "R1" H 2470 3646 50  0000 L CNN
F 1 "10k" H 2470 3555 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 3600 50  0001 C CNN
F 3 "~" H 2400 3600 50  0001 C CNN
	1    2400 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 6030C1E5
P 2450 2000
F 0 "R2" H 2520 2046 50  0000 L CNN
F 1 "10k" H 2520 1955 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2380 2000 50  0001 C CNN
F 3 "~" H 2450 2000 50  0001 C CNN
	1    2450 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 6030C70F
P 8850 5100
F 0 "C4" H 8965 5146 50  0000 L CNN
F 1 "10µ" H 8965 5055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8888 4950 50  0001 C CNN
F 3 "~" H 8850 5100 50  0001 C CNN
	1    8850 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 6030DEB8
P 8450 5100
F 0 "C3" H 8565 5146 50  0000 L CNN
F 1 "10µ" H 8565 5055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8488 4950 50  0001 C CNN
F 3 "~" H 8450 5100 50  0001 C CNN
	1    8450 5100
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 60315B8B
P 2050 2250
F 0 "SW1" H 2050 2535 50  0000 C CNN
F 1 "SW_Push" H 2050 2444 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm_h5mm" H 2050 2450 50  0001 C CNN
F 3 "~" H 2050 2450 50  0001 C CNN
	1    2050 2250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR06
U 1 1 60316750
P 2950 1850
F 0 "#PWR06" H 2950 1700 50  0001 C CNN
F 1 "+3.3V" H 2965 2023 50  0000 C CNN
F 2 "" H 2950 1850 50  0001 C CNN
F 3 "" H 2950 1850 50  0001 C CNN
	1    2950 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 60317089
P 1750 2350
F 0 "#PWR03" H 1750 2100 50  0001 C CNN
F 1 "GND" H 1755 2177 50  0000 C CNN
F 2 "" H 1750 2350 50  0001 C CNN
F 3 "" H 1750 2350 50  0001 C CNN
	1    1750 2350
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 6031F2A6
P 2450 1750
F 0 "#PWR05" H 2450 1600 50  0001 C CNN
F 1 "+3.3V" H 2465 1923 50  0000 C CNN
F 2 "" H 2450 1750 50  0001 C CNN
F 3 "" H 2450 1750 50  0001 C CNN
	1    2450 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2250 1750 2350
Wire Wire Line
	9650 3350 9650 3450
Wire Wire Line
	9650 3450 9550 3450
Connection ~ 9550 3450
Wire Wire Line
	9550 3450 9550 3550
Wire Wire Line
	9550 3450 9450 3450
Wire Wire Line
	9450 3450 9450 3350
$Comp
L power:GND #PWR017
U 1 1 60325E2E
P 9550 3550
F 0 "#PWR017" H 9550 3300 50  0001 C CNN
F 1 "GND" H 9555 3377 50  0000 C CNN
F 2 "" H 9550 3550 50  0001 C CNN
F 3 "" H 9550 3550 50  0001 C CNN
	1    9550 3550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR018
U 1 1 60326CEE
P 9950 2150
F 0 "#PWR018" H 9950 2000 50  0001 C CNN
F 1 "+3.3V" H 9965 2323 50  0000 C CNN
F 2 "" H 9950 2150 50  0001 C CNN
F 3 "" H 9950 2150 50  0001 C CNN
	1    9950 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 6032B0AA
P 3400 3750
F 0 "#PWR07" H 3400 3500 50  0001 C CNN
F 1 "GND" H 3405 3577 50  0000 C CNN
F 2 "" H 3400 3750 50  0001 C CNN
F 3 "" H 3400 3750 50  0001 C CNN
	1    3400 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 602FFE91
P 2400 3800
F 0 "#PWR04" H 2400 3550 50  0001 C CNN
F 1 "GND" H 2405 3627 50  0000 C CNN
F 2 "" H 2400 3800 50  0001 C CNN
F 3 "" H 2400 3800 50  0001 C CNN
	1    2400 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 3600 3400 3600
Text GLabel 2250 3350 0    50   Input ~ 0
READY
Text GLabel 10150 2650 2    50   Output ~ 0
READY
Wire Wire Line
	10050 2650 10150 2650
Wire Wire Line
	2250 3350 2400 3350
Wire Wire Line
	2400 3450 2400 3350
Connection ~ 2400 3350
Wire Wire Line
	2400 3350 2550 3350
Wire Wire Line
	2400 3750 2400 3800
Wire Wire Line
	2250 2250 2450 2250
Wire Wire Line
	2450 2150 2450 2250
Connection ~ 2450 2250
Wire Wire Line
	2450 2250 2550 2250
Wire Wire Line
	8450 4950 8450 4850
Wire Wire Line
	8450 5250 8450 5350
$Comp
L power:+3.3V #PWR015
U 1 1 60317745
P 8850 4750
F 0 "#PWR015" H 8850 4600 50  0001 C CNN
F 1 "+3.3V" H 8865 4923 50  0000 C CNN
F 2 "" H 8850 4750 50  0001 C CNN
F 3 "" H 8850 4750 50  0001 C CNN
	1    8850 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 603186F5
P 8850 5450
F 0 "#PWR016" H 8850 5200 50  0001 C CNN
F 1 "GND" H 8855 5277 50  0000 C CNN
F 2 "" H 8850 5450 50  0001 C CNN
F 3 "" H 8850 5450 50  0001 C CNN
	1    8850 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 6030BADF
P 4050 3100
F 0 "R3" H 4120 3146 50  0000 L CNN
F 1 "1k" H 4120 3055 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3980 3100 50  0001 C CNN
F 3 "~" H 4050 3100 50  0001 C CNN
	1    4050 3100
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D1
U 1 1 60306F60
P 4050 3500
F 0 "D1" H 4043 3717 50  0000 C CNN
F 1 "LED" H 4043 3626 50  0000 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 4050 3500 50  0001 C CNN
F 3 "~" H 4050 3500 50  0001 C CNN
	1    4050 3500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4050 3250 4050 3350
$Comp
L power:GND #PWR010
U 1 1 60323626
P 4050 3750
F 0 "#PWR010" H 4050 3500 50  0001 C CNN
F 1 "GND" H 4055 3577 50  0000 C CNN
F 2 "" H 4050 3750 50  0001 C CNN
F 3 "" H 4050 3750 50  0001 C CNN
	1    4050 3750
	1    0    0    -1  
$EndComp
Text GLabel 3600 2250 2    50   Input ~ 0
DONE
Wire Wire Line
	3600 2250 3450 2250
Text GLabel 10150 3150 2    50   Output ~ 0
DONE
Wire Wire Line
	10150 3150 10050 3150
Text GLabel 2450 3050 0    50   Output ~ 0
RFM_RST
Wire Wire Line
	2550 3050 2450 3050
Text GLabel 8950 2950 0    50   Input ~ 0
RFM_RST
Wire Wire Line
	8950 2950 9050 2950
Text GLabel 3550 2950 2    50   Output ~ 0
MOSI
Wire Wire Line
	3550 2950 3450 2950
Text GLabel 8950 2550 0    50   Input ~ 0
MOSI
Wire Wire Line
	9050 2550 8950 2550
Text GLabel 3550 2850 2    50   Input ~ 0
MISO
Wire Wire Line
	3450 2850 3550 2850
Text GLabel 3550 2750 2    50   Output ~ 0
SCK
Wire Wire Line
	3550 2750 3450 2750
Text GLabel 8950 2450 0    50   Input ~ 0
SCK
Wire Wire Line
	9050 2450 8950 2450
Text GLabel 8950 2650 0    50   Output ~ 0
MISO
Wire Wire Line
	8950 2650 9050 2650
Text GLabel 2450 3250 0    50   Output ~ 0
RFM_CS
Wire Wire Line
	2550 3250 2450 3250
Text GLabel 8950 2750 0    50   Input ~ 0
RFM_CS
Wire Wire Line
	8950 2750 9050 2750
NoConn ~ 10050 2750
NoConn ~ 10050 2850
NoConn ~ 10050 2950
NoConn ~ 10050 3050
$Comp
L power:GND #PWR011
U 1 1 6035CBD9
P 5650 3450
F 0 "#PWR011" H 5650 3200 50  0001 C CNN
F 1 "GND" H 5655 3277 50  0000 C CNN
F 2 "" H 5650 3450 50  0001 C CNN
F 3 "" H 5650 3450 50  0001 C CNN
	1    5650 3450
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack J3
U 1 1 603074CC
P 5750 2050
F 0 "J3" V 5761 2238 50  0000 L CNN
F 1 "Barrel_Jack" V 5852 2238 50  0000 L CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_Altech_AK300-2_P5.00mm" H 5800 2010 50  0001 C CNN
F 3 "~" H 5800 2010 50  0001 C CNN
	1    5750 2050
	0    1    1    0   
$EndComp
Text GLabel 3600 2450 2    50   Input ~ 0
ADC_2
Text GLabel 3600 2550 2    50   Input ~ 0
ADC_1
Wire Wire Line
	3600 2450 3450 2450
Wire Wire Line
	3600 2550 3450 2550
$Comp
L Connector:Conn_01x02_Female J6
U 1 1 6037B0B1
P 9550 5050
F 0 "J6" H 9578 5026 50  0000 L CNN
F 1 "Conn_01x02_Female" H 9578 4935 50  0000 L CNN
F 2 "Connectors_JST:JST_EH_B02B-EH-A_02x2.50mm_Straight" H 9550 5050 50  0001 C CNN
F 3 "~" H 9550 5050 50  0001 C CNN
	1    9550 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 5050 9250 5050
Wire Wire Line
	9250 5050 9250 4850
Wire Wire Line
	9350 5150 9250 5150
Wire Wire Line
	9250 5150 9250 5350
$Comp
L Connector:Conn_01x04_Female J1
U 1 1 6039AF32
P 2350 5150
F 0 "J1" H 2378 5126 50  0000 L CNN
F 1 "Conn_01x04_Female" H 2378 5035 50  0000 L CNN
F 2 "Connectors_JST:JST_EH_B04B-EH-A_04x2.50mm_Straight" H 2350 5150 50  0001 C CNN
F 3 "~" H 2350 5150 50  0001 C CNN
	1    2350 5150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J2
U 1 1 6039C536
P 4600 5150
F 0 "J2" H 4628 5126 50  0000 L CNN
F 1 "Conn_01x06_Female" H 4628 5035 50  0000 L CNN
F 2 "Connectors_JST:JST_EH_B06B-EH-A_06x2.50mm_Straight" H 4600 5150 50  0001 C CNN
F 3 "~" H 4600 5150 50  0001 C CNN
	1    4600 5150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J4
U 1 1 6039E5D7
P 6950 5150
F 0 "J4" H 6978 5126 50  0000 L CNN
F 1 "Conn_01x06_Female" H 6978 5035 50  0000 L CNN
F 2 "Connectors_JST:JST_EH_B06B-EH-A_06x2.50mm_Straight" H 6950 5150 50  0001 C CNN
F 3 "~" H 6950 5150 50  0001 C CNN
	1    6950 5150
	1    0    0    -1  
$EndComp
Text GLabel 4300 5250 0    50   Input ~ 0
MOSI
Text GLabel 4300 5150 0    50   Input ~ 0
SCK
Text GLabel 4300 5450 0    50   Output ~ 0
MISO
Text GLabel 4300 5350 0    50   Input ~ 0
EXT_CS
$Comp
L power:GND #PWR09
U 1 1 603A98E3
P 3800 5150
F 0 "#PWR09" H 3800 4900 50  0001 C CNN
F 1 "GND" H 3805 4977 50  0000 C CNN
F 2 "" H 3800 5150 50  0001 C CNN
F 3 "" H 3800 5150 50  0001 C CNN
	1    3800 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 5050 3800 5050
Wire Wire Line
	3800 5050 3800 5150
$Comp
L power:+3.3V #PWR08
U 1 1 603AE632
P 3800 4850
F 0 "#PWR08" H 3800 4700 50  0001 C CNN
F 1 "+3.3V" H 3815 5023 50  0000 C CNN
F 2 "" H 3800 4850 50  0001 C CNN
F 3 "" H 3800 4850 50  0001 C CNN
	1    3800 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 4850 3800 4950
Wire Wire Line
	3800 4950 4400 4950
Wire Wire Line
	4400 5150 4300 5150
Wire Wire Line
	4400 5250 4300 5250
Wire Wire Line
	4300 5350 4400 5350
Wire Wire Line
	4400 5450 4300 5450
Text GLabel 2050 5350 0    50   Input ~ 0
SDA
Text GLabel 2050 5250 0    50   Input ~ 0
SCL
$Comp
L power:GND #PWR02
U 1 1 603C8C28
P 1550 5250
F 0 "#PWR02" H 1550 5000 50  0001 C CNN
F 1 "GND" H 1555 5077 50  0000 C CNN
F 2 "" H 1550 5250 50  0001 C CNN
F 3 "" H 1550 5250 50  0001 C CNN
	1    1550 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 5150 1550 5150
Wire Wire Line
	1550 5150 1550 5250
$Comp
L power:+3.3V #PWR01
U 1 1 603C8C30
P 1550 4950
F 0 "#PWR01" H 1550 4800 50  0001 C CNN
F 1 "+3.3V" H 1565 5123 50  0000 C CNN
F 2 "" H 1550 4950 50  0001 C CNN
F 3 "" H 1550 4950 50  0001 C CNN
	1    1550 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 4950 1550 5050
Wire Wire Line
	1550 5050 2150 5050
Wire Wire Line
	2050 5350 2150 5350
Text GLabel 6650 5250 0    50   Input ~ 0
SWDIO
Text GLabel 6650 5150 0    50   Input ~ 0
SWCLK
Text GLabel 6650 5450 0    50   Input ~ 0
TX
Text GLabel 6650 5350 0    50   Output ~ 0
NRST
$Comp
L power:GND #PWR013
U 1 1 603D2264
P 6150 5150
F 0 "#PWR013" H 6150 4900 50  0001 C CNN
F 1 "GND" H 6155 4977 50  0000 C CNN
F 2 "" H 6150 5150 50  0001 C CNN
F 3 "" H 6150 5150 50  0001 C CNN
	1    6150 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 5050 6150 5050
$Comp
L power:+3.3V #PWR012
U 1 1 603D226C
P 6150 4850
F 0 "#PWR012" H 6150 4700 50  0001 C CNN
F 1 "+3.3V" H 6165 5023 50  0000 C CNN
F 2 "" H 6150 4850 50  0001 C CNN
F 3 "" H 6150 4850 50  0001 C CNN
	1    6150 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 4850 6150 4950
Wire Wire Line
	6150 4950 6750 4950
Wire Wire Line
	6750 5450 6650 5450
Wire Wire Line
	6750 5350 6650 5350
Wire Wire Line
	6750 5250 6650 5250
Wire Wire Line
	6750 5150 6650 5150
Text GLabel 2350 2450 0    50   Input ~ 0
NRST
Wire Wire Line
	2350 2450 2450 2450
Wire Wire Line
	2450 2450 2450 2250
Text GLabel 3600 2350 2    50   Output ~ 0
TX
Wire Wire Line
	3600 2350 3450 2350
Text GLabel 3550 3050 2    50   Output ~ 0
SCL
Text GLabel 3550 3150 2    50   Output ~ 0
SDA
Wire Wire Line
	3550 3150 3450 3150
Text GLabel 3550 3250 2    50   Output ~ 0
SWDIO
Text GLabel 3550 3350 2    50   Output ~ 0
SWCLK
Wire Wire Line
	3550 3250 3450 3250
Wire Wire Line
	3550 3350 3450 3350
Wire Wire Line
	3450 2650 4050 2650
Text GLabel 2450 2950 0    50   Output ~ 0
EXT_CS
Wire Wire Line
	2550 2950 2450 2950
Wire Wire Line
	1850 2250 1750 2250
Wire Wire Line
	8850 4750 8850 4850
Wire Wire Line
	8850 4850 9250 4850
Connection ~ 8850 4850
Wire Wire Line
	8850 4850 8850 4950
Wire Wire Line
	8450 4850 8850 4850
$Comp
L Device:Antenna_Shield AE1
U 1 1 6044D14D
P 10150 2250
F 0 "AE1" H 10294 2289 50  0000 L CNN
F 1 "Antenna_Shield" H 10294 2198 50  0000 L CNN
F 2 "Connectors_Molex:Molex_Microcoaxial_RF" H 10150 2350 50  0001 C CNN
F 3 "~" H 10150 2350 50  0001 C CNN
	1    10150 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 2450 10150 2450
$Comp
L power:GND #PWR019
U 1 1 6045AB28
P 10600 2550
F 0 "#PWR019" H 10600 2300 50  0001 C CNN
F 1 "GND" H 10605 2377 50  0000 C CNN
F 2 "" H 10600 2550 50  0001 C CNN
F 3 "" H 10600 2550 50  0001 C CNN
	1    10600 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 2450 10600 2450
Wire Wire Line
	10600 2450 10600 2550
Wire Wire Line
	2150 5250 2050 5250
Wire Wire Line
	3450 3050 3550 3050
Wire Wire Line
	6150 5050 6150 5150
Wire Wire Line
	3400 3600 3400 3750
Wire Wire Line
	4050 2650 4050 2950
Wire Wire Line
	4050 3650 4050 3750
Wire Wire Line
	2950 3550 2950 3600
Wire Wire Line
	9550 3350 9550 3450
Wire Wire Line
	3050 2050 3050 1950
Wire Wire Line
	3050 1950 2950 1950
Wire Wire Line
	8850 5250 8850 5350
Wire Wire Line
	9250 5350 8850 5350
Connection ~ 8850 5350
Wire Wire Line
	8850 5350 8850 5450
Wire Wire Line
	8450 5350 8850 5350
Wire Wire Line
	2950 2050 2950 1950
Wire Wire Line
	9550 2250 9550 2200
Wire Wire Line
	9550 2200 9950 2200
Wire Wire Line
	9950 2200 9950 2150
Wire Wire Line
	2950 1850 2950 1950
Connection ~ 2950 1950
Wire Wire Line
	2450 1750 2450 1850
$Comp
L power:PWR_FLAG #FLG01
U 1 1 604EC644
P 9250 4750
F 0 "#FLG01" H 9250 4825 50  0001 C CNN
F 1 "PWR_FLAG" H 9250 4923 50  0000 C CNN
F 2 "" H 9250 4750 50  0001 C CNN
F 3 "~" H 9250 4750 50  0001 C CNN
	1    9250 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 4750 9250 4850
Connection ~ 9250 4850
$Comp
L power:PWR_FLAG #FLG02
U 1 1 604F2073
P 9250 5450
F 0 "#FLG02" H 9250 5525 50  0001 C CNN
F 1 "PWR_FLAG" H 9250 5623 50  0000 C CNN
F 2 "" H 9250 5450 50  0001 C CNN
F 3 "~" H 9250 5450 50  0001 C CNN
	1    9250 5450
	-1   0    0    1   
$EndComp
Wire Wire Line
	9250 5450 9250 5350
Connection ~ 9250 5350
$Comp
L Device:R R5
U 1 1 6055A77D
P 5650 3100
F 0 "R5" H 5720 3146 50  0000 L CNN
F 1 "1M" H 5720 3055 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5580 3100 50  0001 C CNN
F 3 "~" H 5650 3100 50  0001 C CNN
	1    5650 3100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R4
U 1 1 60568606
P 5650 2600
F 0 "R4" H 5720 2646 50  0000 L CNN
F 1 "1M" H 5720 2555 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5580 2600 50  0001 C CNN
F 3 "~" H 5650 2600 50  0001 C CNN
	1    5650 2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	5650 3250 5650 3350
Wire Wire Line
	5650 2750 5650 2850
$Comp
L Device:C C1
U 1 1 60589131
P 5150 3100
F 0 "C1" H 5265 3146 50  0000 L CNN
F 1 "100n" H 5265 3055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5188 2950 50  0001 C CNN
F 3 "~" H 5150 3100 50  0001 C CNN
	1    5150 3100
	1    0    0    -1  
$EndComp
Text GLabel 5050 2850 0    50   Output ~ 0
ADC_1
Wire Wire Line
	5150 3350 5150 3250
Wire Wire Line
	5150 2950 5150 2850
Wire Wire Line
	5050 2850 5150 2850
Wire Wire Line
	5150 2850 5650 2850
Connection ~ 5150 2850
Connection ~ 5650 2850
Wire Wire Line
	5650 2850 5650 2950
Wire Wire Line
	5150 3350 5650 3350
Connection ~ 5650 3350
Wire Wire Line
	5650 3350 5650 3450
Wire Wire Line
	5850 3350 5650 3350
Wire Wire Line
	5650 2350 5650 2450
Wire Wire Line
	5850 2350 5850 3350
$Comp
L power:GND #PWR014
U 1 1 605BC0F7
P 7450 3450
F 0 "#PWR014" H 7450 3200 50  0001 C CNN
F 1 "GND" H 7455 3277 50  0000 C CNN
F 2 "" H 7450 3450 50  0001 C CNN
F 3 "" H 7450 3450 50  0001 C CNN
	1    7450 3450
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack J5
U 1 1 605BC0FD
P 7550 2050
F 0 "J5" V 7561 2238 50  0000 L CNN
F 1 "Barrel_Jack" V 7652 2238 50  0000 L CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_Altech_AK300-2_P5.00mm" H 7600 2010 50  0001 C CNN
F 3 "~" H 7600 2010 50  0001 C CNN
	1    7550 2050
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 605BC103
P 7450 3100
F 0 "R7" H 7520 3146 50  0000 L CNN
F 1 "1M" H 7520 3055 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7380 3100 50  0001 C CNN
F 3 "~" H 7450 3100 50  0001 C CNN
	1    7450 3100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R6
U 1 1 605BC109
P 7450 2600
F 0 "R6" H 7520 2646 50  0000 L CNN
F 1 "10M" H 7520 2555 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7380 2600 50  0001 C CNN
F 3 "~" H 7450 2600 50  0001 C CNN
	1    7450 2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	7450 3250 7450 3350
Wire Wire Line
	7450 2750 7450 2850
$Comp
L Device:C C2
U 1 1 605BC111
P 6950 3100
F 0 "C2" H 7065 3146 50  0000 L CNN
F 1 "100n" H 7065 3055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6988 2950 50  0001 C CNN
F 3 "~" H 6950 3100 50  0001 C CNN
	1    6950 3100
	1    0    0    -1  
$EndComp
Text GLabel 6850 2850 0    50   Output ~ 0
ADC_2
Wire Wire Line
	6950 3350 6950 3250
Wire Wire Line
	6950 2950 6950 2850
Wire Wire Line
	6850 2850 6950 2850
Wire Wire Line
	6950 2850 7450 2850
Connection ~ 6950 2850
Connection ~ 7450 2850
Wire Wire Line
	7450 2850 7450 2950
Wire Wire Line
	6950 3350 7450 3350
Connection ~ 7450 3350
Wire Wire Line
	7450 3350 7450 3450
Wire Wire Line
	7650 3350 7450 3350
Wire Wire Line
	7450 2350 7450 2450
Wire Wire Line
	7650 2350 7650 3350
$EndSCHEMATC
