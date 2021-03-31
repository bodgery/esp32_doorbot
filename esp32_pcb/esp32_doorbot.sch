EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Transistor_FET:2N7002E Q1
U 1 1 603A6F1F
P 1450 5650
F 0 "Q1" H 1654 5696 50  0000 L CNN
F 1 "2N7002E" H 1654 5605 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 1650 5575 50  0001 L CIN
F 3 "http://www.diodes.com/assets/Datasheets/ds30376.pdf" H 1450 5650 50  0001 L CNN
F 4 "ZVN4206GVTR-ND" H 1450 5650 50  0001 C CNN "Digikey #"
	1    1450 5650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 603A7B30
P 1550 5950
F 0 "#PWR0101" H 1550 5700 50  0001 C CNN
F 1 "GND" H 1555 5777 50  0000 C CNN
F 2 "" H 1550 5950 50  0001 C CNN
F 3 "" H 1550 5950 50  0001 C CNN
	1    1550 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 5850 1550 5950
Text GLabel 1100 5650 0    50   Input ~ 0
DOOR
Text GLabel 1900 4950 2    50   Input ~ 0
LATCH_IN
$Comp
L Device:D_Schottky D3
U 1 1 603A8238
P 1350 5150
F 0 "D3" V 1304 5230 50  0000 L CNN
F 1 "D_Schottky" V 1395 5230 50  0000 L CNN
F 2 "Diodes_SMD:SOD-523" H 1350 5150 50  0001 C CNN
F 3 "~" H 1350 5150 50  0001 C CNN
F 4 "31-SDM20U30Q-7TR-ND" V 1350 5150 50  0001 C CNN "Digikey #"
	1    1350 5150
	0    1    1    0   
$EndComp
Text GLabel 1900 5200 2    50   Input ~ 0
LATCH_OUT
Wire Wire Line
	1350 5300 1550 5300
Wire Wire Line
	1550 5300 1550 5450
Text GLabel 1550 4800 1    50   Input ~ 0
12V
Wire Wire Line
	1350 5000 1350 4850
Wire Wire Line
	1350 4850 1550 4850
Wire Wire Line
	1550 4850 1550 4800
Wire Wire Line
	1550 4850 1900 4850
Wire Wire Line
	1900 4850 1900 4950
Connection ~ 1550 4850
Wire Wire Line
	1900 5200 1900 5300
Wire Wire Line
	1900 5300 1550 5300
Connection ~ 1550 5300
Wire Wire Line
	1100 5650 1200 5650
$Comp
L Connector:Conn_01x02_Female J2
U 1 1 60482C20
P 1850 3700
F 0 "J2" H 1878 3676 50  0000 L CNN
F 1 "Conn_01x02_Female" H 1878 3585 50  0000 L CNN
F 2 "Terminal_Blocks:TerminalBlock_Pheonix_MKDS1.5-2pol" H 1850 3700 50  0001 C CNN
F 3 "~" H 1850 3700 50  0001 C CNN
F 4 "ED2561-ND" H 1850 3700 50  0001 C CNN "Digikey #"
	1    1850 3700
	1    0    0    -1  
$EndComp
Text GLabel 1500 3700 0    50   Input ~ 0
LATCH_IN
Text GLabel 1500 3800 0    50   Input ~ 0
LATCH_OUT
Wire Wire Line
	1500 3700 1650 3700
Wire Wire Line
	1650 3800 1500 3800
Text GLabel 4050 1150 0    50   Input ~ 0
12V
$Comp
L power:GND #PWR0104
U 1 1 605FBF90
P 5550 1150
F 0 "#PWR0104" H 5550 900 50  0001 C CNN
F 1 "GND" H 5555 977 50  0000 C CNN
F 2 "" H 5550 1150 50  0001 C CNN
F 3 "" H 5550 1150 50  0001 C CNN
	1    5550 1150
	1    0    0    -1  
$EndComp
Text GLabel 4050 1350 0    50   Input ~ 0
DOOR
Text GLabel 4050 1250 0    50   Input ~ 0
WIEGAND_OUT1
Text GLabel 4850 1350 2    50   Input ~ 0
WIEGAND_OUT2
$Comp
L Regulator_Linear:L7805 U1
U 1 1 60607032
P 4850 2950
F 0 "U1" H 4850 3192 50  0000 C CNN
F 1 "L7805" H 4850 3101 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2Lead" H 4875 2800 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 4850 2900 50  0001 C CNN
F 4 "NCV7805BDTRKGOSCT-ND" H 4850 2950 50  0001 C CNN "Digikey #"
	1    4850 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 606078C6
P 4550 3300
F 0 "C1" H 4665 3346 50  0000 L CNN
F 1 "22uF" H 4665 3255 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4588 3150 50  0001 C CNN
F 3 "~" H 4550 3300 50  0001 C CNN
F 4 "1276-1771-2-ND" H 4550 3300 50  0001 C CNN "Digikey #"
	1    4550 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 60607F53
P 5150 3300
F 0 "C2" H 5265 3346 50  0000 L CNN
F 1 "0.1uF" H 5265 3255 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5188 3150 50  0001 C CNN
F 3 "~" H 5150 3300 50  0001 C CNN
F 4 "478-1529-2-ND" H 5150 3300 50  0001 C CNN "Digkey #"
	1    5150 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 2950 4550 3150
Wire Wire Line
	5150 2950 5150 3150
$Comp
L power:GND #PWR0105
U 1 1 60608A9D
P 4850 3450
F 0 "#PWR0105" H 4850 3200 50  0001 C CNN
F 1 "GND" H 4855 3277 50  0000 C CNN
F 2 "" H 4850 3450 50  0001 C CNN
F 3 "" H 4850 3450 50  0001 C CNN
	1    4850 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 3250 4850 3450
Wire Wire Line
	4850 3450 5150 3450
Connection ~ 4850 3450
Wire Wire Line
	4850 3450 4550 3450
Text GLabel 4400 2950 0    50   Input ~ 0
12V
Text GLabel 5800 2950 2    50   Input ~ 0
5V
Wire Wire Line
	4550 2950 4400 2950
Connection ~ 4550 2950
$Comp
L Device:LED D4
U 1 1 60611FCC
P 5500 3100
F 0 "D4" V 5539 2982 50  0000 R CNN
F 1 "LED" V 5448 2982 50  0000 R CNN
F 2 "LEDs:LED-0603" H 5500 3100 50  0001 C CNN
F 3 "~" H 5500 3100 50  0001 C CNN
F 4 "SML-D12U1WT86TR-ND" V 5500 3100 50  0001 C CNN "Digikey #"
	1    5500 3100
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 6061219C
P 5500 3550
F 0 "R3" H 5570 3596 50  0000 L CNN
F 1 "1K" H 5570 3505 50  0000 L CNN
F 2 "Resistors_SMD:R_1210" V 5430 3550 50  0001 C CNN
F 3 "~" H 5500 3550 50  0001 C CNN
F 4 "2019-RK73B2ETTD102JTR-ND" H 5500 3550 50  0001 C CNN "Digikey #"
	1    5500 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3250 5500 3400
Wire Wire Line
	5150 2950 5500 2950
Connection ~ 5150 2950
Wire Wire Line
	5500 2950 5800 2950
Connection ~ 5500 2950
$Comp
L power:GND #PWR0106
U 1 1 6061699D
P 5500 3750
F 0 "#PWR0106" H 5500 3500 50  0001 C CNN
F 1 "GND" H 5505 3577 50  0000 C CNN
F 2 "" H 5500 3750 50  0001 C CNN
F 3 "" H 5500 3750 50  0001 C CNN
	1    5500 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3750 5500 3700
$Comp
L Device:LED D2
U 1 1 6061801A
P 1200 5800
F 0 "D2" V 1239 5682 50  0000 R CNN
F 1 "LED" V 1148 5682 50  0000 R CNN
F 2 "LEDs:LED-0603" H 1200 5800 50  0001 C CNN
F 3 "~" H 1200 5800 50  0001 C CNN
F 4 "SML-D12U1WT86TR-ND" V 1200 5800 50  0001 C CNN "Digikey #"
	1    1200 5800
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 6061821F
P 1200 6250
F 0 "R2" H 1270 6296 50  0000 L CNN
F 1 "1K" H 1270 6205 50  0000 L CNN
F 2 "Resistors_SMD:R_1210" V 1130 6250 50  0001 C CNN
F 3 "~" H 1200 6250 50  0001 C CNN
F 4 "2019-RK73B2ETTD102JTR-ND" H 1200 6250 50  0001 C CNN "Digikey #"
	1    1200 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 5950 1200 6100
$Comp
L power:GND #PWR0107
U 1 1 6061822A
P 1200 6450
F 0 "#PWR0107" H 1200 6200 50  0001 C CNN
F 1 "GND" H 1205 6277 50  0000 C CNN
F 2 "" H 1200 6450 50  0001 C CNN
F 3 "" H 1200 6450 50  0001 C CNN
	1    1200 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 6450 1200 6400
Connection ~ 1200 5650
Wire Wire Line
	1200 5650 1250 5650
$Comp
L Logic_LevelTranslator:74LVC2T45DC U2
U 1 1 6061B908
P 5250 5650
F 0 "U2" H 5250 5061 50  0000 C CNN
F 1 "74LVC2T45DC" H 5250 4970 50  0000 C CNN
F 2 "Tinkerforge-kicad:VSSOP8" H 5250 4800 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/74LVC_LVCH2T45.pdf" H 5500 5400 50  0001 C CNN
F 4 "1727-4578-2-ND" H 5250 5650 50  0001 C CNN "Digikey #"
	1    5250 5650
	1    0    0    -1  
$EndComp
Text GLabel 5150 4950 1    50   Input ~ 0
5V
Text GLabel 4100 2050 0    50   Input ~ 0
3.3V
Text GLabel 5350 4950 1    50   Input ~ 0
3.3V
Wire Wire Line
	5150 4950 5150 5150
Wire Wire Line
	5350 5150 5350 4950
Text GLabel 5850 5750 2    50   Input ~ 0
WIEGAND_OUT1
Text GLabel 5850 5550 2    50   Input ~ 0
WIEGAND_OUT2
Wire Wire Line
	5650 5550 5850 5550
Wire Wire Line
	5650 5750 5850 5750
Text GLabel 4650 5750 0    50   Input ~ 0
WIEGAND_IN1
Text GLabel 4650 5550 0    50   Input ~ 0
WIEGAND_IN2
Text GLabel 4650 5950 0    50   Input ~ 0
5V
Wire Wire Line
	4650 5950 4850 5950
Wire Wire Line
	4650 5750 4850 5750
Wire Wire Line
	4650 5550 4850 5550
$Comp
L power:GND #PWR0108
U 1 1 6062830E
P 5250 6400
F 0 "#PWR0108" H 5250 6150 50  0001 C CNN
F 1 "GND" H 5255 6227 50  0000 C CNN
F 2 "" H 5250 6400 50  0001 C CNN
F 3 "" H 5250 6400 50  0001 C CNN
	1    5250 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 6400 5250 6150
Text GLabel 4050 1550 0    50   Input ~ 0
READER_LED
Text GLabel 4050 1450 0    50   Input ~ 0
READER_BUZZ
$Comp
L Connector_Generic:Conn_02x10_Odd_Even J3
U 1 1 6063C556
P 4500 1650
F 0 "J3" H 4500 2300 50  0000 L CNN
F 1 "ESP32" H 4450 2200 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x10" H 4500 1650 50  0001 C CNN
F 3 "~" H 4500 1650 50  0001 C CNN
	1    4500 1650
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J4
U 1 1 60640874
P 7550 1550
F 0 "J4" H 7630 1542 50  0000 L CNN
F 1 "Conn_01x06" H 7630 1451 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 7550 1550 50  0001 C CNN
F 3 "~" H 7550 1550 50  0001 C CNN
	1    7550 1550
	1    0    0    -1  
$EndComp
Text GLabel 7150 1350 0    50   Input ~ 0
12V
Text GLabel 7150 1550 0    50   Input ~ 0
WIEGAND_IN1
Text GLabel 7150 1650 0    50   Input ~ 0
WIEGAND_IN2
Text GLabel 7150 1850 0    50   Input ~ 0
READER_LED
Text GLabel 7150 1750 0    50   Input ~ 0
READER_BUZZ
$Comp
L power:GND #PWR0109
U 1 1 60644B08
P 6400 1450
F 0 "#PWR0109" H 6400 1200 50  0001 C CNN
F 1 "GND" H 6405 1277 50  0000 C CNN
F 2 "" H 6400 1450 50  0001 C CNN
F 3 "" H 6400 1450 50  0001 C CNN
	1    6400 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1350 7350 1350
Wire Wire Line
	7150 1550 7350 1550
Wire Wire Line
	7350 1650 7150 1650
Wire Wire Line
	7150 1750 7350 1750
Text GLabel 1200 950  1    50   Input ~ 0
12V
Wire Wire Line
	1200 1850 1200 1950
$Comp
L power:GND #PWR0103
U 1 1 603B0AC4
P 1200 1950
F 0 "#PWR0103" H 1200 1700 50  0001 C CNN
F 1 "GND" H 1205 1777 50  0000 C CNN
F 2 "" H 1200 1950 50  0001 C CNN
F 3 "" H 1200 1950 50  0001 C CNN
	1    1200 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1400 1200 1550
$Comp
L Device:R R1
U 1 1 603AF7E7
P 1200 1700
F 0 "R1" H 1270 1746 50  0000 L CNN
F 1 "1K" H 1270 1655 50  0000 L CNN
F 2 "Resistors_SMD:R_1210" V 1130 1700 50  0001 C CNN
F 3 "~" H 1200 1700 50  0001 C CNN
F 4 "2019-RK73B2ETTD102JTR-ND" H 1200 1700 50  0001 C CNN "Digikey #"
	1    1200 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 603AED23
P 1200 1250
F 0 "D1" V 1239 1132 50  0000 R CNN
F 1 "LED" V 1148 1132 50  0000 R CNN
F 2 "LEDs:LED-0603" H 1200 1250 50  0001 C CNN
F 3 "~" H 1200 1250 50  0001 C CNN
F 4 "SML-D12U1WT86TR-ND" V 1200 1250 50  0001 C CNN "Digikey #"
	1    1200 1250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1200 1100 1200 950 
Wire Wire Line
	7350 1850 7150 1850
Wire Wire Line
	6400 1450 7350 1450
Wire Wire Line
	4200 1150 4050 1150
Wire Wire Line
	4050 1250 4200 1250
Wire Wire Line
	4200 1350 4050 1350
Wire Wire Line
	4050 1450 4200 1450
Wire Wire Line
	4200 1550 4050 1550
Wire Wire Line
	4100 2050 4200 2050
Wire Wire Line
	4700 1350 4850 1350
Wire Wire Line
	4700 1150 5550 1150
NoConn ~ 4700 1250
NoConn ~ 4700 1450
NoConn ~ 4700 1550
NoConn ~ 4700 1650
NoConn ~ 4700 1750
NoConn ~ 4700 1850
NoConn ~ 4700 1950
NoConn ~ 4700 2050
NoConn ~ 4200 1950
NoConn ~ 4200 1850
NoConn ~ 4200 1750
NoConn ~ 4200 1650
$EndSCHEMATC
