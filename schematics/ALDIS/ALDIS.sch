EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "ALDIS"
Date "2021-11-29"
Rev "1"
Comp "FK - Øyvind Skaaden"
Comment1 "Createtd for UKErevyen 2021"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 61A4D8E8
P 6400 4150
F 0 "A1" H 6850 5200 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 7150 5100 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 6400 4150 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 6400 4150 50  0001 C CNN
	1    6400 4150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 61A50B1B
P 3600 3250
F 0 "J1" H 3650 2950 50  0000 R CNN
F 1 "Conn_01x04_Male" H 3950 3450 50  0000 R CNN
F 2 "" H 3600 3250 50  0001 C CNN
F 3 "~" H 3600 3250 50  0001 C CNN
	1    3600 3250
	1    0    0    1   
$EndComp
Wire Wire Line
	4650 4850 4850 4850
Wire Wire Line
	4650 4650 4850 4650
Wire Wire Line
	4650 4450 4850 4450
$Comp
L Device:R R3
U 1 1 61A628D8
P 5000 4850
F 0 "R3" V 4900 4850 50  0000 C CNN
F 1 "220" V 5000 4850 50  0000 C CNN
F 2 "" V 4930 4850 50  0001 C CNN
F 3 "~" H 5000 4850 50  0001 C CNN
	1    5000 4850
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 61A6268D
P 5000 4650
F 0 "R2" V 4900 4650 50  0000 C CNN
F 1 "150" V 5000 4650 50  0000 C CNN
F 2 "" V 4930 4650 50  0001 C CNN
F 3 "~" H 5000 4650 50  0001 C CNN
	1    5000 4650
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 61A62276
P 5000 4450
F 0 "R1" V 4900 4450 50  0000 C CNN
F 1 "150" V 5000 4450 50  0000 C CNN
F 2 "" V 4930 4450 50  0001 C CNN
F 3 "~" H 5000 4450 50  0001 C CNN
	1    5000 4450
	0    1    1    0   
$EndComp
$Comp
L Device:LED_BGCR D1
U 1 1 61A52B59
P 4450 4650
F 0 "D1" H 4450 4300 50  0000 C CNN
F 1 "LED_BGCR" H 4450 5050 50  0000 C CNN
F 2 "" H 4450 4600 50  0001 C CNN
F 3 "~" H 4450 4600 50  0001 C CNN
	1    4450 4650
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 61A80DF5
P 4100 4800
F 0 "#PWR01" H 4100 4550 50  0001 C CNN
F 1 "GND" H 4105 4627 50  0000 C CNN
F 2 "" H 4100 4800 50  0001 C CNN
F 3 "" H 4100 4800 50  0001 C CNN
	1    4100 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4800 4100 4650
Wire Wire Line
	4100 4650 4250 4650
$Comp
L power:GND #PWR04
U 1 1 61A815C5
P 6500 5350
F 0 "#PWR04" H 6500 5100 50  0001 C CNN
F 1 "GND" H 6505 5177 50  0000 C CNN
F 2 "" H 6500 5350 50  0001 C CNN
F 3 "" H 6500 5350 50  0001 C CNN
	1    6500 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 5150 6400 5250
Wire Wire Line
	6400 5250 6500 5250
Wire Wire Line
	6500 5250 6500 5150
Wire Wire Line
	6500 5250 6500 5350
Connection ~ 6500 5250
$Comp
L power:GND #PWR03
U 1 1 61A8578E
P 4750 3550
F 0 "#PWR03" H 4750 3300 50  0001 C CNN
F 1 "GND" H 4755 3377 50  0000 C CNN
F 2 "" H 4750 3550 50  0001 C CNN
F 3 "" H 4750 3550 50  0001 C CNN
	1    4750 3550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 61A4F4C6
P 5000 3500
F 0 "SW1" H 5000 3785 50  0000 C CNN
F 1 "SW_Push" H 5000 3694 50  0000 C CNN
F 2 "" H 5000 3700 50  0001 C CNN
F 3 "~" H 5000 3700 50  0001 C CNN
	1    5000 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 3550 4750 3500
Wire Wire Line
	4750 3500 4800 3500
Wire Wire Line
	5550 3750 5900 3750
Wire Wire Line
	5550 3750 5550 3500
Wire Wire Line
	5550 3500 5200 3500
Wire Wire Line
	3800 3050 6300 3050
Wire Wire Line
	6300 3050 6300 3150
Wire Wire Line
	3800 3150 4400 3150
Wire Wire Line
	4400 3150 4400 4050
Wire Wire Line
	4400 4050 5900 4050
Wire Wire Line
	5900 4150 4300 4150
Wire Wire Line
	4300 4150 4300 3250
Wire Wire Line
	4300 3250 3800 3250
Wire Wire Line
	5150 4450 5900 4450
Wire Wire Line
	5150 4650 5150 4550
Wire Wire Line
	5150 4550 5900 4550
Wire Wire Line
	5150 4850 5250 4850
Wire Wire Line
	5250 4850 5250 4650
Wire Wire Line
	5250 4650 5900 4650
$Comp
L power:GND #PWR02
U 1 1 61AAD0F2
P 4200 3450
F 0 "#PWR02" H 4200 3200 50  0001 C CNN
F 1 "GND" H 4205 3277 50  0000 C CNN
F 2 "" H 4200 3450 50  0001 C CNN
F 3 "" H 4200 3450 50  0001 C CNN
	1    4200 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 3350 4200 3350
Wire Wire Line
	4200 3350 4200 3450
Text Notes 3550 3100 2    50   ~ 0
+5V
Text Notes 3550 3200 2    50   ~ 0
RX
Text Notes 3550 3300 2    50   ~ 0
TX
Text Notes 3550 3400 2    50   ~ 0
GND\n
Text Notes 7000 6850 0    197  ~ 0
ALDIS - Main Schematic
$EndSCHEMATC
