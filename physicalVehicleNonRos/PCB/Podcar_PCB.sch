EESchema Schematic File Version 4
LIBS:Podcar_PCB-cache
EELAYER 26 0
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
L Device:R R1
U 1 1 60F4AC2A
P 7050 2260
F 0 "R1" H 7120 2306 50  0000 L CNN
F 1 "10K" H 7120 2215 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 6980 2260 50  0001 C CNN
F 3 "" H 7050 2260 50  0001 C CNN
	1    7050 2260
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60F4ACA8
P 7430 2260
F 0 "R2" H 7500 2306 50  0000 L CNN
F 1 "10K" H 7500 2215 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 7360 2260 50  0001 C CNN
F 3 "" H 7430 2260 50  0001 C CNN
	1    7430 2260
	1    0    0    -1  
$EndComp
Wire Wire Line
	6810 1920 6810 900 
Wire Wire Line
	6810 900  6960 900 
Wire Wire Line
	6960 900  6960 1040
Wire Wire Line
	6270 5600 6270 5760
Wire Wire Line
	6270 5760 7250 5760
Wire Wire Line
	7250 5760 7250 2410
Wire Wire Line
	7430 2110 7430 1920
Wire Wire Line
	7430 1920 6810 1920
NoConn ~ 6770 5600
NoConn ~ 6670 5600
NoConn ~ 6570 5600
NoConn ~ 6470 5600
NoConn ~ 6370 5600
NoConn ~ 6000 5580
NoConn ~ 5500 5580
NoConn ~ 5280 5580
NoConn ~ 5180 5580
NoConn ~ 6840 4360
NoConn ~ 6910 4360
NoConn ~ 6910 4490
NoConn ~ 6840 4490
Connection ~ 7250 2410
Wire Wire Line
	7250 2410 7050 2410
Wire Wire Line
	7430 2410 7250 2410
$Comp
L pololu_jrk_v3:pololu_jrk_v3 U2
U 1 1 614D4638
P 3400 2020
F 0 "U2" H 3085 1525 60  0000 C CNN
F 1 "pololu_jrk_v3" H 3778 1990 60  0000 L CNN
F 2 "adafruit-mcp4725-1.snapshot:pololu_jrk21v3" H 3425 1535 60  0001 C CNN
F 3 "" H 3425 1535 60  0000 C CNN
	1    3400 2020
	1    0    0    -1  
$EndComp
$Comp
L dac_mcp4725:DAC_MCP4725 U1
U 1 1 615125F6
P 2280 3860
F 0 "U1" H 1930 4210 60  0001 C CNN
F 1 "DAC_MCP4725" H 2557 3865 60  0000 L CNN
F 2 "adafruit-mcp4725-1.snapshot:adafruit_mcp4725-fp" H 2930 4760 60  0001 C CNN
F 3 "" H 2930 4760 60  0000 C CNN
	1    2280 3860
	1    0    0    -1  
$EndComp
Wire Wire Line
	4710 2960 1740 2960
Wire Wire Line
	1740 3810 1930 3810
Wire Wire Line
	4710 2960 4710 3090
Wire Wire Line
	1740 2960 1740 3810
Wire Wire Line
	4810 3090 4810 2880
Wire Wire Line
	4810 2880 1640 2880
Wire Wire Line
	1640 2880 1640 3910
Wire Wire Line
	1640 3910 1930 3910
NoConn ~ 1930 3610
NoConn ~ 1930 3710
NoConn ~ 5900 5580
Wire Wire Line
	5800 5580 5800 5700
Wire Wire Line
	5800 5700 3265 5700
Wire Wire Line
	1750 5700 1750 4110
Wire Wire Line
	1750 4110 1930 4110
NoConn ~ 3700 2090
NoConn ~ 3620 2090
NoConn ~ 3540 2090
NoConn ~ 3540 2150
NoConn ~ 3620 2150
NoConn ~ 3700 2150
Wire Wire Line
	5260 1440 3470 1440
Wire Wire Line
	3470 1440 3470 1790
NoConn ~ 3580 1790
NoConn ~ 3630 1790
$Comp
L arduino:Arduino_Uno_Shield XA1
U 1 1 60F49512
P 5920 4360
F 0 "XA1" H 5770 4490 60  0000 C CNN
F 1 "Arduino_Uno_Shield" H 5240 4470 60  0000 L CNN
F 2 "Arduino:Arduino_Uno_Shield" H 7720 8110 60  0001 C CNN
F 3 "" H 7720 8110 60  0001 C CNN
	1    5920 4360
	1    0    0    -1  
$EndComp
NoConn ~ 5600 5580
NoConn ~ 6740 3090
NoConn ~ 6640 3090
NoConn ~ 6520 3080
NoConn ~ 6420 3080
NoConn ~ 6320 3080
NoConn ~ 6220 3080
NoConn ~ 6120 3080
NoConn ~ 6020 3080
NoConn ~ 5710 3100
NoConn ~ 5610 3100
NoConn ~ 5510 3100
NoConn ~ 5410 3100
NoConn ~ 5310 3100
NoConn ~ 5210 3100
NoConn ~ 5090 3090
NoConn ~ 4990 3090
Wire Wire Line
	5700 5580 5700 5840
Wire Wire Line
	5700 5840 1640 5840
Wire Wire Line
	1640 5840 1640 4010
Wire Wire Line
	1640 4010 1930 4010
NoConn ~ 7980 1040
NoConn ~ 8030 1040
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 615FF63F
P 7800 1620
F 0 "J2" H 7880 1612 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 7880 1521 50  0000 L CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 7800 1620 50  0001 C CNN
F 3 "~" H 7800 1620 50  0001 C CNN
	1    7800 1620
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 615FF6DF
P 5360 1740
F 0 "J1" V 5233 1820 50  0000 L CNN
F 1 "Screw_Terminal_01x02" V 5324 1820 50  0000 L CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 5360 1740 50  0001 C CNN
F 3 "~" H 5360 1740 50  0001 C CNN
	1    5360 1740
	0    1    1    0   
$EndComp
Wire Wire Line
	7930 1040 7930 900 
Wire Wire Line
	7930 900  7550 900 
Wire Wire Line
	7550 900  7550 1620
Wire Wire Line
	7880 1040 7880 950 
Wire Wire Line
	7880 950  7580 950 
Wire Wire Line
	7580 950  7580 1720
Wire Wire Line
	7010 1040 7010 940 
Wire Wire Line
	7010 940  6870 940 
Wire Wire Line
	6870 940  6870 1620
Wire Wire Line
	6870 1620 7050 1620
Connection ~ 7550 1620
Wire Wire Line
	7550 1620 7600 1620
Wire Wire Line
	7050 2110 7050 1620
Connection ~ 7050 1620
Wire Wire Line
	7050 1620 7550 1620
Wire Wire Line
	7430 1920 7430 1720
Wire Wire Line
	7430 1720 7580 1720
Connection ~ 7430 1920
Connection ~ 7580 1720
Wire Wire Line
	7580 1720 7600 1720
Wire Wire Line
	7060 1040 7060 850 
Wire Wire Line
	7060 850  5360 850 
Wire Wire Line
	5360 850  5360 1505
Wire Wire Line
	7110 1040 7110 810 
Wire Wire Line
	7110 810  5260 810 
Wire Wire Line
	5260 810  5260 1440
Connection ~ 5260 1440
Wire Wire Line
	5260 1440 5260 1540
$Comp
L vlp-16_control_unit:vlp-16 U3
U 1 1 6162226B
P 8230 3700
F 0 "U3" H 9479 3309 60  0000 L CNN
F 1 "vlp-16" H 9479 3203 60  0000 L CNN
F 2 "adafruit-mcp4725-1.snapshot:VLP16" H 8230 3700 60  0001 C CNN
F 3 "" H 8230 3700 60  0000 C CNN
	1    8230 3700
	1    0    0    -1  
$EndComp
$Comp
L lcd_display:LCD_Display U4
U 1 1 61623775
P 3710 6400
F 0 "U4" H 3485 6700 60  0001 C CNN
F 1 "LCD_Display" H 4213 6380 60  0000 L CNN
F 2 "adafruit-mcp4725-1.snapshot:LCD_Display" H 3785 6200 60  0001 C CNN
F 3 "" H 3785 6200 60  0000 C CNN
	1    3710 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3180 4230 3180 6425
Wire Wire Line
	3180 6425 3285 6425
Wire Wire Line
	3285 6300 3265 6300
Wire Wire Line
	3265 6300 3265 5700
Connection ~ 3265 5700
Wire Wire Line
	3265 5700 1750 5700
Wire Wire Line
	3180 4230 6910 4230
NoConn ~ 6840 4230
NoConn ~ 3285 6550
Wire Wire Line
	3530 1790 3530 1510
Wire Wire Line
	3530 1510 5360 1505
Connection ~ 5360 1505
Wire Wire Line
	5360 1505 5360 1540
$Comp
L Podcar_PCB-rescue:PSU_12-psu_12 U5
U 1 1 615EFBCA
P 6710 990
F 0 "U5" V 7485 505 60  0001 C CNN
F 1 "PSU_12" H 6975 440 60  0000 L CNN
F 2 "adafruit-mcp4725-1.snapshot:dc_buck_converter-fp" H 7160 690 60  0001 C CNN
F 3 "" H 7160 690 60  0000 C CNN
	1    6710 990 
	1    0    0    -1  
$EndComp
$Comp
L Podcar_PCB-rescue:PSU_16-psu_16 U6
U 1 1 615F433E
P 8100 1140
F 0 "U6" V 8410 825 60  0001 C CNN
F 1 "PSU_16" H 7935 745 60  0000 L CNN
F 2 "adafruit-mcp4725-1.snapshot:dc_buck_converter-fp" H 8080 990 60  0001 C CNN
F 3 "" H 8080 990 60  0000 C CNN
	1    8100 1140
	1    0    0    -1  
$EndComp
$EndSCHEMATC
