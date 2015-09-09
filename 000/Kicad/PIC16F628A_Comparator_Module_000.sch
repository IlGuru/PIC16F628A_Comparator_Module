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
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "PIC16F628A Comparator Module"
Date "2015-09-09"
Rev ""
Comp "Stefano Busnelli"
Comment1 "Prova 000"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L PIC16f628a U1
U 1 1 55F05546
P 3600 2350
F 0 "U1" H 4150 1900 60  0000 C CNN
F 1 "PIC16f628a" H 3000 1900 60  0000 C CNN
F 2 "" H 3600 2350 60  0000 C CNN
F 3 "" H 3600 2350 60  0000 C CNN
	1    3600 2350
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR9
U 1 1 55F0560F
P 3600 3100
F 0 "#PWR9" H 3600 2950 50  0001 C CNN
F 1 "VSS" H 3600 3250 50  0000 C CNN
F 2 "" H 3600 3100 60  0000 C CNN
F 3 "" H 3600 3100 60  0000 C CNN
	1    3600 3100
	-1   0    0    1   
$EndComp
$Comp
L VDD #PWR8
U 1 1 55F05625
P 3600 1600
F 0 "#PWR8" H 3600 1450 50  0001 C CNN
F 1 "VDD" H 3600 1750 50  0000 C CNN
F 2 "" H 3600 1600 60  0000 C CNN
F 3 "" H 3600 1600 60  0000 C CNN
	1    3600 1600
	1    0    0    -1  
$EndComp
$Comp
L POT RV1
U 1 1 55F0563B
P 1100 1350
F 0 "RV1" H 1100 1250 50  0000 C CNN
F 1 "10K" H 1100 1350 50  0000 C CNN
F 2 "" H 1100 1350 60  0000 C CNN
F 3 "" H 1100 1350 60  0000 C CNN
	1    1100 1350
	0    1    1    0   
$EndComp
$Comp
L POT RV2
U 1 1 55F05670
P 1750 1350
F 0 "RV2" H 1750 1250 50  0000 C CNN
F 1 "10K" H 1750 1350 50  0000 C CNN
F 2 "" H 1750 1350 60  0000 C CNN
F 3 "" H 1750 1350 60  0000 C CNN
	1    1750 1350
	0    1    1    0   
$EndComp
$Comp
L VDD #PWR1
U 1 1 55F05693
P 1100 1100
F 0 "#PWR1" H 1100 950 50  0001 C CNN
F 1 "VDD" H 1100 1250 50  0000 C CNN
F 2 "" H 1100 1100 60  0000 C CNN
F 3 "" H 1100 1100 60  0000 C CNN
	1    1100 1100
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR4
U 1 1 55F056AD
P 1750 1100
F 0 "#PWR4" H 1750 950 50  0001 C CNN
F 1 "VDD" H 1750 1250 50  0000 C CNN
F 2 "" H 1750 1100 60  0000 C CNN
F 3 "" H 1750 1100 60  0000 C CNN
	1    1750 1100
	1    0    0    -1  
$EndComp
$Comp
L VSS #PWR2
U 1 1 55F056D4
P 1100 1600
F 0 "#PWR2" H 1100 1450 50  0001 C CNN
F 1 "VSS" H 1100 1750 50  0000 C CNN
F 2 "" H 1100 1600 60  0000 C CNN
F 3 "" H 1100 1600 60  0000 C CNN
	1    1100 1600
	-1   0    0    1   
$EndComp
$Comp
L VSS #PWR5
U 1 1 55F056E8
P 1750 1600
F 0 "#PWR5" H 1750 1450 50  0001 C CNN
F 1 "VSS" H 1750 1750 50  0000 C CNN
F 2 "" H 1750 1600 60  0000 C CNN
F 3 "" H 1750 1600 60  0000 C CNN
	1    1750 1600
	-1   0    0    1   
$EndComp
Text GLabel 1250 1350 2    45   Input ~ 0
AN0
Text GLabel 2500 1900 0    45   Input ~ 0
AN0
Text GLabel 2500 2000 0    45   Input ~ 0
AN1
Text GLabel 1900 1350 2    45   Input ~ 0
AN1
Text GLabel 2500 2400 0    45   Input ~ 0
RESET
$Comp
L R R3
U 1 1 55F05743
P 2400 1250
F 0 "R3" V 2480 1250 50  0000 C CNN
F 1 "10K" V 2400 1250 50  0000 C CNN
F 2 "" V 2330 1250 30  0000 C CNN
F 3 "" H 2400 1250 30  0000 C CNN
	1    2400 1250
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR7
U 1 1 55F05786
P 2400 1100
F 0 "#PWR7" H 2400 950 50  0001 C CNN
F 1 "VDD" H 2400 1250 50  0000 C CNN
F 2 "" H 2400 1100 60  0000 C CNN
F 3 "" H 2400 1100 60  0000 C CNN
	1    2400 1100
	1    0    0    -1  
$EndComp
Text GLabel 2250 1600 0    45   Input ~ 0
RESET
Wire Wire Line
	2250 1600 2400 1600
Wire Wire Line
	2400 1600 2400 1400
$Comp
L R R1
U 1 1 55F05800
P 1200 2350
F 0 "R1" V 1280 2350 50  0000 C CNN
F 1 "330" V 1200 2350 50  0000 C CNN
F 2 "" V 1130 2350 30  0000 C CNN
F 3 "" H 1200 2350 30  0000 C CNN
	1    1200 2350
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 55F05830
P 1850 2350
F 0 "R2" V 1930 2350 50  0000 C CNN
F 1 "330" V 1850 2350 50  0000 C CNN
F 2 "" V 1780 2350 30  0000 C CNN
F 3 "" H 1850 2350 30  0000 C CNN
	1    1850 2350
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 55F05874
P 1200 2700
F 0 "D1" H 1200 2800 50  0000 C CNN
F 1 "LED" H 1200 2600 50  0000 C CNN
F 2 "" H 1200 2700 60  0000 C CNN
F 3 "" H 1200 2700 60  0000 C CNN
	1    1200 2700
	0    -1   -1   0   
$EndComp
$Comp
L LED D2
U 1 1 55F058CA
P 1850 2700
F 0 "D2" H 1850 2800 50  0000 C CNN
F 1 "LED" H 1850 2600 50  0000 C CNN
F 2 "" H 1850 2700 60  0000 C CNN
F 3 "" H 1850 2700 60  0000 C CNN
	1    1850 2700
	0    -1   -1   0   
$EndComp
$Comp
L VSS #PWR3
U 1 1 55F05908
P 1200 2900
F 0 "#PWR3" H 1200 2750 50  0001 C CNN
F 1 "VSS" H 1200 3050 50  0000 C CNN
F 2 "" H 1200 2900 60  0000 C CNN
F 3 "" H 1200 2900 60  0000 C CNN
	1    1200 2900
	-1   0    0    1   
$EndComp
$Comp
L VSS #PWR6
U 1 1 55F0592B
P 1850 2900
F 0 "#PWR6" H 1850 2750 50  0001 C CNN
F 1 "VSS" H 1850 3050 50  0000 C CNN
F 2 "" H 1850 2900 60  0000 C CNN
F 3 "" H 1850 2900 60  0000 C CNN
	1    1850 2900
	-1   0    0    1   
$EndComp
Text GLabel 1100 2100 0    45   Input ~ 0
C1VOUT
Text GLabel 2500 2500 0    45   Input ~ 0
C1VOUT
Text GLabel 1750 2100 0    45   Input ~ 0
C2VOUT
Text GLabel 2500 2600 0    45   Input ~ 0
C2VOUT
Wire Wire Line
	1100 2100 1200 2100
Wire Wire Line
	1200 2100 1200 2200
Wire Wire Line
	1750 2100 1850 2100
Wire Wire Line
	1850 2100 1850 2200
Text Notes 1150 2000 0    45   ~ 0
OPZIONALE
Wire Notes Line
	750  1900 2050 1900
Wire Notes Line
	2050 1900 2050 3150
Wire Notes Line
	2050 3150 750  3150
Wire Notes Line
	750  3150 750  1900
$EndSCHEMATC