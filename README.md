## 2WD Chassis controlled by IBEX RISC-V 

Can A-C4E6/E10 FPGA board be used like Ardunio?
In this project try it, using componets from Ali and soft RISC-V CPU IBEX.

Hardware:
- A-C4E6/E10 (E10) FPGA board 
- L289N motor driver board
- 2WD Chassis with whell encoders
- HC-06 bluetooth - UART module 

## Shematic
 ![Alt text](https://github.com/diakovd/2WD-Chassis-controlled-by-IBEX-RISC-V/blob/master/Schematic_Prints.jpg?raw=true "Title")

## FPGA is based on IBEX system 
(https://github.com/diakovd/ibex_system.git)
 Consyst:
- RAM for program code and data storage - 8k byte
- bus_mux module swith control signal to specific perepherial
- Timer  module count event from motor encoders
- Timer1 module generate PWM output for motors
- UART Universal Asynchronous Receiver/Transmitter for controlling speed and on/off of motors
- UART bootLoader for fast update program to RAM memory
 
## Pinout
-  CycIV-E10 connection discribed in 2WhellPlatform.qsf of quartus project derictiry 

## Folders
- source - TwoWhellPlatform.sv and perepherial modules  
- ms - contane .tcl scripts for modelsim simulation
- qua_pr - contane EP4CE10E22C8 specific RAM, FIFO, PLL and connection of the board's 
- sw - software project for CPU IBEX (Segger Embeded Studio for RISC-V). This project wait command from UART: enable Drive - ON/OFF left/right whell; set rpm Left/right - set whell speed

## Python scripts tools for convertion hex pogram files and send run commands  
1. To convert pogram in intel hex format generated by Segger Studio to:
- .mif IntelFPGA memory intialisation file use IHEXtoMIF.py 
- .hex systemverilog memory intialisation file use IHEXtoSVhex.py
- .coe Xilinx memory intialisation file use IHEXtoCOE.py
2. To reload pogram hex under power 
	run IHEXtoSVhex.py;
	set in UARTboot.py "COMx" number
	connect uart to board
	run UARTboot.py what send SVhex to IBEX system RAM memory 
3. cmd_send.py 
	Send thru UART setup speed,run,stop commands
	
## License

Unless otherwise noted, everything in this repository is covered by the Apache
License, Version 2.0 (see LICENSE for full text).
 
