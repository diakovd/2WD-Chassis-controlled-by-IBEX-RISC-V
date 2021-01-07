`include "../source/defines.sv"

`timescale 1 ps / 1 ps

// `define XilinxBoard

 module ibex_sys_tb;
 
 parameter VENDOR = "Simulation"; //optional "IntelFPGA", "Simulation", "Xilinx"
 parameter int    MEM_SIZE  = 16 * 1024; // 4 kB
 
 logic  Rstn = 0;
 logic 	TX;
 logic 	RX;

 logic [1:0] Mta;
 logic 		 ENa;
 logic [1:0] Mtb;
 logic 		 ENb;
 logic [1:0] Evnt = 0;
	
 initial #200000 Rstn = 1; 
  
`ifdef XilinxBoard
 logic  Clk_14_7456MHz = 0;

 ibex_sys_atrix7 
 #(
	.VENDOR(VENDOR),
	.MEM_SIZE(MEM_SIZE)
 )	
 ibex_sys_atrix7_inst(
	.TX(TX),
	.RX(RX),
	.Clk_14_7456MHz(Clk_14_7456MHz),
    .sys_rst_n(Rstn)	 
 );

 always #34000 Clk_14_7456MHz <= ~Clk_14_7456MHz; 

`else
 logic Clk = 0;
 
 TwoWhellPlatform 
 #(
	.VENDOR(VENDOR),
	.MEM_SIZE(MEM_SIZE)
 ) 
 ibex_sys_cycloneIV_inst(
 
	.TX(TX),
	.RX(RX),
//	output [31:0] LED,
	.Mta(Mta),
	.ENa(ENa),
	.Mtb(Mtb),
	.ENb(ENb),
	.Evnt(Evnt),


    .Clk(Clk),
    .sys_rst_n(Rstn)	 
 );

 always #10000 Clk <= ~Clk;

// always #80000000 Evnt[0] <= ~Evnt[0];
// always #83000000 Evnt[1] <= ~Evnt[1];
 
`endif

 // BootLoader_tb 
 // #(
	// .STPbyte(8'h55),
	// .ONbyte( 8'hAA),
	// .BaudRate(768) //9600
 // )  
 // BootLoader_tb_inst(
	// .TX(RX) //TX UART line
 // ); 
 
 UART_emu 	#(.BaudRate(768)) 
	UART_emu_inst(
	.TX(RX), //TX UART line
	.RX(TX)  //RX UART line
 );
 

 endmodule