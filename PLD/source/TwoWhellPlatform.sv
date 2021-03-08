`include "../source/defines.sv"

 module TwoWhellPlatform(
 
	output TX,
	input  RX,

	output TXble,
	input  RXble,
	output ENble,
	
	input  SWuart,
	
	output [1:0] Mta,
	output 		 ENa,
	output [1:0] Mtb,
	output 		 ENb,
	input  [1:0] Evnt,

	output [7:0] LEDen,
	output	LEDA,
	output	LEDB,
	output	LEDC,
	output	LEDD,
	output	LEDE,
	output	LEDF,
	output	LEDG,
	output	LEDDP,

	input Btn,

    input Clk,
    input sys_rst_n	 
 );
 parameter VENDOR    = "IntelFPGA"; //optional "Simulation" 
 parameter int    MEM_SIZE  = 16 * 1024; // 8 kB
 
 logic      Clk_14_7456MHz;
 logic 		Clk_sys;
 wire       Rstn;

 wire [31:0] IO;
 logic [1:0] PWM;
 
 logic TXs;
 logic RXs;
 logic STap_clk;
 
generate

if(VENDOR == "IntelFPGA") begin

 PLL PLL_inst (
	.areset ( 1'b0),
	.inclk0 ( Clk ),
	.c0 ( Clk_14_7456MHz ),
	.c1 ( Clk_sys ),	
	.c2 ( STap_clk )	
	);

end
else if(VENDOR == "Simulation") begin
 
 initial Clk_14_7456MHz = 0;
 always #(34*(`tm_scale)) Clk_14_7456MHz <= ~Clk_14_7456MHz; 
 
 assign Clk_sys = Clk;

end
endgenerate

 ibex_sys #(
	.VENDOR(VENDOR),
	.MEM_SIZE(MEM_SIZE)
 )
 ibex_sys_inst(
    .TX(TXs),
    .RX(RXs),
	
	.IO(IO),
	
	.LEDen(LEDen),
	.LEDA(LEDA),
	.LEDB(LEDB),
	.LEDC(LEDC),
	.LEDD(LEDD),
	.LEDE(LEDE),
	.LEDF(LEDF),
	.LEDG(LEDG),
	.LEDDP(LEDDP),	
	
	.PWM(PWM),
	.Evnt(Evnt),
	
	.rst_sys_n(sys_rst_n),
	.Clk_14_7456MHz(Clk_14_7456MHz),
	.clk_sys(Clk_sys)
 );
 
 assign TX    = (SWuart)? 1'b1 : TXs; 
 assign TXble = (SWuart)? TXs  : 1'b1; 
 assign RXs   = (SWuart)? RXble: RX;
 assign ENble = (SWuart)? 1'b1 : 1'b0;

 assign Mta =(IO[0])? {1'b0,1'b1} : {1'b1,1'b0};
 assign ENa = (!Btn | IO[2]) & PWM[0];
 assign Mtb =(IO[1])? {1'b1,1'b0} : {1'b0,1'b1};
 assign ENb = (!Btn | IO[3]) & PWM[1];


 endmodule