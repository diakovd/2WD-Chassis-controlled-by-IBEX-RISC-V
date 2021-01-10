`include "../source/defines.sv"

 module TwoWhellPlatform(
 
	output TX,
	input  RX,
	
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
 
generate

if(VENDOR == "IntelFPGA") begin

 PLL PLL_inst (
	.areset ( 1'b0),
	.inclk0 ( Clk ),
	.c0 ( Clk_14_7456MHz ),
	.c1 ( Clk_sys )	
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
    .TX(TX),
    .RX(RX),
	
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

 assign Mta =(IO[0])? {!PWM[0],PWM[0]} : {PWM[0],!PWM[0]};
 assign ENa = !Btn | IO[2];
 assign Mtb =(IO[1])? {PWM[1],!PWM[1]} : {!PWM[1],PWM[1]};
 assign ENb = !Btn | IO[3];


 endmodule