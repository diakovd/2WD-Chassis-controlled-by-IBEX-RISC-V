`include "../source/defines.sv"

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
 int  rpm = 190;
 int T_EV0, T_EV1;
	
 initial #(200*(`tm_scale)) Rstn = 1; 
  
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

 always #(34*(`tm_scale)) Clk_14_7456MHz <= ~Clk_14_7456MHz; 

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

 always #(10*(`tm_scale)) Clk <= ~Clk;

 
// always #(80000*(`tm_scale)) Evnt[0] <= ~Evnt[0];
// always #(83000*(`tm_scale)) Evnt[1] <= ~Evnt[1];
 logic Mta_reg;
 int ctr_Mta = 0;
 int p_Mta = 0;
 int T_Mta  = 0;
 int tic_Mta  = 0;
 int rpm_Mta = 0;
 
 // initial begin
 
  // while(1) begin
    // #1;
	// Mta_reg <= Mta;
	// if(Mta & !Mta_reg) ctr_Mta <= 0;
	// else ctr_Mta <= ctr_Mta + 1;    
  	
	// if(!Mta & Mta_reg) p_Mta <= ctr_Mta;
	
	// if(Mta & !Mta_reg) T_Mta <= ctr_Mta;
	
	// tic_Mta = T_Mta/256;

	// if(p_Mta/tic_Mta > 66) rpm_Mta = p_Mta/tic_Mta - 66;
	// else rpm_Mta = 0;
  // end
 // end
 
 initial begin
	while(1) #((T_EV0/2)*(`tm_scale)) Evnt[0] <= ~Evnt[0];
 end

 initial begin
	while(1) #((T_EV1/2)*(`tm_scale)) Evnt[1] <= ~Evnt[1];
 end

 assign T_EV0 = 60000000/(20*rpm_Mta + 1);
 assign T_EV1 = 60000000/(20*rpm + 1);

 
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