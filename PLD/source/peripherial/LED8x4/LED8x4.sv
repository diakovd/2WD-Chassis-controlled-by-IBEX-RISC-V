
 module TestBoard(
	
	LED1,
	LED2,
	LED3,
	LED4,
	
	LEDA,
	LEDB,
	LEDC,
	LEDD,
	LEDE,
	LEDF,
	LEDG,
	LEDDP,
	
	Clk,
 );
 
 output	LED1;
 output	LED2;
 output	LED3;
 output	LED4;
	
 output	LEDA;
 output	LEDB;
 output	LEDC;
 output	LEDD;
 output	LEDE;
 output	LEDF;
 output	LEDG;
 output	LEDDP;
	
 input Clk;
 
 logic LED1 = 0;
 logic LED2 = 0;
 logic LED3 = 0;
 logic LED4 = 0;
 
 logic [25:0] ctr_sek = 0;
 logic [3:0] digit1 = 0;

 
 
 always@(posedge Clk) begin
	if(ctr_sek == 50000000) begin
		ctr_sek <= 0;
		if(digit1 == 9) digit1 <= 0;
		else digit1 <= digit1 + 1;
	end
	else ctr_sek <= ctr_sek + 1;
 end 
 
 always_comb begin
 
	case(digit1)
		0: begin
			LEDA <= 0;
			LEDB <= 0;
			LEDC <= 0; 
			LEDD <= 0;
			LEDE <= 0;
			LEDF <= 0;
			LEDG <= 1;	
		end
		1: begin
			LEDA <= 1;
			LEDB <= 0;
			LEDC <= 0; 
			LEDD <= 1;
			LEDE <= 1;
			LEDF <= 1;
			LEDG <= 1;	
		end
		2: begin
			LEDA <= 0;
			LEDB <= 0;
			LEDC <= 1; 
			LEDD <= 0;
			LEDE <= 0;
			LEDF <= 1;
			LEDG <= 0;	
		end
		3: begin
			LEDA <= 0;
			LEDB <= 0;
			LEDC <= 0; 
			LEDD <= 0;
			LEDE <= 1;
			LEDF <= 1;
			LEDG <= 0;	
		end
		4: begin
			LEDA <= 1;
			LEDB <= 0;
			LEDC <= 0; 
			LEDD <= 1;
			LEDE <= 1;
			LEDF <= 0;
			LEDG <= 0;	
		end
		5: begin
			LEDA <= 0;
			LEDB <= 1;
			LEDC <= 0; 
			LEDD <= 0;
			LEDE <= 1;
			LEDF <= 0;
			LEDG <= 0;	
		end
		6: begin
			LEDA <= 0;
			LEDB <= 1;
			LEDC <= 0; 
			LEDD <= 0;
			LEDE <= 0;
			LEDF <= 0;
			LEDG <= 0;	
		end
		7: begin
			LEDA <= 0;
			LEDB <= 0;
			LEDC <= 0; 
			LEDD <= 1;
			LEDE <= 1;
			LEDF <= 1;
			LEDG <= 1;	
		end
		8: begin
			LEDA <= 0;
			LEDB <= 0;
			LEDC <= 0; 
			LEDD <= 0;
			LEDE <= 0;
			LEDF <= 0;
			LEDG <= 0;	
		end
		9: begin
			LEDA <= 0;
			LEDB <= 0;
			LEDC <= 0; 
			LEDD <= 0;
			LEDE <= 1;
			LEDF <= 0;
			LEDG <= 0;	
		end
		default: begin
			LEDA <= 1;
			LEDB <= 1;
			LEDC <= 1; 
			LEDD <= 1;
			LEDE <= 1;
			LEDF <= 1;
			LEDG <= 1;	
		end
	endcase
 end
 endmodule 