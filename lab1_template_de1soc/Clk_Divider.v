module Clk_Divider(inclk, outclk, freqCount); 
	input inclk;  
	input [31:0] freqCount; 
	output reg outclk = 0; 
	reg[31:0] count;
	
	always@(posedge inclk) 
	begin 
		
		if (count <freqCount) 
			count <= count + 1;
			
		else
			begin 
				outclk <= ~outclk; 
				count <= 0;  
			end 
	end
endmodule
