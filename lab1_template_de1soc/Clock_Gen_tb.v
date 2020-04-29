module Clock_Gen_tb;

reg inclk;
wire [31:0] divider=32'd2;
wire outclk;

Clock_Gen DUT (inclk,divider,outclk);

initial begin
	inclk = 1; #5;

	forever begin
	inclk = 1; #5;
	inclk = 0; #5;
	end

end

initial
begin

#200;
$stop;
end
endmodule
