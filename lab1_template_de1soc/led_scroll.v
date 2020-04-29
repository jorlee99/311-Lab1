module led_scroll(ledspeed,LED[7:0]);
reg [8:0] ledstates;

parameter sled0 = 8'b00000001;
parameter sled1 = 8'b00000010;
parameter sled2 = 8'b00000100;
parameter sled3 = 8'b00001000;
parameter sled4 = 8'b00010000;
parameter sled5 = 8'b00100000;
parameter sled6 = 8'b01000000;
parameter sled7 = 8'b10000000;

input wire ledspeed;

output reg [7:0] LED;

//LEDS SCROLLING
always @(posedge ledspeed)
begin

case (ledstates)
//flag_leds
9'b0: ledstates = {1'b0,sled1};
9'b0_00000001: ledstates = {1'b0,sled1};
9'b0_00000010: ledstates = {1'b0,sled2};
9'b0_00000100: ledstates = {1'b0,sled3};
9'b0_00001000: ledstates = {1'b0,sled4};
9'b0_00010000: ledstates = {1'b0,sled5};
9'b0_00100000: ledstates = {1'b0,sled6};
9'b0_01000000: ledstates = {1'b1,sled7};

9'b1_00000001: ledstates = {1'b0,sled0};
9'b1_00000010: ledstates = {1'b1,sled0};
9'b1_00000100: ledstates = {1'b1,sled1};
9'b1_00001000: ledstates = {1'b1,sled2};
9'b1_00010000: ledstates = {1'b1,sled3};
9'b1_00100000: ledstates = {1'b1,sled4};
9'b1_01000000: ledstates = {1'b1,sled5};
9'b1_10000000: ledstates = {1'b1,sled6};
default: ledstates = 9'b0_00000001;
endcase

case(ledstates)
9'b0_00000001: LED[7:0] = sled0;
9'b0_00000010: LED[7:0] = sled1;
9'b0_00000100: LED[7:0] = sled2;
9'b0_00001000: LED[7:0] = sled3;
9'b0_00010000: LED[7:0] = sled4;
9'b0_00100000: LED[7:0] = sled5;
9'b0_01000000: LED[7:0] = sled6;
9'b0_10000000: LED[7:0] = sled7;

9'b1_00000001: LED[7:0] = sled0;
9'b1_00000010: LED[7:0] = sled1;
9'b1_00000100: LED[7:0] = sled2;
9'b1_00001000: LED[7:0] = sled3;
9'b1_00010000: LED[7:0] = sled4;
9'b1_00100000: LED[7:0] = sled5;
9'b1_01000000: LED[7:0] = sled6;
9'b1_10000000: LED[7:0] = sled7;
default: LED[7:0] = sled0;

endcase

end

endmodule
