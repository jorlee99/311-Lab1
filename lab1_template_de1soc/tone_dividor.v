module tone_dividor (tone_tog,clock_dividor);
input [2:0] tone_tog;
reg [31:0] tone_test;
output reg [31:0] clock_dividor;

parameter tonedo = 32'h20B;
parameter tonere = 32'd587;
parameter tonemi = 32'd659;
parameter tonefa = 32'd698;
parameter toneso = 32'd783;
parameter tonela = 32'd880;
parameter tonesi = 32'd987;
parameter tonedo2 = 32'd1046;

always@(*)begin
case (tone_tog)

3'b000: tone_test = tonedo;
3'b001: tone_test = tonere;
3'b010: tone_test = tonemi;
3'b011: tone_test = tonefa;
3'b100: tone_test = toneso;
3'b101: tone_test = tonela;
3'b110: tone_test = tonesi;
3'b111: tone_test = tonedo2;

default: tone_test = tonedo;

endcase
end

always@(*) 
begin

case (tone_test)

tonedo: clock_dividor = (32'd47801);
tonere: clock_dividor = (32'd42590);
tonemi: clock_dividor = (32'd37937);
tonefa: clock_dividor = (32'd35817);
toneso: clock_dividor = (32'd31929);
tonela: clock_dividor = (32'd28409);
tonesi: clock_dividor = (32'd25330); 
tonedo2: clock_dividor = (32'd23901);

default: clock_dividor=32'b1;
endcase  
end    

endmodule
