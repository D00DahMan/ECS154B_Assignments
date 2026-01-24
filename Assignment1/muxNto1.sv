module muxNto1 #(
    parameter int N = 8,
    parameter int WIDTH = 16 
) ( 
    input logic [N-1:0][WIDTH-1:0] in_bus,
    input logic [$clog2(N)-1:0] sel,
    output logic [WIDTH-1:0] out
);
    
always_comb begin
    out = '0;
    out = in_bus[sel];
end

endmodule