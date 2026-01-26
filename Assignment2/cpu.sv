module cpu (
    input logic clk,
    input logic reset
);

    // -- Registers & Memory --
    logic [7:0] PC;
    logic [7:0] IR;                  // Instruction Register
    logic [7:0] registers [0:1];     // Register File (r0, r1)
    logic [7:0] memory [0:255];      

    // -- Internal Signals --
    logic halted;                    // Status signal for testbench
        // TODO
            

endmodule