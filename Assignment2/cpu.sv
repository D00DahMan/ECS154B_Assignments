// ============================================================================
// MODULE: Memory (Single-Port RAM)
// ============================================================================
module memory_module (
    input  logic       clk,
    input  logic [7:0] addr,          // Memory address
    input  logic       write_en,      // Write enable
    input  logic [7:0] data_in,       // Data to write
    output logic [7:0] data_out       // Data read
);
    // Memory storage - exposed for testbench initialization
    logic [7:0] mem [0:255];

    // Initialize memory to 0
    initial begin
        for (int i=0; i<256; i++) mem[i] = 0;
    end

    // Asynchronous read
    assign data_out = mem[addr];

    // Synchronous write
    always_ff @(posedge clk) begin
        if (write_en) begin
            mem[addr] <= data_in;
        end
    end
endmodule

// ============================================================================
// MODULE: ALU
// ============================================================================
module alu (
    input  logic [7:0] a,
    input  logic [7:0] b,
    input  logic [1:0] func, // 00: NAND, 01: MULT, 10: SUB, 11: ADD
    output logic [7:0] result,
    output logic       zero // Indicates if result is zero
);
    // TODO: Implement ALU operations
    
endmodule

// ============================================================================
// MODULE: Control Unit
// ============================================================================
module control_unit (
    input  logic       clk,
    input  logic       reset,
    input  logic [3:0] opcode,
    input  logic       alu_zero,  // From ALU, for BEQ
    
    // Status
    output logic       halted,
    
    // Control Signals
    // TODO: Add more control signals as needed
);
    
    typedef enum logic [2:0] {
        // TODO: Define states
    } state_t;

    // TODO: Implement state machine and control signal generation

endmodule

// ============================================================================
// MODULE: CPU (Datapath Top Level)
// ============================================================================
module cpu (
    input logic clk,
    input logic reset
);

    // -- Storage --
    // Registers kept here so the testbench can access 'dut.registers'
    logic [7:0] registers [0:1];
    
    // -- Architectural Registers --
    logic [7:0] PC;
    logic [7:0] IR;

    // TODO: Define other necessary signals

    // ------------------------------------------------------------------------
    // 1. Instruction Decoding
    // ------------------------------------------------------------------------
    assign opcode = IR[7:4];
    assign ds_idx = IR[3];
    assign s_idx  = IR[2];

    assign imm_ext = // TODO
    assign off_beq = // TODO
    assign off_jmp = // TODO

    // ------------------------------------------------------------------------
    // 2. Memory Module Instantiation & Address Multiplexing
    // ------------------------------------------------------------------------
    // TODO: Define memory address logic


    // Memory module instantiation
    memory_module mem_inst (
        .clk(clk),
        .addr(mem_addr),
        .write_en(mem_write),
        .data_in(reg_r1),       // Data to write (from rds register)
        .data_out(mem_data_out) // Data read from memory
    );

    // ------------------------------------------------------------------------
    // 3. Register File Access
    // ------------------------------------------------------------------------
    assign reg_r1 = registers[ds_idx]; // Read Port 1
    assign reg_r2 = registers[s_idx];  // Read Port 2

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            registers[0] <= 0;
            registers[1] <= 0;
        end else if (reg_write) begin
            registers[ds_idx] <= wb_data;
        end
    end

    // ------------------------------------------------------------------------
    // 4. ALU & Datapath Muxes
    // ------------------------------------------------------------------------
    
    // TODO: Define ALU input multiplexing logic

    alu cpu_alu (
        .a(alu_in_a),
        .b(alu_in_b),
        .func(alu_func),
        .result(alu_result),
        .zero(alu_zero)
    );

    // ------------------------------------------------------------------------
    // 5. Control Unit Instance
    // ------------------------------------------------------------------------
    control_unit cu (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .alu_zero(alu_zero),
        .halted(halted),
        // TODO: Connect other control signals
    );

    // TODO

endmodule