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
    always_comb begin
        unique case (func)
            2'b11: result = a + b;
            2'b10: result = a - b;
            2'b01: result = a * b;
            2'b00: result = ~(a & b);
            default: result = 8'h00;
        endcase
        zero = (result == 8'h00);
    end
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

    // TB expects dut.halted
    logic halted;

    // ------------------------------------------------------------------------
    // Internal signals
    // ------------------------------------------------------------------------
    logic [3:0] opcode;
    logic       ds_idx;
    logic       s_idx;

    logic [7:0] imm_ext;
    logic [7:0] off_ext;

    logic [7:0] reg_r1, reg_r2;

    logic [7:0] mem_addr;
    logic       mem_write;
    logic [7:0] mem_data_out;

    logic [7:0] alu_in_a, alu_in_b;
    logic [1:0] alu_func;
    logic [7:0] alu_result;
    logic       alu_zero;

    logic [7:0] wb_data;
    logic       reg_write;

    // ------------------------------------------------------------------------
    // Simple 2-phase controller inside CPU
    // ------------------------------------------------------------------------
    typedef enum logic [0:0] { S_FETCH = 1'b0, S_EXEC = 1'b1 } state_t;
    state_t state;

    // ------------------------------------------------------------------------
    // Decode fields (from IR)
    // ------------------------------------------------------------------------
    assign opcode = IR[7:4];
    assign ds_idx = IR[3];
    assign s_idx  = IR[2];

    // B-type immediate: zero-extend 3-bit imm
    assign imm_ext = {5'b0, IR[2:0]};

    // C-type offset: sign-extend 4-bit offset
    assign off_ext = {{4{IR[3]}}, IR[3:0]};

    // Register reads (combinational)
    assign reg_r1 = registers[ds_idx]; // rds value
    assign reg_r2 = registers[s_idx];  // rs value

    // ------------------------------------------------------------------------
    // Memory address selection
    // FETCH: use PC
    // EXEC:  for addm/lw/sw use rs as address
    // ------------------------------------------------------------------------
    always_comb begin
        mem_addr  = PC;       // default
        mem_write = 1'b0;     // default

        if (!halted) begin
            if (state == S_FETCH) begin
                mem_addr = PC;
            end else begin // S_EXEC
                unique case (opcode)
                    4'h2, // addm
                    4'h6, // lw
                    4'h7: // sw
                        mem_addr = reg_r2;
                    default:
                        mem_addr = PC; // don't care
                endcase

                // sw writes on the clock edge during EXEC
                if (opcode == 4'h7) begin
                    mem_write = 1'b1;
                end
            end
        end
    end

    // Memory module instantiation (TB uses dut.mem_inst.mem[])
    memory_module mem_inst (
        .clk(clk),
        .addr(mem_addr),
        .write_en(mem_write),
        .data_in(reg_r1),       // store rds
        .data_out(mem_data_out)
    );

    // ------------------------------------------------------------------------
    // ALU control + inputs (combinational)
    // ------------------------------------------------------------------------
    assign alu_in_a = reg_r1;

    always_comb begin
        // defaults
        alu_in_b  = reg_r2;
        alu_func  = 2'b11; // ADD
        reg_write = 1'b0;
        wb_data   = alu_result; // default to ALU (assigned after ALU)

        // Only meaningful in EXEC
        if (!halted && state == S_EXEC) begin
            unique case (opcode)
                4'h0: begin // nand
                    alu_func  = 2'b00;
                    alu_in_b  = reg_r2;
                    reg_write = 1'b1;
                end
                4'h1: begin // add
                    alu_func  = 2'b11;
                    alu_in_b  = reg_r2;
                    reg_write = 1'b1;
                end
                4'h2: begin // addm: rds = rds + mem[rs]
                    alu_func  = 2'b11;
                    alu_in_b  = mem_data_out;
                    reg_write = 1'b1;
                end
                4'h3: begin // addi: rds = rds + imm
                    alu_func  = 2'b11;
                    alu_in_b  = imm_ext;
                    reg_write = 1'b1;
                end
                4'h4: begin // sub
                    alu_func  = 2'b10;
                    alu_in_b  = reg_r2;
                    reg_write = 1'b1;
                end
                4'h5: begin // mult
                    alu_func  = 2'b01;
                    alu_in_b  = reg_r2;
                    reg_write = 1'b1;
                end
                4'h6: begin // lw: rds = mem[rs]
                    // no ALU needed; write back memory data
                    reg_write = 1'b1;
                end
                4'h7: begin // sw: mem[rs] = rds
                    reg_write = 1'b0;
                end
                4'h8: begin // beq: if (rds==rs) PC = PC + offset  (PC already PC+1 from FETCH)
                    reg_write = 1'b0;
                end
                4'h9: begin // jmp: PC = PC + offset (PC already PC+1 from FETCH)
                    reg_write = 1'b0;
                end
                4'hF: begin // halt
                    reg_write = 1'b0;
                end
                default: begin
                    reg_write = 1'b0;
                end
            endcase
        end

        // writeback data selection
        if (!halted && state == S_EXEC && opcode == 4'h6) begin
            wb_data = mem_data_out;     // lw
        end else begin
            wb_data = alu_result;       // all ALU-based writes
        end
    end

    // ALU instance
    alu cpu_alu (
        .a(alu_in_a),
        .b(alu_in_b),
        .func(alu_func),
        .result(alu_result),
        .zero(alu_zero)
    );

    // ------------------------------------------------------------------------
    // Sequential: state, PC, IR, registers, halted
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            registers[0] <= 8'd0;
            registers[1] <= 8'd0;
            PC           <= 8'd0;
            IR           <= 8'd0;
            halted       <= 1'b0;
            state        <= S_FETCH;
        end else begin
            if (!halted) begin
                unique case (state)

                    // FETCH: IR <- mem[PC], PC <- PC+1
                    S_FETCH: begin
                        IR    <= mem_data_out;
                        PC    <= PC + 8'd1;
                        state <= S_EXEC;
                    end

                    // EXEC: execute instruction in IR
                    S_EXEC: begin
                        // Register write (if any)
                        if (reg_write) begin
                            registers[ds_idx] <= wb_data;
                        end

                        // Branch/jump PC update:
                        // Note: PC was already incremented in FETCH, so here we add offset to "PC+1".
                        if (opcode == 4'h8) begin // beq
                            if (reg_r1 == reg_r2) begin
                                PC <= PC + off_ext;
                            end
                        end else if (opcode == 4'h9) begin // jmp
                            PC <= PC + off_ext;
                        end

                        // Halt
                        if (opcode == 4'hF) begin
                            halted <= 1'b1;
                        end

                        state <= S_FETCH;
                    end

                    default: state <= S_FETCH;
                endcase
            end
        end
    end

endmodule
