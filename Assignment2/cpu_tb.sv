module cpu_tb;

    // Testbench signals
    logic clk;
    logic reset;

    // Instantiate the CPU
    cpu dut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 10ns period
    end

    initial begin
        $dumpfile("cpu.vcd");
        $dumpvars(0, cpu_tb);
    end

    // Test program
    initial begin
        $display("Starting CPU test...");

        // Reset the CPU
        reset = 1;
        #10;
        reset = 0;
        #5;  // Wait for reset to take effect

        // Load test program into memory
        // Test program from readme:
        // addi r0, 5      # r0 = 5
        // addi r1, 7      # r1 = 7
        // add  r0, r1     # r0 = r0 + r1 = 12
        // lw   r1, r0     # r1 = mem[r0] = mem[12] = 12
        // beq  1          # if (r0 == r1) skip next instruction
        // halt            # should be skipped
        // addm r0, r0     # r0 = r0 + mem[r0] = 12 + mem[12] = 24
        // addi r1, 7      # r1 = r1 + 7 = 19 (since r1=12 after lw)
        // sub  r0, r1     # r0 = r0 - r1 = 24 - 19 = 5
        // sw   r0, r1     # mem[r1] = r0 = mem[19] = 5
        // halt            # end of program

        // Instruction encoding:
        // A-type: Opcode[7:4] | ds[3] | s[2] | extra[1:0]
        // B-type: Opcode[7:4] | ds[3] | Immediate[2:0]
        // C-type: Opcode[7:4] | Offset[3:0]

        dut.mem_inst.mem[0] = 8'b00110101;  // addi r0, 5  (opcode=0011, ds=0, imm=101)
        dut.mem_inst.mem[1] = 8'b00111111;  // addi r1, 7  (opcode=0011, ds=1, imm=111)
        dut.mem_inst.mem[2] = 8'b00010100;  // add  r0, r1 (opcode=0001, ds=0, s=1, extra=00)
        dut.mem_inst.mem[3] = 8'b01101000;  // lw   r1, r0 (opcode=0110, ds=1, s=0, extra=00)
        dut.mem_inst.mem[4] = 8'b10000001;  // beq  offset=1 (opcode=1000, offset=0001)
        dut.mem_inst.mem[5] = 8'b11110000;  // halt        (opcode=1111)
        dut.mem_inst.mem[6] = 8'b00100000;  // addm r0, r0 (opcode=0010, ds=0, s=0, extra=00)
        dut.mem_inst.mem[7] = 8'b00111111;  // addi r1, 7  (opcode=0011, ds=1, imm=111)
        dut.mem_inst.mem[8] = 8'b01000100;  // sub  r0, r1 (opcode=0100, ds=0, s=1, extra=00)
        dut.mem_inst.mem[9] = 8'b01110100;  // sw   r0, r1 (opcode=0111, ds=0, s=1, extra=00)
        dut.mem_inst.mem[10] = 8'b11110000; // halt        (opcode=1111)

        // Pre-initialize memory[12] with value 12 for the lw instruction
        dut.mem_inst.mem[12] = 8'd12;

        // Wait for program to complete
        wait(dut.halted);
        #20;  // Wait a few more cycles

        // Check results
        $display("\n=== Test Results ===");
        $display("r0 = %0d (expected: 5)", dut.registers[0]);
        $display("r1 = %0d (expected: 19)", dut.registers[1]);
        $display("mem[19] = %0d (expected: 5)", dut.mem_inst.mem[19]);
        $display("PC = %0d", dut.PC);

        // Verify correctness
        // Note: Expected values are based on corrected trace:
        if (dut.registers[0] == 8'd5 &&
            dut.registers[1] == 8'd19 &&
            dut.mem_inst.mem[19] == 8'd5) begin
            $display("\n*** TEST PASSED ***");
        end else begin
            $display("\n*** TEST FAILED ***");
            $display("Expected: r0=5, r1=19, mem[19]=5");
        end

        $finish;
    end

    // Timeout watchdog
    initial begin
        #10000;  // Timeout after 10000ns
        $display("\nERROR: Test timeout - CPU may not have halted");
        $finish;
    end

    // Optional: Monitor CPU state during execution
    initial begin
        $monitor("Time=%0t PC=%0d IR=%b halted=%b r0=%0d r1=%0d",
                 $time, dut.PC, dut.IR, dut.halted, dut.registers[0], dut.registers[1]);
    end

endmodule
