# Introduction
In 154A you made a CPU in Logisim. Here, you will make a CPU in SystemVerilog. 

## Here's the 8 bit multi-cycle CPU you are making:
(People who took 154A last quarter may found this familiar)
![CPU](CPU.png)

This is the CPU from the [Generic CPU](https://canvas.ucdavis.edu/courses/1035776/files/29845499?wrap=1)
**Important:** The testbench merely test the correctness of your CPU. Thus, your design is not limited to single bus or multi-cycle implementation. 

## Instructions Set Architecture
| Instruction Format | Opcode | Operation |
|--------------------|--------|-----------|
| nand               | 0000   | rds=~(rds & rs) |
| add                | 0001   | rds=rds+rs |
| addm               | 0010   | rds=rds+mem[rs] |
| addi               | 0011   | rds=rds+imm |
| sub                | 0100   | rds=rds-rs |
| mult               | 0101   | rds=rds*rs |
| lw                 | 0110   | rds=mem[rs] |
| sw                 | 0111   | mem[rs]=rds |
| beq                | 1000   | if (rds==rs) PC=PC+offset (offset is sign extended) |
| jmp                | 1001   | PC=PC+offset (offset is sign extended) |
| halt               | 1111   | stop execution |

### The machine has 3 different instruction formats:  A, B, and C.

A-type: 
| Opcode |  ds  |   s   | extra  |
|--------|------|-------|--------|
| 7-4    |  3   |   2   | 1-0    |

B-type: 
| Opcode |  ds  |   Immediate   |
|--------|------|---------------|
| 7-4    |  3   |     2-0       |

C-type: 
| Opcode |  Offset   |
|--------|-----------|
| 7-4    |   3-0     |

### The ALU can perform 4 functions:

| Operation       | ALU0    | ALU1    |
|-----------------|---------|---------|
| Add             | 1       | 1       |
| Sub             | 1       | 0       |
| Mult            | 0       | 1       |
| Nand            | 0       | 0       |

## Starter Code
You can find the starter code is provided in [cpu.sv](cpu.sv)

## Testing
Below is the test program in assemble.

```
0. addi r0, 5      # r0 = 5
1. addi r1, 7      # r1 = 7
2. add  r0, r1     # r0 = r0 + r1 = 12
3. lw   r1, r0     # r1 = mem[r0] = mem[12] = 12
4. beq  1          # if (r0 == r1) skip next instruction
5. halt            # should be skipped
6. addm r0, r0     # r0 = r0 + mem[r0] = 12 + mem[12] = 24
7. addi r1, 7      # r1 = 12 + 7 = 19
8. sub  r0, r1     # r0 = r0 - r1 = 24 - 19 = 5
9. sw   r0, r1     # mem[r1] = r0 = mem[19] = 5
10. halt            # end of program
```
The provided testbench [cpu_tb.sv](cpu_tb.sv) runs this program and checks that the final values in registers and memory are correct. You can modify the testbench or write your own to test additional cases.

## Submission
under construction
