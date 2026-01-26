# Introduction
In 154A you made a CPU in Logisim. Here, you will make a CPU in SystemVerilog. 

## Here's the 8 bit CPU you are making:
(People who took 154A last quarter may found this familiar)
![CPU](CPU.png)

This is the CPU from the [class notes](https://canvas.ucdavis.edu/courses/1035776/files/29845499?wrap=1)

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
addi r0, 5      # r0 = 5
addi r1, 10     # r1 = 10
add  r0, r1     # r0 = r0 + r1 = 15
lw   r1, r0     # r1 = mem[r0] = mem[15] = 15
beq  1          # if (r0 == r1) skip next instruction
halt            # should be skipped
addm r0, r0     # r0 = r0 + mem[r0] = 15 + mem[15] = 30
addi r1, 7      # r1 = 15 + 7 = 22
sub  r0, r1     # r0 = r0 - r1 = 30 - 22 = 8
sw   r0, r1     # mem[r1] = r0 = mem[22] = 8
halt            # end of program
```
The provided testbench [CPU_tb.sv](CPU_tb.sv) runs this program and checks that the final values in registers and memory are correct. You can modify the testbench or write your own to test additional cases.
