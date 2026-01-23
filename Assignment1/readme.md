# Introduction
This assignment is meant to get you familiar with systemverilog, a hardware description language (HDL). I assume you have used logisim before. While it's very capable and you can simulate a cpu out of it, in the real world people don't use logisim to design chips: instead, they use HDLs like Verilog.

# Setup
Install [Icarus Verilog](https://steveicarus.github.io/iverilog/):
on mac:
```bash
brew install icarus-verilog
```
on ubuntu/wsl:
```bash
sudo apt-get install iverilog
```

Click [GTKWave](https://gtkwave.github.io/gtkwave/) and follow instructions for your operating system to install it.

Read [This tutorial](https://www.asic-world.com/verilog/verilog_one_day.html) to get familiar with verilog/systemverilog syntax. Ask google/chatgpt to learn more about systemverilog.

# N to 1 Multiplexer
Finish the [muxNto1.sv](muxNto1.sv) file to implement an N to 1 multiplexer. The parameter N is defined as a parameter in the module. The width of each input and output is defined by the parameter WIDTH. The select signal will not select an input outside the range of N inputs.

# test
Compile the testbench:
```bash
iverilog -g2012 -o muxNto1_tb.vvp muxNto1.sv muxNto1_tb.sv
```
If it compiles without errors, run the simulation:
```bash
vvp muxNto1_tb.vvp
```
To view the waveform, run:
```bash
gtkwave mux_test.vcd
```
Click `muxNto1_tb` and then double click the signals you want to view on the bottom left panel. 

# Vending Machine
Implement a mealy model of a vending machine in [vending_machine.sv](vending_machine.sv). The vending machine accepts nickel and dime only. When a total of 30 cents is inserted, the machine dispenses a product and returns any change if necessary. Use the testbench [vending_machine_tb.sv](vending_machine_tb.sv) to test your design.

## Test
Use the same commands as above, replacing `muxNto1` with `vending_machine`.

# Submission
Submit your completed `muxNto1.sv`, `vending_machine.sv` files on Gradescope. Assignments due by 11:59 PM on Friday, January 23rd. 
