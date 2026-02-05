FILE=cpu
default: test
all:test

build:
    iverilog -g2012 -o ${FILE}.vvp ./${FILE}.sv ./${FILE}_tb.sv

test: build
    vvp ${FILE}.vvp

clean:
    - rm ./${FILE}.vvp
