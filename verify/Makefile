# Makefile
# Usage: make APP_NAME=test_name

# Compiler
ASM = gpasm
HEXCONV = ihex2vhex.py

# Source file extension
SRC_EXT = asm

# Get the application name from the command line
ifdef APP_NAME
	SOURCE = $(APP_NAME).$(SRC_EXT)
	TARGET = $(APP_NAME).vvp
else
	$(error Please provide the application name using 'make APP_NAME=<name>')
endif

all: $(TARGET)

$(TARGET): $(SOURCE) ../rtl/Tiny_PDP.v Tiny_PDP_tb.v
	$(ASM) $(SOURCE)
	python3 ihex2vhex.py $(APP_NAME).hex > $(APP_NAME).v.hex
	iverilog -o $(APP_NAME).vvp -DTEST="$(APP_NAME).v.hex" ../rtl/Tiny_PDP.v Tiny_PDP_tb.v
	./$(APP_NAME).vvp

clean:
	rm -f $(TARGET)
