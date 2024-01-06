# Makefile

# Compiler
ASM = gpasm

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

$(TARGET): $(SOURCE)
	$(ASM) $(SOURCE)
	python3 ihex2vhex.py $(APP_NAME).hex > $(APP_NAME).v.hex
	iverilog -o $(APP_NAME).vvp -DTEST="$(APP_NAME).v.hex" ms_pic165x.v ms_pic165x_tb.v
	./$(APP_NAME).vvp

clean:
	rm -f $(TARGET)
