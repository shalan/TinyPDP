# Tiny Programmable Digital Peripheral (Tiny PDP)

This repo contains a Programmable Digital Peripheral (PDP). PDP allows us to write very simple assembly programs to emulate different peripherals and communication protocols. Tiny PDP is designed after the Microchip PIC16C5x Instruction Set with extension to make it easier to emulate peripherals. Tiny PDP can be implemented in less than 3000 ASIC gates which makes it very attractive.

## Tiny PDP Architecture

## The Memory

PDP has two types of memories: RAM (for DATA and Special Function Registers) and ROM (for programs). 

- The ROM can be up to 512 instruction words. The instruction word is 12 bits. PDP can fetch instructions only from the ROM. The ROM can be written by the system top provide the machine code of the program to run on Tiny PDP.
- The RAM address space is 32 bytes where the first 16 bytes are used for SFR and the last 16 bytes are used for data. The data space is banked and up to 4 banks can be used making the total available data RAM 64 bytes.

### The Special Function Registers (SFR)

There are 16 SFRs as given by the following table:
| Address | SFR | Description |
|---------|---------|---------|
| 0x00| `LOAD` | Timer load register; `CTRL[4]` selects TMR0/TMR1|
| 0x01| `TMR`    | Timer Register ; `CTRL[4]` selects TMR0/TMR1|
| 0x02| `PORTY_IE` | `PORTY` Inetrrupts Enable|
| 0x03| `STATUS`  | Status Register|
| 0x04| `FSR`     | File Select Register|
| 0x05| `PORTY_EDGE` | `PORTY` Interrupt Edge|
| 0x06| `PORTX`   | Port X|
| 0x07| `PORTY`   | Port Y|
| 0x08| `SHIFT`| Shift Register|
| 0x09| `INDEX`| Index Register|
| 0x0A| `IND`| Indirect Register|
| 0x0B| `CTRL` | Control Register<br>`0-1`: INDEX Auto increment/decrement<br>`2`: SHIFT direction<br>`3`: SHIFT clear<br> `4`: Timer Selection (TMR0/TMR1)<br>`5`: System IRQ<br> `6`: System FIFO Read<br> `7`: System FIFO Write| 
| 0x0C| `FIFO0`| FIFO Register 0|
| 0x0D| `FIFO1`| FIFO Register 1|
| 0x0E| `SYS_STATUS`| System Status Register; emulation program specific format|
| 0x0F| `SYS_CTRL`| System COntrol Register; emulation program specific format |

### Data RAM

Tiny PDP has internal data RAM of up to 64 bytes that can be used for variables. This RAM is made out of up to four 16-byte banks. The bank can be selected using the FSR register.

### The Call Stack
``call`` instruction pushes the ``PC`` into the call stack which is a dedicated 4 words stack memory. That said, Tiny PDP supports only 4 nested calls.

## System Interface

Tiny PDP interfaces with the system through:

- Receive and Transmit FIFOs.
- 8-bit System Status and Control Registers.
- Firmware ROM (Up to 512 12-bit instruction words).
- IRQ line.

## Timers

There are two 8-bit timers (TMR0 and TMR1); each comes with a dedicated 8-bit prescaler and load registers. The timer is an up counter that counts every a number of clock cycles (check the prescaler section). 

Only one timer is accessible at any point of time based on the value of `CTRL[4]`. That means the same set of registers (TMR, LOAD and OPTION) are used for both TMR0 and TMR1 based on the value of `CTRL[4]`. 

`OPTION[3]` is used to disable/enable the selected timer.

### Prescaler
The prescaler effectively divides the system clock by a value $2^n$, where $n$ is the value of `OPTION[2:0]` + 1 with the possible values: `2, 4, ..., 256`.

### Load Register

The load register provides the value of the timer when the timer reaches the value `0`. In other words, the timer is incremented by `1` till it reaches `0` then loaded with the value in the load register and so on.

## I/O Ports

Tiny PDP can control up to 16 I/Os divided into two ports 8-bit ``PORTX`` and ``PORTY``. The direction of of the ports are provided by the ``TRIS`` registers (``0``: output, ``1``: input). ``PORTC`` is capable of detecting input transitions and wake-up Tiny PDP when it sees a positive transition (low to hight) or a negative transition (hight to low). Register ``PORTC_EDGE`` is used to select the direction of the wake-up transition for each pin. Register ``PORTC_IE`` is used to enable or disable the wake-up for each pin.

## Tiny PDP Firmware Development

The [GNU PIC assembler](https://gputils.sourceforge.io/) (gpasm) can be used to assemble the firmware. Also, helper macros and register definitions header are provided.

## Examples

### Square Wave

### PWM

### UART_RX

### UART_TX

### One Wire

### I2S Receiver

### SPI Slave

### I2C Slave
 