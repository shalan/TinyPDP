; The Registers
TMR0_LOAD   equ     00      ; 
TMR0        equ     01      ;
PORTY_IE    equ     02      ;    
STATUS      equ     03      ; STATUS register 
FSR         equ     04      ;
PORTY_EDGE  equ     05      ;
PORTX       equ     06      ; I/O register 
PORTY       equ     07      ; I/O register 
SHIFT       equ     08      ; Shift register 
INDEX       equ     09      ; INDEX register  
INDIRECT    equ     0A      ; Indirect register to be used with the INDEX register 
CONTROL     equ     0B      ; Control register 
FIFO0       equ     0C      ; FIFO register 0
FIFO1       equ     0D      ; FIFO register 1

; Fields
; STATUS register fields
CARRY       equ     0       ; Carry bit in status register
ZERO        equ     2       ;
TO          equ     7

; OPTION register fields
TMR0_EN     equ     3       ; TMR0 Enable bit in OPTION register
TMR0_PR     equ     0       ; TMR0 prescaler selection in OPTION register

; CONTROL register fields
INDEX_AI    equ     0
INDEX_AD    equ     1
SHIFT_DIR   equ     2
SHIFT_CLR   equ     3
IRQ         equ     4
FIFO_RD     equ     5
FIFO_WR     equ     6

; Macros
ps_set      macro   ps
            movlw   ps
            option
            movwf   TMR0_OPTIONS
            endm

tmr0_enable macro
            bsf     TMR0_OPTIONS, 3
            movfw   TMR0_OPTIONS
            option
            endm

tmr0_disable macro
            bcf     TMR0_OPTIONS, 3
            movfw   TMR0_OPTIONS
            option
            endm

portx_set   macro   value
            movlw   value
            movwf   PORTX
            endm

porty_set   macro   value
            movlw   value
            movwf   PORTY
            endm

porty_ne    macro   pin
            movfw   PORTY_EDGE
            andlw   ~(1<<pin)
            movwf   PORTY_EDGE
            endm

porty_pe    macro   pin
            movfw   PORTY_EDGE
            orlw    (1<<pin)
            movwd   PORTY_EDGE
            endm

porty_ie    macro   pin
            movfw   PORTY_IE
            iorlw   (1<<pin)
            movwf   PORTY_IE
            endm

porty_id    macro   pin
            movfw   PORTY_IE
            andlw   ~(1<<pin)
            movwf   PORTY_IE
            endm
            
ll          macro   reg, value
            movlw   value
            movwf   reg
            endm