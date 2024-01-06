;
        LIST    p=16C56 

; uart_rx.asm
;
; UART RX Emulation
;

; RAM allocated variables
count           equ     10
flag            equ     11
TMR0_OPTIONS    equ     12
rdata           equ     13

    include     "tinypdp.h"

    ; Interrupt Vectors
    org 0
reset_vector:
    goto    start
to_vector:
    goto    tmr0_isr
porty_vector:
    goto    porty_isr

    org     4
; TMR0 ISR
tmr0_isr:
    decfsz      flag
    goto        tmr0_isr_not_first
    tmr0_disable
    ps_set      d'0'
    ll          TMR0, d'172'
    ll          TMR0_LOAD, d'172'   ; was 169
    ll          count, d'8'
    ll          CONTROL, h'08'
    movwf       SHIFT               ; a dummy write to clear SHIFT register
    ll          CONTROL, h'04'      ; Disable clearing the SHIFT register
    tmr0_enable
    goto        tmr0_isr_end
tmr0_isr_not_first:
    movfw       PORTY
    andlw       h'01'
    ;movwf       PORTX
    movwf       SHIFT
    decfsz      count
    goto        tmr0_isr_end
tmr0_word_received:
    tmr0_disable
    movfw       SHIFT
    movwf       PORTX
tmr0_isr_end:
    sleep

; PORTY ISR
porty_isr:
    ll          flag, 1
    porty_id    0
    tmr0_enable
    sleep

start:  
    ll          count, d'08'
    ll          PORTY_EDGE, d'0'
    ll          PORTY_IE, d'0'
    ; set PORTX as output and PORTY as input
    clrw             
    tris	    PORTX	        ; PORTX is Output
    ll          PORTX, h'55'
    movlw       h'01'
	tris	    PORTY	        ; PORTY is input
    porty_ne    0
    ps_set      0
    ll          TMR0, d'220'    ; was 216; adjusted for the processing time
    ll          TMR0_LOAD, d'216'
    porty_ie    0
    sleep

done:
    goto    done

    END