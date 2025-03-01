/*
    A peripherals controller with the following features
    - ROM: Up to 512 words
    - RAM: 
        - First 16 bytes are left for SFRs
        - Next 16 bytes are used for data; there are up to 4 16-byte banks 
    - call stack depth is 4
    - Two I/O ports PORTX and PORTY
    - Two 8-bit Timers with 8-bit prescalers
    - OPTION Register:
        - 0-2 : TMR0 Prescaler selection
        - 3   : TMR0 enable
        - 4-6 : TMR1 Prescaler selection
        - 7   : TMR1 enable
    - STATUS Register:
        - 0: C flag
        - 2: Z Flag
        - 6: TO0, TO flag for TMR0
        - 7: TO1, TO flag for TMR1
    - SLEEP for LP operation
        - Wake up on TMR0/1 TO or Portc Events 
    - Ineterrupt Vector table
        - 0: reset
        - 1 TMR0 TO
        - 2 TMR1 TO
        - 3 PORTC Event
        - 4 RDFIFO is not empty [tbd]
        - 5 WRFIFO is empty [tbd]
    - No indirect memory access

    Special Function Registers:
        0: TMR0_LOAD <new>
        1: TMR0    
        2: PORTC_IE <new>
        3: STATUS  
        4: FSR     
        5: PORTC_EDGE <new>
        6: PORTB   
        7: PORTC   
        8: Shift Register
        9: Index Regsiter
        A: Indirect Register
        B: Control Register
            0-1: INDEX Auto increment/decrement
            2-3: SHIFT operation (TBD)
                 00 : Clear
                 11 : Write
                 10 : SHR
                 01 : SHL   
            2: SHIFT direction
            3: SHIFT clear
            4: TMR Selection
            5: System IRQ
            6: System FIFO Read
            7: System FIFO Write
        C: FIFO Register 0
        D: FIFO Register 1
        E: FIFO Register 2
        F: FIFO Register 3
*/

/*
        UART TX:
            RDFIFO_NE:
                tx_data = RDFIFO
                one_cnt = 0
                bit_indx = 0
                Drop TX pin low
                generate TMR0 interrupt with a period = bit time
            TMR0 ISR:
                TX = tx_data & 1
                if(TX) one_cnt++
                bit_indx++
                tx_data = tx_data >> 1
                sleep

        



*/

module Tiny_PDP (
    input   wire            clk,
    input   wire            rst_n,
    input   wire    [11:0]  instr,
    output  reg     [9:0]   pc,
    output  wire    [7:0]   px_out,
    output  wire    [7:0]   px_dir,
    input   wire    [7:0]   px_in,
    output  wire    [7:0]   py_out,
    output  wire    [7:0]   py_dir,
    input   wire    [7:0]   py_in,
    output  wire    [31:0]  fifo_wdata,
    input   wire    [31:0]  fifo_rdata,
    output  wire            fifo_rd,
    output  wire            fifo_wr,
    input   wire            rfifo_ne,
    input   wire            wfifo_nf,
    output  wire            irq
);
/*
    localparam  [3:0]   INDF_ADDR   = 0,
                        TMR0_ADDR   = 1,
                        PCL_ADDR    = 2,
                        STATUS_ADDR = 3,
                        FSR_ADDR    = 4,
                        PORTA_ADDR  = 5,
                        PORTB_ADDR  = 6,
                        PORTC_ADDR  = 7;
*/
    `include "pdp_opcodes.vh"

    wire [3:0]  instr_opcode3   =   instr[11:9];
    wire [3:0]  instr_opcode4   =   instr[11:8];
    wire [5:0]  instr_opcode6   =   instr[11:6];
    wire [7:0]  instr_literal8  =   instr[7:0];
    wire [8:0]  instr_literal9  =   instr[9:0];
    wire [2:0]  instr_bit       =   instr[7:5];
    wire [4:0]  instr_f         =   instr[4:0];
    wire        instr_dest      =   instr[5];
    wire        instr_sfr       =   ~instr[4];

    wire    wr_bu_to_ram    =   (instr_opcode4 == OP_BCF) | 
                                (instr_opcode4 == OP_BSF);

    wire    wr_alu_to_ram   =   (instr_dest & 
                                    (
                                        (instr_opcode6 == OP_IORWF)     |
                                        (instr_opcode6 == OP_ANDWF)     |
                                        (instr_opcode6 == OP_XORWF)     |
                                        (instr_opcode6 == OP_ADDWF)     |
                                        (instr_opcode6 == OP_MOVF )     |
                                        (instr_opcode6 == OP_COMF )     |
                                        (instr_opcode6 == OP_INCF )     |
                                        (instr_opcode6 == OP_DECFSZ)    |
                                        (instr_opcode6 == OP_RRF  )     |
                                        (instr_opcode6 == OP_RLF  )     |
                                        (instr_opcode6 == OP_SWAPF)     |
                                        (instr_opcode6 == OP_INCFSZ)    |
                                        (instr_opcode6 == 6'b0000_10)   |
                                        (instr_opcode6 == 6'b0000_11)
                                    )
                                ) |
                                (instr[11:5] == 7'b0000_001) |
                                (instr[11:5] == 7'b0000_011);

    wire    wr_to_ram   =   wr_bu_to_ram | wr_alu_to_ram;

    wire    wr_to_w     =   (instr == OP_CLRW) |
                            (instr_opcode4 == OP_MOVLW) |
                            (instr_opcode4 == OP_IORLW) |
                            (instr_opcode4 == OP_ANDLW) |
                            (instr_opcode4 == OP_XORLW) |
                            ~instr_dest & (
                                (instr_opcode6 == OP_IORWF)     |
                                (instr_opcode6 == OP_ANDWF)     |
                                (instr_opcode6 == OP_XORWF)     |
                                (instr_opcode6 == OP_ADDWF)     |
                                (instr_opcode6 == OP_MOVF )     |
                                (instr_opcode6 == OP_COMF )     |
                                (instr_opcode6 == OP_INCF )     |
                                (instr_opcode6 == OP_DECFSZ)    |
                                (instr_opcode6 == OP_RRF  )     |
                                (instr_opcode6 == OP_RLF  )     |
                                (instr_opcode6 == OP_SWAPF)     |
                                (instr_opcode6 == OP_INCFSZ)    |
                                (instr_opcode6 == 6'b0000_10)   |
                                (instr_opcode6 == 6'b0000_11)
                            );
    // The Registers
    reg [7:0]   W,
                OPTION,
                TRISA,
                TRISB,
                TRISC;

    // SFR
    wire[7:0]   TMR0        , TMR1;         // RAM[1],
    reg [7:0]   TMR0_LOAD   , TMR1_LOAD,    // RAM[0]
                PORTC_IE    ,               // RAM[2],
                STATUS      ,               // RAM[3],   
                FSR         ,               // RAM[4],
                PORTC_EDGE  ,               // RAM[5],
                PORTB       ,               // RAM[6],
                PORTC       ,               // RAM[7];
                SHIFT       ,               // RAM[8]
                INDEX       ,               // RAM[9]
                CONTROL     ,               // RAM[11]
                FIFO [3:0]  ;               // RAM[15:12]

    reg [7:0]   ALU_out;
    reg [7:0]   bit_mask;
    reg [7:0]   bit_out;
    reg         bit_test;
    reg         C;
    reg         Z;

    reg         TO0;
    reg         TO1;

    //
    // Sleep/Wakeup Logic
    //
    reg         SLEEP;
    
    wire        portc_pedge_event = |(portc_pedge & PORTC_EDGE & PORTC_IE);
    wire        portc_nedge_event = |(portc_nedge & ~PORTC_EDGE & PORTC_IE);

    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            SLEEP <= 0;
        else if(instr == OP_SLEEP)
            SLEEP <= 1;
        else if(TO)
            SLEEP <= 0;
        else if(portc_pedge_event)
            SLEEP <= 0;
        else if(portc_nedge_event)
            SLEEP <= 0;     
        
    //
    // The Call Stack
    //
    reg [7:0]   call_stack [3:0];
    reg [2:0]   tos;

    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            tos <= 'b0;
        else if(!SLEEP)
            if(instr_opcode4 == OP_CALL) begin
                call_stack[tos] <= pc + 1;
                tos <= tos + 1'b1;
            end else if(instr_opcode4 == OP_RETLW) begin
                tos <= tos - 1'b1;
            end

    //
    // The timers
    //
    wire tmr0_wr = wr_to_ram && SFR_access && (SFR_addr==1) && ~CONTROL[4];
    wire tmr1_wr = wr_to_ram && SFR_access && (SFR_addr==1) && CONTROL[4];

    pdp_timer timer_0 (
        .clk(clk),
        .rst_n(rst_n),
        .en(OPTION[3]),         
        .tmr_wr(wr_to_ram && SFR_access && (SFR_addr==1) && ~CONTROL[4]),     
        .pr_sel(OPTION[2:0]),     
        .data_in(TAM_in),    
        .reload(TMR0_LOAD),     
        .tmr(TMR0),    
        .to(TO0)          
    );

    pdp_timer timer_1 (
        .clk(clk),
        .rst_n(rst_n),
        .en(OPTION[7]),         
        .tmr_wr(tmr1_wr),     
        .pr_sel(OPTION[6:4]),     
        .data_in(TAM_in),    
        .reload(TMR1_LOAD),     
        .tmr(TMR1),    
        .to(TO1)          
    );
/*
    // TMR0
    reg[7:0]    pr;
    wire        tmr0_inc;
    reg         pr_prev;
    reg         tof_prev;

    // The prescaler
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pr <= 0;
        else if(OPTION[3])
            pr <= pr + 1;

    // Generate the inc pulse
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pr_prev <= 0;
        else
            pr_prev <= pr[OPTION[2:0]];
    assign tmr0_inc = ~pr_prev & pr[OPTION[2:0]];

    // The timer
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            TMR0 <= 8'b0;
        else if(wr_to_ram && SFR_access && (SFR_addr==1) && ~CONTROL[4])
            TMR0 <= RAM_in;
        else if(tmr0_inc)
            if(TOF)
                TMR0 <= TMR0_LOAD;
            else
                TMR0 <= TMR0 + 1'b1;

    // Generate the TO pulse
    always@(posedge clk)
        tof_prev <= TOF;
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            TO <= 0;
        else
            TO <= TOF & !tof_prev;

    // TMR1
    reg[7:0] pr1;
    wire tmr1_inc;
    reg pr1_prev;
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pr1 <= 0;
        else if(OPTION[3])
            pr1 <= pr1 + 1;

    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pr1_prev <= 0;
        else
            pr1_prev <= pr1[OPTION[2:0]];

    assign tmr1_inc = ~pr1_prev & pr1[OPTION[2:0]];

    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            TMR1 <= 8'b0;
        else if(wr_to_ram && SFR_access && (SFR_addr==1) && CONTROL[4])
            TMR1 <= RAM_in;
        else if(tmr1_inc)
            if(TOF1)
                TMR1 <= TMR1_LOAD;
            else
                TMR1 <= TMR1 + 1'b1;

    reg tof1_prev;

    always@(posedge clk)
        tof1_prev <= TOF1;

    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            TO1 <= 0;
        else
            TO1 <= TOF1 & !tof1_prev;

    // PORTC Wake from Sleep Logic
    reg [7:0] portc_prev;
    reg [7:0] portc_pedge;
    reg [7:0] portc_nedge;

    always @(posedge clk, negedge rst_n)
        if(!rst_n) begin
            portc_pedge <= 0;
            portc_nedge <= 0;
            portc_prev <= 0;
        end else begin
            portc_prev <= TRISC & py_in;
            portc_pedge <= ~portc_prev & (py_in & TRISC);
            portc_nedge <= portc_prev & ~(py_in & TRISC);
        end
*/
    //
    // The Registers File
    //
    reg [7:0]   RAM [32:0];    
    reg [7:0]   RAM_out;
    wire        RAM_access  =   instr_f[4];
    wire        SFR_access  =   ~RAM_access;
    wire [5:0]  RAM_addr    =   {FSR[6], FSR[5], instr_f[3:0]};
    wire [3:0]  SFR_addr    =   instr_f[3:0];
    wire [5:0]  IND_addr    =   {FSR[6], FSR[5], INDEX[3:0]}; 
                            
    always @* begin
        if(SFR_access)
            case (SFR_addr)
                0 : RAM_out = CONTROL[4] ? TMR1_LOAD : TMR0_LOAD;
                1 : RAM_out = CONTROL[4] ? TMR1 : TMR0;
                2 : RAM_out = PORTC_IE;
                3 : RAM_out = {TO1, TO0, 3'd0, Z, 1'b0, C};
                4 : RAM_out = FSR;
                5 : RAM_out = PORTC_EDGE;
                6 : RAM_out = (PORTB&(~TRISB)) | (px_in&TRISB);
                7 : RAM_out = (PORTC&(~TRISC)) | (py_in&TRISC);
                8 : RAM_out = SHIFT;
                9 : RAM_out = INDEX;
                10: RAM_out = RAM[IND_addr];
                11: RAM_out = CONTROL;
                12: RAM_out = fifo_rdata[7:0];
                13: RAM_out = fifo_rdata[15:8];
                12: RAM_out = fifo_rdata[23:15];
                13: RAM_out = fifo_rdata[31:24];
                default: RAM_out = 'b0;
            endcase
        else begin
            RAM_out = RAM[RAM_addr];
        end
    end

    wire [7:0] RAM_in = wr_alu_to_ram ? ALU_out : bit_out;

    always @(posedge clk, negedge rst_n)
        if(!rst_n) begin
            TMR0_LOAD   <=  'b0;
            TMR1_LOAD   <=  'b0;
            FSR         <=  'b0;
            PORTB       <=  'b0;
            PORTC       <=  'b0;
            PORTC_IE    <=  'b0;
            PORTC_EDGE  <=  'b0;
            SHIFT       <=  'b0;
            FIFO[0]     <=  'b0;
            FIFO[1]     <=  'b0;
        end else if(!SLEEP)
            if(wr_to_ram) begin
                if(SFR_access) begin
                    case (SFR_addr)
                        0 : if(CONTROL[4]) TMR1_LOAD <= RAM_in; else TMR0_LOAD <= RAM_in;
                        2 : PORTC_IE        <=  RAM_in;
                        4 : FSR             <=  RAM_in;
                        5 : PORTC_EDGE      <=  RAM_in;
                        6 : PORTB           <=  RAM_in;
                        7 : PORTC           <=  RAM_in;
                        8 : SHIFT           <=  CONTROL[3] ? 8'b0 : 
                                                CONTROL[2] ? {RAM_in[0], SHIFT[7:1]} : {SHIFT[6:0], RAM_in[0]};
                        10: RAM[IND_addr]   <=  RAM_in;
                        12: FIFO[0]         <=  RAM_in;
                        13: FIFO[1]         <=  RAM_in;
                    endcase
                end else begin
                    RAM [RAM_addr] <= RAM_in;
                    $display("RAM - WR %x to %x", ALU_out, RAM_addr);
                end
            end
    
    //
    // IND (Indirect) Register
    //
    always @(posedge clk, negedge rst_n)
        if(!rst_n) 
            INDEX <= 'b0;
        else if((wr_to_ram==1) && (SFR_access) && (SFR_addr == 9))
            INDEX <= RAM_in;
        else if((wr_to_ram==1) && (SFR_access) && (SFR_addr == 10)) begin
            if(CONTROL[0])
                INDEX <= INDEX + 1;
            else if(CONTROL[1])
                INDEX <= INDEX - 1;
        end else if((instr_opcode6==OP_MOVF) && (SFR_access) && (SFR_addr == 10)) begin
            if(CONTROL[0])
                INDEX <= INDEX + 1;
            else if(CONTROL[1])
                INDEX <= INDEX - 1;
        end 
    
    //
    // Control Register
    //
    always @(posedge clk, negedge rst_n)
        if(!rst_n) 
            CONTROL <= 'b0;
        else if((wr_to_ram==1) && (SFR_access) && (SFR_addr == 11))
            CONTROL <= RAM_in;
    
    //
    // STATUS Register
    //
    always @(posedge clk, negedge rst_n)
        if(!rst_n) begin
           C    <= 'b0;
           Z    <= 'b0;
        end else if(!SLEEP)
            if(instr_opcode6 == OP_RRF)
                C <= RAM_out[0];
            else if(instr_opcode6 == OP_RLF)
                C <= RAM_out[7];
            else if((wr_to_ram==1) && (SFR_access) && (SFR_addr == 3)) begin
                    C <= RAM_in[0];
                    Z <= RAM_in[2];
            end else begin
                C <= CF;
                Z <= ZF;
            end
    
    //
    // Program Counter
    //
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pc <= 'b0;
        else
            if(SLEEP & TO0)   
                pc <= 1;
            else if(SLEEP & TO1) 
                pc <= 2;
            else if(SLEEP & |(portc_nedge_event | portc_pedge_event))
                pc <= 3;
            else if(!SLEEP)
                if(instr_opcode3 == OP_GOTO)
                    pc <= instr_literal9;
                else if((instr_opcode4 == OP_BTFSC) || (instr_opcode4 == OP_BTFSS)) begin
                    if(bit_test) pc <= pc + 'd2;
                    else pc <= pc + 'd1;
                end else if((instr_opcode6 == OP_DECFSZ) || (instr_opcode6 == OP_INCFSZ)) begin
                    if(ZF) pc <= pc + 'd2;
                    else pc <= pc + 'd1;        
                end else if(instr_opcode4 == OP_CALL)
                    pc <= instr_literal8;
                else if(instr_opcode4 == OP_RETLW)
                    pc <= call_stack[tos-1];
                else
                    pc <= pc + 'd1;

    //
    // I/O Registers
    //
    always @(posedge clk, negedge rst_n)
        if(!rst_n) begin
            //W       <= 'b0;
            OPTION  <= 'b0;
            //TRISA   <= 'b0;
            TRISB   <= 'b0;
            TRISC   <= 'b0;
        end
        else if(!SLEEP)
            if (instr_opcode4 == 4'b0000)
                case (instr)
                    OP_OPTION   :   OPTION  <= W;
                    //OP_TRISA    :   TRISA   <= W;
                    OP_TRISB    :   TRISB   <= W;
                    OP_TRISC    :   TRISC   <= W;
                endcase

    //
    // ALU + BMU
    //
    pdp_alu ALU (
        .W(W),
        .F(RAM_out),
        .L(instr_literal8),
        .opcode(instr_opcode6),
        .C(C),
        .out(ALU_out),
        .CF(CF),
        .ZF(ZF)
    );

    pdp_bmu BMU (
        .bit_sel(instr_bit),
        .opcode(instr_opcode4),
        .F(RAM_out),
        .out(bit_out),
        .test(bit_test)
    );

    //
    // W Register
    //
    always @(posedge clk, negedge rst_n)
        if(!rst_n) 
            W <= 'b0;
        else if(!SLEEP) 
            if(instr_opcode4 == OP_RETLW)
                W <= instr_literal8;
            else 
                if(wr_to_w) W <= ALU_out;

/*
    always @* begin
        CF = 0;
        casex (instr_opcode6)
            6'b0000_10          :   {CF,ALU_out} = RAM_out - W;
            6'b0000_11          :   ALU_out = RAM_out - 1;
            6'b0000_00          :   ALU_out = W;
            6'b0000_01          :   ALU_out = 'b0;
            OP_IORWF            :   ALU_out = RAM_out | W;
            OP_ANDWF            :   ALU_out = RAM_out & W;
            OP_XORWF            :   ALU_out = RAM_out ^ W;
            OP_ADDWF            :   {CF,ALU_out} = RAM_out + W;                   
            OP_MOVF             :   ALU_out = RAM_out;
            OP_COMF             :   ALU_out = ~RAM_out;
            OP_INCF             :   ALU_out = RAM_out + 1;
            OP_DECFSZ           :   ALU_out = RAM_out - 1;
            OP_INCFSZ           :   ALU_out = RAM_out + 1;
            OP_SWAPF            :   ALU_out = {RAM_out[3:0], RAM_out[7:4]};
            OP_RRF              :   ALU_out = {C,RAM_out[7:1]};
            OP_RLF              :   ALU_out = {RAM_out[6:0],C};
            {OP_MOVLW, 2'bxx}   :   ALU_out = instr_literal8;
            {OP_IORLW, 2'bxx}   :   ALU_out = W | instr_literal8;
            {OP_ANDLW, 2'bxx}   :   ALU_out = W & instr_literal8;
            {OP_XORLW, 2'bxx}   :   ALU_out = W ^ instr_literal8;
            default             :   ALU_out = 8'b1;
        endcase
    end

    always @*
        ZF = (ALU_out == 8'b0);
*/
    
/*
    always @*
        begin
            case(instr_bit)
                3'b000  : bit_mask = 8'b0000_0001;
                3'b001  : bit_mask = 8'b0000_0010;
                3'b010  : bit_mask = 8'b0000_0100;
                3'b011  : bit_mask = 8'b0000_1000;
                3'b100  : bit_mask = 8'b0001_0000;
                3'b101  : bit_mask = 8'b0010_0000;
                3'b110  : bit_mask = 8'b0100_0000;
                3'b111  : bit_mask = 8'b1000_0000;
            endcase
        end

    always @* begin    
        case (instr_opcode4)
            OP_BCF  :   bit_out = RAM_out & ~bit_mask;
            OP_BSF  :   bit_out = RAM_out | bit_mask;
            default :   bit_out = 8'b0;
        endcase
    end

    always @* begin    
        case (instr_opcode4)
            OP_BTFSC    :   bit_test = ~|(RAM_out & bit_mask);
            OP_BTFSS    :   bit_test =  |(RAM_out & bit_mask);
            default     :   bit_test = 1'b0;
        endcase
    end
*/
    

    assign px_dir       = TRISB;
    assign px_dir       = TRISC;

    assign px_out       = PORTB;
    assign py_out       = PORTC;

    assign fifo_wdata   = {FIFO[3], FIFO[2], FIFO[1], FIFO[0]};

    assign irq          = CONTROL[5];
    assign fifo_rd      = CONTROL[6];
    assign fifo_wr      = CONTROL[7];
    
endmodule


/*
    8-bit timer with a prescaler
    ~100 cells
*/
module pdp_timer (
    input           clk,
    input           rst_n,

    input           en,         // Timer Enable
    input           tmr_wr,     // Timer write data_in
    input   [2:0]   pr_sel,     // Prescaler divisor select

    input   [7:0]   data_in,    // Timer data to be written
    input   [7:0]   reload,     // The reload value

    output  [7:0]   tmr,        // The current TImer value 

    output          to          // Time out indicator

);

    reg     [7:0]   pr;
    reg     [7:0]   TMR;
    wire            tmr_en;
    reg             pr_prev;
    reg             tof_prev;
    wire            TOF = (TMR == 0);
    reg             TO;

    // 8-bit prescaler
    // That divides by 2, 4, ...., 256
    // pr_sel selects one of these divisors
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pr <= 0;
        else if(en)
            pr <= pr + 1;
        else if(~en)
            pr <= 0;

    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pr_prev <= 0;
        else
            pr_prev <= pr[pr_sel];
    
            assign tmr_en = ~pr_prev & pr[pr_sel];

    // The timer
    // 8-bit down counter 
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            TMR <= 8'b0;
        else if(tmr_wr)
            TMR <= data_in;
        else if(tmr_en)
            if(TOF)
                TMR <= reload;
            else
                TMR <= TMR - 1'b1;

    // Generate the TO pulse
    always@(posedge clk)
        tof_prev <= TOF;

    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            TO <= 0;
        else
            TO <= TOF & !tof_prev;

    assign to       =   TO;
    assign tmr      =   TMR;

endmodule

/*
    PDP ALU
    ~500 cells; can be reduced to 250 using FA cells
*/

module pdp_alu (
    input  [7:0]    W,
    input  [7:0]    F,
    input  [7:0]    L,
    input  [5:0]    opcode,
    input           C,
    output [7:0]    out,
    output          CF,
    output          ZF
);

    `include "pdp_opcodes.vh"

    always @* begin
        CF = 0;
        casex (opcode)
            6'b0000_10          :   {CF,out} = F - W;
            6'b0000_11          :   out = F - 8'b1;
            6'b0000_00          :   out = W;
            6'b0000_01          :   out = 1'b0;
            OP_IORWF            :   out = F | W;
            OP_ANDWF            :   out = F & W;
            OP_XORWF            :   out = F ^ W;
            OP_ADDWF            :   {CF,out} = F + W;                   
            OP_MOVF             :   out = F;
            OP_COMF             :   out = ~F;
            OP_INCF             :   out = F + 8'b1;
            OP_DECFSZ           :   out = F - 8'b1;
            OP_INCFSZ           :   out = F + 8'b1;
            OP_SWAPF            :   out = {F[3:0], F[7:4]};
            OP_RRF              :   out = {C,F[7:1]};
            OP_RLF              :   out = {F[6:0],C};
            {OP_MOVLW, 2'bxx}   :   out = L;
            {OP_IORLW, 2'bxx}   :   out = W | L;
            {OP_ANDLW, 2'bxx}   :   out = W & L;
            {OP_XORLW, 2'bxx}   :   out = W ^ L;
            default             :   out = 8'hFF;
        endcase
    end

    always @*
        ZF = (out == 8'b0);

endmodule


/*
    PDP Bit Manipulation Unit (BMU)
*/

module pdp_bmu(
    input [2:0]     bit_sel,
    input [3:0]     opcode,
    input [7:0]     F,
    
    output [7:0]    out,
    output          test

);

    `include "pdp_opcodes.vh"
    
    reg [7:0] bit_mask;

    always @*
    begin
        case(bit_sel)
            3'b000  : bit_mask = 8'b0000_0001;
            3'b001  : bit_mask = 8'b0000_0010;
            3'b010  : bit_mask = 8'b0000_0100;
            3'b011  : bit_mask = 8'b0000_1000;
            3'b100  : bit_mask = 8'b0001_0000;
            3'b101  : bit_mask = 8'b0010_0000;
            3'b110  : bit_mask = 8'b0100_0000;
            3'b111  : bit_mask = 8'b1000_0000;
        endcase
    end

    always @* begin    
        case (opcode)
            OP_BCF      :   out = F & ~bit_mask;
            OP_BSF      :   out = F | bit_mask;
            default     :   out = 8'h0;
        endcase
    end

    always @* begin    
        case (opcode)
            OP_BTFSC    :   test = ~|(F & bit_mask);
            OP_BTFSS    :   test =  |(F & bit_mask);
            default     :   test = 1'b0;
        endcase
    end

endmodule