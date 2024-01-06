/*
    A PIC16C57 like peripherals controller with the following features
    - ROM: 512 words
    - RAM: 
        - First 16 bytes are left for SFRs
        - Next 16 bytes are used for data; there are 4 16-byte banks 
    - call stack depth is 4
    - Two I/O ports PORTB and PORTC
    - NO PCL register
    - 8-bit TMR0
    - OPTION Register:
        - 0-2 : TMR0 Prescaler selection
        - 3   : TMR0 enable
    - STATUS Register:
        - 0: C flag
        - 2: Z Flag
        - 7: TO flag
    - SLEEP for LP operation
        - Wake up on TMR0 TO or Portc Event 
    - Ineterrupt Vector table
        - 0: reset
        - 1 TMR0 TO
        - 2 PORTC Event
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
            1-2: SHIFT direction
            3: SHIFT clear
            4: TMR Selection
            5: System IRQ
            6: System FIFO Read
            7: System FIFO Write
        C: FIFO Register 0
        D: FIFO Register 1
*/

module ms_pic165x (
    input   wire            clk,
    input   wire            rst_n,
    input   wire    [11:0]  instr,
    output  reg     [9:0]   pc,
    output  wire    [7:0]   py_out,
    output  wire    [7:0]   px_dir,
    input   wire    [7:0]   px_in,
    output  wire    [7:0]   py_out,
    output  wire    [7:0]   py_dir,
    input   wire    [7:0]   py_in,
    output  wire    [31:0]  fifo_wdata,
    input   wire    [31:0]  fifo_rdata,
    output  wire            fifo_rd,
    output  wire            fifo_wr,
    output  wire            irq
);
    localparam  [3:0]   INDF_ADDR   = 0,
                        TMR0_ADDR   = 1,
                        PCL_ADDR    = 2,
                        STATUS_ADDR = 3,
                        FSR_ADDR    = 4,
                        PORTA_ADDR  = 5,
                        PORTB_ADDR  = 6,
                        PORTC_ADDR  = 7;

    localparam  [11:0]  OP_NOP    = 12'b0000_0000_0000,   // No Operation
                        OP_OPTION = 12'b0000_0000_0010,   // Set Option Register
                        OP_SLEEP  = 12'b0000_0000_0011,   // Set Sleep Register
                        OP_CLRWDT = 12'b0000_0000_0100,   // Clear Watchdog Timer
                        OP_TRISA  = 12'b0000_0000_0101,   // Set Port A Tristate Control Reg
                        OP_TRISB  = 12'b0000_0000_0110,   // Set Port B Tristate Control Reg
                        OP_TRISC  = 12'b0000_0000_0111,   // Set Port C Tristate Control Reg
                        OP_CLRW   = 12'b0000_0100_0000;   // W = 0; Z;

    localparam  [11:0]  OP_MOVWF  =  12'b0000_001x_xxxx;         // F = W;
    localparam  [11:0]  OP_CLRF   =  12'b0000_011x_xxxx;         // F = 0; Z;

    localparam  [11:0]  OP_SUBWF  =  12'b0000_10xx_xxxx,  // D ? F = F - W : W = F - W; Z, C, DC;
                        OP_DECF   =  12'b0000_11xx_xxxx,  // D ? F = F - 1 : W = F - 1; Z;

                        OP_IORWF  =  6'b0001_00,  // D ? F = F | W : W = F | W; Z;
                        OP_ANDWF  =  6'b0001_01,  // D ? F = F & W : W = F & W; Z;
                        OP_XORWF  =  6'b0001_10,  // D ? F = F ^ W : W = F ^ W; Z;
                        OP_ADDWF  =  6'b0001_11,  // D ? F = F + W : W = F + W; Z, C, DC;
                        
                        OP_MOVF   =  6'b0010_00,  // D ? F = F     : W = F    ; Z;
                        OP_COMF   =  6'b0010_01,  // D ? F = ~F    : W = ~F   ; Z;
                        OP_INCF   =  6'b0010_10,  // D ? F = F + 1 : W = F + 1; Z;
                        OP_DECFSZ =  6'b0010_11,  // D ? F = F - 1 : W = F - 1; skip if Z;
                        
                        OP_RRF    =  6'b0011_00,  // D ? F = {C,F[7:1]} : W={C,F[7:1]};C=F[0]
                        OP_RLF    =  6'b0011_01,  // D ? F = {F[6:0],C} : W={F[6:0],C};C=F[7]
                        OP_SWAPF  =  6'b0011_10,  // D ? F = t : W = t; t = {F[3:0], F[7:4]}
                        OP_INCFSZ =  6'b0011_11;  // D ? F = F - 1 : W = F - 1; skip if Z

    localparam  [3:0]   OP_BCF    =  4'b0100,     // F = F & ~(1 << bit);
                        OP_BSF    =  4'b0101,     // F = F |  (1 << bit);
                        OP_BTFSC  =  4'b0110,     // skip if F[bit] == 0;
                        OP_BTFSS  =  4'b0111,     // skip if F[bit] == 1;
                        OP_RETLW  =  4'b1000,     // W = L; Pop(PC = TOS);
                        OP_CALL   =  4'b1001,     // Push(TOS=PC+1); PC={PA[2:0],0,L[7:0]};
                        OP_MOVLW  =  4'b1100,     // W = L[7:0];
                        OP_IORLW  =  4'b1101,     // W = L[7:0] | W; Z;
                        OP_ANDLW  =  4'b1110,     // W = L[7:0] & W; Z;
                        OP_XORLW  =  4'b1111;     // W = L[7:0] ^ W; Z;

    localparam  [2:0]   OP_GOTO   =  3'b101;      // PC = {PA[2:0], L[8:0]};

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
    // The SFRs
    reg [7:0]   W,
                OPTION,
                TRISA,
                TRISB,
                TRISC;

    reg [7:0]   TMR0_LOAD   , TMR1_LOAD,  // RAM[0]
                TMR0        , TMR1,  // RAM[1],
                PORTC_IE    ,   // RAM[2],
                STATUS      ,   // RAM[3],   
                FSR         ,   // RAM[4],
                PORTC_EDGE  ,   // RAM[5],
                PORTB       ,   // RAM[6],
                PORTC       ,   // RAM[7];
                SHIFT       ,   // RAM[8]
                INDEX       ,
                CONTROL     ,
                FIFO [1:0]  ;

    reg [7:0]   ALU_out;
    reg [7:0]   bit_mask;
    reg [7:0]   bit_out;
    reg         bit_test;
    reg         CF, C;
    reg         ZF, Z;

    wire        TOF = (TMR0 == 0);
    reg         TO;
    wire        TOF1 = (TMR1 == 0);
    reg         TO1;


    // Sleep FF
    reg     SLEEP;
    wire    portc_pedge_event = |(portc_pedge & PORTC_EDGE & PORTC_IE);
    wire    portc_nedge_event = |(portc_nedge & ~PORTC_EDGE & PORTC_IE);
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
        

    // The Stack
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

    // TMR0
    reg[7:0] pr;
    wire tmr0_inc;
    reg pr_prev;
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pr <= 0;
        else if(OPTION[3])
            pr <= pr + 1;

    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pr_prev <= 0;
        else
            pr_prev <= pr[OPTION[2:0]];

    assign tmr0_inc = ~pr_prev & pr[OPTION[2:0]];

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

    reg tof_prev;

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
            portc_prev <= TRISC & pc_in;
            portc_pedge <= ~portc_prev & (pc_in & TRISC);
            portc_nedge <= portc_prev & ~(pc_in & TRISC);
        end

    // The Registers File
    reg [7:0]   RAM [32:0];     // 2 x 16-bytes banks
    reg [7:0]   RAM_out;

    wire        RAM_access = instr_f[4];
    wire        SFR_access = ~RAM_access;
    wire [5:0]  RAM_addr = {FSR[6], FSR[5], instr_f[3:0]};
    wire [3:0]  SFR_addr = instr_f[3:0];

    wire [5:0]  IND_addr =  {FSR[6], FSR[5], INDEX[3:0]}; 
                            
    always @* begin
        if(SFR_access)
            case (SFR_addr)
                0 : RAM_out = CONTROL[4] ? TMR1_LOAD : TMR0_LOAD;
                1 : RAM_out = CONTROL[4] ? TMR1 : TMR0;
                2 : RAM_out = PORTC_IE;
                3 : RAM_out = {TO, 4'd0, Z, 1'b0, C};
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
    
    // IND Register
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
            
    // Control Register
    always @(posedge clk, negedge rst_n)
        if(!rst_n) 
            CONTROL <= 'b0;
        else if((wr_to_ram==1) && (SFR_access) && (SFR_addr == 11))
            CONTROL <= RAM_in;
        
    // STATUS Register
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
    
    always @(posedge clk, negedge rst_n)
        if(!rst_n)
            pc <= 'b0;
        else
            if(SLEEP & TO) 
                pc <= 1;
            else if(SLEEP & |(portc_nedge_event | portc_pedge_event))
                pc <= 2;
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

    // Special Registers
    always @(posedge clk, negedge rst_n)
        if(!rst_n) begin
            W       <= 'b0;
            OPTION  <= 'b0;
            TRISA   <= 'b0;
            TRISB   <= 'b0;
            TRISC   <= 'b0;
        end
        else if(!SLEEP)
            if (instr_opcode4 == 4'b0000)
                case (instr)
                    OP_OPTION   :   OPTION  <= W;
                    OP_TRISA    :   TRISA   <= W;
                    OP_TRISB    :   TRISB   <= W;
                    OP_TRISC    :   TRISC   <= W;
                endcase

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

    // W Register
    always @(posedge clk, negedge rst_n)
        if(!rst_n) 
            W <= 'b0;
        else if(!SLEEP) 
            if(instr_opcode4 == OP_RETLW)
                W <= instr_literal8;
            else 
                if(wr_to_w) W <= ALU_out;

    assign px_dir       = TRISB;
    assign px_dir       = TRISC;

    assign py_out       = PORTB;
    assign py_out       = PORTC;

    assign fifo_wdata   = {FIFO[1], FIFO[0]};

    assign irq          = CONTROL[5];
    assign fifo_rd      = CONTROL[6];
    assign fifo_wr      = CONTROL[7];
    
endmodule