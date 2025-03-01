localparam  [11:0]      OP_NOP    = 12'b0000_0000_0000,   // No Operation
                        OP_OPTION = 12'b0000_0000_0010,   // Set Option Register
                        OP_SLEEP  = 12'b0000_0000_0011,   // Set Sleep Register
                        OP_CLRWDT = 12'b0000_0000_0100,   // Clear Watchdog Timer
                        OP_TRISA  = 12'b0000_0000_0101,   // Set Port A Tristate Control Reg
                        OP_TRISB  = 12'b0000_0000_0110,   // Set Port B Tristate Control Reg
                        OP_TRISC  = 12'b0000_0000_0111,   // Set Port C Tristate Control Reg
                        OP_CLRW   = 12'b0000_0100_0000;   // W = 0; Z;

localparam  [11:0]      OP_MOVWF  =  12'b0000_001x_xxxx;         // F = W;
localparam  [11:0]      OP_CLRF   =  12'b0000_011x_xxxx;         // F = 0; Z;

localparam  [11:0]      OP_SUBWF  =  12'b0000_10xx_xxxx,  // D ? F = F - W : W = F - W; Z, C, DC;
                        OP_DECF   =  12'b0000_11xx_xxxx;  // D ? F = F - 1 : W = F - 1; Z;

localparam  [ 5:0]      OP_IORWF  =  6'b0001_00,  // D ? F = F | W : W = F | W; Z;
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

localparam  [3:0]       OP_BCF    =  4'b0100,     // F = F & ~(1 << bit);
                        OP_BSF    =  4'b0101,     // F = F |  (1 << bit);
                        OP_BTFSC  =  4'b0110,     // skip if F[bit] == 0;
                        OP_BTFSS  =  4'b0111,     // skip if F[bit] == 1;
                        OP_RETLW  =  4'b1000,     // W = L; Pop(PC = TOS);
                        OP_CALL   =  4'b1001,     // Push(TOS=PC+1); PC={PA[2:0],0,L[7:0]};
                        OP_MOVLW  =  4'b1100,     // W = L[7:0];
                        OP_IORLW  =  4'b1101,     // W = L[7:0] | W; Z;
                        OP_ANDLW  =  4'b1110,     // W = L[7:0] & W; Z;
                        OP_XORLW  =  4'b1111;     // W = L[7:0] ^ W; Z;

localparam  [2:0]       OP_GOTO   =  3'b101;      // PC = {PA[2:0], L[8:0]};