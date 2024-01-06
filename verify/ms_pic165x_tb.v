`include                        "ms_tb.vh"

`timescale			            1ns/1ps
`default_nettype		        none

`define     SQUARE_ON_PORTC     0
`define     SERIAL_ON_PORTC     1
`define     MS_TB_SIMTIME       150_000
`define     TEST_FILE(file)     `"file`"


module ms_pic165x_tb #(parameter GEN_SQUARE=0, GEN_SERIAL=1) ();

    wire [11:0] instr;
    wire [9:0]  pc;
    wire [7:0]  pb_out;
    wire [7:0]  pb_dir;
    reg  [7:0]  pb_in;
    wire [7:0]  pc_out;
    wire [7:0]  pc_dir;
    wire [7:0]  pc_in;

    /*
    generate
        if(`SQUARE_ON_PORTC == 1) begin
            reg portc_square = 0;
            always #5_000 portc_square <= !portc_square;
            assign pc_in = {6'd0,portc_square,portc_square};
        end else 
        if(`SERIAL_ON_PORTC == 1) begin*/
            `include "ms_tasks.vh"
            reg tx;
            integer i;
            initial begin
                tx = 1;
                #55_937;
                send_serial_8N1(8680.56, i, 8'h75);
            end
            assign pc_in = {6'd0,1'b0,tx};
        //end
    //endgenerate

    `MS_TB_CLK(50)
    `MS_TB_SRSTN(999)
    `MS_TB_DUMP("pic165x_tb.vcd", ms_pic165x_tb, 0)
    `MS_TB_FINISH

	// Instantiate the Unit Under Test (UUT)
	ms_pic165x DUV (
        .clk(clk),
        .rst_n(rstn),
        .instr(instr),
        .pc(pc),
        .pb_out(pb_out),
        .pb_dir(pb_dir),
        .pb_in(pb_in),
        .pc_out(pc_out),
        .pc_dir(pc_dir),
        .pc_in(pc_in)
    );

    reg [11:0] ROM [4095:0];
    initial begin
        $readmemh(`TEST_FILE(`TEST), ROM);
    end

    assign instr = ROM[pc];
        
endmodule