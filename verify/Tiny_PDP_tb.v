`include                        "../IP_Utilities/verify/tb_macros.vh"

`timescale			            1ns/1ps
`default_nettype		        none

`define     SQUARE_ON_PORTC     0
`define     SERIAL_ON_PORTC     1
`define     MS_TB_SIMTIME       150_000
`define     TEST_FILE(file)     `"file`"


module Tiny_PDP_tb #(parameter GEN_SQUARE=0, GEN_SERIAL=1) ();

    wire [11:0] instr;
    wire [9:0]  pc;
    wire [7:0]  px_out;
    wire [7:0]  px_dir;
    reg  [7:0]  px_in;
    wire [7:0]  py_out;
    wire [7:0]  py_dir;
    wire [7:0]  py_in;

    `include "../IP_Utilities/verify/serial_tasks.vh"

    reg         tx;
    integer     i;
    initial 
    begin
        tx = 1;
        #55_937;
        send_serial_8N1(8680.56, i, 8'h75);
    end
    
    assign pc_in = {6'd0,1'b0,tx};

    `TB_CLK(clk, 50)
    `TB_SRSTN(rst_n, clk, 999)
    `TB_DUMP("Tiny_PDP_tb.vcd", Tiny_PDP_tb, 0)
    `TB_FINISH(`MS_TB_SIMTIME)       
    
	// Instantiate the Unit Under Test (UUT)
	Tiny_PDP DUV (
        .clk(clk),
        .rst_n(rst_n),
        .instr(instr),
        .pc(pc),
        .px_out(px_out),
        .px_dir(px_dir),
        .px_in(px_in),
        .py_out(py_out),
        .py_dir(py_dir),
        .py_in(py_in)
    );

    reg [11:0] ROM [4095:0];
    initial begin
        $readmemh(`TEST_FILE(`TEST), ROM);
    end

    assign instr = ROM[pc];
        
endmodule