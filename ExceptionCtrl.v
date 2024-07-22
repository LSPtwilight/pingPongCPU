`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/06/28 19:43:54
// Design Name: 
// Module Name: ExceptionCtrl
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ExceptionCtrl(
    input [7:0] STATUS,
    input [7:0] EX_SCAUSE,
    input [7:0] INTMASK,
    output EXL_Set,
    output INT_Signal,
    output reg [2:0] INT_PEND
    );
    
    wire [7:0] SRC_PEND;
    assign SRC_PEND = EX_SCAUSE & INTMASK;
    assign INT_Signal = (|SRC_PEND) & STATUS[1] & ~STATUS[0];
    assign EXL_Set = INT_Signal;
    
    always@(*)
    begin
        if(SRC_PEND[7]) INT_PEND <= 3'b111;
        else if(SRC_PEND[6]) INT_PEND <= 3'b110;
        else if(SRC_PEND[5]) INT_PEND <= 3'b101;
        else if(SRC_PEND[4]) INT_PEND <= 3'b100;
        else if(SRC_PEND[3]) INT_PEND <= 3'b011;
        else if(SRC_PEND[2]) INT_PEND <= 3'b010;
        else if(SRC_PEND[1]) INT_PEND <= 3'b001;
        else if(SRC_PEND[0]) INT_PEND <= 3'b000;
    end
    
endmodule
