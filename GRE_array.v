`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/06/14 22:27:22
// Design Name: 
// Module Name: pipeRegisters
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

module GRE_array #(parameter WIDTH = 32)(
    input clk, rst, write_enable, flush,
    input [WIDTH-1:0] in,
    output reg [WIDTH-1:0] out
    );
    //warning
    always@(posedge clk, posedge rst)//?
    begin
        if(rst)
            out = 0;
        else if(write_enable)
        begin
            if(flush)
                out = 0;
            else
                out = in;
        end
    end
    
    /*always@(posedge rst)
    begin
        out = 0;
    end*/
    
endmodule
