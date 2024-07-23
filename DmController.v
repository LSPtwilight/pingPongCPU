`timescale 1ns / 1ps
`define dm_word 3'b000
`define dm_halfword 3'b001
`define dm_halfword_unsigned 3'b010
`define dm_byte 3'b011
`define dm_byte_unsigned 3'b100
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/06/24 12:28:21
// Design Name: 
// Module Name: dm_controller
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

module Dm_Controller(mem_w, EX_Addr_in, MEM_Addr_in, Data_write, EX_DMType, MEM_DMType,
  Data_read_from_dm, Data_read, Data_write_to_dm, wea_mem);
  input mem_w;
  input [31:0]EX_Addr_in;
  input [31:0]MEM_Addr_in;
  input [31:0]Data_write;
  input [2:0]EX_DMType;
  input [2:0]MEM_DMType;
  input [31:0]Data_read_from_dm;
  output reg [31:0]Data_read;
  output reg [31:0]Data_write_to_dm;
  output [3:0]wea_mem;
  
    // dm_ctrl == DMType:
    // dm_word 3'b000
    // dm_halfword 3'b001
    // dm_halfword_unsigned 3'b010
    // dm_byte 3'b011
    // dm_byte_unsigned 3'b100

    assign wea_mem[3] = mem_w & ( ~EX_DMType[1] & ~EX_DMType[0] | ~EX_DMType[1] & EX_DMType[0] & EX_Addr_in[1]
                        | EX_DMType[1] & EX_DMType[0] & EX_Addr_in[1] & EX_Addr_in[0]);
    assign wea_mem[2] = mem_w & ( ~EX_DMType[1] & ~EX_DMType[0] | ~EX_DMType[1] & EX_DMType[0] & EX_Addr_in[1]
                        | EX_DMType[1] & EX_DMType[0] & EX_Addr_in[1] & ~EX_Addr_in[0]);
    assign wea_mem[1] = mem_w & ( ~EX_DMType[1] & ~EX_DMType[0] | ~EX_DMType[1] & EX_DMType[0] & ~EX_Addr_in[1]
                        | EX_DMType[1] & EX_DMType[0] & ~EX_Addr_in[1] & EX_Addr_in[0]);
    assign wea_mem[0] = mem_w & ( ~EX_DMType[1] & ~EX_DMType[0] | ~EX_DMType[1] & EX_DMType[0] & ~EX_Addr_in[1]
                        | EX_DMType[1] & EX_DMType[0] & ~EX_Addr_in[1] & ~EX_Addr_in[0]);
    
    always@(*)
    begin
        case(EX_DMType)
        `dm_word: begin Data_write_to_dm <= Data_write; end
        `dm_halfword: begin Data_write_to_dm <= {2{Data_write[15:0]}}; end
        `dm_byte: begin Data_write_to_dm <= {4{Data_write[7:0]}}; end
        default: begin Data_write_to_dm <= Data_write; end
        endcase
    end
    
    //loads
    always@(*)
    begin
        if(MEM_DMType==`dm_word) begin Data_read <= Data_read_from_dm; end
        else if(MEM_DMType==`dm_halfword)
        begin
            case(MEM_Addr_in[1])
            1'b0: begin Data_read <= {{16{Data_read_from_dm[15]}}, Data_read_from_dm[15:0]}; end
            1'b1: begin Data_read <= {{16{Data_read_from_dm[31]}}, Data_read_from_dm[31:16]}; end
            endcase
        end
        else if(MEM_DMType==`dm_halfword_unsigned)
        begin
            case(MEM_Addr_in[1])
            1'b0: begin Data_read <= {{16{1'b0}}, Data_read_from_dm[15:0]}; end
            1'b1: begin Data_read <= {{16{1'b0}}, Data_read_from_dm[31:16]}; end
            endcase 
        end
        else if(MEM_DMType==`dm_byte)
        begin
            case(MEM_Addr_in[1:0])
            2'b00: begin Data_read <= {{24{Data_read_from_dm[7]}}, Data_read_from_dm[7:0]}; end
            2'b01: begin Data_read <= {{24{Data_read_from_dm[15]}}, Data_read_from_dm[15:8]}; end
            2'b10: begin Data_read <= {{24{Data_read_from_dm[23]}}, Data_read_from_dm[23:16]}; end
            2'b11: begin Data_read <= {{24{Data_read_from_dm[31]}}, Data_read_from_dm[31:24]}; end
            endcase
        end
        else if(MEM_DMType==`dm_byte_unsigned)
        begin
            case(MEM_Addr_in[1:0])
            2'b00: begin Data_read <= {{24{1'b0}}, Data_read_from_dm[7:0]}; end
            2'b01: begin Data_read <= {{24{1'b0}}, Data_read_from_dm[15:8]}; end
            2'b10: begin Data_read <= {{24{1'b0}}, Data_read_from_dm[23:16]}; end
            2'b11: begin Data_read <= {{24{1'b0}}, Data_read_from_dm[31:24]}; end
            endcase
        end
    end
    
endmodule
