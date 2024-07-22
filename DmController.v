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

module Dm_Controller(mem_w, Addr_in, Data_write, DMType, 
  Data_read_from_dm, Data_read, Data_write_to_dm, wea_mem);
  input mem_w;
  input [31:0]Addr_in;
  input [31:0]Data_write;
  input [2:0]DMType;
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

    assign wea_mem[3] = mem_w & ( ~DMType[1] & ~DMType[0] | ~DMType[1] & DMType[0] & Addr_in[1]
                        | DMType[1] & DMType[0] & Addr_in[1] & Addr_in[0]);
    assign wea_mem[2] = mem_w & ( ~DMType[1] & ~DMType[0] | ~DMType[1] & DMType[0] & Addr_in[1]
                        | DMType[1] & DMType[0] & Addr_in[1] & ~Addr_in[0]);
    assign wea_mem[1] = mem_w & ( ~DMType[1] & ~DMType[0] | ~DMType[1] & DMType[0] & ~Addr_in[1]
                        | DMType[1] & DMType[0] & ~Addr_in[1] & Addr_in[0]);
    assign wea_mem[0] = mem_w & ( ~DMType[1] & ~DMType[0] | ~DMType[1] & DMType[0] & ~Addr_in[1]
                        | DMType[1] & DMType[0] & ~Addr_in[1] & ~Addr_in[0]);
    
    always@(*)
    begin
        case(DMType)
        `dm_word: begin Data_write_to_dm <= Data_write; end
        `dm_halfword: begin Data_write_to_dm <= {2{Data_write[15:0]}}; end
        `dm_byte: begin Data_write_to_dm <= {4{Data_write[7:0]}}; end
        default: begin Data_write_to_dm <= Data_write; end
        endcase
    end
    
    //loads
    always@(*)
    begin
        if(DMType==`dm_word) begin Data_read <= Data_read_from_dm; end
        else if(DMType==`dm_halfword)
        begin
            case(Addr_in[1])
            1'b0: begin Data_read <= {{16{Data_read_from_dm[15]}}, Data_read_from_dm[15:0]}; end
            1'b1: begin Data_read <= {{16{Data_read_from_dm[31]}}, Data_read_from_dm[31:16]}; end
            endcase
        end
        else if(DMType==`dm_halfword_unsigned)
        begin
            case(Addr_in[1])
            1'b0: begin Data_read <= {{16{1'b0}}, Data_read_from_dm[15:0]}; end
            1'b1: begin Data_read <= {{16{1'b0}}, Data_read_from_dm[31:16]}; end
            endcase 
        end
        else if(DMType==`dm_byte)
        begin
            case(Addr_in[1:0])
            2'b00: begin Data_read <= {{24{Data_read_from_dm[7]}}, Data_read_from_dm[7:0]}; end
            2'b01: begin Data_read <= {{24{Data_read_from_dm[15]}}, Data_read_from_dm[15:8]}; end
            2'b10: begin Data_read <= {{24{Data_read_from_dm[23]}}, Data_read_from_dm[23:16]}; end
            2'b11: begin Data_read <= {{24{Data_read_from_dm[31]}}, Data_read_from_dm[31:24]}; end
            endcase
        end
        else if(DMType==`dm_byte_unsigned)
        begin
            case(Addr_in[1:0])
            2'b00: begin Data_read <= {{24{1'b0}}, Data_read_from_dm[7:0]}; end
            2'b01: begin Data_read <= {{24{1'b0}}, Data_read_from_dm[15:8]}; end
            2'b10: begin Data_read <= {{24{1'b0}}, Data_read_from_dm[23:16]}; end
            2'b11: begin Data_read <= {{24{1'b0}}, Data_read_from_dm[31:24]}; end
            endcase
        end
    end
    
endmodule
