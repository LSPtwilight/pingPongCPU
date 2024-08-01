`timescale 1ns / 1ps
`include "ctrl_encode_def.v"



module ForwardingUnit#(parameter WIDTH = 5) (
    input MEM_RegWrite,// EX/MEM.RegWrite
    input [WIDTH-1:0] MEM_rd,// EX/MEM.RegisterRd
    input WB_RegWrite,// MEM/WB.RegWrite
    input [WIDTH-1:0] WB_rd,// MEM/WB.RegisterRd
    input [WIDTH-1:0] EX_rs,// EX/MEM.RegisterRs1/2
    output [1:0] ForwardSignal//00: from regfile. 10: from MEM_aluout. 01: from WB_WD
    );
    
    wire MEM_Forward;
    assign MEM_Forward = MEM_rd==EX_rs && MEM_RegWrite==1'b1;//~(|(MEM_rd ^ EX_rs)) & MEM_RegWrite;
    wire WB_Forward;
    assign WB_Forward = WB_rd==EX_rs && WB_RegWrite==1'b1 && MEM_Forward==1'b0;//~(|(WB_rd ^ EX_rs)) & WB_RegWrite & ~MEM_Forward;
    assign ForwardSignal = {MEM_Forward, WB_Forward};
    
endmodule
