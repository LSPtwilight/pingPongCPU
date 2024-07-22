`include "ctrl_encode_def.v"

module NPC(PC, EX_pc, ID_pc, SEPC, NPCOp, INT_Signal, INT_PEND, IMM, NPC,EX_RD1, branch_flag, stall_signal);  // next pc module
    
   input  [31:0] PC;        // pc
   input  [31:0] EX_pc;
   input  [31:0] ID_pc;
   input  [31:0] SEPC;
   input  [4:0]  NPCOp;     // next pc operation
   input INT_Signal;
   input [2:0] INT_PEND;
   input  [31:0] IMM;       // immediate
	input [31:0] EX_RD1;
    input branch_flag;
    input stall_signal;
   output reg [31:0] NPC;   // next pc
   
   wire [31:0] PCPLUS4;
   wire [31:0] SEPCPLUS4;
   
   //wire [31:0] INT0_ADDRESS = 32'b0;
   
   assign PCPLUS4 = PC + 4; // pc + 4
   assign SEPCPLUS4 = SEPC + 4;
   
    always @(*) begin
        if(INT_Signal)
        begin
            case(INT_PEND)
            3'b111: NPC<=32'd168;
            3'b110: NPC<=32'd144;
            3'b101: NPC<=32'd120;
            3'b100: NPC<=32'd96;
            3'b011: NPC<=32'd72;
            3'b010: NPC<=32'd48;
            3'b001: NPC<=32'd24;
            3'b000: NPC<=32'd0;
            endcase
        end
        else if(stall_signal) begin
            NPC <= PC;
        end
        else begin
            case (NPCOp)
            `NPC_PLUS4:       NPC = PCPLUS4;
            `NPC_BRANCH:      NPC = branch_flag? (EX_pc+IMM):PCPLUS4;
            `NPC_JUMP:        NPC = EX_pc+IMM;
		    `NPC_JALR:        NPC = 32'h0;
		    `NPC_SEPC:        NPC = SEPC;
		    `NPC_SEPC_PLUS4:  NPC = SEPCPLUS4;
            `NPC_JIRL:        NPC = EX_RD1+IMM;
            //`NPC_BANDBL:      NPC = ID_pc+IMM;
            `NPC_BANDBL:      NPC = EX_pc+IMM;
            default:     NPC = PCPLUS4;
            endcase
        end
    end // end always
   
endmodule
