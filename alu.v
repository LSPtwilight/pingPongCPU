`include "ctrl_encode_def.v"

module alu(A, B, ALUOp, C, Zero, PC, csr_data, div_result);
           
   input  signed [31:0] A, B;
   input         [4:0]  ALUOp;
   input [31:0] PC;
   input [31:0] csr_data;
   input [63:0] div_result;
   output signed [31:0] C;
   output Zero;
   
   reg [31:0] C;
   integer    i;
   
   wire [63:0] product = A*B;
   wire [63:0] product_u = $unsigned(A) * $unsigned(B);

   always @( * ) begin
      case ( ALUOp )
`ALUOp_nop:C=A;
`ALUOp_lui:C=B;
`ALUOp_auipc:C=PC+B;
`ALUOp_add:C=A+B;
`ALUOp_sub:C=A-B;
`ALUOp_bne:C={31'b0,(A==B)};
`ALUOp_blt:C={31'b0,(A>=B)};
`ALUOp_bge:C={31'b0,(A<B)};
`ALUOp_bltu:C={31'b0,($unsigned(A)>=$unsigned(B))};
`ALUOp_bgeu:C={31'b0,($unsigned(A)<$unsigned(B))};
`ALUOp_slt:C={31'b0,(A<B)};
`ALUOp_sltu:C={31'b0,($unsigned(A)<$unsigned(B))};
`ALUOp_xor:C=A^B;
`ALUOp_or:C=A|B;
`ALUOp_and:C=A&B;
`ALUOp_sll:C=A<<B[4:0];
`ALUOp_srl:C=A>>B;
`ALUOp_sra:C=A>>>B;
`ALUOp_nor:C=~(A|B);
`ALUOp_beq:C={31'b0,(A!=B)};
`ALUOp_mulw:C= product[31:0];
`ALUOp_mulhw:C= product[63:32];
`ALUOp_mulhwu:C= product_u[63:32];
`ALUOp_divw: C  = div_result[31: 0];
`ALUOp_divwu: C = div_result[31: 0];
`ALUOp_modw: C  = div_result[63:32];
`ALUOp_modwu: C = div_result[63:32];
`ALUOp_CSRs: C = csr_data;
default: C=32'h0;

      endcase
   end // end always
   
   assign Zero = (C == 32'b0);

endmodule
    
