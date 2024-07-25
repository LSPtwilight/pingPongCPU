 `include "ctrl_encode_def.v"

//123
module ctrl(STATUS,
            PC, inst,
            Op, Funct7, Funct3, Zero, 
            RegWrite, MemWrite, MemRead,
            EXTOp, ALUOp, NPCOp, imm4alu, imm4pc,
            ALUSrc1, ALUSrc2, GPRSel, WDSel,DMType,
            ID_SCAUSE, EXL_Clear,
            rdOut, rs1, rs2, // reg 
            csr_num, csr_mask_en, csr_write,//csr
            ERTN,
            SYS, BRK, INE  // exception signals
            );
            
   input [7:0] STATUS;
  
   input [31:0] inst;
   input [31:0] PC;
   output [31:0] imm4alu;
   output [31:0] imm4pc;

   input  [6:0] Op;       // opcode
   input  [6:0] Funct7;    // funct7
   input  [2:0] Funct3;    // funct3
   input        Zero;
   
   output       RegWrite; // control signal for register write
   output       MemWrite; // control signal for memory write
   output       MemRead;
   output [5:0] EXTOp;    // control signal to signed extension
   output [4:0] ALUOp;    // ALU opertion
   output [4:0] NPCOp;    // next pc operation
   output       ALUSrc1;   // ALU source for A
   output       ALUSrc2;   // ALU source for B
   output [2:0] DMType;
   output [1:0] GPRSel;   // general purpose register selection
   output [1:0] WDSel;    // (register) write data selection
   output [7:0] ID_SCAUSE;

   //// reg out 
   output [4:0]  rdOut;
   output [ 4:0] rs1;
   output [ 4:0] rs2;
   output EXL_Clear;

   output [13:0] csr_num;
   output csr_mask_en;
   output csr_write;

   output ERTN;
   output SYS;
   output BRK;
   output INE;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
wire [11:0] alu_op;
wire        load_op;
wire        src1_is_pc;
wire        src2_is_imm;
wire        res_from_mem;
wire        dst_is_r1;
wire        gr_we;
wire        mem_we;
wire        src_reg_is_rd;
wire [4: 0] dest;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm4alu;
wire [31:0] imm4pc;
wire [31:0] br_offs;
wire [31:0] jirl_offs;
// temp
wire [31:0] pc;

wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rd;
wire [ 4:0] rj;
wire [ 4:0] rk;
wire [ 4:0] i5;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;

wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

wire        inst_add_w;
wire        inst_sub_w;
wire        inst_slt;
wire        inst_sltu;
wire        inst_nor;
wire        inst_and;
wire        inst_or;
wire        inst_xor;
wire        inst_slli_w;
wire        inst_srli_w;
wire        inst_srai_w;
wire        inst_addi_w;
wire        inst_ld_w;
wire        inst_st_w;
wire        inst_jirl;
wire        inst_b;
wire        inst_bl;
wire        inst_beq;
wire        inst_bne;
wire        inst_lu12i_w;
wire        inst_slti;
wire        inst_sltui;
wire        inst_andi;
wire        inst_ori;
wire        inst_xori;
wire        inst_sll_w;
wire        inst_srl_w;
wire        inst_sra_w;
wire        inst_pcaddu12i;
wire        inst_st_b;
wire        inst_st_h;
wire        inst_ld_b;
wire        inst_ld_h;
wire        inst_ld_hu;
wire        inst_ld_bu;
wire        inst_blt;
wire        inst_bge;
wire        inst_bltu;
wire        inst_bgeu;
wire        inst_mul_w;
wire        inst_mulh_w;
wire        inst_mulh_wu;
wire        inst_div_w;
wire        inst_mod_w;
wire        inst_div_wu;
wire        inst_mod_wu;
wire        inst_csrrd;
wire        inst_csrwr;
wire        inst_csrxchg;
wire        inst_ertn;
wire        inst_break;
wire        inst_syscall;

wire        need_ui5;
wire        need_si12;
wire        need_si16;
wire        need_si20;
wire        need_si26;
wire        need_ui12;
wire        src2_is_4;

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;
wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;

wire [31:0] alu_src1   ;
wire [31:0] alu_src2   ;
wire [31:0] alu_result ;

wire [31:0] mem_result;
   
//temp test
assign pc        =0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
assign op_31_26  = inst[31:26];
assign op_25_22  = inst[25:22];
assign op_21_20  = inst[21:20];
assign op_19_15  = inst[19:15];

assign rd   = inst[ 4: 0];
assign rj   = inst[ 9: 5];
assign rk   = inst[14:10];

assign i5   = inst[14:10];
assign i12  = inst[21:10];
assign i20  = inst[24: 5];
assign i16  = inst[25:10];
assign i26  = {inst[ 9: 0], inst[25:10]};

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));


assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];

// load from mem
assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];

assign inst_jirl   = op_31_26_d[6'h13];
assign inst_b      = op_31_26_d[6'h14];
assign inst_bl     = op_31_26_d[6'h15];
assign inst_beq    = op_31_26_d[6'h16];
assign inst_bne    = op_31_26_d[6'h17];
assign inst_lu12i_w= op_31_26_d[6'h05] & ~inst[25];

// added 
assign inst_slti = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi = op_31_26_d[6'h00] & op_25_22_d[4'b1101];
assign inst_ori = op_31_26_d[6'h00] & op_25_22_d[4'b1110];
assign inst_xori = op_31_26_d[6'h00] & op_25_22_d[4'b1111];
assign inst_sll_w=op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'b01110];
assign inst_srl_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'b01111];
assign inst_sra_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'b10000];
assign inst_pcaddu12i = op_31_26_d[6'h07] & ~inst[25];
assign inst_st_b = op_31_26_d[6'h0a] & op_25_22_d[4'h4];
assign inst_st_h = op_31_26_d[6'h0a] & op_25_22_d[4'h5];
assign inst_ld_b = op_31_26_d[6'h0a] & op_25_22_d[4'h0];
assign inst_ld_h = op_31_26_d[6'h0a] & op_25_22_d[4'h1];
assign inst_ld_hu = op_31_26_d[6'h0a] & op_25_22_d[4'h9];
assign inst_ld_bu = op_31_26_d[6'h0a] & op_25_22_d[4'h8];
assign inst_blt = op_31_26_d[6'b011000];
assign inst_bge = op_31_26_d[6'b011001];
assign inst_bltu = op_31_26_d[6'b011010];
assign inst_bgeu = op_31_26_d[6'b011011];

// mul and div
assign inst_mul_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_mod_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_div_wu  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_wu  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];

assign inst_csrrd   = (inst[31:24]==8'h04) & (rj==5'b0);
assign inst_csrwr   = (inst[31:24]==8'h04) & (rj==5'b1);
assign inst_csrxchg = (inst[31:24]==8'h04) & !(rj==5'b0) & !(rj==5'b1);

assign csr_num = inst[23:10];
assign csr_mask_en = inst_csrxchg;
assign csr_write = inst_csrwr | inst_csrxchg;

assign inst_ertn    = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10]
                    & (inst[14:10]==5'h0e);
assign ERTN = inst_ertn;

assign inst_syscall = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
assign inst_break   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];

assign SYS = inst_syscall;
assign BRK = inst_break;

assign MemRead = inst_ld_b | inst_ld_h | inst_ld_w | inst_ld_bu | inst_ld_hu;

/*---------------------------------------------------------*/
assign need_ui5  = inst_slli_w | inst_srli_w | inst_srai_w ;
assign need_si12 = inst_addi_w | inst_ld_w | inst_ld_h | inst_ld_hu | inst_ld_b | inst_ld_bu | inst_st_w | inst_st_b | inst_st_h | inst_slti | inst_sltui ;
assign need_ui12 = inst_andi | inst_ori | inst_xori ;
assign need_si16 = inst_jirl | inst_beq | inst_bne | inst_blt | inst_bltu | inst_bge | inst_bgeu ;
assign need_si20 = inst_lu12i_w | inst_pcaddu12i ; 
assign need_si26 = inst_b | inst_bl ;
assign src2_is_4 = inst_jirl | inst_bl ;

assign imm4alu = src2_is_4 ? 32'h4                  :
             need_si20 ? {i20[19:0], 12'b0}         :
             need_si16 ? {{14{i16[15]}},{i16,2'b00}}:
             need_si26 ? {{ 4{i26[25]}},{i26,2'b00}}:
             need_ui5  ? {27'b0, i5[4:0]}           :
             need_ui12 ? {20'b0, i12[11:0]}         :
            /*need_si12*/{{20{i12[11]}}, i12[11:0]} ;

assign imm4pc = need_si26 ? {{ 4{i26[25]}},{i26,2'b00}} : 
                need_si16 ? {{14{i16[15]}},{i16,2'b00}} :
                32'h0;


// ctrl
assign src_reg_is_rd = inst_beq | inst_bne | inst_blt | inst_bltu | inst_bge | inst_bgeu | inst_st_w | inst_st_b |inst_st_h 
                     | inst_csrwr | inst_csrxchg ;

assign src1_is_pc    = inst_jirl | inst_bl | inst_pcaddu12i ;

assign src2_is_imm   = inst_slli_w    |
                       inst_srli_w    |
                       inst_srai_w    |
                       inst_addi_w    |
                       inst_ld_w      | inst_ld_h | inst_ld_hu | inst_ld_b | inst_ld_bu |
                       inst_st_w      | inst_st_b | inst_st_h |
                       inst_lu12i_w   |
                       inst_jirl      |
                       inst_bl        |
                       inst_pcaddu12i |
                       inst_slti      |
                       inst_sltui     |
                       inst_andi      |
                       inst_ori       |
                       inst_xori;

assign res_from_mem  = inst_ld_w | inst_ld_h | inst_ld_hu | inst_ld_b | inst_ld_bu ;
assign dst_is_r1     = inst_bl;
assign gr_we         = (~inst_st_w & ~inst_st_b & ~inst_st_h & ~inst_beq & ~inst_bne & ~inst_b & ~inst_blt & ~inst_bltu & ~inst_bge & ~inst_bgeu);
assign mem_we        = inst_st_w | inst_st_b | inst_st_h ;
assign dest          = dst_is_r1 ? 5'd1 : rd;

assign rf_raddr1 = rj;
assign rf_raddr2 = src_reg_is_rd ? rd :rk;

assign rdOut = dest;
assign rs1   = rf_raddr1;
assign rs2   = rf_raddr2;

assign rj_value  = rf_rdata1;
assign rkd_value = rf_rdata2;

//assign rj_eq_rd = (rj_value == rkd_value);
//// assign br_taken = (   inst_beq  &&  rj_eq_rd
////                    || inst_bne  && !rj_eq_rd
////                    || inst_jirl
////                    || inst_bl
////                    || inst_b
////                   ) && valid;
//assign br_target = (inst_beq || inst_bne || inst_bl || inst_b) ? (pc + br_offs) :
//                                                   /*inst_jirl*/ (rj_value + jirl_offs);
//
//assign alu_src1 = src1_is_pc  ? pc[31:0] : rj_value;
//assign alu_src2 = src2_is_imm ? imm : rkd_value;
//
//assign data_sram_we    = mem_we && valid;
//assign data_sram_addr  = alu_result;
//assign data_sram_wdata = rkd_value;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //Op[6]&Op[5]&Op[4]&Op[3]&Op[2]&Op[1]&Op[0];
   
   //syscall 7'b1111111
   wire syscall = Op[6]&Op[5]&Op[4]&Op[3]&Op[2]&Op[1]&Op[0];
   //break 7'b0000001
   wire break = ~Op[6]&~Op[5]&~Op[4]&~Op[3]&~Op[2]&~Op[1]&Op[0];
   //ERET
   wire ERET = Op[6]&Op[5]&Op[4]&Op[3]&Op[2]&Op[1]&~Op[0] & STATUS[0];
   //ERETN
   wire ERETN = Op[6]&Op[5]&Op[4]&Op[3]&Op[2]&~Op[1]&~Op[0] & STATUS[0];
   
   //LUI
   wire LUI = ~Op[6]&Op[5]&Op[4]&~Op[3]&Op[2]&Op[1]&Op[0];
   //AUIPC
   wire AUIPC = ~Op[6]&~Op[5]&Op[4]&~Op[3]&Op[2]&Op[1]&Op[0];
   
  // r format 0110011
    wire rtype  = ~Op[6]&Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0110011
    wire i_add  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // add 0000000 000
    wire i_sub  = rtype& ~Funct7[6]& Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // sub 0100000 000
    wire i_or   = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]& Funct3[1]&~Funct3[0]; // or 0000000 110
    wire i_and  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]& Funct3[1]& Funct3[0]; // and 0000000 111
    wire i_xor  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]&~Funct3[1]&~Funct3[0]; // xor 0000000 100
    wire i_sll  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]& Funct3[0]; // sll 0000000 001
    wire i_slt  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]& Funct3[1]&~Funct3[0]; // slt 0000000 010
    wire i_sltu = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]& Funct3[1]& Funct3[0]; // slt 0000000 011
    wire i_srl  = rtype& ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]&~Funct3[1]& Funct3[0]; // srl 0000000 101
    wire i_sra  = rtype& ~Funct7[6]& Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]& Funct3[2]&~Funct3[1]& Funct3[0]; // sra 0100000 101

 // i format load 0000011
    wire itype_l  = ~Op[6]&~Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0000011
    wire i_lb = itype_l&~Funct3[2]&~Funct3[1]&~Funct3[0];//lb 000
    wire i_lh = itype_l&~Funct3[2]&~Funct3[1]& Funct3[0];//lh 001
    wire i_lw = itype_l&~Funct3[2]& Funct3[1]&~Funct3[0];//lw 010
    wire i_lbu = itype_l& Funct3[2]&~Funct3[1]&~Funct3[0];//lbu 100
    wire i_lhu = itype_l& Funct3[2]&~Funct3[1]& Funct3[0];//lhu 101
    
// i format 0010011
    wire itype_r  = ~Op[6]&~Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0010011
    wire i_addi  =  itype_r& ~Funct3[2]& ~Funct3[1]& ~Funct3[0]; // addi 000
    wire i_ori  =  itype_r& Funct3[2]& Funct3[1]&~Funct3[0]; // ori 110
    wire i_andi =  itype_r& Funct3[2]& Funct3[1]& Funct3[0]; // andi 111
    wire i_xori = itype_r& Funct3[2]&~Funct3[1]&~Funct3[0]; // xori 100
    wire i_slli = itype_r&~Funct3[2]&~Funct3[1]&Funct3[0]; // slli 001
    wire i_slti = itype_r&~Funct3[2]& Funct3[1]&~Funct3[0]; // slti 010
	wire i_sltiu= itype_r&~Funct3[2]& Funct3[1]& Funct3[0]; // sltiu 011
	wire i_srli = itype_r& Funct3[2]&~Funct3[1]& Funct3[0]; // srli 101
	wire i_srai = itype_r& Funct3[2]&~Funct3[1]& Funct3[0]&Funct7[5]; // srai 01000000 101
	
 //jalr
	  wire i_jalr =Op[6]&Op[5]&~Op[4]&~Op[3]&Op[2]&Op[1]&Op[0];//jalr 1100111

  // s format 0100011
    wire stype  = ~Op[6]&Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];//0100011
    wire i_sb   =  stype& ~Funct3[2]&~Funct3[1]&~Funct3[0]; // sb 000
    wire i_sh   =  stype& ~Funct3[2]&~Funct3[1]& Funct3[0]; // sh 001
    wire i_sw   =  stype& ~Funct3[2]& Funct3[1]&~Funct3[0]; // sw 010
    
  // sb format 1100011
    wire sbtype  = Op[6]&Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];//1100011
    wire i_beq  = sbtype& ~Funct3[2]& ~Funct3[1]&~Funct3[0]; // beq 000
    wire i_bne  = sbtype& ~Funct3[2]& ~Funct3[1]& Funct3[0]; // bne 001
    wire i_blt  = sbtype&  Funct3[2]& ~Funct3[1]&~Funct3[0]; // blt 100
    wire i_bge  = sbtype&  Funct3[2]& ~Funct3[1]& Funct3[0]; // bge 101
    wire i_bltu = sbtype&  Funct3[2]&  Funct3[1]&~Funct3[0]; // bltu 110
    wire i_bgeu = sbtype&  Funct3[2]&  Funct3[1]& Funct3[0]; // bgeu 111
	
 // j format
    wire i_jal  = Op[6]& Op[5]&~Op[4]& Op[3]& Op[2]& Op[1]& Op[0];  // jal 1101111

  // generate control signals
  assign RegWrite   = gr_we; // register write
  assign MemWrite   = mem_we;                           // memory write
  assign ALUSrc1    = src1_is_pc;
  assign ALUSrc2    = src2_is_imm;
  //itype_r | stype | i_jalr | LUI | AUIPC | itype_l | inst_b | // ALU B is from instruction immediate
  //                   inst_slli_w | inst_srli_w | inst_srai_w | inst_addi_w | inst_ld_w | inst_st_w | inst_lu12i_w| inst_jirl | inst_bl;   

  // signed extension
  // EXT_CTRL_ITYPE_SHAMT 6'b100000
  // EXT_CTRL_ITYPE	      6'b010000
  // EXT_CTRL_STYPE	      6'b001000
  // EXT_CTRL_BTYPE	      6'b000100
  // EXT_CTRL_UTYPE	      6'b000010
  // EXT_CTRL_JTYPE	      6'b000001
  assign EXTOp[5]    = i_slli | i_srai | i_srli | i_slti;
  assign EXTOp[4]    = i_ori | i_andi | i_jalr | i_addi | i_xori | i_sltiu | itype_l;
  assign EXTOp[3]    = stype ; 
  assign EXTOp[2]    = sbtype ; 
  assign EXTOp[1]    = LUI | AUIPC ;
  assign EXTOp[0]    = i_jal ;         

  // WDSel_FromALU 2'b00
  // WDSel_FromMEM 2'b01
  // WDSel_FromPC  2'b10
  assign WDSel[1] = /*i_jal | i_jalr | */inst_jirl | inst_bl ;
  assign WDSel[0] = /*itype_l | */inst_ld_w | inst_ld_b | inst_ld_bu | inst_ld_h | inst_ld_hu/*| inst_st_w*/;

// NPC_PLUS4      5'b00000
// NPC_BRANCH     5'b00001
// NPC_JUMP       5'b00010
// NPC_JALR       5'b00100
// NPC_SEPC       5'b01000
// NPC_SEPC_PLUS4 5'b10000
// NPC_JIRL       5'b11000
// NPC_B AND BL   5'b00110
    assign NPCOp[4] = /*ERETN | */inst_jirl;
    assign NPCOp[3] = /*ERET | */inst_ertn | inst_jirl;
    assign NPCOp[2] = i_jalr | inst_b | inst_bl;
    assign NPCOp[1] = i_jal | inst_b | inst_bl;
    assign NPCOp[0] = inst_beq | inst_bne | inst_bge | inst_blt | inst_bltu | inst_bgeu ; //& Zero;
    

// ALUOp_nop 5'b00000
// ALUOp_lui 5'b00001
// ALUOp_auipc 5'b00010
// ALUOp_add 5'b00011
// ALUOp_sub 5'b00100
// ALUOp_bne 5'b00101
// ALUOp_blt 5'b00110
// ALUOp_bge 5'b00111
// ALUOp_bltu 5'b01000
// ALUOp_bgeu 5'b01001
// ALUOp_slt 5'b01010
// ALUOp_sltu 5'b01011
// ALUOp_xor 5'b01100
// ALUOp_or 5'b01101
// ALUOp_and 5'b01110
// ALUOp_sll 5'b01111
// ALUOp_srl 5'b10000
// ALUOp_sra 5'b10001
// ALUOp_nor 5'b10010
// ALUOp_beq 5'b10011
// ALUOp_mulw 5'b10100
// ALUOp_mulhw 5'b10101
// ALUOp_mulhwu 5'b10110
// ALUOp_divw 5'b10111
// ALUOp_modw 5'b11000
// ALUOp_divwu 5'b11001
// ALUOp_modwu 5'b11010
// ALUOp_CSRs 5'b11011

assign ALUOp[4] = inst_srli_w | inst_srai_w | inst_nor | inst_srl_w | inst_sra_w | inst_beq 
                | inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu
                | inst_csrrd | inst_csrwr | inst_csrxchg;
    
assign ALUOp[3] = inst_sltu | inst_slt | inst_and | inst_or | inst_xor | inst_slli_w | inst_slti | inst_sltui | inst_andi | inst_ori | inst_xori
                | inst_sll_w | inst_bltu | inst_bgeu
                | inst_mod_w | inst_div_wu | inst_mod_wu
                | inst_csrrd | inst_csrwr | inst_csrxchg;
    
assign ALUOp[2] = inst_sub_w | inst_and | inst_or | inst_xor | inst_slli_w | inst_andi |inst_ori | inst_xori | inst_sll_w | inst_bne | inst_blt
                  | inst_bge
                  | inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w;

assign ALUOp[1] = inst_add_w | inst_sltu | inst_slt | inst_and | inst_slli_w | inst_addi_w | inst_ld_w | inst_st_w | inst_nor 
                | inst_jirl | inst_b | inst_bl | inst_slti | inst_sltui | inst_andi | inst_sll_w | inst_pcaddu12i | inst_beq | inst_blt | inst_bge
                | inst_ld_h | inst_ld_hu | inst_ld_b | inst_ld_bu | inst_st_b | inst_st_h
                | inst_mulh_wu | inst_div_w | inst_mod_wu
                | inst_csrrd | inst_csrwr | inst_csrxchg;
	  
assign ALUOp[0] = inst_add_w | inst_sltu | inst_or | inst_slli_w | inst_srai_w | inst_addi_w | inst_ld_w | inst_st_w 
                | inst_jirl | inst_b | inst_bl | inst_lu12i_w | inst_sltui | inst_ori | inst_sll_w | inst_sra_w | inst_pcaddu12i | inst_beq | inst_bne | inst_bge
                | inst_ld_h | inst_ld_hu | inst_ld_b | inst_ld_bu | inst_st_b | inst_st_h | inst_bgeu
                | inst_mulh_w | inst_div_w | inst_div_wu
                | inst_csrrd | inst_csrwr | inst_csrxchg;
	
//DM_Type: 
//assign DMType = {2'b0, inst_st_w};

// dm_word 3'b000
// dm_halfword 3'b001
// dm_halfword_unsigned 3'b010
// dm_byte 3'b011
// dm_byte_unsigned 3'b100
    //assign DMType[2] = i_lbu;
    //assign DMType[1] = i_lb | i_lhu | i_sb;
    //assign DMType[0] = i_lb | i_lh | i_sb | i_sh;
    assign DMType[2] = inst_ld_bu;
    assign DMType[1] = inst_ld_b | inst_ld_hu | inst_st_b;
    assign DMType[0] = inst_ld_b | inst_ld_h | inst_st_b | inst_st_h;


    assign INE = ~
    (        (PC == 32'b0) // reset
    |        inst_add_w
    |        inst_sub_w
    |        inst_slt
    |        inst_sltu
    |        inst_nor
    |        inst_and
    |        inst_or
    |        inst_xor
    |        inst_slli_w
    |        inst_srli_w
    |        inst_srai_w
    |        inst_addi_w
    |        inst_ld_w
    |        inst_st_w
    |        inst_jirl
    |        inst_b
    |        inst_bl
    |        inst_beq
    |        inst_bne
    |        inst_lu12i_w
    |        inst_slti
    |        inst_sltui
    |        inst_andi
    |        inst_ori
    |        inst_xori
    |        inst_sll_w
    |        inst_srl_w
    |        inst_sra_w
    |        inst_pcaddu12i
    |        inst_st_b
    |        inst_st_h
    |        inst_ld_b
    |        inst_ld_h
    |        inst_ld_hu
    |        inst_ld_bu
    |        inst_blt
    |        inst_bge
    |        inst_bltu
    |        inst_bgeu
    |        inst_mul_w
    |        inst_mulh_w
    |        inst_mulh_wu
    |        inst_div_w
    |        inst_mod_w
    |        inst_div_wu
    |        inst_mod_wu
    |        inst_csrrd
    |        inst_csrwr
    |        inst_csrxchg
    |        inst_ertn
    |        inst_break
    |        inst_syscall
    );
    
    /*wire UNDEFINED = ~(syscall | break | ERET | ERETN | LUI | AUIPC | i_add  | i_sub | i_or | i_and | i_xor | i_sll | i_slt | i_sltu | i_srl | i_sra |
                i_lb | i_lh | i_lw | i_lbu | i_lhu | i_addi | i_ori | i_andi | i_xori | i_slli | i_slti | i_sltiu | i_srli | i_srai |
                i_jalr | i_sb | i_sh | i_sw | i_beq | i_bne | i_blt | i_bge | i_bltu | i_bgeu | i_jal);
    
    //assign SCAUSE = {2'b0, 1'b0, 2'b0, break, syscall, 1'b0};
    assign ID_SCAUSE = {5'b0, break, syscall, 1'b0};
    assign EXL_Clear = ERET | ERETN;*/

endmodule