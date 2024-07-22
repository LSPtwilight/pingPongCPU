`include "ctrl_encode_def.v"
module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire [ 3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,

    output wire        inst_sram_en,
    output wire        data_sram_en
);

assign inst_sram_we    = 1'b0;
assign inst_sram_en     = 1'b1;
assign inst_sram_wdata = 32'h0;
assign inst_sram_addr=NPC;

reg         reset;
always @(posedge clk) reset <= ~resetn;

    wire        RegWrite;    // control signal to register write
    wire [5:0]  EXTOp;       // control signal to signed extension
    wire [4:0]  ALUOp;       // ALU opertion
    wire [4:0]  NPCOp;       // next PC operation

    wire [1:0]  WDSel;       // (register) write data selection
    wire [1:0]  GPRSel;      // general purpose register selection
   
    wire        ALUSrc1;      // ALU source for A
    wire        ALUSrc2;      // ALU source for B
    wire        Zero;        // ALU ouput zero
    wire        [2:0] DM_type;
    wire [31:0] pc;    
    wire [31:0] NPC;         // next PC


    wire [4:0]  rs1;          // rs
    wire [4:0]  rs2;          // rt
    wire [4:0]  rd;          // rd
    wire [6:0]  Op;          // opcode
    wire [6:0]  Funct7;       // funct7
    wire [2:0]  Funct3;       // funct3
    wire [11:0] Imm12;       // 12-bit immediate
    wire [31:0] Imm32;       // 32-bit immediate
    wire [19:0] IMM;         // 20-bit immediate (address)
    wire [4:0]  A3;          // register address for write
    reg [31:0] WD;          // register write data
    wire [31:0] RD1,RD2;         // register data specified by rs
    wire [31:0] A;            //operator for ALU A
    wire [31:0] B;           // operator for ALU B
	
	wire [4:0] iimm_shamt;
	wire [11:0] iimm,simm,bimm;
	wire [19:0] uimm,jimm;
	wire [31:0] imm4alu,imm4pc;
	
    wire MemRead;

	//Exception wires and regs
	wire [7:0] ID_SCAUSE;
	wire [7:0] EX_SCAUSE;
	wire [2:0] INT_PEND;
	reg [31:0] SEPC;
	reg [7:0] STATUS;
	reg [7:0] INTMASK;
	wire EXL = STATUS[0];
	wire IE = STATUS[1];
	wire ID_EXL_Clear;
	wire EXL_Clear;
	wire EXL_Set;
	wire INT_Signal;
	
	//EX wires
	wire [4:0] EX_rd;
    wire [4:0] EX_rs1;
    wire [4:0] EX_rs2;
    wire [31:0] EX_imm4alu, EX_imm4pc;
    wire [31:0] EX_RD1;
    wire [31:0] EX_RD2;
    wire        EX_RegWrite;//RFWr
    wire        EX_MemWrite;//DMWr
    wire [4:0] EX_ALUOp;
    wire [4:0] EX_NPCOp;
    wire       EX_ALUSrc1;
    wire       EX_ALUSrc2;
    wire [2:0] EX_DMType;
    wire [1:0] EX_WDSel;
    wire EX_MemRead;
    wire [31:0] EX_pc;
    wire [31:0] ID_pc;
	
	//MEM wires
	wire [4:0] MEM_rd;
	wire [31:0] MEM_RD2;
	wire [31:0] MEM_aluout;
	wire        MEM_RegWrite;
	wire        MEM_MemWrite;
	wire [2:0] MEM_DMType;
	wire [1:0] MEM_WDSel;

    assign DMType = MEM_DMType;
    
    //WB wires
    wire [4:0]  WB_rd;
    wire [31:0] WB_aluout;
    wire [31:0] WB_MemData;
    wire        WB_RegWrite;
    wire [1:0]  WB_WDSel;
	wire [31:0] WB_pc;
	
	//Forwarding control wires
	wire [1:0] ForwardA;//first alu source: alu_in1
	wire [1:0] ForwardB;//second alu source:alu_in2
	reg [31:0] alu_in1;//to be chosen by forwarding unit
	reg [31:0] alu_in2;
	
	//Hazard detection unit wires
	wire stall_signal;
	
    wire [31:0] aluout;
    assign data_sram_addr = (EX_MemRead|EX_MemWrite)? aluout: 32'h0;
	assign data_sram_wdata = EX_MemWrite? alu_in2: 32'h0;
    assign data_sram_we = {4{EX_MemWrite}};
    //assign B = (EX_ALUSrc) ? EX_immout : EX_RD2;//whether from EXT

	
	wire [31:0] instr;
	
	assign iimm_shamt=instr[24:20];
	assign iimm=instr[31:20];
	assign simm={instr[31:25],instr[11:7]};
	assign bimm={instr[31],instr[7],instr[30:25],instr[11:8]};
	assign uimm=instr[31:12];
	assign jimm={instr[31],instr[19:12],instr[20],instr[30:21]};
   
    assign Op = instr[6:0];  // instruction
    assign Funct7 = instr[31:25]; // funct7
    assign Funct3 = instr[14:12]; // funct3
    // assign rs1 = instr[19:15];  // rs1
    // assign rs2 = instr[24:20];  // rs2
    // assign rd = instr[11:7];  // rd
    assign Imm12 = instr[31:20];// 12-bit immediate
    assign IMM = instr[31:12];  // 20-bit immediate
    
    //loads
    //wire MemRead  = ~Op[6]&~Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0000011
    
    
   
   wire ID_MemWrite;
   wire [2:0] ID_DMType;
   
   // instantiation of control unit
	ctrl U_ctrl(
	    .STATUS(STATUS),//.reset(reset),
        .inst(instr),
		.Op(7'b0), 
        .Funct7(7'b0), 
        .Funct3(3'b0), 
        .Zero(Zero), 
		.RegWrite(RegWrite), 
        .MemWrite(ID_MemWrite),
        .MemRead(MemRead),
		.EXTOp(EXTOp), 
        .ALUOp(ALUOp), 
        .NPCOp(NPCOp), 
        .imm4alu(imm4alu),
        .imm4pc(imm4pc),
		.ALUSrc1(ALUSrc1), 
        .ALUSrc2(ALUSrc2),
        .GPRSel(GPRSel), 
        .WDSel(WDSel),
		.DMType(ID_DMType), 
		.ID_SCAUSE(ID_SCAUSE), 
        .EXL_Clear(ID_EXL_Clear),
        .rs1(rs1), 
        .rs2(rs2), 
        .rdOut(rd)
	);
 // instantiation of pc unit
	//PC U_PC(.clk(clk), .rst(reset), .write(~stall_signal), .NPC(NPC), .PC(pc) );
    PC U_PC(.clk(clk), .rst(reset), .NPC(NPC), .PC(pc) );
	//NPC U_NPC(.PC(pc), .EX_pc(EX_pc), .ID_pc(ID_pc), .SEPC(SEPC), .NPCOp(NPCOp),
	//           .INT_Signal(INT_Signal), .INT_PEND(INT_PEND), .IMM(immout/*?*/), .NPC(NPC), .aluout(aluout));

	NPC U_NPC(.PC(pc), .EX_pc(EX_pc), .ID_pc(EX_pc), .SEPC(SEPC), .NPCOp(EX_NPCOp),
	           .INT_Signal(INT_Signal), .INT_PEND(INT_PEND), .IMM(EX_imm4pc/*?*/), .NPC(NPC), .EX_RD1(EX_RD1), .branch_flag(Zero), .stall_signal(stall_signal));

	EXT U_EXT(
		.iimm_shamt(iimm_shamt), .iimm(iimm), .simm(simm), .bimm(bimm),
		.uimm(uimm), .jimm(jimm),
		.EXTOp(EXTOp)//.immout(immout)
	);
	RF U_RF(
		.clk(clk), .rst(reset),
		.RFWr(WB_RegWrite), 
		.A1(rs1), .A2(rs2), .A3(WB_rd), 
		.WD(WD), 
		.RD1(RD1), .RD2(RD2)
		//.reg_sel(reg_sel),
		//.reg_data(reg_data)
	);
// instantiation of alu unit
	alu U_alu(.A(A), .B(B), .ALUOp(EX_ALUOp), .C(aluout), .Zero(Zero), .PC(EX_pc));
	
//please connnect the CPU by yourself

//WD MUX
always @*
begin
	case(WB_WDSel)
		`WDSel_FromALU: WD<=WB_aluout;
		`WDSel_FromMEM: WD<=WB_MemData;
		`WDSel_FromPC:  WD<=WB_pc+4;
        //`WDSel_FromImm: WD<=WB_immout;
	endcase
end



//-----Exceptions-----------------

    ExceptionCtrl exception_ctrl(
    .STATUS(STATUS),
    .EX_SCAUSE(EX_SCAUSE),
    //.INTMASK(SW),//
    .INTMASK(8'b0),
    .EXL_Set(EXL_Set),
    .INT_Signal(INT_Signal),
    .INT_PEND(INT_PEND)
    );

    always@(posedge clk, posedge reset)
    begin
        if(reset)
        begin
        SEPC <= 32'b0;
        STATUS <= 8'h02;//��ȫ���ж�
        INTMASK <= 8'hFF;
        end
        else begin
        STATUS[0] <= (STATUS[0] | EXL_Set) & ~EXL_Clear;
        if(INT_Signal) begin SEPC <= EX_pc; end
        end
    end
    
    // dm_word 3'b000
// dm_halfword 3'b001
// dm_halfword_unsigned 3'b010
// dm_byte 3'b011
// dm_byte_unsigned 3'b100
    wire BadAddr = (EX_MemRead | EX_MemWrite) & (~EX_DMType[2]&~EX_DMType[1]&~EX_DMType[0]&(aluout[0]|aluout[1]) 
                   | ((~EX_DMType[2]&~EX_DMType[1]& EX_DMType[0] | ~EX_DMType[2]& EX_DMType[1]&~EX_DMType[0]) & aluout[0]));
   // ALUOp_add 5'b00011
   // ALUOp_sub 5'b00100
    wire IntOverflow = ~EX_ALUOp[4]&~EX_ALUOp[3]&~EX_ALUOp[2]& EX_ALUOp[1]& EX_ALUOp[0] & ( A[31]&B[31]&~aluout[31] | ~A[31]&~B[31]&aluout[31]);


//-----Branch or Jump-------------

    wire branch_b_and_bl = (EX_NPCOp==`NPC_BANDBL);
    wire branch_bxx = ((EX_NPCOp==`NPC_BRANCH)&&Zero);
    wire branch_jirl = (EX_NPCOp==`NPC_JIRL);
    wire Branch_or_Jump = branch_b_and_bl|branch_bxx|branch_jirl;

//-----HazardDetectionUnit--------

    HazardDetectionUnit hazardunit(.EX_MemRead(EX_MemRead),
    .ID_rs1(rs1),
    .ID_rs2(rs2),
    .EX_rd(EX_rd),
    .stall_signal(stall_signal)
    );

//-----ForwardingUnit-------------
    ForwardingUnit forwadingUnitA(.MEM_RegWrite(MEM_RegWrite), .MEM_rd(MEM_rd), 
        .WB_RegWrite(WB_RegWrite), .WB_rd(WB_rd), .EX_rs(EX_rs1), .ForwardSignal(ForwardA));
    ForwardingUnit forwadingUnitB(.MEM_RegWrite(MEM_RegWrite), .MEM_rd(MEM_rd), 
        .WB_RegWrite(WB_RegWrite), .WB_rd(WB_rd), .EX_rs(EX_rs2), .ForwardSignal(ForwardB));
    // MUX Gate 
    always @(*)
    begin
        case(ForwardA)
            2'b00: alu_in1 <= EX_RD1;//from regfile
            2'b10: alu_in1 <= MEM_aluout;//from MEM
            2'b01: alu_in1 <= WD;    // from WB
        endcase
    end
    always @(*)
    begin
        case(ForwardB)
            2'b00: alu_in2 <= EX_RD2;//from regfile
            2'b10: alu_in2 <= MEM_aluout;//from MEM
            2'b01: alu_in2 <= WD;    // from WB
        endcase
    end
    
    assign A = (EX_ALUSrc1) ? EX_pc : alu_in1;
    assign B = (EX_ALUSrc2) ? EX_imm4alu : alu_in2;//whether from EXT

//-----pipe registers--------------

    //IF_ID: [31:0]PC [31:0]instr
    wire IF_ID_write_enable = ~stall_signal;
    wire IF_ID_flush = Branch_or_Jump | INT_Signal;
    wire [71:0] IF_ID_in;
    assign IF_ID_in[31:0] = pc;//?????????????????????
    assign IF_ID_in[63:32] = inst_sram_rdata;
    //assign IF_ID_in[71:64] = {EXINT, 7'b0};
    assign IF_ID_in[71:64] = 8'b0;
    wire [71:0] IF_ID_out;
    //assign instr = IF_ID_out[63:32];
    assign instr = IF_ID_out[63:32];
    //assign ID_pc = IF_ID_out[31:0];
    GRE_array #(.WIDTH(72))
    IF_ID
    (.clk(clk), .rst(reset), .write_enable(IF_ID_write_enable), .flush(IF_ID_flush),
    .in(IF_ID_in), .out(IF_ID_out));

    gnrl_dff #(32) if_id_pc(clk, reset, IF_ID_write_enable, IF_ID_flush, pc, ID_pc);
    
    

    //ID_EX
    wire ID_EX_write_enable = 1;
    wire ID_EX_flush = stall_signal | Branch_or_Jump | INT_Signal;
    wire [203:0] ID_EX_in;
    assign ID_EX_in[31:0] = IF_ID_out[31:0];//PC
    assign ID_EX_in[36:32] = rd;
    assign ID_EX_in[41:37] = rs1;
    assign ID_EX_in[46:42] = rs2;
    assign ID_EX_in[78:47] = imm4alu;
    assign ID_EX_in[110:79] = RD1;
    assign ID_EX_in[142:111] = RD2;
    assign ID_EX_in[143] = RegWrite;//RFWr
    assign ID_EX_in[144] = ID_MemWrite;//DMWr
    assign ID_EX_in[149:145] = ALUOp;
    assign ID_EX_in[154:150] = NPCOp;
    assign ID_EX_in[155] = ALUSrc1;
    assign ID_EX_in[158:156] = ID_DMType;
    assign ID_EX_in[160:159] = WDSel;
    assign ID_EX_in[161] = MemRead;//warning
    assign ID_EX_in[169:162] = IF_ID_out[71:64] | ID_SCAUSE;
    assign ID_EX_in[170] = ID_EXL_Clear;
    assign ID_EX_in[202:171] = imm4pc;
    assign ID_EX_in[203] = ALUSrc2;
    //assign ID_pc = ID_EX_in[31:0]; 
    wire [203:0] ID_EX_out;
    assign EX_rd = ID_EX_out[36:32];
    assign EX_rs1 = ID_EX_out[41:37];
    assign EX_rs2 = ID_EX_out[46:42];
    assign EX_imm4alu = ID_EX_out[78:47];
    assign EX_RD1 = ID_EX_out[110:79];
    assign EX_RD2 = ID_EX_out[142:111];
    assign EX_RegWrite = ID_EX_out[143];//RFWr
    assign EX_MemWrite = ID_EX_out[144];//DMWr
    assign EX_ALUOp = ID_EX_out[149:145];
    assign EX_NPCOp = {ID_EX_out[154:150]};
    assign EX_ALUSrc1 = ID_EX_out[155];
    assign EX_DMType = ID_EX_out[158:156];
    assign EX_WDSel = ID_EX_out[160:159];
    assign EX_MemRead = ID_EX_out[161];
    //assign EX_pc = ID_EX_out[31:0];
    assign EX_imm4pc = ID_EX_out[202:171];
    assign EX_ALUSrc2 = ID_EX_out[203];
    
    assign EX_SCAUSE = {ID_EX_out[169:167], ID_EX_out[166:165] | {BadAddr, IntOverflow}, ID_EX_out[164:162]};
    assign EXL_Clear = ID_EX_out[170];

    GRE_array #(.WIDTH(204))
    ID_EX
    (.clk(clk), .rst(reset), .write_enable(ID_EX_write_enable), .flush(ID_EX_flush),
    .in(ID_EX_in), .out(ID_EX_out));

    gnrl_dff #(32) id_ex_pc(clk, reset, ID_EX_write_enable, ID_EX_flush, ID_pc, EX_pc);
    
    //EX_MEM
    wire EX_MEM_write_enable = 1;
    wire EX_MEM_flush = INT_Signal;//warning
    wire [108:0] EX_MEM_in;
    assign EX_MEM_in[31:0] = ID_EX_out[31:0];//PC
    assign EX_MEM_in[36:32] = EX_rd;//rd
    assign EX_MEM_in[68:37] = alu_in2;//RD2 updated!!!
    assign EX_MEM_in[100:69] = aluout;
    assign EX_MEM_in[101] = EX_RegWrite;
    assign EX_MEM_in[102] = EX_MemWrite;
    assign EX_MEM_in[105:103] = EX_DMType;
    assign EX_MEM_in[107:106] = EX_WDSel;

    assign EX_MEM_in[108] = EX_MemRead;

    wire [108:0] EX_MEM_out;
    assign MEM_rd = EX_MEM_out[36:32];
    assign MEM_RD2 = EX_MEM_out[68:37];
    assign MEM_aluout = EX_MEM_out[100:69];
    assign MEM_RegWrite = EX_MEM_out[101];
    assign MEM_MemWrite = EX_MEM_out[102];
    assign MEM_DMType = EX_MEM_out[105:103];
    assign MEM_WDSel = EX_MEM_out[107:106];
    wire MEM_MemRead;
    assign MEM_MemRead = EX_MEM_out[108];
    GRE_array #(.WIDTH(109))
    EX_MEM
    (.clk(clk), .rst(reset), .write_enable(EX_MEM_write_enable), .flush(EX_MEM_flush),
    .in(EX_MEM_in), .out(EX_MEM_out));

    wire [31:0] MEM_pc;
    gnrl_dff #(32) ex_mem_pc(clk, reset, EX_MEM_write_enable, EX_MEM_flush, EX_pc, MEM_pc);
    
    assign data_sram_en = EX_MemRead|EX_MemWrite;


    //MEM_WB
    wire MEM_WB_write_enable = 1;
    wire MEM_WB_flush = 0;
    wire [103:0] MEM_WB_in;
    assign MEM_WB_in[31:0] = EX_MEM_out[31:0];
    assign MEM_WB_in[36:32] = MEM_rd;
    assign MEM_WB_in[68:37] = MEM_aluout;
    assign MEM_WB_in[100:69] = data_sram_rdata;
    assign MEM_WB_in[101] = MEM_RegWrite;
    assign MEM_WB_in[103:102] = MEM_WDSel;
    wire [103:0] MEM_WB_out;
    //assign WB_pc = MEM_WB_out[31:0];
    assign WB_rd = MEM_WB_out[36:32];
    assign WB_aluout = MEM_WB_out[68:37];
    assign WB_MemData = MEM_WB_out[100:69];
    assign WB_RegWrite = MEM_WB_out[101];
    assign WB_WDSel = MEM_WB_out[103:102];
    GRE_array #(.WIDTH(104))
    MEM_WB
    (.clk(clk), .rst(reset), .write_enable(MEM_WB_write_enable), .flush(MEM_WB_flush),
    .in(MEM_WB_in), .out(MEM_WB_out));

    gnrl_dff #(32) mem_wb_pc(clk, reset, MEM_WB_write_enable, MEM_WB_flush, MEM_pc, WB_pc);


assign debug_wb_pc       = WB_pc;
assign debug_wb_rf_we   = {4{WB_RegWrite}};
assign debug_wb_rf_wnum  = WB_rd;
assign debug_wb_rf_wdata = WD;

endmodule