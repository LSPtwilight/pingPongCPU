`include "ctrl_encode_def.v"
`include "CSR_def.v"

module mycpu_top(
    input  wire        aclk,
    input  wire        aresetn,
    // inst sram interface
    //output wire        inst_sram_we,
    //output wire [31:0] inst_sram_addr,
    //output wire [31:0] inst_sram_wdata,
    //input  wire [31:0] inst_sram_rdata,
    // data sram interface
    //output wire [ 3:0] data_sram_we,
    //output wire [31:0] data_sram_addr,
    //output wire [31:0] data_sram_wdata,
    //input  wire [31:0] data_sram_rdata,

    output    wire    [ 3 : 0 ]    arid    ,
    output    wire    [ 31: 0 ]    araddr  ,
    output    wire    [ 7 : 0 ]    arlen   ,
    output    wire    [ 2 : 0 ]    arsize  ,
    output    wire    [ 1 : 0 ]    arburst ,
    output    wire    [ 1 : 0 ]    arlock  ,
    output    wire    [ 3 : 0 ]    arcache ,
    output    wire    [ 2 : 0 ]    arprot  ,
    output    wire                 arvalid ,
    input     wire                 arready ,
               
    input     wire    [ 3 : 0 ]    rid     ,
    input     wire    [ 31: 0 ]    rdata   ,
    input     wire    [ 1 : 0 ]    rresp   ,
    input     wire                 rlast   ,
    input     wire                 rvalid  ,
    output    wire                 rready  ,
               
    output    wire    [ 3 : 0 ]    awid    ,
    output    wire    [ 31: 0 ]    awaddr  ,
    output    wire    [ 7 : 0 ]    awlen   ,
    output    wire    [ 2 : 0 ]    awsize  ,
    output    wire    [ 1 : 0 ]    awburst ,
    output    wire    [ 1 : 0 ]    awlock  ,
    output    wire    [ 3 : 0 ]    awcache ,
    output    wire    [ 2 : 0 ]    awprot  ,
    output    wire                 awvalid ,
    input     wire                 awready ,
    
    output    wire    [ 3 : 0 ]    wid     ,
    output    wire    [ 31: 0 ]    wdata   ,
    output    wire    [ 1 : 0 ]    wstrb   ,
    output    wire                 wlast   ,
    output    wire                 wvalid  ,
    input     wire                 wready  ,
    
    input     wire    [ 3 : 0 ]     bid     ,
    input     wire    [ 1 : 0 ]     bresp   ,
    input     wire                  bvalid  ,
    output    wire                  bready  ,

    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,

    output wire        inst_sram_en,
    output wire        data_sram_en
);

wire [31:0] NPC;         // next PC


wire        inst_sram_we        ;
wire [31:0] inst_sram_addr      ;
wire [31:0] inst_sram_wdata     ;
wire [31:0] inst_sram_rdata     ;

reg inst_sram_en_req;

assign inst_sram_we    = 1'b0;
assign inst_sram_en    = inst_sram_en_req;
assign inst_sram_wdata = 32'h0;
assign inst_sram_addr=NPC;

wire clk = aclk;



wire [ 3:0] data_sram_we        ;
wire [31:0] data_sram_addr      ;
wire [31:0] data_sram_wdata     ;
wire [31:0] data_sram_rdata     ;

wire wr_data = |data_sram_we;
wire wr_inst = |inst_sram_we;

reg [2:0] size_data;
wire [2:0] size_inst;
wire data_ok_data;
wire data_ok_inst;
wire addr_ok_data;
wire addr_ok_inst;

wire req_inst_success;


sram2axi my_sram2axi(
.clk(clk),
.resetn(aresetn),
//axi接口
.arid    (arid   ),
.araddr  (araddr ),
.arlen   (arlen  ),
.arsize  (arsize ),
.arburst (arburst),
.arlock  (arlock ),
.arcache (arcache),
.arprot  (arprot ),
.arvalid (arvalid),
.arready (arready),
.rid     (rid    ),
.rdata   (rdata  ),
.rresp   (rresp  ),
.rlast   (rlast  ),
.rvalid  (rvalid ),
.rready  (rready ),
.awid    (awid   ),
.awaddr  (awaddr ),
.awlen   (awlen  ),
.awsize  (awsize ),
.awburst (awburst),
.awlock  (awlock ),
.awcache (awcache),
.awprot  (awprot ),
.awvalid (awvalid),
.awready (awready),
.wid     (wid    ),
.wdata   (wdata  ),
.wstrb   (wstrb  ),
.wlast   (wlast  ),
.wvalid  (wvalid ),
.wready  (wready ),
.bid     (bid    ),
.bresp   (bresp  ),
.bvalid  (bvalid ),
.bready  (bready ),

//类sram接口
.req_data                        ( data_sram_en ),
.wr_data                         ( wr_data ),
.size_data                       ( size_data ),
.addr_data                       ( data_sram_addr ),
.wstrb_data                      ( data_sram_we ),
.wdata_data                      ( data_sram_wdata ),
.addr_ok_data                    ( addr_ok_data ),
.data_ok_data                    ( data_ok_data ),
.rdata_data                      ( data_sram_rdata ),

.req_inst                        ( inst_sram_en ),
.wr_inst                         ( wr_inst ),
.size_inst                       ( size_inst ),
.addr_inst                       ( inst_sram_addr ),
.wstrb_inst                      ( 4'b0 ),
.wdata_inst                      ( inst_sram_wdata ),
.addr_ok_inst                    ( addr_ok_inst ),
.data_ok_inst                    ( data_ok_inst ),
.rdata_inst                      ( inst_sram_rdata )


);


reg         reset;
always @(posedge clk) reset <= ~aresetn;

assign req_inst_success = (!inst_sram_en_req)&&(addr_ok_inst)&&(!reset);

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

    wire [13:0] csr_num;
    wire csr_mask_en;
    wire csr_write;
    wire [31:0] csr_data;
	
    wire MemRead;

	//Exception wires and regs
	/*wire [7:0] ID_SCAUSE;
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
	wire INT_Signal;*/

    // divs
    wire signed_div;
    wire div_start;
    wire div_ready;
    wire div_busy;
    wire [63:0] div_result;

    // muls
    wire signed_mul;
    wire mul_start;
    wire mul_ready;
    wire mul_busy;
    wire [63:0] mul_result;

    // exceptions
    wire        exc_sig;
    /*--exception request signals--*/
    wire        INT;
    wire        PIL;
    wire        PIS;
    wire        PIF;
    wire        PME;
    wire        PPI;
    wire        ADEF;
    wire        ADEM;
    wire        ALE;
    wire        SYS;
    wire        BRK;
    wire        INE;
    wire        IPE;
    wire        FPD;
    wire        FPE;
    wire        TLBR;
    /*-----------------------------*/
    wire        ERTN;
    wire        EX_ERTN;
    wire        IE;
    wire [31:0] ERA;
    wire [31:0] EENTRY;
    wire        IF_exc_req_in;
    wire        IF_exc_req_out;
    wire [ 5:0] IF_Ecode_in;
    wire [ 8:0] IF_EsubCode_in;
    wire [ 5:0] IF_Ecode_out;
    wire [ 8:0] IF_EsubCode_out;
    wire        ID_exc_req_in;
    wire        ID_exc_req_out;
    wire [ 5:0] ID_Ecode_in;
    wire [ 8:0] ID_EsubCode_in;
    wire [ 5:0] ID_Ecode_out;
    wire [ 8:0] ID_EsubCode_out;
    wire        EX_exc_req_in;
    wire        EX_exc_req_out;
    wire [ 5:0] EX_Ecode_in;
    wire [ 8:0] EX_EsubCode_in;
    wire [ 5:0] EX_Ecode_out;
    wire [ 8:0] EX_EsubCode_out;
    wire        MEM_exc_req_in;
    wire        MEM_exc_req_out;
    wire [ 5:0] MEM_Ecode_in;
    wire [ 8:0] MEM_EsubCode_in;
    wire [ 5:0] MEM_Ecode_out;
    wire [ 8:0] MEM_EsubCode_out;
	
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

    wire [31:0] EX_csr_data;
    reg  [31:0] alu_csr_data;
    wire [13:0] EX_csr_num;
    wire EX_csr_mask_en;
    wire EX_csr_write;
    wire [31:0] EX_csr_wd;
    wire Branch_or_Jump;
	
	//MEM wires
	wire [4:0] MEM_rd;
	wire [31:0] MEM_RD2;
	wire [31:0] MEM_aluout;
	wire        MEM_RegWrite;
	wire        MEM_MemWrite;
	wire [2:0] MEM_DMType;
	wire [1:0] MEM_WDSel;
    wire [13:0] MEM_csr_num;
    wire MEM_csr_write;
    wire [31:0] MEM_csr_wd;

    //assign DMType = MEM_DMType;
    
    //WB wires
    wire [4:0]  WB_rd;
    wire [31:0] WB_aluout;
    wire [31:0] WB_MemData;
    wire        WB_RegWrite;
    wire [1:0]  WB_WDSel;
	wire [31:0] WB_pc;
    wire [13:0] WB_csr_num;
    wire WB_csr_write;
    wire [31:0] WB_csr_wd;
	
	//Forwarding control wires
	wire [1:0] ForwardA;//first alu source: alu_in1
	wire [1:0] ForwardB;//second alu source:alu_in2
	reg [31:0] alu_in1;//to be chosen by forwarding unit
	reg [31:0] alu_in2;
    wire [1:0] ForwardCSR2ALU;
	
	//Hazard detection unit wires
	wire stall_signal;
	
    wire [31:0] aluout;
    assign data_sram_addr = (EX_MemRead|EX_MemWrite)? aluout: 32'h0;
	//assign data_sram_wdata = EX_MemWrite? alu_in2: 32'h0;
    //assign data_sram_we = {4{EX_MemWrite}};
    //assign data_sram_addr = (MEM_MemRead|MEM_MemWrite)? MEM_aluout: 32'h0;
    //assign data_sram_wdata = MEM_MemWrite? MEM_RD2: 32'h0;
    //assign data_sram_we = {4{MEM_MemWrite}};
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
   wire [31:0] ID_pc_;
   
   // instantiation of control unit
	ctrl U_ctrl(
	    .STATUS(STATUS),//.reset(reset),
        .PC(ID_pc_),
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
        .rdOut(rd),
        .csr_num(csr_num), 
        .csr_mask_en(csr_mask_en), 
        .csr_write(csr_write),
        .ERTN(ERTN),
        .SYS(SYS), .BRK(BRK), .INE(INE)
	);
 // instantiation of pc unit
	//PC U_PC(.clk(clk), .rst(reset), .write(~stall_signal), .NPC(NPC), .PC(pc) );
    PC U_PC(.clk(clk), .rst(reset), .NPC(NPC), .PC(pc) );
	//NPC U_NPC(.PC(pc), .EX_pc(EX_pc), .ID_pc(ID_pc), .SEPC(SEPC), .NPCOp(NPCOp),
	//           .INT_Signal(INT_Signal), .INT_PEND(INT_PEND), .IMM(immout/*?*/), .NPC(NPC), .aluout(aluout));

	NPC U_NPC(.PC(pc), .EX_pc(EX_pc), .ID_pc(EX_pc), .SEPC(ERA), .NPCOp(EX_NPCOp),
	           .exc_sig(exc_sig), .EENTRY(EENTRY), .IMM(EX_imm4pc/*?*/), .NPC(NPC), .EX_RD1(EX_RD1), 
               .branch_flag(Zero), .stall_signal(stall_signal | div_busy | mul_busy),
               .req_inst_success(req_inst_success));

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

    CSR U_CSR(
        .clk(clk), 
        .rst(reset),
        /* reading and writing */
        .csr_write  (WB_csr_write)/*WB*/, 
        .read_addr  (csr_num)     /*ID*/,
        .write_addr (WB_csr_num)  /*WB*/, 
        .WD         (WB_csr_wd)   /*WB*/, 
        //output
        .RDout      (csr_data),   /*ID*/
        /* forwarding */
        .MEM_csr_write (MEM_csr_write),
        .MEM_csr_num   (MEM_csr_num),
        .MEM_csr_wd    (MEM_csr_wd),
        /* external INT signals */
        .HWI_in(8'b0),
        //.TI_in (1'b0),
        .IPI_in(1'b0),
        // CPU states
        //.IE(IE),//output
        /* exceptions */
        .exc_sig   (exc_sig),
        .Ecode     (MEM_Ecode_out),
        .EsubCode  (MEM_EsubCode_out),
        .MEM_aluout(MEM_aluout), // for BADV
        .PC        (MEM_pc),//interupted instruction
        .ERTN      (EX_ERTN),
        .EENTRY_out(EENTRY),//output
        .ERA_out   (ERA),//output
        .INT(INT)
    );

    csr_alu U_csr_alu(
        .rd(/*EX_RD2*/alu_in2),
        .rj(/*EX_RD1*/alu_in1), 
        .csr_data(/*EX_csr_data*/alu_csr_data),
        .csr_num(EX_csr_num), /*write_addr*/
        .mask_en(EX_csr_mask_en), /*for csrxchg*/
        //output
        .csr_wd(EX_csr_wd)
    );
// instantiation of alu unit
	alu U_alu(.A(A), .B(B), .ALUOp(EX_ALUOp), .C(aluout), .Zero(Zero), .PC(EX_pc), 
        .csr_data(alu_csr_data), .div_result(div_result), .mul_result(mul_result));

// div module
    div u_div(.clk(clk), .resetn(~reset),
	
	.opdata1_i   (alu_in1   ),
	.opdata2_i   (alu_in2   ),
	.signed_div_i(signed_div),
	.start_i     (div_start ),
	.annul_i     (exc_sig | Branch_or_Jump),	// cancel division, seems never happen
	
	.result_o    (div_result),
	.ready_o     (div_ready )
    );
    
    assign signed_div = EX_ALUOp==`ALUOp_divw || EX_ALUOp==`ALUOp_modw;
    assign div_start = EX_ALUOp == `ALUOp_divw
                    || EX_ALUOp == `ALUOp_modw
                    || EX_ALUOp == `ALUOp_divwu
                    || EX_ALUOp == `ALUOp_modwu;
    assign div_busy = div_start & ~div_ready;

// mul module
    mul u_mul(.clk(clk), .resetn(~reset),
	
	.opdata1_i   (alu_in1),
	.opdata2_i   (alu_in2),
	.signed_mul_i(signed_mul),
	.start_i     (mul_start),
	.annul_i     (exc_sig | Branch_or_Jump),	// cancel division
	
	.result_o    (mul_result),
	.ready_o     (mul_ready)
    );

    assign signed_mul = EX_ALUOp==`ALUOp_mulw || EX_ALUOp==`ALUOp_mulhw;
    assign mul_start  = EX_ALUOp==`ALUOp_mulhw|| EX_ALUOp==`ALUOp_mulhwu || EX_ALUOp==`ALUOp_mulw;
    assign mul_busy   = mul_start & ~mul_ready;
	
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

//-----dm_controller--------------
wire [31:0] data_read/* = data_sram_rdata*/;
wire [3:0] wea_mem;
Dm_Controller dm_controller(
  // EX
  .mem_w(EX_MemWrite), .EX_Addr_in(data_sram_addr), 
  .Data_write(EX_MemWrite? alu_in2: 32'h0), .EX_DMType(EX_DMType), 
  // MEM
  .MEM_Addr_in(MEM_aluout), .MEM_DMType(MEM_DMType), .Data_read_from_dm(data_sram_rdata), 
  // output
  .Data_read(data_read/*MEM*/), .Data_write_to_dm(data_sram_wdata/*EX*/), .wea_mem(wea_mem/*EX*/)
  );

assign data_sram_we = wea_mem & {4{~exc_sig}} & {4{~EX_exc_req_out}};

//ex阶段给出地址、使能的同时也给出size，在ex-mem上升沿握手
always@(*) begin
    case(EX_DMType)
        `dm_word:               size_data = 3'd2;
        `dm_halfword:           size_data = 3'd1;
        `dm_halfword_unsigned:  size_data = 3'd1;
        `dm_byte:               size_data = 3'd0;
        `dm_byte_unsigned:      size_data = 3'd0;
        default:                size_data = 3'd0;
    endcase
end

assign size_inst = 3'd2;

//generate req_inst according to current situation
always@(posedge clk) begin
    if(!reset)                  inst_sram_en_req <= 1'b0;
    else if(IF_ID_flush)        inst_sram_en_req <= 1'b0;
    //else if(req_inst_success)   inst_sram_en_req <= 1'b0;
    else if(addr_ok_inst)       inst_sram_en_req <= 1'b1;
    else                        inst_sram_en_req <= 1'b0;
end



//-----Exceptions-----------------

// ADEF
assign ADEF = pc[0] | pc[1];
// ALE
assign ALE = (EX_MemRead | EX_MemWrite) & (~EX_DMType[2]&~EX_DMType[1]&~EX_DMType[0]&(aluout[0]|aluout[1]) 
            | ((~EX_DMType[2]&~EX_DMType[1]& EX_DMType[0] | ~EX_DMType[2]& EX_DMType[1]&~EX_DMType[0]) & aluout[0]));

// IF
assign IF_exc_req_in  = INT;
assign IF_Ecode_in    = 6'b0;
assign IF_EsubCode_in = 9'b0;

assign IF_exc_req_out = IF_exc_req_in | ADEF;
assign IF_Ecode_out   = IF_exc_req_in==1'b1 ? IF_Ecode_in :
                        {6{ADEF}} & `Ecode_ADEF;
                        //... TLBs
assign IF_EsubCode_out= IF_exc_req_in==1'b1 ? IF_EsubCode_in :
                        {9{ADEF}} & `EsubCode_ADEF;
                        //... TLBs

// ID
assign ID_exc_req_out  = ID_exc_req_in | SYS | BRK | INE;
assign ID_Ecode_out    = ID_exc_req_in==1'b1 ? ID_Ecode_in : 
                         {6{SYS}} & `Ecode_SYS 
                        |{6{BRK}} & `Ecode_BRK
                        |{6{INE}} & `Ecode_INE;
                         //...
assign ID_EsubCode_out = ID_exc_req_in==1'b1 ? ID_EsubCode_in:
                         {9{SYS}} & `EsubCode_SYS
                        |{9{BRK}} & `EsubCode_BRK
                        |{9{INE}} & `EsubCode_INE;
                         //...

// EX
assign EX_exc_req_out  = EX_exc_req_in | ALE;
assign EX_Ecode_out    = EX_exc_req_in==1'b1 ? EX_Ecode_in : 
                         {6{ALE}} & `Ecode_ALE;
                         //...
assign EX_EsubCode_out = EX_exc_req_in==1'b1 ? EX_EsubCode_in :
                         {9{ALE}} & `EsubCode_ALE;
                         //...

// MEM
assign MEM_exc_req_out  = MEM_exc_req_in;
assign MEM_Ecode_out    = MEM_exc_req_in==1'b1 ? MEM_Ecode_in : 
                         6'b0;
                         //...
assign MEM_EsubCode_out = MEM_exc_req_in==1'b1 ? MEM_EsubCode_in:
                         9'b0;
                         //...

assign exc_sig = MEM_exc_req_out/* & IE*/;

//-----Branch or Jump-------------

    wire branch_b_and_bl = (EX_NPCOp==`NPC_BANDBL);
    wire branch_bxx = ((EX_NPCOp==`NPC_BRANCH)&&Zero);
    wire branch_jirl = (EX_NPCOp==`NPC_JIRL);
    assign Branch_or_Jump = branch_b_and_bl|branch_bxx|branch_jirl | EX_ERTN;

//-----HazardDetectionUnit--------

    HazardDetectionUnit hazardunit(.EX_MemRead(EX_MemRead),
    .ID_rs1(rs1),
    .ID_rs2(rs2),
    .EX_rd(EX_rd),
    .stall_signal(stall_signal)
    );

//-----ForwardingUnit-------------

    // ALU to ALU  and  ALU to CSR
    ForwardingUnit forwadingUnitA(.MEM_RegWrite(MEM_RegWrite), .MEM_rd(MEM_rd), 
        .WB_RegWrite(WB_RegWrite), .WB_rd(WB_rd), .EX_rs(EX_rs1), .ForwardSignal(ForwardA));
    ForwardingUnit forwadingUnitB(.MEM_RegWrite(MEM_RegWrite), .MEM_rd(MEM_rd), 
        .WB_RegWrite(WB_RegWrite), .WB_rd(WB_rd), .EX_rs(EX_rs2), .ForwardSignal(ForwardB));
    // MUX Gate 
    always @(*)
    begin
        case(ForwardA)
            2'b00: alu_in1 <= EX_rs1 != 5'b0 ? EX_RD1 : 32'b0;     //from regfile
            2'b10: alu_in1 <= EX_rs1 != 5'b0 ? MEM_aluout : 32'b0; //from MEM
            2'b01: alu_in1 <= EX_rs1 != 5'b0 ? WD : 32'b0;         // from WB
        endcase
    end
    always @(*)
    begin
        case(ForwardB)
            2'b00: alu_in2 <= EX_rs2 != 5'b0 ? EX_RD2 : 32'b0;     //from regfile
            2'b10: alu_in2 <= EX_rs2 != 5'b0 ? MEM_aluout : 32'b0; //from MEM
            2'b01: alu_in2 <= EX_rs2 != 5'b0 ? WD : 32'b0;         // from WB
        endcase
    end
    
    assign A = (EX_ALUSrc1) ? EX_pc : alu_in1;
    assign B = (EX_ALUSrc2) ? EX_imm4alu : alu_in2;//whether from EXT

    // CSR to ALU  and  CSR to CSR
    ForwardingUnit #(.WIDTH(14)) forwardingCSR2ALU(.MEM_RegWrite(MEM_csr_write), .MEM_rd(MEM_csr_num), 
        .WB_RegWrite(WB_csr_write), .WB_rd(WB_csr_num), .EX_rs(EX_csr_num), .ForwardSignal(ForwardCSR2ALU));
    
    always @(*)
    begin
        case(ForwardCSR2ALU)
            /*2'b00: alu_csr_data <= csr_num != `TICLR ? EX_csr_data : 32'b0;// from regfile
            2'b10: alu_csr_data <= csr_num != `TICLR ? MEM_csr_wd : 32'b0; // from MEM
            2'b01: alu_csr_data <= csr_num != `TICLR ? WB_csr_wd : 32'b0;  // from WB*/
            2'b00: alu_csr_data <= EX_csr_data;// from regfile
            2'b10: alu_csr_data <= MEM_csr_wd; // from MEM
            2'b01: alu_csr_data <= WB_csr_wd;  // from WB
        endcase
    end

    /*// EENTRY
    ForwardingUnit #(.WIDTH(14)) forwardingCSR2ALU(.MEM_RegWrite(1'b0), .MEM_rd(14'b0), 
        .WB_RegWrite(WB_csr_write), .WB_rd(WB_csr_num), .EX_rs(`EENTRY), .ForwardSignal(ForwardEENTRY));
    always @(*)
    begin
        case(ForwardCSR2ALU)
            2'b00: alu_csr_data <= EENTRY;// from regfile
            2'b10: alu_csr_data <= MEM_csr_wd; // from MEM
            2'b01: alu_csr_data <= WB_csr_wd;  // from WB
        endcase
    end*/

//-----pipe registers--------------

    //IF_ID: [31:0]PC [31:0]instr
    wire IF_ID_write_enable = ~(stall_signal | div_busy | mul_busy) | exc_sig | Branch_or_Jump/*?*/;
    wire IF_ID_flush = Branch_or_Jump | exc_sig;
    wire [79:0] IF_ID_in;
    assign IF_ID_in[31:0] = pc;//?????????????????????
    assign IF_ID_in[63:32] = inst_sram_rdata;
    //assign IF_ID_in[71:64] = {EXINT, 7'b0};
    //assign IF_ID_in[71:64] = 8'b0;
    assign IF_ID_in[64]    = IF_exc_req_out;
    assign IF_ID_in[70:65] = IF_Ecode_out;
    assign IF_ID_in[79:71] = IF_EsubCode_out;

    wire [79:0] IF_ID_out;
    //assign instr = IF_ID_out[63:32];
    assign instr = IF_ID_out[63:32];
    assign ID_pc_ = IF_ID_out[31:0];
    assign ID_exc_req_in  = IF_ID_out[64];
    assign ID_Ecode_in    = IF_ID_out[70:65];
    assign ID_EsubCode_in = IF_ID_out[79:71];
    GRE_array #(.WIDTH(80))
    IF_ID
    (.clk(clk), .rst(reset), .write_enable(IF_ID_write_enable), .flush(IF_ID_flush),
    .in(IF_ID_in), .out(IF_ID_out));

    gnrl_dff #(32) if_id_pc(clk, reset, IF_ID_write_enable, IF_ID_flush, pc, ID_pc);
    
    

    //ID_EX
    wire ID_EX_write_enable = ~(div_busy | mul_busy) | Branch_or_Jump | exc_sig | stall_signal;
    wire ID_EX_flush = stall_signal | Branch_or_Jump | exc_sig;
    wire [268:0] ID_EX_in;
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
    assign ID_EX_in[217:204] = csr_num;
    assign ID_EX_in[218] = csr_mask_en;
    assign ID_EX_in[219] = csr_write;
    assign ID_EX_in[251:220] = csr_data;
    assign ID_EX_in[252]     = ID_exc_req_out;
    assign ID_EX_in[258:253] = ID_Ecode_out;
    assign ID_EX_in[267:259] = ID_EsubCode_out;
    assign ID_EX_in[268]     = ERTN;
    //assign ID_pc = ID_EX_in[31:0]; 
    wire [268:0] ID_EX_out;
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
    assign EX_csr_num = ID_EX_out[217:204];
    assign EX_csr_mask_en = ID_EX_out[218];
    assign EX_csr_write = ID_EX_out[219];
    assign EX_csr_data = ID_EX_out[251:220];
    assign EX_exc_req_in  = ID_EX_out[252];
    assign EX_Ecode_in    = ID_EX_out[258:253];
    assign EX_EsubCode_in = ID_EX_out[267:259];
    assign EX_ERTN        = ID_EX_out[268];
    
    //assign EX_SCAUSE = {ID_EX_out[169:167], ID_EX_out[166:165] | {BadAddr, IntOverflow}, ID_EX_out[164:162]};
    //assign EXL_Clear = ID_EX_out[170];

    GRE_array #(.WIDTH(269))
    ID_EX
    (.clk(clk), .rst(reset), .write_enable(ID_EX_write_enable), .flush(ID_EX_flush),
    .in(ID_EX_in), .out(ID_EX_out));

    gnrl_dff #(32) id_ex_pc(clk, reset, ID_EX_write_enable, ID_EX_flush, ID_pc, EX_pc);
    
    //EX_MEM
    wire EX_MEM_write_enable = 1;
    wire EX_MEM_flush = exc_sig | div_busy | mul_busy;//warning
    wire [171:0] EX_MEM_in;
    assign EX_MEM_in[31:0] = ID_EX_out[31:0];//PC
    assign EX_MEM_in[36:32] = EX_rd;//rd
    assign EX_MEM_in[68:37] = alu_in2;//RD2 updated!!!
    assign EX_MEM_in[100:69] = aluout;
    assign EX_MEM_in[101] = EX_RegWrite;
    assign EX_MEM_in[102] = EX_MemWrite;
    assign EX_MEM_in[105:103] = EX_DMType;
    assign EX_MEM_in[107:106] = EX_WDSel;
    assign EX_MEM_in[108] = EX_MemRead;
    assign EX_MEM_in[122:109] = EX_csr_num;
    assign EX_MEM_in[123] = EX_csr_write;
    assign EX_MEM_in[155:124] = EX_csr_wd;
    assign EX_MEM_in[156]     = EX_exc_req_out;
    assign EX_MEM_in[162:157] = EX_Ecode_out;
    assign EX_MEM_in[171:163] = EX_EsubCode_out;

    wire [171:0] EX_MEM_out;
    assign MEM_rd = EX_MEM_out[36:32];
    assign MEM_RD2 = EX_MEM_out[68:37];
    assign MEM_aluout = EX_MEM_out[100:69];
    assign MEM_RegWrite = EX_MEM_out[101];
    assign MEM_MemWrite = EX_MEM_out[102];
    assign MEM_DMType = EX_MEM_out[105:103];
    assign MEM_WDSel = EX_MEM_out[107:106];
    wire MEM_MemRead;
    assign MEM_MemRead = EX_MEM_out[108];
    assign MEM_csr_num = EX_MEM_out[122:109];
    assign MEM_csr_write = EX_MEM_out[123];
    assign MEM_csr_wd = EX_MEM_out[155:124];
    assign MEM_exc_req_in  = EX_MEM_out[156];
    assign MEM_Ecode_in    = EX_MEM_out[162:157];
    assign MEM_EsubCode_in = EX_MEM_out[171:163];
    GRE_array #(.WIDTH(172))
    EX_MEM
    (.clk(clk), .rst(reset), .write_enable(EX_MEM_write_enable), .flush(EX_MEM_flush),
    .in(EX_MEM_in), .out(EX_MEM_out));

    wire [31:0] MEM_pc;
    gnrl_dff #(32) ex_mem_pc(clk, reset, EX_MEM_write_enable, EX_MEM_flush, EX_pc, MEM_pc);
    
    assign data_sram_en = (EX_MemRead|EX_MemWrite)/* & ~exc_sig*/;
    //assign data_sram_en = EX_MemRead|MEM_MemWrite;


    //MEM_WB
    wire MEM_WB_write_enable = 1;
    wire MEM_WB_flush = exc_sig;
    wire [150:0] MEM_WB_in;
    assign MEM_WB_in[31:0] = EX_MEM_out[31:0];
    assign MEM_WB_in[36:32] = MEM_rd;
    assign MEM_WB_in[68:37] = MEM_aluout;
    //assign MEM_WB_in[100:69] = data_sram_rdata;
    assign MEM_WB_in[100:69] = data_read;
    assign MEM_WB_in[101] = MEM_RegWrite;
    assign MEM_WB_in[103:102] = MEM_WDSel;
    assign MEM_WB_in[117:104] = MEM_csr_num;
    assign MEM_WB_in[118] = MEM_csr_write;
    assign MEM_WB_in[150:119] = MEM_csr_wd;
    wire [150:0] MEM_WB_out;
    //assign WB_pc = MEM_WB_out[31:0];
    assign WB_rd = MEM_WB_out[36:32];
    assign WB_aluout = MEM_WB_out[68:37];
    assign WB_MemData = MEM_WB_out[100:69];
    assign WB_RegWrite = MEM_WB_out[101];
    assign WB_WDSel = MEM_WB_out[103:102];
    assign WB_csr_num = MEM_WB_out[117:104];
    assign WB_csr_write = MEM_WB_out[118];
    assign WB_csr_wd = MEM_WB_out[150:119];
    GRE_array #(.WIDTH(151))
    MEM_WB
    (.clk(clk), .rst(reset), .write_enable(MEM_WB_write_enable), .flush(MEM_WB_flush),
    .in(MEM_WB_in), .out(MEM_WB_out));

    gnrl_dff #(32) mem_wb_pc(clk, reset, MEM_WB_write_enable, MEM_WB_flush, MEM_pc, WB_pc);


assign debug_wb_pc       = WB_pc;
assign debug_wb_rf_we   = {4{WB_RegWrite}};
assign debug_wb_rf_wnum  = WB_rd;
assign debug_wb_rf_wdata = WD;

endmodule