`include "CSR_def.v"

module CSR(    input         clk, 
               input         rst,
               input         csr_write, 
               input  [13:0] read_addr, write_addr, 
               input  [31:0] WD, 
               output [31:0] RDout,
               // for forwarding
               input         MEM_csr_write,
               input  [13:0] MEM_csr_num,
               input  [31:0] MEM_csr_wd,
               // external INT signals
               input  [ 7:0] HWI_in,
               //input         TI_in,
               input         IPI_in,
               // CPU states
               //output        IE,
               // exceptions
               input         exc_sig,
               input  [5: 0] Ecode,
               input  [8: 0] EsubCode,
               input  [31:0] PC,
               input         ERTN,
               output [31:0] EENTRY_out,
               output [31:0] ERA_out,
               output wire INT
               );

  reg [31:0] RD;

  reg [31:0] CRMD;
  reg [31:0] PRMD;
  reg [31:0] ECFG;
  reg [31:0] ESTAT;
  reg [31:0] ERA;
  reg [31:0] BADV;
  reg [31:0] EENTRY;
  reg [31:0] SAVE0;
  reg [31:0] SAVE1;
  reg [31:0] SAVE2;
  reg [31:0] SAVE3;
  reg [31:0] TID;
  reg [31:0] TCFG;
  //reg [31:0] TVAL;
  reg [31:0] TICLR;

  // simple built-in timer
  reg [`TIMER_WIDTH-1:0] timer;

  always @(posedge clk, posedge rst)begin
    if (rst) begin    //  reset
        timer <= `TIMER_WIDTH'b1;
        CRMD  <= 32'h8; // DA = 1
        //EUEN[FPUen] <= 0;
        ECFG[`LIE1] <= 10'b0;
        ECFG[`LIE2] <= 2'b0; 
        ESTAT[1:0] <= 2'b0;
        TCFG[`En] <= 0;
        //LLBCTL[KLO] <= 0;
    end
    else begin
      // timer
      if(TCFG[`En]==1'b1)
        timer <= timer - 1;
      if(timer==`TIMER_WIDTH'b0 && TCFG[`En]==1'b1)begin
        ESTAT[`TI] <= 1;
        if(TCFG[`Periodic]==1'b1)begin
          timer <= {TCFG[`InitVal], 2'b0};
        end
        else begin
          timer <= `TIMER_WIDTH'b1; //??
          TCFG[`En] <= 1'b0;
        end
      end
      // sample the external interuption signals
      ESTAT[`HWI] <= HWI_in;
      //ESTAT[`TI ] <= TI_in;
      ESTAT[`IPI] <= IPI_in;
      // writes
      if (csr_write) begin
          case(write_addr)
              `CRMD:   CRMD   <= WD;
              `PRMD:   PRMD   <= WD;
              `ECFG:   ECFG   <= WD;
              `ESTAT:  ESTAT  <= WD;
              `ERA:    ERA    <= WD;
              `BADV:   BADV   <= WD;
              `EENTRY: EENTRY <= WD;
              `SAVE0:  SAVE0  <= WD;
              `SAVE1:  SAVE1  <= WD;
              `SAVE2:  SAVE2  <= WD;
              `SAVE3:  SAVE3  <= WD;
              `TID:    TID    <= WD;
              `TCFG:   TCFG   <= WD;
              `TVAL:   ;
              `TICLR:  ESTAT[`TI] <= ESTAT[`TI] & ~WD[0];
          endcase
      end
      // return from exceptions
      // jump in EX
      if (ERTN) begin
        CRMD[`PLV] <= (MEM_csr_write==1'b1 && MEM_csr_num==`PRMD) ? MEM_csr_wd[`PPLV] : // MEM to EX
                      (csr_write    ==1'b1 && write_addr ==`PRMD) ? WD[`PPLV] :         // WB to EX
                      PRMD[`PPLV];
        CRMD[`IE ] <= (MEM_csr_write==1'b1 && MEM_csr_num==`PRMD) ? MEM_csr_wd[`PIE] : // MEM to EX
                      (csr_write    ==1'b1 && write_addr ==`PRMD) ? WD[`PIE] :         // WB to EX
                      PRMD[`PIE];
      end
      // trigger exceptions
      // jump in MEM
      else if (exc_sig /*& IE*/) begin
        PRMD[`PPLV] <= (csr_write==1'b1 && write_addr==`CRMD) ? WD[`PLV] : CRMD[`PLV]; // WB to MEM
        PRMD[`PIE ] <= (csr_write==1'b1 && write_addr==`CRMD) ? WD[`IE ] : CRMD[`IE ]; // WB to MEM
        CRMD[`PLV ] <= 0;
        CRMD[`IE  ] <= 0;
        ERA         <= PC;
        ESTAT[`Ecode] <= Ecode;
        ESTAT[`EsubCode] <= EsubCode;
      end
    end
  end

  // reads
  always@(*)begin
    case(read_addr)
        `CRMD:   RD <= {23'b0, CRMD[8:0]};
        `PRMD:   RD <= {29'b0, PRMD[2:0]};
        `ECFG:   RD <= {19'b0/*31:13*/, ECFG[12:11], 1'b0/*10*/, ECFG[9:0]};
        `ESTAT:  RD <= csr_write == 1'b1 && write_addr == `TICLR ?  
                       {1'b0/*31*/, ESTAT[30:16]/*Ecode, EsubCode*/, 3'b0/*15:13*/, ESTAT[12:11], 1'b0/*10*/, ESTAT[9:0]} & ~(WD[0] << `TI): 
                       {1'b0/*31*/, ESTAT[30:16]/*Ecode, EsubCode*/, 3'b0/*15:13*/, ESTAT[12:11], 1'b0/*10*/, ESTAT[9:0]};
        `ERA:    RD <= ERA;
        `BADV:   RD <= BADV;
        `EENTRY: RD <= {EENTRY[31:6]/*VA*/, 6'b0/*5:0*/};
        `SAVE0:  RD <= SAVE0;
        `SAVE1:  RD <= SAVE1;
        `SAVE2:  RD <= SAVE2;
        `SAVE3:  RD <= SAVE3;
        `TID:    RD <= TID;
        `TCFG:   RD <= {{(32-`TIMER_WIDTH){1'b0}}, TCFG[(`TIMER_WIDTH-1): 0]};
        `TVAL:   RD <= {{(32-`TIMER_WIDTH){1'b0}}, timer};
        `TICLR:  RD <= 32'b0;
        default: RD <= 32'b0;
    endcase
  end
    
  assign RDout      = ((csr_write==1'b1)&&(write_addr==read_addr)) ? WD : RD; 
  assign ERA_out    = (MEM_csr_write==1'b1 && MEM_csr_num==`ERA) ? MEM_csr_wd : // MEM to EX
                      (csr_write==1'b1     && write_addr==`ERA ) ? WD         : // WB to EX
                      ERA;
  //assign IE         = ((csr_write==1'b1)&&(write_addr==`CRMD)) ? WD[`IE] : CRMD[`IE];
  assign EENTRY_out = ((csr_write==1'b1)&&(write_addr==`EENTRY)) ? WD : EENTRY; //WB to MEM
  
  wire [11:0] LIE = {ECFG[12:11], ECFG[9:0]};
  wire [11:0] IS  = {ESTAT[12], ESTAT[11], ESTAT[9:2], ESTAT[1:0]};
  wire [11:0] int_vec = LIE & IS;
  assign INT = (|int_vec) & CRMD[`IE];
  // assign RD1 = (A1 != 0) ? rf[A1] : 0;
  // assign RD2 = (A2 != 0) ? rf[A2] : 0;
  //assign RD1 = (A1==0) ? 0: ((RFWr==1'b1)&&(A1==A3)) ? WD: rf[A1];
  //assign RD2 = (A2==0) ? 0: ((RFWr==1'b1)&&(A2==A3)) ? WD: rf[A2];
  //assign reg_data = rf[reg_sel];//(reg_sel != 0) ? rf[reg_sel] : 0; 

endmodule 
