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
               input         TI_in,
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
               output [31:0] ERA_out
               );

  reg [31:0] RD;

  reg [31:0] CRMD;
  reg [31:0] PRMD;
  reg [31:0] ESTAT;
  reg [31:0] ERA;
  reg [31:0] EENTRY;
  reg [31:0] SAVE0;
  reg [31:0] SAVE1;
  reg [31:0] SAVE2;
  reg [31:0] SAVE3;

//warning: posedge
  //always @(negedge clk, posedge rst)begin
  always @(posedge clk, posedge rst)begin
    if (rst) begin    //  reset
        CRMD  <= 32'h8; // DA = 1
        //EUEN[FPUen] <= 0;
        //ECFG[LIE1] <= 10'b0;
        //ECFG[LIE2] <= 2'b0; 
        ESTAT[1:0] <= 2'b0;
        //EENTRY <= 32'b0;
        //TCFG[En] <= 0;
        //LLBCTL[KLO] <= 0;

    end
    else begin
      // writes
      ESTAT[`HWI] <= HWI_in;
      ESTAT[`TI ] <= TI_in;
      ESTAT[`IPI] <= IPI_in;
      if (csr_write) begin
          case(write_addr)
              `CRMD:   CRMD   <= WD;
              `PRMD:   PRMD   <= WD;
              `ESTAT:  ESTAT  <= WD;
              `ERA:    ERA    <= WD;
              `EENTRY: EENTRY <= WD;
              `SAVE0:  SAVE0  <= WD;
              `SAVE1:  SAVE1  <= WD;
              `SAVE2:  SAVE2  <= WD;
              `SAVE3:  SAVE3  <= WD;
          endcase
      end
      // EX
      if (ERTN) begin
        CRMD[`PLV] <= (MEM_csr_write==1'b1 && MEM_csr_num==`PRMD) ? MEM_csr_wd[`PPLV] : // MEM to EX
                      (csr_write    ==1'b1 && write_addr ==`PRMD) ? WD[`PPLV] :         // WB to EX
                      PRMD[`PPLV];
        CRMD[`IE ] <= (MEM_csr_write==1'b1 && MEM_csr_num==`PRMD) ? MEM_csr_wd[`PIE] : // MEM to EX
                      (csr_write    ==1'b1 && write_addr ==`PRMD) ? WD[`PIE] :         // WB to EX
                      PRMD[`PIE];
      end
      // trigger exceptions
      // MEM
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
  always@(read_addr)begin
    case(read_addr)
        `CRMD:   RD <= {23'b0, CRMD[8:0]};
        `PRMD:   RD <= {29'b0, PRMD[2:0]};
        `ESTAT:  RD <= {1'b0/*31*/, ESTAT[30:16]/*Ecode, EsubCode*/, 3'b0/*15:13*/, ESTAT[12:11], 1'b0/*10*/, ESTAT[9:0]};
        `ERA:    RD <= ERA;
        `EENTRY: RD <= {EENTRY[31:6]/*VA*/, 6'b0/*5:0*/};
        `SAVE0:  RD <= SAVE0;
        `SAVE1:  RD <= SAVE1;
        `SAVE2:  RD <= SAVE2;
        `SAVE3:  RD <= SAVE3;
        default: RD <= 32'b0;
    endcase
  end
    
  assign RDout      = ((csr_write==1'b1)&&(write_addr==read_addr)) ? WD : RD; 
  assign ERA_out    = (MEM_csr_write==1'b1 && MEM_csr_num==`ERA) ? MEM_csr_wd : // MEM to EX
                      (csr_write==1'b1     && write_addr==`ERA ) ? WD         : // WB to EX
                      ERA;
  //assign IE         = ((csr_write==1'b1)&&(write_addr==`CRMD)) ? WD[`IE] : CRMD[`IE];
  assign EENTRY_out = ((csr_write==1'b1)&&(write_addr==`EENTRY)) ? WD : EENTRY; //WB to MEM
  // assign RD1 = (A1 != 0) ? rf[A1] : 0;
  // assign RD2 = (A2 != 0) ? rf[A2] : 0;
  //assign RD1 = (A1==0) ? 0: ((RFWr==1'b1)&&(A1==A3)) ? WD: rf[A1];
  //assign RD2 = (A2==0) ? 0: ((RFWr==1'b1)&&(A2==A3)) ? WD: rf[A2];
  //assign reg_data = rf[reg_sel];//(reg_sel != 0) ? rf[reg_sel] : 0; 

endmodule 
