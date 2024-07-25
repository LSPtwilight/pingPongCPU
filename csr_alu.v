`include "CSR_def.v"

module csr_alu(
    input  [31:0] rd,
    input  [31:0] rj, /*mask*/
    input  [31:0] csr_data,
    input  [13:0] csr_num, /*write_addr*/
    input         mask_en, /*for csrxchg*/
    output reg [31:0] csr_wd
    );

    always@(*)begin
        case(csr_num)
            `CRMD:   csr_wd <= (mask_en ? (rj & rd | ~rj & csr_data) : rd) & 32'h1FF;  // [8:0] RW [31:9] reserved 0
            `PRMD:   csr_wd <= (mask_en ? (rj & rd | ~rj & csr_data) : rd) & 32'h7;    // [2:0] RW [31:3] reserved 0
            `ESTAT:  csr_wd <= mask_en ? rj & rd & 32'h3 | ~(rj & 32'h3) & csr_data 
                                       : rd & 32'h3 | ~32'h3 & csr_data; // [1:0] RW
            `ERA:    csr_wd <= mask_en ? rj & rd | ~rj & csr_data : rd; // [31:0] RW
            `EENTRY: csr_wd <= (mask_en ? (rj & rd | ~rj & csr_data) : rd) & ~32'h7F; // [31:6] RW
            `SAVE0:  csr_wd <= mask_en ? (rj & rd | ~rj & csr_data) : rd; // [31:0] RW
            `SAVE1:  csr_wd <= mask_en ? (rj & rd | ~rj & csr_data) : rd; // [31:0] RW
            `SAVE2:  csr_wd <= mask_en ? (rj & rd | ~rj & csr_data) : rd; // [31:0] RW
            `SAVE3:  csr_wd <= mask_en ? (rj & rd | ~rj & csr_data) : rd; // [31:0] RW
            default: csr_wd <= 32'b0;
        endcase
    end
endmodule
    
