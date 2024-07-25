`define CRMD      14'h0
`define PRMD      14'h1
`define EUEN      14'h2
`define ECFG      14'h4
`define ESTAT     14'h5
`define ERA       14'h6
`define BADV      14'h7
`define EENTRY    14'hc
`define TLBIDX    14'h10
`define TLBEHI    14'h11
`define TLBELO0   14'h12
`define TLBELO1   14'h13
`define ASID      14'h18
`define PGDL      14'h19
`define PGDH      14'h1A
`define PGD       14'h1B
`define CPUID     14'h20
`define SAVE0     14'h30
`define SAVE1     14'h31
`define SAVE2     14'h32
`define SAVE3     14'h33
`define TID       14'h40
`define TCFG      14'h41
`define TVAL      14'h42
`define TICLR     14'h44
`define LLBCTL    14'h60
`define TLBRENTRY 14'h88
`define CTAG      14'h98
`define DMW0      14'h180
`define DMW1      14'h181

// CRMD
`define PLV  1:0
`define IE   2
`define DA   3
`define PG   4
`define DATF 6:5
`define DATM 8:7

// PRMD
`define PPLV 1:0
`define PIE  2

// ECFG
`define LIE1 9:0
`define LIE2 12:11

// ESTAT
`define SW       1:0
`define HWI      9:2
`define TI       11
`define IPI      12
`define Ecode    21:16
`define EsubCode 30:22

//TIMER
`define TIMER_WIDTH   24

// TCFG
`define En       0
`define Periodic 1
`define InitVal  (`TIMER_WIDTH-1):2

`define Ecode_INT     6'h0
`define EsubCode_INT   9'b0
`define Ecode_PIL     6'h1
`define EsubCode_PIL   9'b0
`define Ecode_PIS     6'h2
`define EsubCode_PIS   9'b0
`define Ecode_PIF     6'h3
`define EsubCode_PIF   9'b0
`define Ecode_PME     6'h4
`define EsubCode_PME   9'b0
`define Ecode_PPI     6'h7
`define EsubCode_PPI   9'b0
`define Ecode_ADEF    6'h8
`define EsubCode_ADEF  9'b0
`define Ecode_ADEM    6'h8
`define EsubCode_ADEM  9'b1
`define Ecode_ALE     6'h9
`define EsubCode_ALE   9'b0
`define Ecode_SYS     6'hB
`define EsubCode_SYS   9'b0
`define Ecode_BRK     6'hC
`define EsubCode_BRK   9'b0
`define Ecode_INE     6'hD
`define EsubCode_INE   9'b0
`define Ecode_IPE     6'hE
`define EsubCode_IPE   9'b0
`define Ecode_FPD     6'hF
`define EsubCode_FPD   9'b0
`define Ecode_FPE     6'h12
`define EsubCode_FPE   9'b0
`define Ecode_TLBR    6'h3F
`define EsubCode_TLBR  9'b0
