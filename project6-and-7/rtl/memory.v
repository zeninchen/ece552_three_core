`default_nettype none
// memory stage: handle memory read/write and pass data to writeback stage
module memory_stage (
    input  wire        i_clk,
    input  wire        i_rst,
    input wire [31:0] i_next_pc,
    input  wire [31:0] i_alu_result,    //alu result
    input  wire [31:0] i_pc_4,

    // input  wire        i_MemRead,
    input wire [1:0]  i_WriteRegSrc,
    // input wire        i_MemWrite,
    input wire        [3:0] i_Mask, // for store instruction byte/halfword/word selection
    // input wire [31:0] i_wb_data,

    input wire [31:0] i_rs1_data,
    input wire [31:0] i_rs2_data,
    input  wire [2:0]  i_funct3,      // decode: inst[14:12]
    input wire [31:0] i_inst, // for jalr target calculation and misaligned detection
    input wire [31:0] i_pc,
    input wire        i_misaligned, // for misaligned load/store detection

    output wire        o_misaligned, // pass through for misaligned load/store detection
    output wire [31:0] o_pc,
    output wire [31:0] o_inst, // pass through for hazard detection    
    output wire [2:0]  o_funct3, // for memory access byte/halfword/word selection
    
    output wire [31:0] o_rs1_data,
    output wire [31:0] o_rs2_data,


    // output  wire        o_MemRead,
    output  wire [1:0]  o_WriteRegSrc,
    // output wire        o_MemWrite,
    output wire [3:0]  o_Mask, // pass through for store instruction byte/halfword/word selection

    output  wire [31:0] o_pc_4,
    output  wire [31:0] o_next_pc,
    output  wire [31:0] o_alu_result,   //alu result
    // output wire [31:0] o_wb_data

    //extra signals for p6
    input wire i_ex_mem_MemRead,
    input wire i_ex_mem_MemWrite,

    //ren signals
    input wire i_dmem_ready,
    input wire i_dmem_valid,
    output wire o_dmem_ren,
    output wire o_dmem_wen,
    output wire o_memory_wait
);
    assign o_next_pc = i_next_pc;
    assign o_Mask = i_Mask; // pass through for store instruction byte/halfword/word selection
    assign o_rs1_data = i_rs1_data;
    assign o_rs2_data = i_rs2_data;
    assign o_inst = i_inst; // pass through for hazard detection
    assign o_pc = i_pc; // pass through for jalr target calculation
    assign o_misaligned = i_misaligned; // pass through for misaligned load/store detection

    assign o_pc_4 = i_pc_4; // for jal/jalr
    assign o_WriteRegSrc = i_WriteRegSrc; // for writeback mux control

    assign o_alu_result = i_alu_result; // pass through for
    assign o_funct3 = i_funct3; // pass through for memory access byte/halfword/word selection

    //multicycle memory access control
    reg dmem_busy;
    assign o_memory_wait = dmem_busy && !i_dmem_valid;

    assign o_dmem_ren = i_dmem_ready && !dmem_busy && i_ex_mem_MemRead; // only send read request when not stalled, mem is ready, and it's a load instruction
    assign o_dmem_wen = i_dmem_ready && !dmem_busy && i_ex_mem_MemWrite; // only send write request when not stalled, mem is ready, and it's a store instruction
    always @(posedge i_clk) begin
        if (i_rst)
            dmem_busy <= 1'b0;
        else if (i_dmem_valid)
            dmem_busy <= 1'b0;
        else if (o_dmem_ren||o_dmem_wen)
            dmem_busy <= 1'b1;
    end
endmodule
`default_nettype wire