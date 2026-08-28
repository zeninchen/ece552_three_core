module hazard_detect(
    //we should have consenus that if addr is 0 -> no hazard// not used
    //outside; we should implment, for I format or U J, format we load 
    // the unused register's port with 0(mux) to avoid the hazard
    input wire [4:0] i_if_idrs1_addr,
    input wire [4:0] i_if_idrs2_addr,
    // from ID/EX stage
    input wire [4:0] i_id_ex_rd_addr,
    input wire i_id_ex_reg_write_en,
    input wire i_id_ex_MemRead,
    input wire i_id_ex_predicted_taken,
    // from EX/MEM stage
    input wire [4:0] i_ex_mem_rd_addr,
    input wire i_ex_mem_reg_write_en,


    //we can just that this nop instruction to progragate in the reg
    output wire o_nop_ex,
    output wire o_nop_id,
    output wire o_nop_mem,
    output wire o_nop_wb, // for the nop in mem stage when the mem stage is stalling, we need to insert nop in the writeback stage as well, because the instruction in mem stage is not finished yet, so we need to stall the writeback stage as well.
    //output wire o_nop_mem,

    output wire o_stall_decode,
    output wire o_stall_execute,
    output wire o_stall_fetch,
    output wire o_stall_mem,
    //branch/jump hazard detection (for branch instructions, we need to stall if the instruction in EX/MEM stage is writing to a register that is used as rs1 or rs2 in the current instruction, because we need the value to calculate the branch target and make the branch decision)
    //right now our branch resolve in execute stage, so we only need to check the hazard with the instruction in EX/MEM stage
    input wire i_ex_not_pc_4, //ex stage not_pc_4 have priority than id stage not_pc_4
    input wire i_ex_is_pc_4,
    input wire i_id_not_pc_4, // for hazard detection to determine if the instruction in id stage is a control transfer instruction or not (for better timing, we use the signal generated in id stage, which is used for next PC calculation in hart, to determine if the instruction in id stage is control transfer or not)
    input wire i_id_MemWrite,
    //also only stall when there is a load instruction in the EX/MEM stage, because if it's not a load instruction, we can still forward the data from EX/MEM stage to EX stage, but if it's a load instruction, we can't forward the data because the data is not ready until the end of MEM stage, so we have to stall the pipeline until the data is ready.
    input wire i_ex_mem_MemRead,
    input wire i_ex_mem_MemWrite,

    //valid signals
    input wire  i_dmem_valid,         
    input  wire i_imem_valid,
    //ready signals
    input wire i_dmem_ready,
    input wire i_imem_ready,

    //ren signals
    input wire i_memory_wait,

    // input wire i_id_ex_valid
    input wire i_id_ex_valid
);
    wire load_use_hazard;
    wire dmem_access_in_mem;
    //correctly we flush in exstage
    wire flush_id_ex;
    wire flush_id;
    //for now, the branch is resolved in ex, so if a branch is happening, we need to flush the instruction,
    //we can just insert a nop in the execute stage.
    //when predict taken but it's not take and predict notaken but taken
    assign flush_id_ex = (((i_ex_not_pc_4 && !i_id_ex_predicted_taken)
        || (i_id_ex_predicted_taken && i_ex_is_pc_4))
        && i_id_ex_valid && !i_memory_wait);
    assign flush_id = i_id_not_pc_4;
// I am thinking that for stall, we just need the decode stage to hold the instruction
// and for the execute stage, we insert the nop
    // assign o_stall_decode = (i_id_ex_reg_write_en && (i_id_ex_rd_addr != 0) && 
    //                     ((i_id_ex_rd_addr == i_if_idrs1_addr) || (i_id_ex_rd_addr == i_if_idrs2_addr))) ||
    //                    (i_ex_mem_reg_write_en && (i_ex_mem_rd_addr != 0) &&
    //                     ((i_ex_mem_rd_addr == i_if_idrs1_addr) || (i_ex_mem_rd_addr == i_if_idrs2_addr))); // Stall decode if there's a hazard, we don't stall when there is a flush
    //we don't have to stall decode now, if we have forwarding, but we do need to stall when the next stages are stalling.
    //lw and then sw: no need stall.
    wire load_use_rs1 =
        (i_id_ex_rd_addr == i_if_idrs1_addr);

    wire load_use_rs2 =
        (i_id_ex_rd_addr == i_if_idrs2_addr);

    assign load_use_hazard =
        i_id_ex_valid &&
        i_id_ex_MemRead &&
        (i_id_ex_rd_addr != 5'd0) &&
        (
            load_use_rs1 ||
            (load_use_rs2 && !i_id_MemWrite)
        );
    //mem stall also ned to stall decode
    assign o_stall_decode = load_use_hazard; // Stall decode if execute stage is stalling, we don't stall when there is a flush
    //we only stall execute when there is rs1 collision with the rd in the mem stage
    //no hazard for rs2 if we are doing mem write (if_id)
    // assign o_stall_execute = (i_ex_mem_reg_write_en && i_ex_mem_MemRead && (i_ex_mem_rd_addr != 0) && 
    //                         (i_ex_mem_rd_addr == i_if_idrs1_addr))||
    //                         (!i_id_MemWrite&& i_ex_mem_reg_write_en && i_ex_mem_MemRead &&
    //                         (i_ex_mem_rd_addr != 0) && (i_ex_mem_rd_addr == i_if_idrs2_addr));
    assign o_stall_execute = 1'b0;
    assign o_nop_ex = flush_id_ex || load_use_hazard; // insert nop if there's a load-to-use hazard or flush is needed
    assign o_nop_id = flush_id_ex; // with branch resolution in ID, only EX-stage redirects need an ID-stage nop
    assign o_nop_mem = 1'b0; // insert nop in mem stage when execute stage is stalling.

    //we stall the fetch, when the imem is not valid,
    // cycle. It should not globally stall fetch/decode retirement.
    
    assign o_stall_fetch = 1'b0;

    // Stall MEM only for an actual in-flight data-memory transaction.

    assign o_stall_mem = i_memory_wait; // stall mem stage when there is a memory access that is not ready yet, we can just use the o_memory_wait signal from memory module to determine if we need to stall the mem stage or not, because the o_memory_wait signal is only high when there is a memory access that is not ready yet, and it's low when there is no memory access or the memory access is ready, so we can just use this signal to stall the mem stage when there is a memory access that is not ready yet.
    //we also need to add nop for writeback stage when mem stage is stalling
    assign o_nop_wb = 1'b0; // insert nop in writeback stage when mem stage is stalling, because the instruction in mem stage is not finished yet, so we need to stall the writeback stage as well.


endmodule
