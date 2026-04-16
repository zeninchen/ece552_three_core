module forward(
    //from the execute stage
    input wire [4:0] i_id_ex_rs1_addr,
    input wire [4:0] i_id_ex_rs2_addr,

    // for us to do ex to ex forwarding, we need to know the rs1 and rs2 address in ex stage, and the rd address in mem stage,
    // and the reg write enable signal in ex stage
    input wire [4:0] i_ex_mem_rd_addr,
    input wire i_ex_mem_reg_write_en,

    output wire o_ex_to_ex_forward_rs1_en, //higher priority than mem to ex forwarding
    output wire o_ex_to_ex_forward_rs2_en,

    //for us to do mem to ex forwarding, we need to know the rs1 and rs2 address in ex stage, and the rd address in mem stage,
    // and the reg write enable signal in mem stage
    input wire [4:0] i_mem_wb_rd_addr,
    input wire i_mem_wb_reg_write_en,

    output wire o_mem_to_ex_forward_rs1_en,
    output wire o_mem_to_ex_forward_rs2_en,

    //for us to do mem to mem forwarding, we can only do rs2 forwarding, 
    //because rs1 is used for address calculation, and we need the value in the mem stage to calculate the address, 
    //so we can't forward from mem to mem for rs1, but for rs2, we can forward from mem to mem, because rs2 is only used for store data, and we can get the value from the mem stage to do the store operation
    input wire [4:0] i_ex_mem_rs2_addr, // the rs2 address in mem stage
    input wire i_ex_mem_MemWrite,
    output wire o_mem_to_mem_forward_rs2_en

);
    //note that ex to ex forwarding has higher priority than mem to ex forwarding, because the instruction in mem stage is older than the instruction in ex stage, 
    //so if there is a hazard with both ex stage and mem stage, we should forward from ex stage, because the value in ex stage is more up-to-date than the value in mem stage
    //so the mux so select the ex to ex forwarding first
    assign o_ex_to_ex_forward_rs1_en = i_ex_mem_reg_write_en 
    && (i_ex_mem_rd_addr != 0) && (i_ex_mem_rd_addr == i_id_ex_rs1_addr);

    assign o_ex_to_ex_forward_rs2_en = i_ex_mem_reg_write_en 
    && (i_ex_mem_rd_addr != 0) && (i_ex_mem_rd_addr == i_id_ex_rs2_addr);

    assign o_mem_to_ex_forward_rs1_en = i_mem_wb_reg_write_en 
    && (i_mem_wb_rd_addr != 0) && (i_mem_wb_rd_addr == i_id_ex_rs1_addr);

    assign o_mem_to_ex_forward_rs2_en = i_mem_wb_reg_write_en 
    && (i_mem_wb_rd_addr != 0) && (i_mem_wb_rd_addr == i_id_ex_rs2_addr);

    //only when storing we can 
    assign o_mem_to_mem_forward_rs2_en = i_ex_mem_MemWrite && i_mem_wb_reg_write_en 
    && (i_mem_wb_rd_addr != 0) && (i_mem_wb_rd_addr == i_ex_mem_rs2_addr);

endmodule