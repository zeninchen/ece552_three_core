module fetch #(
    parameter RESET_ADDR = 32'h00000000
) (
    input  wire        i_clk,
    input  wire        i_rst,  
    input  wire [31:0] i_target_addr, 
    input  wire        i_not_pc_4, // from execute stage, for next PC calculation in hart (for non-control-transfer instructions)   
    input wire        i_stall, // from hazard detection, to stall the fetch stage when there's a hazard (stall the decode stage and insert nop in execute stage)
    // input  wire [31:0] i_imem_rdata, 

    // output wire [31:0] o_inst,        
    output reg  [31:0] o_pc,          // PC
    output wire [31:0] o_pc_4,         // PC + 4

    //ren signals
    input wire i_imem_ready,
    input wire i_imem_valid,
    output wire o_imem_ren,
    output wire o_fetch_wait
);
    reg imem_busy;
    // PC + 4
    assign o_pc_4 = o_pc + 4;
    assign o_fetch_wait = imem_busy && !i_imem_valid;

    // update PC 
    always @(posedge i_clk) begin
        if (i_rst)
            o_pc <= RESET_ADDR; // reset 
        ///////////////
        //change the selector, so it will correctly select the pc+4
        ////////////////////
        else if (i_not_pc_4)
            o_pc <= i_target_addr; // for branch/jal target or jalr target
        //when stalled, the PC should not update, so we can just let the PC hold the value until the stall is over, and we can determine whether to update the PC or not in the next PC calculation in hart by using the i_not_pc_4 signal from execute stage, which indicates whether the instruction in execute stage is a control transfer instruction or not (for better timing, we use the signal generated in execute stage, which is used for next PC calculation in hart, to determine if the instruction is control transfer or not)
        else if (i_stall || o_fetch_wait)
            o_pc <= o_pc; // hold the current PC when stalled
        
        else
            o_pc <= o_pc_4; // default PC update to PC + 4
    end

    // assign o_inst = i_imem_rdata;
    assign o_imem_ren = i_imem_ready && !i_stall && !imem_busy;    
    always @(posedge i_clk) begin
        if (i_rst)
            imem_busy <= 1'b0;
        else if (i_imem_valid)
            imem_busy <= 1'b0;
        else if (o_imem_ren)
            imem_busy <= 1'b1;
    end

endmodule

//assign o_imem_raddr = {o_pc[31:2], 2'b00};