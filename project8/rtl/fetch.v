module fetch #(
    parameter RESET_ADDR = 32'h00000000
) (
    input  wire        i_clk,
    input  wire        i_rst,  
    input  wire [31:0] i_if_instr,
    input  wire [31:0] i_target_addr, 
    input  wire [31:0] i_id_ex_pc_4_before_predict,
    input wire         i_id_ex_predict_taken,
    input  wire        i_not_pc_4, // from execute stage, for next PC calculation in hart (for non-control-transfer instructions)   
    input  wire        i_is_pc_4, // for branch predictor update in fetch stage, to determine whether the instruction in execute stage is a control transfer instruction or not (for better timing, we use the signal generated in execute stage, which is used for next PC calculation in hart, to determine if the instruction in execute stage is control transfer or not)
    input wire        i_stall, // from hazard detection, to stall the fetch stage when there's a hazard (stall the decode stage and insert nop in execute stage)
    // input  wire [31:0] i_imem_rdata, 

    // output wire [31:0] o_inst,        
    output reg  [31:0] o_pc,          // PC
    output wire [31:0] o_pc_4,         // PC + 4

    //ren signals
    input wire i_imem_ready,
    input wire i_imem_valid,
    output wire o_imem_ren,
    output wire o_fetch_wait,

    // flush signals
    input wire id_ex_valid,
    input wire id_ex_Jalr,
    input wire id_ex_Branch,
    output wire o_fetch_accept,
    output wire o_flush_fetch, // for the flush signal in fetch stage, we can just use the i_not_pc_4 signal from execute stage, because when a flush happens, the i_not_pc_4 signal will be high, and we can use it to determine whether to flush the instruction in fetch stage or not
    output wire [31:0] o_pc_4_before_predict, // for when we predict taken but it's infact not taken, then we need to revert back to orginal pc, and we can use the saved pc to revert back to the original pc
    output wire o_predict_taken // for the branch predictor, to determine whether we predict taken or not, and we can use it in the execute stage to determine whether to update the branch predictor or not

);
    reg imem_busy;
    reg flushed;
    reg [1:0] branch_predictor; //simple branch predictor
    wire is_jal = (i_if_instr[6:0] == 7'b1101111); // jal
    wire predict_taken = branch_predictor[1]; // predict taken when the branch predictor is in strong taken or weak taken state
    assign o_predict_taken = predict_taken|| is_jal; // for the branch predictor, to determine whether we predict taken or not, and we can use it in the execute stage to determine whether to update the branch predictor or not, and we also need to consider jal, because jal is for sure taken, so we need to predict taken for jal regardless of the state of the branch predictor
    wire [31:0] branch_address_offset;
    wire is_branch = (i_if_instr[6:0] == 7'b1100011); // beq, bne, blt, bge, bltu, bgeu
    assign o_flush_fetch = flushed;
    // PC + 4
    assign o_pc_4 = o_pc + 4;
    assign o_fetch_wait = imem_busy && !i_imem_valid;
    assign o_fetch_accept = !i_stall && i_imem_valid && !flushed;
    
    assign o_pc_4_before_predict = o_pc_4; // for when we predict taken but it's infact not taken, then we need to revert back to orginal pc, and we can use the saved pc to revert back to the original pc
    // update PC 
    always @(posedge i_clk) begin
        if (i_rst)
            o_pc <= RESET_ADDR; // reset 
        ///////////////
        //change the selector, so it will correctly select the pc+4
        ////////////////////
        else if (i_not_pc_4&& id_ex_valid && (!i_id_ex_predict_taken || id_ex_Jalr)) // when the instruction in execute stage is a control transfer instruction but we predict not taken, we need to update the PC to the target address, because we have already fetched the next instruction based on the wrong prediction, so we need to update the PC to the correct target address to fetch the correct instruction, and we can use the i_not_pc_4 signal from execute stage to determine whether the instruction in execute stage is a control transfer instruction or not, because when a control transfer instruction is in execute stage, the i_not_pc_4 signal will be high, and we can use it to determine whether to update the PC or not
            o_pc <= i_target_addr; // for branch/jal target or jalr target
        //when stalled, the PC should not update, so we can just let the PC hold the value until the stall is over, and we can determine whether to update the PC or not in the next PC calculation in hart by using the i_not_pc_4 signal from execute stage, which indicates whether the instruction in execute stage is a control transfer instruction or not (for better timing, we use the signal generated in execute stage, which is used for next PC calculation in hart, to determine if the instruction is control transfer or not)
        //we also default not predict taken for jalr
        else if (i_is_pc_4 && id_ex_valid && i_id_ex_predict_taken && !id_ex_Jalr) // when the instruction in execute stage is a control transfer instruction and we predict not taken, we need to update the PC to the target address, because we have already fetched the next instruction based on the wrong prediction, so we need to update the PC to the correct target address to fetch the correct instruction, and we can use the i_is_pc_4 signal from execute stage to determine whether the instruction in execute stage is a control transfer instruction or not, because when a control transfer instruction is in execute stage, the i_is_pc_4 signal will be high, and we can use it to determine whether to updatethe PC or not
            o_pc <= i_id_ex_pc_4_before_predict; // for branch/jal target or jalr target
        else if (o_fetch_accept) begin// when not stalled and imem is valid and not flushed, we can updatethe PC to PC+4, otherwise we holdthe PC untilthe stall is over orthe imem is valid orthe flush is over
            //jal is for sure taken
            if((is_branch && predict_taken) || is_jal) // when the instruction is a branch or jal and we predict taken, we update the PC to the branch target address, which is calculated by adding the branch address offset to the current PC, and the branch address offset is generated by the immediate generator based on the instruction in fetch stage, and we can use the i_if_instr signal to determine whether the instruction in fetch stage is a branch or jal or not, because when the instruction in fetch stage is a branch or jal, the opcode field of the instruction will be 1100011 for branch instructions and 1101111 for jal instructions, so we can use the opcode field of the instruction to determine whether it's a branch or jal or not
                o_pc <= o_pc + branch_address_offset; // for branch/jal target
            else
                o_pc <= o_pc_4; //propagate PC, when not stalled and not waiting for imem, we can update the PC to PC+4, otherwise we hold the PC until the stall is over or the imem is valid
        end
        //default hold the pc
    end
    
    assign o_imem_ren = !i_stall && !imem_busy; // when imem is ready and not stalled and not busy, we can send the read request to imem, otherwise we hold the read request until the imem is ready and not stalled and not busy
    always @(posedge i_clk) begin
        if (i_rst)
            imem_busy <= 1'b0;
        // else if ((i_imem_valid && !flushed) || i_not_pc_4) //we can only clear when it's valid or the flush consume one valid alreadys
        else if (i_imem_valid)
            imem_busy <= 1'b0;
        else if (o_imem_ren)
            imem_busy <= 1'b1;
    end

    //if a flush have happened, we need to flush a imem_valid too
    always @(posedge i_clk) begin
        if (i_rst)
            flushed <= 1'b0;
        // else if (i_not_pc_4)
        //don't let the jal take in to calculation because it's for sure taken
        else if (((i_not_pc_4&& id_ex_valid && (!i_id_ex_predict_taken || id_ex_Jalr)) 
                || (i_is_pc_4 && id_ex_valid && i_id_ex_predict_taken&& !id_ex_Jalr))
                && imem_busy)// when we make a wrong prediction, we need to flush the instruction in fetch stage, and we can use the i_not_pc_4 signal from execute stage to determine whether the instruction in execute stage is a control transfer instruction or not, because when a control transfer instruction is in execute stage, the i_not_pc_4 signal will be high, and we can use it to determine whether to flush the instruction in fetch stage or not, and we also need to check if the imem is busy, because if the imem is not busy, it means that we have already consumed the imem_valid signal for the instruction in fetch stage, so we don't need to flush it again, but if the imem is busy, it means that we have not consumed the imem_valid signal for the instruction in fetch stage yet, so we need to flush it by setting the flushed signal to 1
            flushed <= 1'b1; // when a flush happens, we set the flushed signal to 1, and we will keep it until the instruction in execute stage is retired, because the instruction in execute stage is the one that causes the flush, and we need to make sure that the instruction in execute stage is retired before we clear the flushed signal, otherwise we might have some corner case where the instruction in execute stage is not retired yet but we have already cleared the flushed signal, which will cause some problem in the next PC calculation in hart, because the next PC calculation in hart will use the i_not_pc_4 signal from execute stage to determine whether to update the PC or not, if we have already cleared the flushed signal but the instruction in execute stage is not retired yet, then we might update the PC based on the wrong i_not_pc_4 signal from execute stage, which will cause some problem in the next PC calculation in hart.
        else if (flushed && i_imem_valid)   //must be flush condition
            flushed <= 1'b0; // when the instruction in execute stage is retired, we clear the flushed signal
    end


    //2 bit branch predictor prediction
    always @(posedge i_clk) begin
        if (i_rst)
            branch_predictor <= 2'b01; // weak not taken
        //we don't let jalr affect our predictor
        else if (id_ex_Branch &&i_not_pc_4 && id_ex_valid && branch_predictor != 2'b11 && !id_ex_Jalr) begin// when a control transfer instruction is in execute stage, we update the branch predictor based on whether the branch is taken or not, we can use the i_not_pc_4 signal from execute stage to determine whether the branch is taken or not, because when a control transfer instruction is in execute stage, the i_not_pc_4 signal will be high, and we can use it to determine whether the branch is taken or not
            branch_predictor <= branch_predictor + 1; // taken +1 when not full
        end else if (id_ex_Branch && !i_not_pc_4 && id_ex_valid && branch_predictor != 2'b00 && !id_ex_Jalr ) begin
            branch_predictor <= branch_predictor - 1; // not taken -1 when not full
        end// when a control transfer instruction is in execute stage, but it's not taken, we update the branch predictor to not taken
    end
    //instaniate immediate generator
    wire [5:0] w_imm_format;
    wire [6:0] w_opcode = i_if_instr[6:0];
    
    assign w_imm_format[0] = (w_opcode == 7'b0110011); // R-type
    assign w_imm_format[1] = (w_opcode == 7'b0010011) || // I-type (Imm)
                             (w_opcode == 7'b0000011) || // I-type (Load)
                             (w_opcode == 7'b1100111);   // I-type (Jalr)
    assign w_imm_format[2] = (w_opcode == 7'b0100011); // S-type
    assign w_imm_format[3] = (w_opcode == 7'b1100011); // B-type
    assign w_imm_format[4] = (w_opcode == 7'b0110111) || // U-type (LUI)
                             (w_opcode == 7'b0010111);   // U-type (AUIPC)
    assign w_imm_format[5] = (w_opcode == 7'b1101111); // J-type
    imm u_immg_fetch (
        .i_inst (i_if_instr),
        .i_format(w_imm_format),
        .o_immediate(branch_address_offset)
    );
endmodule
