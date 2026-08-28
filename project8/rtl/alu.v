`default_nettype none

// The arithmetic logic unit (ALU) is responsible for performing the core
// calculations of the processor. It takes two 32-bit operands and outputs
// a 32 bit result based on the selection operation - addition, comparison,
// shift, or logical operation. This ALU is a purely combinational block, so
// you should not attempt to add any registers or pipeline it.
module alu (
    // NOTE: Both 3'b010 and 3'b011 are used for set less than operations and
    // your implementation should output the same result for both codes. The
    // reason for this will become clear in project 3.
    //
    // Major operation selection.
    // 3'b000: addition/subtraction if `i_sub` asserted
    // 3'b001: shift left logical
    // 3'b010,
    // 3'b011: set less than/unsigned if `i_unsigned` asserted
    // 3'b100: exclusive or
    // 3'b101: shift right logical/arithmetic if `i_arith` asserted
    // 3'b110: or
    // 3'b111: and
    input  wire [ 2:0] i_opsel,
    // When asserted, addition operations should subtract instead.
    // This is only used for `i_opsel == 3'b000` (addition/subtraction).
    input  wire        i_sub,
    // When asserted, comparison operations should be treated as unsigned.
    // This is used for branch comparisons and set less than unsigned. For
    // b ranch operations, the ALU result is not used, only the comparison
    // results.
    input  wire        i_unsigned,
    // When asserted, right shifts should be treated as arithmetic instead of
    // logical. This is only used for `i_opsel == 3'b101` (shift right).
    input  wire        i_arith,
    // First 32-bit input operand.
    input  wire [31:0] i_op1,
    // Second 32-bit input operand.
    input  wire [31:0] i_op2,
    // 32-bit output result. Any carry out should be ignored.
    output wire [31:0] o_result,
    // Equality result. This is used externally to determine if a branch
    // should be taken.
    output wire        o_eq,
    // Set less than result. This is used externally to determine if a branch
    // should be taken.
    output wire        o_slt
);
    // TODO: Fill in your implementation here.

    // Equality result
    assign o_eq = (i_op1 == i_op2);

    // Set less than result
    //assign o_slt = (i_unsigned) ? (i_op1 < i_op2) : ($signed(i_op1) < $signed(i_op2));
    wire slt_signed;
    assign slt_signed = (i_op1[31] ^ i_op2[31]) ? i_op1[31] : (i_op1 < i_op2); // signed comparison: if signs differ, the one with sign bit 1 is smaller; if signs are the same, compare normally
    assign o_slt = (i_unsigned) ? (i_op1 < i_op2) : slt_signed;

     // shift amount
    wire [4:0] shamt;
    assign shamt = i_op2[4:0];

    // Shift left logical result
    wire [31:0] sll_result;
    wire [31:0] sll1, sll2, sll3, sll4, sll5;
    assign sll1 = shamt[0] ? {i_op1[30:0], 1'b0} : i_op1;                 // << 1
    assign sll2 = shamt[1] ? {sll1[29:0], 2'b00} : sll1;                  // << 2
    assign sll3 = shamt[2] ? {sll2[27:0], 4'b0000} : sll2;                // << 4
    assign sll4 = shamt[3] ? {sll3[23:0], 8'b00000000} : sll3;           // << 8
    assign sll5 = shamt[4] ? {sll4[15:0], 16'b0000000000000000} : sll4;// << 16
    assign sll_result = sll5;
    
    // Shift right logical result
    wire [31:0] srl_result;
    wire [31:0] srl1, srl2, srl3, srl4, srl5;
    assign srl1 = shamt[0] ? {1'b0, i_op1[31:1]} : i_op1;                 // >> 1
    assign srl2 = shamt[1] ? {2'b00, srl1[31:2]} : srl1;                  // >> 2
    assign srl3 = shamt[2] ? {4'b0000, srl2[31:4]} : srl2;                // >> 4
    assign srl4 = shamt[3] ? {8'b00000000, srl3[31:8]} : srl3;           // >> 8
    assign srl5 = shamt[4] ? {16'b0000000000000000, srl4[31:16]} : srl4;// >> 16
    assign srl_result = srl5;
    
    // Shift right arithmetic result
    wire [31:0] sra_result;
    wire [31:0] sra1, sra2, sra3, sra4, sra5;
    assign sra1 = shamt[0] ? {{1{i_op1[31]}},  i_op1[31:1]} : i_op1;            // >> 1
    assign sra2 = shamt[1] ? {{2{i_op1[31]}},  sra1[31:2]}  : sra1;             // >> 2
    assign sra3 = shamt[2] ? {{4{i_op1[31]}},  sra2[31:4]}  : sra2;             // >> 4
    assign sra4 = shamt[3] ? {{8{i_op1[31]}},  sra3[31:8]}  : sra3;             // >> 8
    assign sra5 = shamt[4] ? {{16{i_op1[31]}}, sra4[31:16]} : sra4;             // >> 16
    assign sra_result = sra5;

    // ALU result
    reg [31:0] result;
    always @(*) begin
        case (i_opsel)
            3'b000: result = (i_sub) ? (i_op1 - i_op2) : (i_op1 + i_op2);
            3'b001: result = sll_result;
            3'b010,
            3'b011: result = {31'b0, o_slt};
            3'b100: result = i_op1 ^ i_op2;
            3'b101: result = (i_arith) ? sra_result : srl_result;
            // 3'b101: result = $signed(i_op1) >>> i_op2[4:0] ;
            3'b110: result = i_op1 | i_op2;
            3'b111: result = i_op1 & i_op2;
            default: result = 32'b0; // default case to avoid latches
        endcase
    end
    assign o_result = result;
endmodule

`default_nettype wire

