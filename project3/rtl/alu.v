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
    wire [31:0] add_sub_result;
    wire [31:0] shift_left_result;
    wire [31:0] shift_right_result;
    wire [31:0] xor_result;
    wire [31:0] or_result;
    wire [31:0] and_result;
    wire [31:0] slt_result;

    //addition/subtraction
    assign add_sub_result = i_sub ? (i_op1 - i_op2) : (i_op1 + i_op2);

    //shift left logical
    assign shift_left_result = i_op1 << i_op2[4:0];

    //shift right logical/arithmetic
    //somehow the signed right shift operator >>> does not work in this case, so we have to do it manually 
    //by filling the sign bit if the sign bit is 1 and the shift amount is not 0
    //assign shift_right_result = i_arith ? $signed({i_op1}) >>> i_op2[4:0] : i_op1 >> i_op2[4:0];
    wire [4:0] shamt = i_op2[4:0];
    wire [31:0] logical = i_op1 >> shamt;
    // If sign bit is 1, make a mask with top `shamt` bits = 1, else 0.
    wire [31:0] sign_fill = (i_op1[31] && (shamt != 0)) ? ~(32'hFFFF_FFFF >> shamt)
                                                        : 32'h0000_0000;
    wire [31:0] arith = logical | sign_fill;
    assign shift_right_result = i_arith ? arith : logical;
    
    //xor
    assign xor_result = i_op1 ^ i_op2;

    //or
    assign or_result = i_op1 | i_op2;

    //and
    assign and_result = i_op1 & i_op2;

    //set less than/unsigned
    assign slt_result = {31'b0, o_slt};

    //result selection
    assign o_result = (i_opsel == 3'b000) ? add_sub_result :
                      (i_opsel == 3'b001) ? shift_left_result :
                      (i_opsel == 3'b010 || i_opsel == 3'b011) ? slt_result :
                      (i_opsel == 3'b100) ? xor_result :
                      (i_opsel == 3'b101) ? shift_right_result :
                      (i_opsel == 3'b110) ? or_result :
                      (i_opsel == 3'b111) ? and_result :
                      32'b0;
    //equality result
    assign o_eq = (i_op1 == i_op2);

    //set less than result
    assign o_slt = i_unsigned ? (i_op1 < i_op2) :
                             ($signed(i_op1) < $signed(i_op2));;
endmodule

`default_nettype wire
