`default_nettype none

// The register file is effectively a single cycle memory with 32-bit words
// and depth 32. It has two asynchronous read ports, allowing two independent
// registers to be read at the same time combinationally, and one synchronous
// write port, allowing a register to be written to on the next clock edge.
// The register `x0` is hardwired to zero, and writes to it are ignored.
module rf #(
    // When this parameter is set to 1, "RF bypass" mode is enabled. This
    // allows data at the write port to be observed at the read ports
    // immediately without having to wait for the next clock edge. This is
    // a common forwarding optimization in a pipelined core (project 5), but
    // will cause a single-cycle processor to behave incorrectly.
    //
    // You are required to implement and test both modes. In project 3 and 4,
    // you will set this to 0, before enabling it in project 5.
    parameter BYPASS_EN = 0
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Both read register ports are asynchronous (zero-cycle). That is, read
    // data is visible combinationally without having to wait for a clock.
    //
    // Register read port 1, with input address [0, 31] and output data.
    input  wire [ 4:0] i_rs1_raddr,
    output wire [31:0] o_rs1_rdata,
    // Register read port 2, with input address [0, 31] and output data.
    input  wire [ 4:0] i_rs2_raddr,
    output wire [31:0] o_rs2_rdata,
    // The register write port is synchronous. When write is enabled, the
    // write data is visible after the next clock edge.
    // Write register enable, address [0, 31] and input data.
    input  wire        i_rd_wen,
    input  wire [ 4:0] i_rd_waddr,
    input  wire [31:0] i_rd_wdata
);
    reg [31:0] Register_File[31:0];
	
	//Synchronous Write Logic
    genvar k;
    generate
        for (k = 0; k < 32; k = k + 1) begin : reg_file_init
            always @(posedge i_clk) begin
                if (i_rst) begin
                    Register_File[k] <= 32'd0;
                end else if (i_rd_wen && (i_rd_waddr == k) && (k != 0)) begin
                    Register_File[k] <= i_rd_wdata;
                end
            end
        end
    endgenerate
	
	//Asynchronous Read Logic
	assign o_rs1_rdata = (i_rs1_raddr == 5'd0) ? 32'd0 :
						(BYPASS_EN && i_rd_wen && (i_rs1_raddr==i_rd_waddr)) ? 
						i_rd_wdata : Register_File[i_rs1_raddr];
	
	assign o_rs2_rdata = (i_rs2_raddr == 5'd0) ? 32'd0 :
						(BYPASS_EN && i_rd_wen && (i_rs2_raddr==i_rd_waddr)) ? 
						i_rd_wdata : Register_File[i_rs2_raddr];
						
endmodule

`default_nettype wire

