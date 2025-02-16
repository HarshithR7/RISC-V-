`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/13/2025 12:15:16 PM
// Design Name: 
// Module Name: register_rw
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module register_rw(
    input clk,
    input reset,
    input reg_write,          // Write enable signal
    input [4:0] read_reg1,    // Address of source register 1
    input [4:0] read_reg2,    // Address of source register 2
    input [4:0] write_reg,    // Address of destination register
    input [31:0] write_data,  // Data to write to destination register
    output reg [31:0] read_data1, // Data from source register 1
    output reg [31:0] read_data2  // Data from source register 2
);
    reg [31:0] registers [0:31]; // Shared register memory

    integer i;

    // Initialize registers on reset
always @(*) begin
//always @(*) begin
    if (reset) begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] <= 32'b0;
        end    
    end
    else if (reg_write && write_reg != 0)
            registers[write_reg] <= write_data; // Write data to register
            

end

    // Read operations (combinational)
    always @(* ) begin
        // Read data from source registers (x0 is hardwired to 0)
         read_data1 <=/* (read_reg1 == 5'b0) ? 32'b0 :*/ registers[read_reg1];
         read_data2 <= /*(read_reg2 == 5'b0) ? 32'b0 :*/ registers[read_reg2];
    end
    always @(*) begin
    $display("Time=%t | rs1=%d | rs2=%d | reg_write=%b| rs1_data=%h | rs2_data=%h", 
             $time, read_reg1, read_reg2,reg_write, read_data1, read_data2);
end

endmodule

