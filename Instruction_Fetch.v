`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/06/2025 10:56:42 AM
// Design Name: 
// Module Name: instruction_memory
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

module instruction_fetch(
    input clk,
    input [31:0] pc,
    output reg [31:0] instruction
);
    reg [31:0] memory [0:1023];
    
    
    initial begin
        // Load instructions from file or initialize directly
        $readmemh("instructions.mem", memory);
        $display("Instruction memory loaded successfully.");
    end
    always @(posedge clk) begin
        instruction = memory[pc[11:2]];
    end
    initial begin
    $monitor("Time: %0t | PC: %h | Instruction: %h", $time, pc, instruction);
end


endmodule
