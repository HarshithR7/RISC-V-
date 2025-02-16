`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/15/2025 09:46:48 PM
// Design Name: 
// Module Name: Instruction_decode_control_unit
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


module Instruction_decode_control_unit(
    input clk,
    input [31:0] instruction,     // 32-bit instruction input
    
    // Output signals for instruction decode
    output wire [6:0] opcode,     // Opcode (7 bits)
    output wire [4:0] rs1,        // Source register 1 (5 bits)
    output wire [4:0] rs2,        // Source register 2 (5 bits)
    output wire [4:0] rd,         // Destination register (5 bits)
    output wire [2:0] func3,      // Function code 3 (3 bits)
    output wire [6:0] func7,      // Function code 7 (7 bits, R-type)
    output reg [31:0] imm,        // Immediate value
    
    // Control signals
    output reg reg_dst,
    output reg branch,
    output reg mem_read,
    output reg mem_to_reg,
    output reg [3:0] alu_op,
    output reg mem_write,
    output reg reg_write,
    output reg jump,
    output reg branch_type,
    output reg is_shift,
    output reg alu_src               // ALU source selection signal
);
    // Extract common fields from the instruction
    assign opcode = instruction[6:0];         // Bits 0-6
    assign rs1    = instruction[19:15];       // Bits 15-19
    assign rs2    = instruction[24:20];       // Bits 20-24
    assign rd     = instruction[11:7];        // Bits 7-11
    assign func3  = instruction[14:12];       // Bits 12-14
    assign func7  = instruction[31:25];       // Bits 25-31 (R-type only)

    // Opcode definitions for RV32I ISA
    localparam R_TYPE    = 7'b0110011;
    localparam I_TYPE    = 7'b0010011;
    localparam S_TYPE    = 7'b0100011;
    localparam B_TYPE    = 7'b1100011;
    localparam LUI_TYPE  = 7'b0110111;
    localparam AUIPC     = 7'b0010111;
    localparam JAL       = 7'b1101111;
    localparam JALR      = 7'b1100111;
    localparam LOAD      = 7'b0000011;

    // ALU Operation codes
    localparam ALU_ADD   = 4'b0010;
    localparam ALU_SUB   = 4'b1010;
    localparam ALU_AND   = 4'b0100;
    localparam ALU_OR    = 4'b0101;
    localparam ALU_XOR   = 4'b0011;
    localparam ALU_SLL   = 4'b0110;
    localparam ALU_SRL   = 4'b0111;
    localparam ALU_SRA   = 4'b1000;
    localparam ALU_SLT   = 4'b1011;
    localparam ALU_SLTU  = 4'b1100;

    // Immediate generation using always block
    always @(posedge clk) begin
        case (opcode)
            LOAD, I_TYPE, JALR: 
                imm = {{20{instruction[31]}}, instruction[31:20]};  // Immediate for LOAD, I-Type, JALR
            S_TYPE: 
                imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};  // Immediate for S-Type
            B_TYPE: 
                imm = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};  // Immediate for B-Type
            JAL: 
                imm = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};  // Immediate for J-Type
            default: 
                imm = 32'b0;
        endcase
    end

    // Control logic generation based on opcode and function codes
    always @(posedge clk) begin
        // Default control signal values
        reg_dst     = 0;
        branch      = 0;
        mem_read    = 0;
        mem_to_reg  = 0;
        alu_op      = ALU_ADD;
        mem_write   = 0;
        reg_write   = 0;
        jump        = 0;
        is_shift    = 0;
        branch_type = 0;
        alu_src     = 0;  // Default: use register as second ALU operand

        case (opcode)
            // R-Type Instructions
            R_TYPE: begin
                reg_dst   = 1;
                reg_write = 1;
                alu_src   = 0;  // ALU source is register for R-Type
                case ({func7, func3})
                    10'b0000000_000: alu_op = ALU_ADD;   // ADD
                    10'b0100000_000: alu_op = ALU_SUB;   // SUB
                    10'b0000000_111: alu_op = ALU_AND;   // AND
                    10'b0000000_110: alu_op = ALU_OR;    // OR
                    10'b0000000_100: alu_op = ALU_XOR;   // XOR
                    10'b0000000_001: begin
                        alu_op = ALU_SLL;                 // SLL
                        is_shift = 1;
                    end
                    10'b0000000_101: begin
                        alu_op = ALU_SRL;                 // SRL
                        is_shift = 1;
                    end
                    10'b0100000_101: begin
                        alu_op = ALU_SRA;                 // SRA
                        is_shift = 1;
                    end
                    10'b0000000_010: alu_op = ALU_SLT;   // SLT
                    10'b0000000_011: alu_op = ALU_SLTU;  // SLTU
                endcase
            end

            // I-Type Instructions (ALU operations)
            I_TYPE: begin
                reg_write = 1;
                alu_src   = 1;  // ALU source is immediate for I-Type
                case (func3)
                    3'b000: alu_op = ALU_ADD;   // ADDI
                    3'b010: alu_op = ALU_SLT;   // SLTI
                    3'b011: alu_op = ALU_SLTU;  // SLTIU
                    3'b100: alu_op = ALU_XOR;   // XORI
                    3'b110: alu_op = ALU_OR;    // ORI
                    3'b111: alu_op = ALU_AND;   // ANDI
                    3'b001: begin
                        if (func7 == 7'b0000000) begin
                            alu_op = ALU_SLL;   // SLLI
                            is_shift = 1;
                        end
                    end
                    3'b101: begin
                        if (func7 == 7'b0000000) begin
                            alu_op = ALU_SRL;   // SRLI
                            is_shift = 1;
                        end
                        else if (func7 == 7'b0100000) begin
                            alu_op = ALU_SRA;   // SRAI
                            is_shift = 1;
                        end
                    end
                endcase
            end

            // Load Instructions
            LOAD: begin
                mem_to_reg = 1;
                reg_write = 1;
                mem_read  = 1;
                alu_src   = 1;  // ALU source is immediate for Load instructions
                alu_op    = ALU_ADD;   // Address calculation
            end

            // Store Instructions (S-Type)
            S_TYPE: begin
                mem_write = 1;
                alu_src   = 1;  // ALU source is immediate for Store instructions
                alu_op    = ALU_ADD;   // Address calculation
            end

            // Branch Instructions (B-Type)
            B_TYPE: begin
                branch    = 1;
                alu_src   = 0;  // ALU source is register for branch comparison
                alu_op    = ALU_SUB;   // Branch comparison (e.g., BEQ, BNE)
                branch_type = func3;
            end

            // JAL (J-Type)
            JAL: begin
                jump      = 1;
                reg_write = 1;
            end

            // JALR (I-Type)
            JALR: begin
                jump      = 1;
                reg_write = 1;
                alu_src   = 1;  // ALU source is immediate for JALR
                alu_op    = ALU_ADD;   // PC + immediate
            end

            // LUI (U-Type)
            LUI_TYPE: begin
                reg_write = 1;
                alu_op    = ALU_ADD;  // Loading upper immediate
                alu_src   = 1;  // Use immediate value
            end
        endcase
    end
endmodule
