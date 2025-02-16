`timescale 1ns / 1ps

module execute1 (
    input clk,
    input wire [6:0] opcode,               // Opcode from instruction
    input wire [31:0] rs1_data,            // Data from rs1 (source register 1)
    input wire [31:0] rs2_data,            // Data from rs2 (source register 2) or immediate
    input wire [31:0] imm,                 // Immediate value
    input wire [31:0] pc,                  // Current program counter (PC)
    input wire [4:0] rd,                   // Destination register
    input wire branch,                     // Branch signal from control unit
    input wire jump,                       // Jump signal from control unit
    input wire alu_src,                    // ALU source signal: 1 for immediate, 0 for register
    input wire [3:0] alu_op,               // ALU operation code (from control unit)
    output reg [31:0] alu_result,          // Result from ALU
    output reg [31:0] mem_addr,            // Memory address for LOAD/S-type instruction
    output reg [31:0] next_pc,             // Next program counter
    output reg branch_taken,               // Indicates if branch was taken
    output reg [4:0] rd_out                // Destination register (for R, I, LOAD types)
);

    reg [31:0] alu_operand_2;              // ALU second operand (rs2 or immediate)
    //assign opcode = instruction[6:0]; 
    always @(*) begin
        alu_operand_2 = alu_src ? imm : rs2_data;  // Select ALU second operand
        branch_taken = 1'b0;                       // Default values
        next_pc = pc + 4;
        rd_out = rd;
        mem_addr = 32'h0;

        $display("Time=%t | Opcode=%h | PC=%h | rs1_data=%h | rs2_data=%h | Imm=%h|mem_addr=%h| alu_src=%b | alu_operand_2=%h", 
                 $time, opcode, pc, rs1_data, rs2_data, imm,mem_addr,alu_src, alu_operand_2);
       // $display("Time=%t | alu_src=%b | alu_operand_2=%h", $time, alu_src, alu_operand_2);
        //$display("Time=%t | rs1_data=%h | imm=%h | mem_addr=%h", $time, rs1_data, imm, mem_addr);


        case (opcode)
            7'b0110011: begin  // R-Type instructions
                $display("R-Type Instruction Detected");
                // ALU result already computed based on alu_op
            end

            7'b0010011: begin  // I-Type instructions (e.g., ADDI)
                $display("I-Type Instruction Detected");
                // ALU result already computed based on alu_op
            end

            7'b0000011: begin  // LOAD instructions (e.g., LW)
                mem_addr = rs1_data + imm;         // Compute memory address
                $display("LOAD Instruction Detected | MemAddr=%h", mem_addr);
            end

            7'b0100011: begin  // STORE instructions (e.g., SW)
                mem_addr = rs1_data + imm;         // Compute memory address
                $display("STORE Instruction Detected | MemAddr=%h", mem_addr);
            end

            7'b1100011: begin  // BRANCH instructions (e.g., BEQ)
                case (alu_op)
                    4'b0000: branch_taken = (rs1_data == rs2_data);      // BEQ
                    4'b0001: branch_taken = (rs1_data != rs2_data);      // BNE
                    4'b0010: branch_taken = ($signed(rs1_data) < $signed(rs2_data));   // BLT
                    4'b0011: branch_taken = ($signed(rs1_data) >= $signed(rs2_data));  // BGE
                endcase

                if (branch_taken) begin
                    next_pc = pc + imm;           // Update PC if branch is taken
                    $display("BRANCH Taken | NextPC=%h", next_pc);
                end else begin
                    $display("BRANCH Not Taken");
                end
            end

            7'b1101111: begin  // JAL instruction (J-Type)
                next_pc = pc + imm;               // Compute target PC for jump
                $display("JAL Instruction Detected | NextPC=%h", next_pc);
            end

            7'b1100111: begin  // JALR instruction (I-Type)
                next_pc = (rs1_data + imm) & ~32'b1;   // Compute target PC for jump and align to word boundary
                $display("JALR Instruction Detected | NextPC=%h", next_pc);
            end

            default: begin
                alu_result = 32'h0;
                next_pc = pc + 4;
                branch_taken = 1'b0;
                mem_addr = 32'h0;
                rd_out = rd;
                $display("Unknown or Unsupported Opcode Detected");
            end
        endcase

        case (alu_op)   // Perform ALU operation based on alu_op signal
            4'b0010: alu_result = rs1_data + alu_operand_2;    // ADD
            4'b1010: alu_result = rs1_data - alu_operand_2;    // SUB
            4'b0100: alu_result = rs1_data & alu_operand_2;    // AND
            4'b0101: alu_result = rs1_data | alu_operand_2;    // OR
            4'b0011: alu_result = rs1_data ^ alu_operand_2;    // XOR
            4'b0110: alu_result = rs1_data << alu_operand_2[4:0];   // SLL
            4'b0111: alu_result = rs1_data >> alu_operand_2[4:0];   // SRL
            4'b1000: alu_result = $signed(rs1_data) >>> alu_operand_2[4:0];   // SRA
        //     localparam ALU_SLT   = 4'b1011;
        //      localparam ALU_SLTU  = 4'b1100;
        
        endcase
        



        $display("ALU Operation Completed | ALUOp=%b | ALUResult=%h", alu_op, alu_result);
        $display("---------------------------------------------");
    end

endmodule
