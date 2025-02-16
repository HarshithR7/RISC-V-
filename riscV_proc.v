

`timescale 1ns / 1ps

module riscv_processor(
    input clk,
    input reset,
    output wire [31:0] alu_result,
    output wire [31:0] output_mem_read
);
    // Internal wires
    wire [31:0] pc, next_pc;
    wire [31:0] instruction;
    wire [31:0] read_data1, read_data2;      // Data from register file
    wire [31:0] write_back_data;            // Data to write back to register file
    wire [31:0] imm;
    wire [4:0] rs1, rs2, rd;
    wire reg_write, mem_to_reg;

    // Control signals
    wire branch, jump, alu_src, mem_read, mem_write;
    wire [3:0] alu_op;                      // ALU operation code
    wire [2:0] func3;                       // Function code (3 bits)
    wire [6:0] func7;                       // Function code (7 bits)

    // MEM/WB pipeline register wires
    wire [31:0] mem_wb_read_data;           // Data read from memory (LOAD instructions)
    wire [31:0] mem_wb_alu_result;          // ALU result from EX/MEM stage
    wire [4:0] mem_wb_write_reg;            // Write-back destination register address
    wire mem_wb_reg_write;                  // Write enable signal for WB stage
    wire mem_wb_mem_to_reg;                 // Control signal for memory-to-register

    // Memory address and data wires
    wire [31:0] mem_addr;                   // Memory address
    wire [31:0] read_data;                  // Data read from memory

    // Program Counter Module
    Program_counter pc_module (
        .clk(clk),
        .reset(reset),
        .pc_in(next_pc),
        .pc_out(pc)
    );

    // Instruction Fetch Module
    instruction_fetch if_module (
        .clk(clk),
        .pc(pc),
        .instruction(instruction)
    );

    // Instruction Decode and Control Unit Module (Combining decoding and control signals)
    Instruction_decode_control_unit id_cu_module (
        .clk(clk),
        .instruction(instruction),          // Input instruction
        .rs1(rs1),                          // Source register 1
        .rs2(rs2),                          // Source register 2
        .rd(rd),                            // Destination register
        .imm(imm),                          // Immediate value
        .alu_src(alu_src),                  // ALU source signal
        .branch(branch),                    // Branch control signal
        .jump(jump),                        // Jump control signal
        .reg_write(reg_write),              // Register write control signal
        .mem_read(mem_read),                // Memory read control signal
        .mem_write(mem_write),              // Memory write control signal
        .mem_to_reg(mem_to_reg),            // Memory-to-register control signal
        .alu_op(alu_op),                    // ALU operation code
        .func3(func3),                      // Function code 3 bits
        .func7(func7)                       // Function code 7 bits
    );

    // Unified Register File Module
    register_rw regfile (
        .clk(clk),
        .reset(reset),
        .reg_write(reg_write),              // Write enable signal from MEM/WB stage
        .read_reg1(rs1),                    // Read register 1 address
        .read_reg2(rs2),                    // Read register 2 address
        .write_reg(rd),                     // Write register address from MEM/WB stage
        .write_data(alu_result),                   // Data to write back to register file
        .read_data1(read_data1),            // Output data for rs1
        .read_data2(read_data2)             // Output data for rs2
    );

    // Execute Module
    execute1 ex_module (
        .clk(clk),
        .opcode(instruction[6:0]),          // Extract opcode from instruction
        .rs1_data(read_data1),              // Data from rs1 in register file
        .rs2_data(read_data2),              // Data from rs2 in register file
        .imm(imm),
        .pc(pc),
        .rd(rd),
        .branch(branch),
        .jump(jump),
        .alu_src(alu_src),
        .alu_op(alu_op),                    // Corrected to match width (4 bits)
        .alu_result(alu_result),
        .mem_addr(mem_addr),                // Corrected to match width (32 bits)
        .next_pc(next_pc),
        .branch_taken(branch_taken)
    );

    // Data Memory Module
    data_memory dmem (
        .clk(clk),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .func3(func3),                      // Corrected to match width (3 bits)
        .mem_addr(mem_addr),                // Corrected to match width (32 bits)
        .write_data(alu_result),            // Data from rs2 in register file
        .read_data(output_mem_read)               // Corrected to match width (32 bits)
    );

endmodule
