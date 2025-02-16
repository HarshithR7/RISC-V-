`timescale 1ns / 1ps

module data_memory(
    input clk,
    input mem_read,
    input mem_write,
    input [2:0] func3,
    input [31:0] mem_addr,
    input [31:0] write_data,
    output reg [31:0] read_data
);

    // 4KB byte-addressable memory (1024 x 32-bit words)
    reg [31:0] memory [0:1023];
    wire [9:0] word_addr = mem_addr[11:2];  // Word-aligned address
    
    // Initialize memory with data.mem contents
    initial begin
        // First initialize all memory locations to 0
        for (integer i = 0; i < 1024; i = i + 1)
            memory[i] = 32'b0;
            
        // Then load data.mem contents
        $readmemh("data.mem", memory);
        $display("Memory initialized with data.mem contents");
    end

    // Synchronous write with debug
    always @(posedge clk) begin
        if (mem_write) begin
            memory[word_addr] <= write_data;
            $display("Time=%0t | MEMORY WRITE | Address=%h | Data=%h", 
                     $time, word_addr, write_data);
        end
    end
    
    // Asynchronous read with debug
    always @(*) begin
        read_data = mem_read ? memory[word_addr] : 32'b0;
        if (mem_read)
            $display("Time=%0t | MEMORY READ | Address=%h | Data=%h", 
                     $time, word_addr, read_data);
    end

    // Memory operation monitoring
    always @(mem_read or mem_write or mem_addr or write_data) begin
        $display("Time=%0t | Memory Operation:", $time);
        $display("  MemRead=%b | MemWrite=%b", mem_read, mem_write);
        $display("  Address=%h | WordAddr=%h", mem_addr, word_addr);
        $display("  WriteData=%h | ReadData=%h", write_data, read_data);
        $display("---------------------------------------------");
    end

endmodule
