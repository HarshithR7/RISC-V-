`timescale 1ns / 1ps

module riscv_processor_tb;

    // Testbench signals
    reg clk;
    reg reset;
    wire [31:0] alu_result;

    // Instantiate the RISC-V Processor module
    riscv_processor uut (
        .clk(clk),
        .reset(reset),
        .alu_result(alu_result)
    );

    // Clock generation
    initial begin
        clk = 0;          // Initialize clock to 0
        forever #50 clk = ~clk;
    end

   // In testbench
initial begin
    // Initialize signals
    clk = 0;
    reset = 1;  // Assert reset initially
    
    // Hold reset for a few clock cycles
    #10;
    reset = 0;  // De-assert reset
    
    // Continue simulation
    #1000;
    $finish;
end
    // Monitor important signals
    initial begin
        $monitor("Time: %t, Reset: %b, alu_result: %h", 
                 $time, reset, alu_result);
    end

    // Optional: Add waveform dumping
    initial begin
        $dumpfile("riscv_processor.vcd");
        $dumpvars(0, riscv_processor_tb);
    end

endmodule
