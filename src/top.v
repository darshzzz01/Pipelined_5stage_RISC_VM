module top_tb;
  //Ports
  reg  clk = 1'b0;
  reg  we = 1'b0;
  reg  reset = 1'b0;
  reg [31:0] data;
  reg [1:0] p_sel = 2'b00;
  reg [15:0] P0 = 15'd0;
  reg [15:0] P1 = 15'd0;

  wire [15:0] P0_out;
  wire [15:0] P1_out;
  wire [15:0] data_out;

  top  U0 (
         //==========Input========//
         .clk(clk),
         .we(we),
         .reset(reset),
         .data(data),
         .p_sel(p_sel),
         .P0(P0),
         .P1(P1),
         //==========Outputs======//
         .P0_out(P0_out),
         .P1_out(P1_out),
         .data_out(data_out)
       );

  always #5     clk = ! clk ;

  initial
  begin
    U0.U7.R0.reg_data = 16'b0100000100000001;
    U0.U7.R1.reg_data = 16'b0011100000000001;
    U0.U7.R2.reg_data = 16'd0;
    U0.U7.R3.reg_data = 16'd0;
    U0.U7.R4.reg_data = 16'd0;
    U0.U7.R5.reg_data = 16'd0;
    U0.U7.R6.reg_data = 16'd0;
    U0.U7.R7.reg_data = 16'd0;
  end

endmodule

module top(
    input                           clk,                            // Input Clock
    input                           we,                             // RAM Write Enable
    input                           reset,                          // CHIP Reset signal
    input       [31:0]              data,                           // Clock input
    input       [1:0]               p_sel,
    input       [15:0]              P0,
    input       [15:0]              P1,

    output      [15:0]              P0_out,
    output      [15:0]              P1_out,
    output      [15:0]              data_out
  );

  wire          [4:0]               addr,addr_reg;


  wire          [31:0]              instr,instr_reg;
  wire          [3:0]               op;
  wire          [2:0]               rs,rA,rB;
  wire                              jz;
  wire          [4:0]               start,end_addr;
  wire          [15:0]              out_ALU,out_A,out_B;



  AddrGen  U1(
             .clk(clk),
             .reset(reset),
             .jz(1'b0),
             .start(5'd0),
             .end_addr(5'd31),
             .count(addr)
           );

  register_5bit  U2 (
                   .clk(clk),
                   .rst(reset),
                   .din(addr),
                   .en(1'b1),
                   .dout(addr_reg)
                 );

  ProgramMem  U3(
                .clk(clk),
                .we(we),
                .data(data),
                .addr(addr_reg),
                .data_out(instr)
              );

  register_32bit  U4 (
                    .clk(clk),
                    .reset(reset),
                    .enable(1'b1),
                    .data_in(instr),
                    .data_out(instr_reg)
                  );

  inst_fetch  U5 (
                .instr(instr_reg),
                .op(op),
                .rs(rs),
                .rA(rA),
                .rB(rB),
                .jz(jz),
                .start(start),
                .end_addr(end_addr)
              );

  ALU_16bit  U6 (
                .A(out_A),
                .B(out_B),
                .opcode(op),
                .result(out_ALU)
             );

  RegBank  U7 (
             .clk(clk),
             .reset(reset),
             .data_in(out_ALU),
             .p_sel(p_sel),
             .sel(rs),
             .sel_A(rA),
             .sel_B(rB),
             .P0(P0),
             .P1(P1),

             .out_A(out_A),
             .out_B(out_B),
             .P0_out(P0_out),
             .P1_out(P1_out)
           );

endmodule

module ProgramMem (
    input                           clk,                            // Input Clock
    input                           we,                             // RAM Write Enable
    input [31:0]                    data,                           // Clock input
    input [4:0]                     addr,                           // Address input (assuming 32 locations)
    output [31:0]                   data_out                        // Data output
  );

  // Declare memory array
  reg [31:0] memory [31:0];                                         // 32 locations, each 32 bits wide
  initial
  begin
    memory[0] = 32'b0000_010_000_001_0000000000000000000; 
    memory[1] = 32'b0001_010_000_001_0000000000000000000;
    memory[2] = 32'b0010_010_000_001_0000000000000000000;
    memory[3] = 32'b1001_010_000_001_0000000000000000000;
  end
  // Variable to hold the registered read address
  reg [4:0] addr_reg;
  // Read and write operations
  always @(posedge clk)
  begin
    if(we)
      memory[addr] <= data;
    addr_reg <= addr;
  end

  assign data_out = memory[addr_reg];

endmodule

module AddrGen (
    input clk,                                                      // Clock input
    input reset,                                                    // Reset input
    input jz,
    input [4:0]start,
    input [4:0]end_addr,

    output [4:0] count                                              // 8-bit counter output
  );

  // Define the counter
  reg [4:0] counter;
  reg [4:0] R0;
  reg [4:0] R1;

  // Synchronous counter
  always @(posedge clk)
  begin
    if (reset)
      counter <= 5'b00000;
    else
      counter <= counter + 5'd1;

    if(jz)
    begin
      counter <= start;
      R0 <= end_addr;
      R1 <= counter;
    end

    if(counter == R0)
    begin
      counter <= R1;
    end
  end
  // Output the counter value
  assign count = counter;
endmodule

module register_5bit(
    input clk,       // Clock input
    input rst,       // Reset input
    input [4:0] din, // Data input
    input en,        // Enable input
    output [4:0] dout // Data output
  );

  // Register declaration
  reg [4:0] reg_data;

  // Register behavior
  always @(posedge clk or posedge rst)
  begin
    if (rst)
    begin
      reg_data <= 5'd0; // Reset the register to all zeros
    end
    else if (en)
    begin
      reg_data <= din; // Write data to the register when en is high
    end
  end

  // Assign output
  assign dout = reg_data;

endmodule

module register_32bit (
    input wire clk,         // Clock input
    input wire reset,       // Reset input
    input wire enable,      // Enable input
    input wire [31:0] data_in, // Data input
    output [31:0] data_out // Data output
  );

  // Register declaration
  reg [31:0] reg_data;

  // Clock edge sensitive process
  always @(posedge clk or posedge reset)
  begin
    if (reset)
    begin
      // Reset condition
      reg_data <= 32'b0;
    end
    else if (enable)
    begin
      // If enable is high, update the register with data_in
      reg_data <= data_in;
    end
  end

  // Assigning output
  assign data_out = reg_data;

endmodule


module inst_fetch(
    input [31:0] instr,
    output [3:0] op,
    output [2:0] rs, rA, rB,
    output jz,
    output [4:0] start, end_addr
  );
  assign op = instr[31:28];
  assign rs = instr[27:25];
  assign rA = instr[24:22];
  assign rB = instr[21:19];

  assign jz = instr[0];
  assign start = instr[5:1];
  assign end_addr = instr[10:6];

endmodule

module ALU_16bit(
    input [15:0] A,               //input [31:0] A,
    input [15:0] B,               //input [31:0] B,
    input [3:0] opcode,
    output reg [31:0] result
  );
    wire [31:0] w2; // Wire to hold the product from the vedic multiplier

    // Instantiating the vedic multiplier
    vedic_multiplier_16bit n0 (
        .A(A),
        .B(B),
        .product(w2)
    );
  always @(*)
  begin
    case(opcode)
      4'b0000:
        result =A + B;
      4'b0001:
        result = A - B;
      4'b0010:
        result = w2;
      4'b0011:
        result = A & B;
      4'b0100:
        result = A | B;
      4'b0101:
        result = A ^ B;
      4'b0110:
        result = (A>B)?1:0;
      4'b0111:
        result = (A<B)?1:0;
      4'b1001:
        result = A;
      default:
        result = 16'h0;
    endcase
  end
endmodule

module RegBank(
    input clk,
    input reset,
    input [15:0] data_in,    //input [31:0] data_in,
    input [1:0] p_sel,
    input [2:0] sel,
    input [2:0] sel_A,
    input [2:0] sel_B,
    input [15:0] P0,
    input [15:0] P1,

    output [15:0] out_A,     //output [31:0] out_A,
    output [15:0] out_B,     //output [31:0] out_B,

    output [15:0] P0_out,   //output [31:0] P0_out,
    output [15:0] P1_out   //output [31:0] P1_out
  );


  wire [15:
        0]w0,w1,w2,w3,w4,w5,w6,w7,w0_p0,w1_p1;  // wire [31:0]w0,w1,w2,w3,w4,w5,w6,w7,w0_p0,w1_p1;
  wire [7:
        0]Reg_EN;

  decoder_3to8  decoder_3to8_inst (
                  .input_A(sel),
                  .y(Reg_EN)
                );

  assign w0_p0 = p_sel[0] ? data_in : P0;
  assign w1_p1 = p_sel[1] ? data_in : P1;

  Reg_16_bit  R0 (
                .clk(clk),
                .reset(reset),
                .enable(Reg_EN[0]),
                .data_in(w0_p0),
                .data_out(w0)
              );

  Reg_16_bit  R1 (
                .clk(clk),
                .reset(reset),
                .enable(Reg_EN[1]),
                .data_in(w1_p1),
                .data_out(w1)
              );

  Reg_16_bit  R2 (
                .clk(clk),
                .reset(reset),
                .enable(Reg_EN[2]),
                .data_in(data_in),
                .data_out(w2)
              );

  Reg_16_bit  R3 (
                .clk(clk),
                .reset(reset),
                .enable(Reg_EN[3]),
                .data_in(data_in),
                .data_out(w3)
              );

  Reg_16_bit  R4 (
                .clk(clk),
                .reset(reset),
                .enable(Reg_EN[4]),
                .data_in(data_in),
                .data_out(w4)
              );

  Reg_16_bit  R5 (
                .clk(clk),
                .reset(reset),
                .enable(Reg_EN[5]),
                .data_in(data_in),
                .data_out(w5)
              );

  Reg_16_bit  R6 (
                .clk(clk),
                .reset(reset),
                .enable(Reg_EN[6]),
                .data_in(data_in),
                .data_out(w6)
              );

  Reg_16_bit  R7 (
                .clk(clk),
                .reset(reset),
                .enable(Reg_EN[7]),
                .data_in(data_in),
                .data_out(w7)
              );

  mux_8to1  U0 (
              .input_0(w0),
              .input_1(w1),
              .input_2(w2),
              .input_3(w3),
              .input_4(w4),
              .input_5(w5),
              .input_6(w6),
              .input_7(w7),
              .select(sel_A),
              .output_Y(out_A)
            );

  mux_8to1  U1 (
              .input_0(w0),
              .input_1(w1),
              .input_2(w2),
              .input_3(w3),
              .input_4(w4),
              .input_5(w5),
              .input_6(w6),
              .input_7(w7),
              .select(sel_B),
              .output_Y(out_B)
            );

  assign P0_out = w0;
  assign P1_out = w1;

endmodule

module mux_8to1 (
    input [15:0] input_0,
    input [15:0] input_1,
    input [15:0] input_2,
    input [15:0] input_3,
    input [15:0] input_4,
    input [15:0] input_5,
    input [15:0] input_6,
    input [15:0] input_7,
    input [2:0] select,
    output reg [15:0] output_Y
  );

  always @*
  begin
    case(select)
      3'b000:
        output_Y = input_0;
      3'b001:
        output_Y = input_1;
      3'b010:
        output_Y = input_2;
      3'b011:
        output_Y = input_3;
      3'b100:
        output_Y = input_4;
      3'b101:
        output_Y = input_5;
      3'b110:
        output_Y = input_6;
      3'b111:
        output_Y = input_7;
      default:
        output_Y = 8'b0;
    endcase
  end

endmodule

module decoder_3to8 (
    input wire [2:0] input_A,         // 3-bit input
    output reg [7:0] y                // 8-bit output
  );

  // Decoder logic
  always @*
  begin
    case(input_A)
      3'b000:
        y = 8'b00000001;
      3'b001:
        y = 8'b00000010;
      3'b010:
        y = 8'b00000100;
      3'b011:
        y = 8'b00001000;
      3'b100:
        y = 8'b00010000;
      3'b101:
        y = 8'b00100000;
      3'b110:
        y = 8'b01000000;
      3'b111:
        y = 8'b10000000;
      default:
        y = 8'b00000000; // Default case
    endcase
  end

endmodule

module Reg_16_bit (
    input clk,                      // Clock input
    input reset,                    // Reset input
    input enable,                   // Enable input
    input [15:0] data_in,           //input [31:0] data_in,   // Data input
    output [15:0] data_out          //output [15:0] data_out  // Data output
  );

  // Define the register
  reg [15:
       0] reg_data;              //reg [31:0] reg_data;

  // Synchronous reset
  always @(posedge clk or posedge reset)
  begin
    if (reset)
      reg_data <= 16'd0;  // reg_data <= 32'd0;
    else if (enable)
      reg_data <= data_in;
  end

  // Output the registered data
  assign data_out = reg_data;

endmodule

module vedic_multiplier_16bit(
    input [15:0] A, // First 16-bit input
    input [15:0] B, // Second 16-bit input
    output [31:0] product // 32-bit output product
);

    wire [15:0] p0, p1, p2, p3;
    wire [7:0] A1, A0, B1, B0;

    // Splitting the 16-bit inputs into two 8-bit segments
    assign A1 = A[15:8];
    assign A0 = A[7:0];
    assign B1 = B[15:8];
    assign B0 = B[7:0];

    // Vedic multipliers for 8x8 bit multiplication
    vedic_multiplier_8bit VM0 (.A(A0), .B(B0), .product(p0));
    vedic_multiplier_8bit VM1 (.A(A1), .B(B0), .product(p1));
    vedic_multiplier_8bit VM2 (.A(A0), .B(B1), .product(p2));
    vedic_multiplier_8bit VM3 (.A(A1), .B(B1), .product(p3));

    // Calculating final product
    wire [23:0] sum1, sum2, sum3;

    assign sum1 = {8'b0, p0[15:0]};
    assign sum2 = {p1[15:0], 8'b0} + {p2[15:0], 8'b0};
    assign sum3 = {p3[15:0], 16'b0};

    assign product = sum1 + sum2 + sum3;

endmodule

module vedic_multiplier_8bit(
    input [7:0] A, // First 8-bit input
    input [7:0] B, // Second 8-bit input
    output [15:0] product // 16-bit output product
);

    wire [7:0] p0, p1, p2, p3;
    wire [3:0] A1, A0, B1, B0;

    // Splitting the 8-bit inputs into two 4-bit segments
    assign A1 = A[7:4];
    assign A0 = A[3:0];
    assign B1 = B[7:4];
    assign B0 = B[3:0];

    // Vedic multipliers for 4x4 bit multiplication
    vedic_multiplier_4bit VM0 (.A(A0), .B(B0), .product(p0));
    vedic_multiplier_4bit VM1 (.A(A1), .B(B0), .product(p1));
    vedic_multiplier_4bit VM2 (.A(A0), .B(B1), .product(p2));
    vedic_multiplier_4bit VM3 (.A(A1), .B(B1), .product(p3));

    // Calculating final product
    wire [11:0] sum1, sum2, sum3;

    assign sum1 = {4'b0, p0[7:0]};
    assign sum2 = {p1[7:0], 4'b0} + {p2[7:0], 4'b0};
    assign sum3 = {p3[7:0], 8'b0};

    assign product = sum1 + sum2 + sum3;

endmodule

module vedic_multiplier_4bit(
    input [3:0] A, // First 4-bit input
    input [3:0] B, // Second 4-bit input
    output [7:0] product // 8-bit output product
);

    wire [3:0] p0, p1, p2, p3;
    wire [1:0] A1, A0, B1, B0;

    // Splitting the 4-bit inputs into two 2-bit segments
    assign A1 = A[3:2];
    assign A0 = A[1:0];
    assign B1 = B[3:2];
    assign B0 = B[1:0];

    // Vedic multipliers for 2x2 bit multiplication
    vedic_multiplier_2bit VM0 (.A(A0), .B(B0), .product(p0));
    vedic_multiplier_2bit VM1 (.A(A1), .B(B0), .product(p1));
    vedic_multiplier_2bit VM2 (.A(A0), .B(B1), .product(p2));
    vedic_multiplier_2bit VM3 (.A(A1), .B(B1), .product(p3));

    // Calculating final product
    wire [5:0] sum1, sum2, sum3;

    assign sum1 = {2'b0, p0[3:0]};
    assign sum2 = {p1[3:0], 2'b0} + {p2[3:0], 2'b0};
    assign sum3 = {p3[3:0], 4'b0};

    assign product = sum1 + sum2 + sum3;

endmodule

module vedic_multiplier_2bit(
    input [1:0] A, // First 2-bit input
    input [1:0] B, // Second 2-bit input
    output [3:0] product // 4-bit output product
);

    assign product = A * B; // Direct multiplication for 2x2 bit

endmodule
