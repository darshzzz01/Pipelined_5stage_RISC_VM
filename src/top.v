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
    // U0.U7.R0.reg_data = 16'b0_10000_0100000000;
    // U0.U7.R1.reg_data = 16'b0_01110_0000000001;
    // U0.U7.R2.reg_data = 16'd0;
    // U0.U7.R3.reg_data = 16'd0;
    // U0.U7.R4.reg_data = 16'd0;
    // U0.U7.R5.reg_data = 16'd0;
    // U0.U7.R6.reg_data = 16'd0;
    // U0.U7.R7.reg_data = 16'd0;
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

  ALU_16bit  U6 (
               .A(out_A),
               .B(out_B),
               .opcode(op),
               .result(out_ALU)
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
    output reg [15:0] result
  );

  wire [15:0] w0,w1,w2;
  wire [15:0] t0;
  assign t0 = (opcode == 4'b0001) ? {1'b1,B[14:0]} : B;
  ieee_16add  U0 (
                .A(A),
                .B(t0),
                .result(w0)
              );
  ieee_16mul  U3 (
                .A(A),
                .B(B),
                .result(w2)
              );
  always @(*)
  begin
    case(opcode)
      4'b0000:
        result = w0;
      4'b0001:
        result = w0;
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

//float adder adds floating point numbers.
module ieee_16add(//Ports
    input [15:0] A, B,
    output [15:0] result);

  reg precisionLost;

  //Reassing numbers as big and small
  reg [15:
       0] bigNum, smallNum; //to seperate big and small numbers
  //Decode big and small number
  wire [9:
        0] big_fra, small_fra; //to hold fraction part
  wire [4:
        0] big_ex_pre, small_ex_pre;
  wire [4:
        0] big_ex, small_ex; //to hold exponent part
  wire big_sig, small_sig; //to hold signs
  wire [10:
        0] big_float, small_float; //to hold as float number with integer
  reg [10:
       0] sign_small_float, shifted_small_float; //preparing small float
  wire [4:
        0] ex_diff; //difrence between exponentials
  reg [9:
       0] sum_shifted; //Shift fraction part of sum
  reg [3:
       0] shift_am;
  wire neg_exp;
  //Extensions for higher precision
  reg [9:
       0] small_extension;
  wire [9:
        0] sum_extension;

  wire [10:
        0] sum; //sum of numbers with integer parts
  wire sum_carry;
  wire sameSign;
  wire zeroSmall;
  wire inf_num; //at least on of the operands is inf.

  wire [4:
        0] res_exp_same_s, res_exp_diff_s;

  //Flags
  assign zero = (A[14:0] == B[14:0]) & (~A[15] == B[15]);
  assign overflow = ((&big_ex[4:1] & ~big_ex[0]) & sum_carry & sameSign) | inf_num;
  assign NaN = (&A[14:10] & |A[9:0]) | (&B[14:10] & |B[9:0]);
  assign inf_num = (&A[14:10] & ~|A[9:0]) | (&B[14:10] & ~|B[9:0]); //check for infinate number
  //Get result
  assign result[15] = big_sig; //result sign same as big sign
  assign res_exp_same_s = big_ex + {4'd0, (~zeroSmall & sum_carry & sameSign)} - {4'd0,({1'b0,result[9:0]} == sum)};
  assign res_exp_diff_s = (neg_exp | (shift_am == 4'd10)) ? 5'd0 : (~shift_am + big_ex + 5'd1);
  assign result[14:
                10] = ((sameSign) ? res_exp_same_s : res_exp_diff_s) | {5{overflow}}; //result exponent
  assign result[9:
                0] = ((zeroSmall) ? big_fra : ((sameSign) ? ((sum_carry) ? sum[10:1] : sum[9:0]) : ((neg_exp) ? 10'd0 : sum_shifted))) & {10{~overflow}};

  //decode numbers
  assign {big_sig, big_ex_pre, big_fra} = bigNum;
  assign {small_sig, small_ex_pre, small_fra} = smallNum;
  assign sameSign = (big_sig == small_sig);
  assign zeroSmall = ~(|small_ex | |small_fra);
  assign big_ex = big_ex_pre + {4'd0, ~|big_ex_pre};
  assign small_ex = small_ex_pre + {4'd0, ~|small_ex_pre};

  //add integer parts
  assign big_float = {|big_ex_pre, big_fra};
  assign small_float = {|small_ex_pre, small_fra};
  assign ex_diff = big_ex - small_ex; //diffrence between exponents
  assign {sum_carry, sum} = sign_small_float + big_float; //add numbers
  assign sum_extension = small_extension;

  //Get shift amount for subtraction
  assign neg_exp = (big_ex < shift_am);
  always@*
  begin
    casex(sum)
      11'b1xxxxxxxxxx:
        shift_am = 4'd0;
      11'b01xxxxxxxxx:
        shift_am = 4'd1;
      11'b001xxxxxxxx:
        shift_am = 4'd2;
      11'b0001xxxxxxx:
        shift_am = 4'd3;
      11'b00001xxxxxx:
        shift_am = 4'd4;
      11'b000001xxxxx:
        shift_am = 4'd5;
      11'b0000001xxxx:
        shift_am = 4'd6;
      11'b00000001xxx:
        shift_am = 4'd7;
      11'b000000001xx:
        shift_am = 4'd8;
      11'b0000000001x:
        shift_am = 4'd9;
      default:
        shift_am = 4'd10;
    endcase
  end

  //Shift result for sub.
  always@*
  begin
    case (shift_am)
      4'd0:
        sum_shifted =  sum[9:0];
      4'd1:
        sum_shifted = {sum[8:0],sum_extension[9]};
      4'd2:
        sum_shifted = {sum[7:0],sum_extension[9:8]};
      4'd3:
        sum_shifted = {sum[6:0],sum_extension[9:7]};
      4'd4:
        sum_shifted = {sum[5:0],sum_extension[9:6]};
      4'd5:
        sum_shifted = {sum[4:0],sum_extension[9:5]};
      4'd6:
        sum_shifted = {sum[3:0],sum_extension[9:4]};
      4'd7:
        sum_shifted = {sum[2:0],sum_extension[9:3]};
      4'd8:
        sum_shifted = {sum[1:0],sum_extension[9:2]};
      4'd9:
        sum_shifted = {sum[0],  sum_extension[9:1]};
      default:
        sum_shifted = sum_extension;
    endcase
    case (shift_am)
      4'd0:
        precisionLost = |sum_extension;
      4'd1:
        precisionLost = |sum_extension[8:0];
      4'd2:
        precisionLost = |sum_extension[7:0];
      4'd3:
        precisionLost = |sum_extension[6:0];
      4'd4:
        precisionLost = |sum_extension[5:0];
      4'd5:
        precisionLost = |sum_extension[4:0];
      4'd6:
        precisionLost = |sum_extension[3:0];
      4'd7:
        precisionLost = |sum_extension[2:0];
      4'd8:
        precisionLost = |sum_extension[1:0];
      4'd9:
        precisionLost = |sum_extension[0];
      default:
        precisionLost = 1'b0;
    endcase
  end

  //take small number to exponent of big number
  always@*
  begin
    case (ex_diff)
      5'h0:
        {shifted_small_float,small_extension} = {small_float,10'd0};
      5'h1:
        {shifted_small_float,small_extension} = {small_float,9'd0};
      5'h2:
        {shifted_small_float,small_extension} = {small_float,8'd0};
      5'h3:
        {shifted_small_float,small_extension} = {small_float,7'd0};
      5'h4:
        {shifted_small_float,small_extension} = {small_float,6'd0};
      5'h5:
        {shifted_small_float,small_extension} = {small_float,5'd0};
      5'h6:
        {shifted_small_float,small_extension} = {small_float,4'd0};
      5'h7:
        {shifted_small_float,small_extension} = {small_float,3'd0};
      5'h8:
        {shifted_small_float,small_extension} = {small_float,2'd0};
      5'h9:
        {shifted_small_float,small_extension} = {small_float,1'd0};
      5'ha:
        {shifted_small_float,small_extension} = small_float;
      5'hb:
        {shifted_small_float,small_extension} = small_float[10:1];
      5'hc:
        {shifted_small_float,small_extension} = small_float[10:2];
      5'hd:
        {shifted_small_float,small_extension} = small_float[10:3];
      5'he:
        {shifted_small_float,small_extension} = small_float[10:4];
      5'hf:
        {shifted_small_float,small_extension} = small_float[10:5];
      5'h10:
        {shifted_small_float,small_extension} = small_float[10:5];
      5'h11:
        {shifted_small_float,small_extension} = small_float[10:6];
      5'h12:
        {shifted_small_float,small_extension} = small_float[10:7];
      5'h13:
        {shifted_small_float,small_extension} = small_float[10:8];
      5'h14:
        {shifted_small_float,small_extension} = small_float[10:9];
      5'h15:
        {shifted_small_float,small_extension} = small_float[10];
      5'h16:
        {shifted_small_float,small_extension} = 0;
    endcase
  end

  always@* //if signs are diffrent take 2s compliment of small number
  begin
    if(sameSign)
    begin
      sign_small_float = shifted_small_float;
    end
    else
    begin
      sign_small_float = ~shifted_small_float + 11'b1;
    end
  end

  always@* //determine big number
  begin
    if(B[14:10] > A[14:10])
    begin
      bigNum = B;
      smallNum = A;
    end
    else if(B[14:10] == A[14:10])
    begin
      if(B[9:0] > A[9:0])
      begin
        bigNum = B;
        smallNum = A;
      end
      else
      begin
        bigNum = A;
        smallNum = B;
      end
    end
    else
    begin
      bigNum = A;
      smallNum = B;
    end
  end
endmodule

//float multi multiplier floating point numbers.
module ieee_16mul(A, B, result);
  //Operands
  input [15:0] A, B;
  output [15:0] result;
  //Decode numbers
  wire sign1, sign2, signR; //hold signs
  wire [4:0] ex1, ex2, exR; //hold exponents
  wire [4:0] ex1_pre, ex2_pre, exR_calc; //hold exponents
  reg [4:0] exSubCor;
  wire [4:0] exSum_fault;
  wire ex_cannot_correct;
  wire [9:0] fra1, fra2, fraR; //hold fractions
  reg [9:0] fraSub, fraSub_corrected;
  wire [20:0] float1;
  wire [10:0] float2;
  wire exSum_sign;
  wire [6:0] exSum;
  wire [5:0] exSum_prebais, exSum_abs; //exponent sum
  wire [11:0] float_res, float_res_preround; //result
  wire [9:0] float_res_fra;
  wire [9:0] dump_res; //Lost precision
  reg [21:0] res_full;
  wire [21:0] res_full_preshift;
  reg [20:0] mid[10:0];
  wire inf_num; //at least on of the operands is inf.
  wire subNormal;
  wire zero_num_in, zero_calculated;

  //Partial flags
  assign zero_num_in = ~(|A[14:0] & |B[14:0]);
  assign zero_calculated = (subNormal & (fraSub == 10'd0)) | (exSum_sign & (~|res_full[20:11]));
  assign ex_cannot_correct = {1'b0,exSubCor} > exSum_abs; //?: or >=

  //Flags
  assign zero = zero_num_in | zero_calculated;
  assign NaN = (&A[14:10] & |A[9:0]) | (&B[14:10] & |B[9:0]);
  assign inf_num = (&A[14:10] & ~|A[9:0]) | (&B[14:10] & ~|B[9:0]); //check for infinate number
  assign overflow = inf_num | (~exSum[6] & exSum[5]);
  assign subNormal = ~|float_res[11:10];
  assign precisionLost = |dump_res | (exSum_prebais < 6'd15);

  //decode-encode numbers
  assign {sign1, ex1_pre, fra1} = A;
  assign {sign2, ex2_pre, fra2} = B;
  assign ex1 = ex1_pre + {4'd0, ~|ex1_pre};
  assign ex2 = ex2_pre + {4'd0, ~|ex2_pre};
  assign result = {signR, exR, fraR};

  //exponentials are added
  assign exSum = exSum_prebais - 7'd15;
  assign exSum_prebais = {1'b0,ex1} + {1'b0,ex2};
  assign exSum_abs = (exSum_sign) ? (~exSum[5:0] + 6'd1) : exSum[5:0];
  assign exSum_sign = exSum[6];

  //Get floating numbers
  assign float1 = {|ex1_pre, fra1, 10'd0};
  assign float2 = {|ex2_pre, fra2};

  //Calculate result
  assign signR = (sign1 ^ sign2);
  assign exR_calc = exSum[4:0] + {4'd0, float_res[11]} + (~exSubCor & {5{subNormal}}) + {4'd0, subNormal};
  assign exR = (exR_calc | {5{overflow}}) & {5{~(zero | exSum_sign | ex_cannot_correct)}};
  assign fraR = ((exSum_sign) ? res_full[20:11] :((subNormal) ? fraSub_corrected : float_res_fra)) & {10{~(zero | overflow)}} ;
  assign float_res_fra = (float_res[11]) ? float_res[10:1] : float_res[9:0];
  assign float_res = float_res_preround + {10'd0,dump_res[9]}; //? possibly generates wrong result due to overflow
  assign {float_res_preround, dump_res} = res_full_preshift;
  assign res_full_preshift = mid[0] + mid[1] + mid[2] + mid[3] + mid[4] + mid[5] + mid[6] + mid[7] + mid[8] + mid[9] + mid[10];
  assign exSum_fault = exSubCor - exSum_abs[4:0];
  always@*
  begin
    if(exSum_sign)
    case(exSum_abs)
      6'h0:
        res_full = res_full_preshift;
      6'h1:
        res_full = (res_full_preshift >> 1);
      6'h2:
        res_full = (res_full_preshift >> 2);
      6'h3:
        res_full = (res_full_preshift >> 3);
      6'h4:
        res_full = (res_full_preshift >> 4);
      6'h5:
        res_full = (res_full_preshift >> 5);
      6'h6:
        res_full = (res_full_preshift >> 6);
      6'h7:
        res_full = (res_full_preshift >> 7);
      6'h8:
        res_full = (res_full_preshift >> 8);
      6'h9:
        res_full = (res_full_preshift >> 9);
      6'ha:
        res_full = (res_full_preshift >> 10);
      6'hb:
        res_full = (res_full_preshift >> 11);
      6'hc:
        res_full = (res_full_preshift >> 12);
      6'hd:
        res_full = (res_full_preshift >> 13);
      6'he:
        res_full = (res_full_preshift >> 14);
      6'hf:
        res_full = (res_full_preshift >> 15);
      default:
        res_full = (res_full_preshift >> 16);
    endcase
    else
      res_full = res_full_preshift;
  end
  always@*
  begin
    if(ex_cannot_correct)
    case(exSum_fault)
      5'h0:
        fraSub_corrected = fraSub;
      5'h1:
        fraSub_corrected = (fraSub >> 1);
      5'h2:
        fraSub_corrected = (fraSub >> 2);
      5'h3:
        fraSub_corrected = (fraSub >> 3);
      5'h4:
        fraSub_corrected = (fraSub >> 4);
      5'h5:
        fraSub_corrected = (fraSub >> 5);
      5'h6:
        fraSub_corrected = (fraSub >> 6);
      5'h7:
        fraSub_corrected = (fraSub >> 7);
      5'h8:
        fraSub_corrected = (fraSub >> 8);
      5'h9:
        fraSub_corrected = (fraSub >> 9);
      default:
        fraSub_corrected = 10'h0;
    endcase
    else
      fraSub_corrected = fraSub;
  end

  always@* //create mids from fractions
  begin
    mid[0] = (float1 >> 10) & {21{float2[0]}};
    mid[1] = (float1 >> 9)  & {21{float2[1]}};
    mid[2] = (float1 >> 8)  & {21{float2[2]}};
    mid[3] = (float1 >> 7)  & {21{float2[3]}};
    mid[4] = (float1 >> 6)  & {21{float2[4]}};
    mid[5] = (float1 >> 5)  & {21{float2[5]}};
    mid[6] = (float1 >> 4)  & {21{float2[6]}};
    mid[7] = (float1 >> 3)  & {21{float2[7]}};
    mid[8] = (float1 >> 2)  & {21{float2[8]}};
    mid[9] = (float1 >> 1)  & {21{float2[9]}};
    mid[10] = float1        & {21{float2[10]}};
  end
  //Corrections for subnormal normal op
  always@*
  begin
    casex(res_full)
      22'b001xxxxxxxxxxxxxxxxxxx:
      begin
        fraSub = res_full[18:9];
      end
      22'b0001xxxxxxxxxxxxxxxxxx:
      begin
        fraSub = res_full[17:8];
      end
      22'b00001xxxxxxxxxxxxxxxxx:
      begin
        fraSub = res_full[16:7];
      end
      22'b000001xxxxxxxxxxxxxxxx:
      begin
        fraSub = res_full[15:6];
      end
      22'b0000001xxxxxxxxxxxxxxx:
      begin
        fraSub = res_full[14:5];
      end
      22'b00000001xxxxxxxxxxxxxx:
      begin
        fraSub = res_full[13:4];
      end
      22'b000000001xxxxxxxxxxxxx:
      begin
        fraSub = res_full[12:3];
      end
      22'b0000000001xxxxxxxxxxxx:
      begin
        fraSub = res_full[11:2];
      end
      22'b00000000001xxxxxxxxxxx:
      begin
        fraSub = res_full[10:1];
      end
      22'b000000000001xxxxxxxxxx:
      begin
        fraSub = res_full[9:0];
      end
      22'b0000000000001xxxxxxxxx:
      begin
        fraSub = {res_full[8:0], 1'd0};
      end
      22'b00000000000001xxxxxxxx:
      begin
        fraSub = {res_full[7:0], 2'd0};
      end
      22'b000000000000001xxxxxxx:
      begin
        fraSub = {res_full[6:0], 3'd0};
      end
      22'b0000000000000001xxxxxx:
      begin
        fraSub = {res_full[5:0], 4'd0};
      end
      22'b00000000000000001xxxxx:
      begin
        fraSub = {res_full[4:0], 5'd0};
      end
      22'b000000000000000001xxxx:
      begin
        fraSub = {res_full[3:0], 6'd0};
      end
      22'b0000000000000000001xxx:
      begin
        fraSub = {res_full[2:0], 7'd0};
      end
      22'b00000000000000000001xx:
      begin
        fraSub = {res_full[1:0], 8'd0};
      end
      22'b000000000000000000001x:
      begin
        fraSub = {res_full[0], 9'd0};
      end
      default:
      begin
        fraSub = 10'd0;
      end
    endcase
  end
  always@*
  begin
    casex(res_full)
      22'b001xxxxxxxxxxxxxxxxxxx:
      begin
        exSubCor = 5'd1;
      end
      22'b0001xxxxxxxxxxxxxxxxxx:
      begin
        exSubCor = 5'd2;
      end
      22'b00001xxxxxxxxxxxxxxxxx:
      begin
        exSubCor = 5'd3;
      end
      22'b000001xxxxxxxxxxxxxxxx:
      begin
        exSubCor = 5'd4;
      end
      22'b0000001xxxxxxxxxxxxxxx:
      begin
        exSubCor = 5'd5;
      end
      22'b00000001xxxxxxxxxxxxxx:
      begin
        exSubCor = 5'd6;
      end
      22'b000000001xxxxxxxxxxxxx:
      begin
        exSubCor = 5'd7;
      end
      22'b0000000001xxxxxxxxxxxx:
      begin
        exSubCor = 5'd8;
      end
      22'b00000000001xxxxxxxxxxx:
      begin
        exSubCor = 5'd9;
      end
      22'b000000000001xxxxxxxxxx:
      begin
        exSubCor = 5'd10;
      end
      22'b0000000000001xxxxxxxxx:
      begin
        exSubCor = 5'd11;
      end
      22'b00000000000001xxxxxxxx:
      begin
        exSubCor = 5'd12;
      end
      22'b000000000000001xxxxxxx:
      begin
        exSubCor = 5'd13;
      end
      22'b0000000000000001xxxxxx:
      begin
        exSubCor = 5'd14;
      end
      22'b00000000000000001xxxxx:
      begin
        exSubCor = 5'd15;
      end
      22'b000000000000000001xxxx:
      begin
        exSubCor = 5'd16;
      end
      22'b0000000000000000001xxx:
      begin
        exSubCor = 5'd17;
      end
      22'b00000000000000000001xx:
      begin
        exSubCor = 5'd18;
      end
      22'b000000000000000000001x:
      begin
        exSubCor = 5'd19;
      end
      default:
      begin
        exSubCor = 5'd0;
      end
    endcase
  end
endmodule
