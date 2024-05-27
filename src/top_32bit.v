module top_32bit_tb;
  //Ports
  reg  clk = 1'b0;
  reg  we = 1'b0;
  reg  reset = 1'b0;
  reg [31:0] data;
  reg [1:0] p_sel = 2'b00;
  reg [31:0] P0 = 32'd0;
  reg [31:0] P1 = 32'd0;

  wire [31:0] P0_out;
  wire [31:0] P1_out;
  wire [31:0] data_out;

  top_32bit  U0 (
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
    U0.U7.R0.reg_data = 32'b0_1000000001000000000000000000000;
    U0.U7.R1.reg_data = 32'b0_0111111000000000000000000000000;
    U0.U7.R2.reg_data = 32'd0;
    U0.U7.R3.reg_data = 32'd0;
    U0.U7.R4.reg_data = 32'd0;
    U0.U7.R5.reg_data = 32'd0;
    U0.U7.R6.reg_data = 32'd0;
    U0.U7.R7.reg_data = 32'd0;
  end

endmodule

module top_32bit(
    input                           clk,                            // Input Clock
    input                           we,                             // RAM Write Enable
    input                           reset,                          // CHIP Reset signal
    input       [31:0]              data,                           // Clock input
    input       [1:0]               p_sel,
    input       [31:0]              P0,
    input       [31:0]              P1,

    output      [31:0]              P0_out,
    output      [31:0]              P1_out,
    output      [31:0]              data_out
  );

  wire          [4:0]               addr,addr_reg;


  wire          [31:0]              instr,instr_reg;
  wire          [3:0]               op;
  wire          [2:0]               rs,rA,rB;
  wire                              jz;
  wire          [4:0]               start,end_addr;
  wire          [31:0]              out_ALU,out_A,out_B;



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

  ALU_32bit  U6 (
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
  reg [4:0] counter = 5'd0;
  reg [4:0] R0 = 5'd15;
  reg [4:0] R1 = 5'd0;

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

module ALU_32bit(
    input [31:0] A,               //input [31:0] A,
    input [31:0] B,               //input [31:0] B,
    input [3:0] opcode,
    output reg [31:0] result
  );

  wire [31:0] w0,w1,w2;
  wire t0;
  assign t0 = opcode == 4'b0000;
  ieee_32add  U0 (
                .a(A),
                .b(B),
                .add_sub_signal(t0),
                .res(w0)
              );
  ieee_32mul  U3 (
                .a(A),
                .b(B),
                .res(w2)
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
        result = 32'd0;
    endcase
  end
endmodule

module RegBank(
    input clk,
    input reset,
    input [31:0] data_in,    //input [31:0] data_in,
    input [1:0] p_sel,
    input [2:0] sel,
    input [2:0] sel_A,
    input [2:0] sel_B,
    input [31:0] P0,
    input [31:0] P1,

    output [31:0] out_A,     //output [31:0] out_A,
    output [31:0] out_B,     //output [31:0] out_B,

    output [31:0] P0_out,   //output [31:0] P0_out,
    output [31:0] P1_out   //output [31:0] P1_out
  );


  wire [31:0]w0,w1,w2,w3,w4,w5,w6,w7,w0_p0,w1_p1;  // wire [31:0]w0,w1,w2,w3,w4,w5,w6,w7,w0_p0,w1_p1;
  wire [7:
        0]Reg_EN;

  decoder_3to8  decoder_3to8_inst (
                  .input_A(sel),
                  .y(Reg_EN)
                );

  assign w0_p0 = p_sel[0] ? data_in : P0;
  assign w1_p1 = p_sel[1] ? data_in : P1;

  register_32bit  R0 (
                    .clk(clk),
                    .reset(reset),
                    .enable(Reg_EN[0]),
                    .data_in(w0_p0),
                    .data_out(w0)
                  );

  register_32bit  R1 (
                    .clk(clk),
                    .reset(reset),
                    .enable(Reg_EN[1]),
                    .data_in(w1_p1),
                    .data_out(w1)
                  );

  register_32bit  R2 (
                    .clk(clk),
                    .reset(reset),
                    .enable(Reg_EN[2]),
                    .data_in(data_in),
                    .data_out(w2)
                  );

  register_32bit  R3 (
                    .clk(clk),
                    .reset(reset),
                    .enable(Reg_EN[3]),
                    .data_in(data_in),
                    .data_out(w3)
                  );

  register_32bit  R4 (
                    .clk(clk),
                    .reset(reset),
                    .enable(Reg_EN[4]),
                    .data_in(data_in),
                    .data_out(w4)
                  );

  register_32bit  R5 (
                    .clk(clk),
                    .reset(reset),
                    .enable(Reg_EN[5]),
                    .data_in(data_in),
                    .data_out(w5)
                  );

  register_32bit  R6 (
                    .clk(clk),
                    .reset(reset),
                    .enable(Reg_EN[6]),
                    .data_in(data_in),
                    .data_out(w6)
                  );

  register_32bit  R7 (
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
    input [31:0] input_0,
    input [31:0] input_1,
    input [31:0] input_2,
    input [31:0] input_3,
    input [31:0] input_4,
    input [31:0] input_5,
    input [31:0] input_6,
    input [31:0] input_7,
    input [2:0] select,
    output reg [31:0] output_Y
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

// Multiplication
module ieee_32mul(
    a,
    b,
    res
  );

  input [31:0] a;
  input [31:0] b;
  output [31:0] res;

  wire exception,overflow,underflow;
  wire sign,product_round,normalised,zero;
  wire [8:0] exponent,sum_exponent;
  wire [22:0] product_mantissa;
  wire [23:0] op_a,op_b;
  wire [47:0] product,product_normalised; //48 Bits


  assign sign = a[31] ^ b[31];   													// XOR of 32nd bit

  assign exception = (&a[30:23]) | (&b[30:23]);											// Execption sets to 1 when exponent of any a or b is 255
  // If exponent is 0, hidden bit is 0



  assign op_a = (|a[30:23]) ? {1'b1,a[22:0]} : {1'b0,a[22:0]};

  assign op_b = (|b[30:23]) ? {1'b1,b[22:0]} : {1'b0,b[22:0]};

  assign product = op_a * op_b;													// Product

  assign product_round = |product_normalised[22:0];  									        // Last 22 bits are ORed for rounding off purpose

  assign normalised = product[47] ? 1'b1 : 1'b0;

  assign product_normalised = normalised ? product : product << 1;								// Normalized value based on 48th bit

  assign product_mantissa = product_normalised[46:24] + {21'b0,(product_normalised[23] & product_round)};				// Mantissa

  assign zero = exception ? 1'b0 : (product_mantissa == 23'd0) ? 1'b1 : 1'b0;

  assign sum_exponent = a[30:23] + b[30:23];

  assign exponent = sum_exponent - 8'd127 + normalised;

  assign overflow = ((exponent[8] & !exponent[7]) & !zero) ;									// Overall exponent is greater than 255 then Overflow

  assign underflow = ((exponent[8] & exponent[7]) & !zero) ? 1'b1 : 1'b0; 							// Sum of exponents is less than 255 then Underflow

  assign res = exception ? 32'd0 : zero ? {sign,31'd0} : overflow ? {sign,8'hFF,23'd0} : underflow ? {sign,31'd0} : {sign,exponent[7:0],product_mantissa};


endmodule

module ieee_32add(a,b,add_sub_signal,res);

  input [31:0] a,b;
  input add_sub_signal;														// If 1 then addition otherwise subtraction
  output [31:0] res;

  wire exception;

  wire operation_add_sub_signal;
  wire enable;
  wire output_sign,perform;

  wire [31:0] op_a,op_b;
  wire [23:0] significand_a,significand_b;
  wire [7:0] exponent_diff,exp_a,exp_b;


  wire [23:0] significand_b_add_sub;
  wire [7:0] exp_b_add_sub;

  wire [24:0] significand_add;
  wire [30:0] add_sum;

  wire [23:0] significand_sub_complement;
  wire [24:0] significand_sub;
  wire [30:0] sub_diff;
  wire [24:0] subtraction_diff;
  wire [7:0] exp_sub;

  assign {enable,op_a,op_b} = (a[30:0] < b[30:0]) ? {1'b1,b,a} : {1'b0,a,b};							// For operations always op_a must not be less than b

  assign exp_a = op_a[30:23];
  assign exp_b = op_b[30:23];

  assign exception = (&op_a[30:23]) | (&op_b[30:23]);										// Exception flag sets 1 if either one of the exponent is 255.

assign output_sign = add_sub_signal ? enable ? !op_a[31] : op_a[31] : op_a[31] ;

  assign operation_add_sub_signal = add_sub_signal ? op_a[31] ^ op_b[31] : ~(op_a[31] ^ op_b[31]);				// Assign significand values according to Hidden Bit.

  assign significand_a = (|op_a[30:23]) ? {1'b1,op_a[22:0]} : {1'b0,op_a[22:0]};							// If exponent is zero,hidden bit = 0,else 1
  assign significand_b = (|op_b[30:23]) ? {1'b1,op_b[22:0]} : {1'b0,op_b[22:0]};

  assign exponent_diff = op_a[30:23] - op_b[30:23];										// Exponent difference calculation

  assign significand_b_add_sub = significand_b >> exponent_diff;

  assign exp_b_add_sub = op_b[30:23] + exponent_diff;

  assign perform = (op_a[30:23] == exp_b_add_sub);										// Checking if exponents are same

  // Add Block //
  assign significand_add = (perform & operation_add_sub_signal) ? (significand_a + significand_b_add_sub) : 25'd0;

  assign add_sum[22:0] = significand_add[24] ? significand_add[23:1] : significand_add[22:0];					// res will be most 23 bits if carry generated, else least 22 bits.

  assign add_sum[30:23] = significand_add[24] ? (1'b1 + op_a[30:23]) : op_a[30:23];						// If carry generates in sum value then exponent is added with 1 else feed as it is.

  // Sub Block //
  assign significand_sub_complement = (perform & !operation_add_sub_signal) ? ~(significand_b_add_sub) + 24'd1 : 24'd0 ;

  assign significand_sub = perform ? (significand_a + significand_sub_complement) : 25'd0;

  priority_encoder pe(significand_sub,op_a[30:23],subtraction_diff,exp_sub);

  assign sub_diff[30:23] = exp_sub;

  assign sub_diff[22:0] = subtraction_diff[22:0];


  // Output //
  assign res = exception ? 32'b0 : ((!operation_add_sub_signal) ? {output_sign,sub_diff} : {output_sign,add_sum});

endmodule

// Priority Encoder

module priority_encoder(significand,exp_a,Significand,exp_sub);


  input [24:0] significand;
  input [7:0] exp_a;
  output [24:0] Significand;
  reg [24:0] Significand;
  output [7:0] exp_sub;

  reg [4:0] shift;

  always @(significand)
  begin
    casex (significand)
      25'b1_1xxx_xxxx_xxxx_xxxx_xxxx_xxxx :
      begin
        Significand = significand;
        shift = 5'd0;
      end
      25'b1_01xx_xxxx_xxxx_xxxx_xxxx_xxxx :
      begin
        Significand = significand << 1;
        shift = 5'd1;
      end

      25'b1_001x_xxxx_xxxx_xxxx_xxxx_xxxx :
      begin
        Significand = significand << 2;
        shift = 5'd2;
      end

      25'b1_0001_xxxx_xxxx_xxxx_xxxx_xxxx :
      begin
        Significand = significand << 3;
        shift = 5'd3;
      end

      25'b1_0000_1xxx_xxxx_xxxx_xxxx_xxxx :
      begin
        Significand = significand << 4;
        shift = 5'd4;
      end

      25'b1_0000_01xx_xxxx_xxxx_xxxx_xxxx :
      begin
        Significand = significand << 5;
        shift = 5'd5;
      end

      25'b1_0000_001x_xxxx_xxxx_xxxx_xxxx :
      begin						// 24'h020000
        Significand = significand << 6;
        shift = 5'd6;
      end

      25'b1_0000_0001_xxxx_xxxx_xxxx_xxxx :
      begin						// 24'h010000
        Significand = significand << 7;
        shift = 5'd7;
      end

      25'b1_0000_0000_1xxx_xxxx_xxxx_xxxx :
      begin						// 24'h008000
        Significand = significand << 8;
        shift = 5'd8;
      end

      25'b1_0000_0000_01xx_xxxx_xxxx_xxxx :
      begin						// 24'h004000
        Significand = significand << 9;
        shift = 5'd9;
      end

      25'b1_0000_0000_001x_xxxx_xxxx_xxxx :
      begin						// 24'h002000
        Significand = significand << 10;
        shift = 5'd10;
      end

      25'b1_0000_0000_0001_xxxx_xxxx_xxxx :
      begin						// 24'h001000
        Significand = significand << 11;
        shift = 5'd11;
      end

      25'b1_0000_0000_0000_1xxx_xxxx_xxxx :
      begin						// 24'h000800
        Significand = significand << 12;
        shift = 5'd12;
      end

      25'b1_0000_0000_0000_01xx_xxxx_xxxx :
      begin						// 24'h000400
        Significand = significand << 13;
        shift = 5'd13;
      end

      25'b1_0000_0000_0000_001x_xxxx_xxxx :
      begin						// 24'h000200
        Significand = significand << 14;
        shift = 5'd14;
      end

      25'b1_0000_0000_0000_0001_xxxx_xxxx  :
      begin						// 24'h000100
        Significand = significand << 15;
        shift = 5'd15;
      end

      25'b1_0000_0000_0000_0000_1xxx_xxxx :
      begin						// 24'h000080
        Significand = significand << 16;
        shift = 5'd16;
      end

      25'b1_0000_0000_0000_0000_01xx_xxxx :
      begin						// 24'h000040
        Significand = significand << 17;
        shift = 5'd17;
      end

      25'b1_0000_0000_0000_0000_001x_xxxx :
      begin						// 24'h000020
        Significand = significand << 18;
        shift = 5'd18;
      end

      25'b1_0000_0000_0000_0000_0001_xxxx :
      begin						// 24'h000010
        Significand = significand << 19;
        shift = 5'd19;
      end

      25'b1_0000_0000_0000_0000_0000_1xxx :
      begin						// 24'h000008
        Significand = significand << 20;
        shift = 5'd20;
      end

      25'b1_0000_0000_0000_0000_0000_01xx :
      begin						// 24'h000004
        Significand = significand << 21;
        shift = 5'd21;
      end

      25'b1_0000_0000_0000_0000_0000_001x :
      begin						// 24'h000002
        Significand = significand << 22;
        shift = 5'd22;
      end

      25'b1_0000_0000_0000_0000_0000_0001 :
      begin						// 24'h000001
        Significand = significand << 23;
        shift = 5'd23;
      end

      25'b1_0000_0000_0000_0000_0000_0000 :
      begin						// 24'h000000
        Significand = significand << 24;
        shift = 5'd24;
      end
      default :
      begin
        Significand = (~significand) + 1'b1;
        shift = 8'd0;
      end

    endcase
  end
  assign exp_sub = exp_a - shift;

endmodule
