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

module tb_vedic_multiplier_16bit;

  // Inputs
  reg [15:0] A;
  reg [15:0] B;

  // Outputs
  wire [31:0] product;

  // Instantiate the Unit Under Test (UUT)
  vedic_multiplier_16bit uut (
    .A(A), 
    .B(B), 
    .product(product)
  );

  initial begin
    // Initialize inputs
    A = 0;
    B = 0;

    // Apply test vectors
    #10; // wait for 10 time units
    A = 16'h000F; B = 16'h0001; // 15 * 15 = 225
    #10;
    A = 16'h000F; B = 16'h0003;
    #10;
    A = 16'h0077; B = 16'h000A; // arbitrary values
    #10;

    // Finish simulation
    // $stop;
  end
  
//   initial begin
//     $monitor("Time = %0d: A = %h, B = %h, product = %h", $time, A, B, product);
//   end

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
