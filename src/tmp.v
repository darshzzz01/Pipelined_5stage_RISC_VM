module ALU_16bit(
    input [15:0] A,               //input [31:0] A,
    input [15:0] B,               //input [31:0] B,
    input [3:0] opcode,
    output reg [15:0] result,
    output reg zero
  );

  always @(*)
  begin
    case(opcode)
      4'b0000:
        result = A + B;
      4'b0001:
        result = A - B;
      4'b0010:
        result = A * B;
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

    if (result == 16'h0)
      zero = 1;
    else
      zero = 0;
  end

endmodule
