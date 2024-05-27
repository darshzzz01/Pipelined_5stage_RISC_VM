.main clear
# vdel -all -lib work
quit -sim

vlog -reportprogress 300 -work work "../RISC_pipelined/src/top_32bit.v"
vsim -voptargs=+acc work.top_32bit_tb

add wave -noupdate -radix binary /top_32bit_tb/U0/clk
add wave -noupdate -radix binary /top_32bit_tb/U0/reset
add wave -noupdate -radix float32 /top_32bit_tb/U0/U7/R0/reg_data
add wave -noupdate -radix float32 /top_32bit_tb/U0/U7/R1/reg_data
add wave -noupdate -radix float32 /top_32bit_tb/U0/out_ALU
add wave -noupdate -radix float32 /top_32bit_tb/U0/U7/R2/reg_data
add wave -noupdate -radix unsigned /top_32bit_tb/U0/op
add wave -noupdate -radix unsigned /top_32bit_tb/U0/addr

add wave -noupdate -radix unsigned -color blue /top_32bit_tb/U0/U1/*

# add wave -noupdate -radix binary /top_32bit_tb/U0/addr_reg
# add wave -noupdate -radix unsigned /top_32bit_tb/U0/instr
# add wave -noupdate -radix unsigned /top_32bit_tb/U0/instr_reg
# add wave -noupdate -radix unsigned /top_32bit_tb/U0/op
# add wave -noupdate /top_32bit_tb/U0/U7/out_A
# add wave -noupdate /top_32bit_tb/U0/U7/out_B
# add wave -noupdate /top_32bit_tb/U0/U6/t0
# add wave -noupdate /top_32bit_tb/U0/out_ALU

# add wave -noupdate -radix binary /top_tb/U0/data_out


run 80 ns	  
WaveRestoreZoom {0 ns} {210 ns}
configure wave -namecolwidth 226
configure wave -valuecolwidth 100
configure wave -timelineunits ns