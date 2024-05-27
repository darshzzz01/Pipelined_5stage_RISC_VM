.main clear
# vdel -all -lib work
quit -sim

vlog -reportprogress 300 -work work "../RISC_pipelined/src/top.v"
vsim -voptargs=+acc work.top_tb

radix define float#10#decimal -float -fraction 10 -base signed -precision 6

add wave -noupdate -radix binary /top_tb/U0/clk
add wave -noupdate -radix unsigned /top_tb/U0/addr
add wave -noupdate -radix unsigned /top_tb/U0/addr_reg
add wave -noupdate -radix unsigned /top_tb/U0/instr
add wave -noupdate -radix unsigned /top_tb/U0/instr_reg
add wave -noupdate -radix unsigned /top_tb/U0/op
add wave -noupdate /top_tb/U0/U7/out_A
add wave -noupdate /top_tb/U0/U7/out_B
add wave -noupdate /top_tb/U0/U6/t0
add wave -noupdate /top_tb/U0/out_ALU

# add wave -noupdate -radix binary /top_tb/U0/data_out


run 80 ns	  
WaveRestoreZoom {0 ns} {210 ns}
configure wave -timelineunits ns