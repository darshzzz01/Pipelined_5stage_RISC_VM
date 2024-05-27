.main clear
quit -sim
vlog -reportprogress 300 -work work ../src/vedic_mul.v
vsim -voptargs=+acc work.tb_vedic_multiplier_16bit

add wave -noupdate /tb_vedic_multiplier_16bit/*


run 300 ns

WaveRestoreZoom {0 ns} {80 ns}