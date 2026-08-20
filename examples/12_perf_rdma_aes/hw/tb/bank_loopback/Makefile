# Encrypt-bank -> decrypt-bank loopback (cocotb + Icarus).
#
#   nix-shell -p iverilog python3Packages.cocotb python3Packages.cryptography \
#             --run "make"
#
#   make ENGINES=1 | make ENGINES=2 (default) | make ENGINES=4
#
# Note: top_aes_gcm.v is a 34 MB generated netlist with a 131k-token line.
# Icarus takes minutes to compile it; that is not a hang.

TOPLEVEL_LANG = verilog
SIM ?= icarus

RTL = ../../src/hdl
ENGINES ?= 2

VERILOG_SOURCES += $(RTL)/top_aes_gcm.v
VERILOG_SOURCES += $(RTL)/aes_gcm_stream.v
VERILOG_SOURCES += $(RTL)/aes_gcm_encryption.v
VERILOG_SOURCES += $(RTL)/aes_gcm_decryption_taglast.v
VERILOG_SOURCES += $(RTL)/axis_strip_tail_beat.v
VERILOG_SOURCES += $(RTL)/endian_swap.v
VERILOG_SOURCES += $(RTL)/axis_demux.v
VERILOG_SOURCES += $(RTL)/axis_mux.v
VERILOG_SOURCES += $(RTL)/axis_fifo_adapter.v
VERILOG_SOURCES += $(RTL)/axis_fifo.v
VERILOG_SOURCES += $(RTL)/axis_adapter.v
VERILOG_SOURCES += $(RTL)/aes_gcm_bank.v
VERILOG_SOURCES += tb_bank_loopback.v

TOPLEVEL = tb_bank_loopback
MODULE   = test_bank_loopback

COMPILE_ARGS += -Ptb_bank_loopback.NUM_AES_ENGINES=$(ENGINES)
export ENGINES

include $(shell cocotb-config --makefiles)/Makefile.sim
