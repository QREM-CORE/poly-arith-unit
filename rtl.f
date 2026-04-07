# --- Packages (Must be compiled first) ---
rtl/poly_arith_pkg.sv

# --- Modular Functional Units ---
rtl/mod_add.sv
rtl/mod_div_by_2.sv
rtl/mod_mul.sv
rtl/mod_sub.sv
rtl/mod_uni_add_sub.sv

# --- Misc Modules ---
rtl/delay_n.sv

# --- MAC Adder ---
rtl/mac_adder.sv

# --- Twiddle Factor Generation ---
rtl/tf_rom.sv
rtl/tf_addr_gen.sv

# --- Processing Elements (PEs) ---
rtl/pe0.sv
rtl/pe1.sv
rtl/pe2.sv
rtl/pe3.sv
rtl/pe_unit.sv

# --- PAU Controller ---
rtl/pau_controller.sv
rtl/pau_top.sv
