# Polynomial Arithmetic Unit (PAU) for ML-KEM (SystemVerilog)

![Language](https://img.shields.io/badge/Language-SystemVerilog-blue)
![Standard](https://img.shields.io/badge/Standard-FIPS%20203-green)
![Simulation](https://img.shields.io/badge/Simulation-Modelsim%20%7C%20Verilator-blueviolet)
![PE-Architecture](https://img.shields.io/badge/PE-Reconfigurable_Radix--4%2F2-ff69b4)
![Target](https://img.shields.io/badge/Target-ASIC%2FFPGA-orange)
![Status](https://img.shields.io/badge/Status-WIP-yellow)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

This repository contains the SystemVerilog RTL for a highly optimized Polynomial Arithmetic Unit, designed to accelerate the core mathematical operations required by the NIST FIPS 203 (ML-KEM / Kyber) post-quantum cryptography standard.

This unit is designed for efficient FPGA and ASIC synthesis, handling all polynomial arithmetic over the Kyber ring $R_q = \mathbb{Z}_{3329}[X]/(X^{256}+1)$.

## 🏗 Architecture Overview

The datapath is heavily inspired by the "Unified Polynomial Arithmetic Module (UniPAM)" architecture proposed by Inha University. Instead of instantiating separate, resource-heavy blocks for different polynomial operations, this design aggressively reuses a unified set of Processing Elements (PEs) by dynamically reconfiguring their internal routing.

Supported hardware operations include:
* **NTT (Number Theoretic Transform):** Mixed Radix-4 / Radix-2 Decimation-in-Time.
* **INTT (Inverse NTT):** Mixed Radix-2 / Radix-4 Decimation-in-Frequency.
* **CWM (Coordinate-Wise Multiplication):** Base multiplication of polynomials in the NTT domain.
* **Point-wise Addition & Subtraction:** Standard polynomial accumulation.

## 📂 Module Hierarchy

The repository is structured into three main logical domains:

### 1. Processing Elements (PEs)
The computational muscle of the unit. The PEs are deeply pipelined to maximize clock frequency and throughput.
* `pe_unit.sv`: The top-level wrapper that dynamically routes data from the SRAMs and Twiddle ROMs into the individual PEs based on the requested operation. Includes pipeline synchronization delays.
* `pe0.sv`, `pe1.sv`, `pe2.sv`, `pe3.sv`: The individual processing elements containing the combinational multiplexing logic to morph between Radix-4, Radix-2, and standard modular arithmetic modes.

### 2. Modular Arithmetic Operations
Low-level mathematical building blocks designed strictly for 12-bit integer arithmetic modulo $q = 3329$.
* `mod_mul.sv`: Standard modular multiplier.
* `mod_add.sv` / `mod_sub.sv`: Modular adders and subtractors.
* `mod_uni_add_sub.sv`: Unified adder/subtractor.
* `mod_div_by_2.sv`: Optimized shift logic for modular division.

### 3. Twiddle Factor Management
The control plane for Number Theoretic Transforms, managing the deterministic lookup of powers of the root of unity ($\zeta$).
* `tf_rom.sv`: A highly compressed, 4-ROM memory architecture storing the forward twiddle factors, inverse twiddle factors, and $\omega_4$ constants. Utilizes a pre-negation trick for INTT to save physical subtractors in the datapath.
* `tf_addr_gen.sv`: A standalone, FSM-driven address generator that mimics the nested loop structures of the Cooley-Tukey and Gentleman-Sande schedules. It operates on a "Pass-at-a-Time" basis to allow the top-level controller to safely flush pipelines and swap memory pointers.

### 4. Utilities
* `poly_arith_pkg.sv`: Global package containing operation opcodes (`pe_mode_e`), typedefs (`coeff_t`), and hardware constants.
* `delay_n.sv`: Configurable shift-register pipeline delays used for data synchronization.

## 🚀 Current Status & Roadmap
- [x] Base modular arithmetic blocks
- [x] PE 0-3 internal routing and structural synthesis
- [x] Twiddle Factor ROM and Address Generator FSM
- [x] Cycle-accurate testbenches for NTT/INTT schedules
- [ ] Top-level AU (Arithmetic Unit) Controller integration

## 📚 References
* NIST FIPS 203: Module-Lattice-Based Key-Encapsulation Mechanism Standard.
* H. Jung, Q. D. Truong and H. Lee, *"Highly-Efficient Hardware Architecture for ML-KEM PQC Standard,"* IEEE Open Journal of Circuits and Systems, 2025.
