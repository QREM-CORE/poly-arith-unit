# Import the central build system
include build-tools/common.mk

# =========================================================
# Test Vector Generation Logic
# =========================================================
VECTOR_SCRIPT := verif/scripts/gen_pau_test_vectors.py
VECTOR_STAMP  := verif/vectors/.generated_stamp

$(VECTOR_STAMP): $(VECTOR_SCRIPT)
	@echo "=== Generating PAU test vectors ==="
	python3 $(VECTOR_SCRIPT)
	@touch $(VECTOR_STAMP)

# Hook the vector generation into the testbench target
run_poly_arith_unit_tb: $(VECTOR_STAMP)

# Add to cleanup
EXTRA_CLEAN = $(VECTOR_STAMP) verif/vectors/
