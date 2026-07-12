# Import the central build system
include build-tools/common.mk

# =========================================================
# Test Vector Generation Logic
# =========================================================
VECTOR_SCRIPT := verif/scripts/gen_pau_test_vectors.py
VECTOR_STAMP  := verif/vectors/.generated_stamp

VERIF_PY_DEPS_STAMP := verif/mlkem-python/.deps_installed

$(VERIF_PY_DEPS_STAMP): verif/mlkem-python/requirements.txt
	@echo "=== Installing python dependencies ==="
	pip3 install -r verif/mlkem-python/requirements.txt --break-system-packages || pip3 install -r verif/mlkem-python/requirements.txt
	@touch $(VERIF_PY_DEPS_STAMP)

$(VECTOR_STAMP): $(VECTOR_SCRIPT) $(VERIF_PY_DEPS_STAMP)
	@echo "=== Generating PAU test vectors ==="
	python3 $(VECTOR_SCRIPT)
	@touch $(VECTOR_STAMP)

# Hook the vector generation into the testbench target
run_poly_arith_unit_tb: $(VECTOR_STAMP)

# Add to cleanup
EXTRA_CLEAN = $(VECTOR_STAMP) verif/vectors/
