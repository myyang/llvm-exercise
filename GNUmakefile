
# ----
# vars
# ----

CXX				?= clang++
LLVM_CONFIG		?= llvm-config

# Automake-like silent rules
V ?= 0
AT_RUN			  = $(AT_RUN_$(V))
AT_RUN_0		  = @echo "  RUN          "$@;
AT_RUN_1		  =

LLVM_CONFIG_FLAGS :=`$(LLVM_CONFIG) --cxxflags --ldflags --system-libs --libs core`

CMDS = \
	   toy

toy:
	$(AT_RUN) $(CXX) -g kaleidoscope.cpp $(LLVM_CONFIG_FLAGS) -o toy
