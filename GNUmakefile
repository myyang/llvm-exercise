
# ----
# vars
# ----

CXX				?= clang++
CXXFLAGS		?=-g -std=c++17
LLVM_CONFIG		?= llvm-config

# Automake-like silent rules
V ?= 0
AT_RUN			  = $(AT_RUN_$(V))
AT_RUN_0		  = @echo "  RUN          "$@;
AT_RUN_1		  =

LLVM_CONFIG_FLAGS :=`$(LLVM_CONFIG) --cxxflags --ldflags --system-libs --libs all`

CMDS = \
	clean \
	toy \
	sample

clean:
	$(AT_RUN) rm -rf ./bin/*

toy:
	$(AT_RUN) $(CXX) $(CXXFLAGS) src/kaleidoscope.cpp $(LLVM_CONFIG_FLAGS) -o bin/toy

sample:
	$(AT_RUN) $(CXX) $(CXXFLAGS) src/sample.cpp $(LLVM_CONFIG_FLAGS) -o bin/sample
