PACKAGES       = stages/test stmlib/utils stages stmlib/dsp tides2/ramp

VPATH          = $(PACKAGES)

TARGET         = stages_test
PERF_TARGET    = stages_perf
BUILD_ROOT     = build/
BUILD_DIR      = $(BUILD_ROOT)$(TARGET)/
COMMON_CC	   = \
		ramp_extractor.cc \
    segment_generator.cc \
		resources.cc \
		units.cc \
		random.cc \
		quantizer.cc \
		braids_quantizer.cc
CC_FILES       = stages_test.cc $(COMMON_CC)
PERF_CC_FILES  = stages_perf.cc $(COMMON_CC)
OBJ_FILES      = $(CC_FILES:.cc=.o)
PERF_OBJ_FILES = $(PERF_CC_FILES:.cc=.o)
OBJS           = $(patsubst %,$(BUILD_DIR)%,$(OBJ_FILES)) $(STARTUP_OBJ)
DEPS           = $(OBJS:.o=.d)
DEP_FILE       = $(BUILD_DIR)depends.mk
OPEN           = $(shell type -p open || echo xdg-open)

PERF_CC_FILES  = stages_perf.cc $(COMMON_CC)
PERF_OBJ_FILES = $(PERF_CC_FILES:.cc=.o)
PERF_OBJS      = $(patsubst %,$(BUILD_DIR)%,$(PERF_OBJ_FILES)) $(STARTUP_OBJ)
PERF_DEPS      = $(PERF_OBJS:.o=.d)

all:  stages_test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)%.o: %.cc
	g++ -c -DTEST -g -Wall -Werror -msse2 -Wno-unused-variable -O2 -I. $< -o $@

$(BUILD_DIR)%.d: %.cc
	g++ -MM -DTEST -I. $< -MF $@ -MT $(@:.d=.o)

stages_test:  $(OBJS)
	g++ -g -o $(TARGET) $(OBJS) -lm -lprofiler -L/opt/local/lib

stages_perf: $(PERF_OBJS)
	g++ -g -o $(PERF_TARGET) $(PERF_OBJS) -lm -L/opt/local/lib

depends:  $(DEPS)
	cat $(DEPS) > $(DEP_FILE)

$(DEP_FILE):  $(BUILD_DIR) $(DEPS)
	cat $(DEPS) > $(DEP_FILE)

profile:	stages_test
	env CPUPROFILE_FREQUENCY=1000 CPUPROFILE=$(BUILD_DIR)/stages.prof ./stages_test && pprof --pdf ./stages_test $(BUILD_DIR)/stages.prof > profile.pdf && ${OPEN} profile.pdf

valgrind:	stages_test
	valgrind --tool=callgrind ./stages_test

perf:		stages_perf
	./stages_perf

clean:
	rm $(BUILD_DIR)*.*

include $(DEP_FILE)
