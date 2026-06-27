# Makefile — minimal local build. CMakeLists.txt is the "real" build
# system if you'd rather use that; this Makefile is for `make && ./build/...`
# convenience.

CXX        ?= g++
CXXFLAGS    = -std=c++17 -O2 -pthread -Iinclude \
              -Wall -Wextra -Wpedantic -Wshadow \
              -Wnon-virtual-dtor -Wold-style-cast -Wcast-align \
              -Wconversion -Wsign-conversion
LDFLAGS     = -pthread

BUILD_DIR   = build

BINARY      = $(BUILD_DIR)/sfp
EKF_BINARY  = $(BUILD_DIR)/ekf_localization

TESTS       = $(BUILD_DIR)/test_ring_buffer \
              $(BUILD_DIR)/test_sensor \
              $(BUILD_DIR)/test_estimator \
              $(BUILD_DIR)/test_logger \
              $(BUILD_DIR)/test_kalman \
              $(BUILD_DIR)/test_histogram \
              $(BUILD_DIR)/test_ekf

.PHONY: all test clean

all: $(BINARY) $(EKF_BINARY) $(TESTS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BINARY): src/main.cpp include/ring_buffer.hpp include/messages.hpp include/config.hpp include/stats.hpp include/histogram.hpp include/sensor.hpp include/estimator.hpp include/kalman.hpp include/logger.hpp include/pipeline_config.hpp include/benchmark.hpp include/platform.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) src/main.cpp -o $@ $(LDFLAGS)

$(EKF_BINARY): apps/ekf_localization.cpp include/ekf.hpp include/matrix.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) apps/ekf_localization.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_ring_buffer: tests/test_ring_buffer.cpp include/ring_buffer.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_ring_buffer.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_sensor: tests/test_sensor.cpp include/ring_buffer.hpp include/messages.hpp include/config.hpp include/sensor.hpp include/platform.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_sensor.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_estimator: tests/test_estimator.cpp include/ring_buffer.hpp include/messages.hpp include/config.hpp include/sensor.hpp include/stats.hpp include/histogram.hpp include/estimator.hpp include/kalman.hpp include/platform.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_estimator.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_logger: tests/test_logger.cpp include/ring_buffer.hpp include/messages.hpp include/config.hpp include/logger.hpp tests/test_util.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_logger.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_kalman: tests/test_kalman.cpp include/kalman.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_kalman.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_histogram: tests/test_histogram.cpp include/histogram.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_histogram.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_ekf: tests/test_ekf.cpp include/ekf.hpp include/matrix.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_ekf.cpp -o $@ $(LDFLAGS)

test: $(TESTS)
	@for t in $(TESTS); do \
		echo "==> Running $$t"; \
		$$t || exit 1; \
	done

clean:
	rm -rf $(BUILD_DIR)
