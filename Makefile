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

TESTS       = $(BUILD_DIR)/test_ring_buffer \
              $(BUILD_DIR)/test_sensor \
              $(BUILD_DIR)/test_estimator \
              $(BUILD_DIR)/test_logger \
              $(BUILD_DIR)/test_kalman

.PHONY: all test clean

all: $(BINARY) $(TESTS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BINARY): src/main.cpp include/ring_buffer.hpp include/messages.hpp include/config.hpp include/stats.hpp include/sensor.hpp include/estimator.hpp include/kalman.hpp include/logger.hpp include/pipeline_config.hpp include/benchmark.hpp include/platform.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) src/main.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_ring_buffer: tests/test_ring_buffer.cpp include/ring_buffer.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_ring_buffer.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_sensor: tests/test_sensor.cpp include/ring_buffer.hpp include/messages.hpp include/config.hpp include/sensor.hpp include/platform.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_sensor.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_estimator: tests/test_estimator.cpp include/ring_buffer.hpp include/messages.hpp include/config.hpp include/sensor.hpp include/stats.hpp include/estimator.hpp include/kalman.hpp include/platform.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_estimator.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_logger: tests/test_logger.cpp include/ring_buffer.hpp include/messages.hpp include/config.hpp include/logger.hpp tests/test_util.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_logger.cpp -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_kalman: tests/test_kalman.cpp include/kalman.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) tests/test_kalman.cpp -o $@ $(LDFLAGS)

test: $(TESTS)
	@for t in $(TESTS); do \
		echo "==> Running $$t"; \
		$$t || exit 1; \
	done

clean:
	rm -rf $(BUILD_DIR)
