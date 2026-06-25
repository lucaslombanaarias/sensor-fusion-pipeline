// platform.hpp — tiny cross-platform shims for the pipeline.
//
// The only thing here today is timer resolution. On Windows the default
// system timer granularity is ~15.6 ms, so any sub-millisecond
// sleep_until overshoots massively: a 200 Hz (5 ms) estimator loop or a
// 1 kHz sensor cannot come close to its target rate. Requesting the
// finest available period (1 ms on virtually all hardware) for the
// process lifetime fixes that.
//
// ScopedHighResTimer is an RAII guard: construct one at the top of
// main() and the elevated resolution stays in effect until it goes out
// of scope. On POSIX it is an empty no-op — sleep_until is already
// fine-grained there, which is why the headline Linux numbers don't
// need this at all.

#pragma once

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX   // keep std::min/std::max usable elsewhere
#  endif
#  include <windows.h>
#  include <timeapi.h>
#  pragma comment(lib, "winmm.lib")
#endif

namespace sfp {

class ScopedHighResTimer {
public:
#if defined(_WIN32)
    ScopedHighResTimer() noexcept {
        TIMECAPS tc;
        if (timeGetDevCaps(&tc, sizeof(tc)) == MMSYSERR_NOERROR) {
            period_ = (tc.wPeriodMin < 1u) ? 1u : tc.wPeriodMin;
            active_ = (timeBeginPeriod(period_) == TIMERR_NOERROR);
        }
    }
    ~ScopedHighResTimer() {
        if (active_) timeEndPeriod(period_);
    }

private:
    unsigned int period_ = 1;
    bool         active_ = false;
#else
    ScopedHighResTimer() noexcept = default;
    ~ScopedHighResTimer()         = default;
#endif

    ScopedHighResTimer(const ScopedHighResTimer&)            = delete;
    ScopedHighResTimer& operator=(const ScopedHighResTimer&) = delete;
    ScopedHighResTimer(ScopedHighResTimer&&)                 = delete;
    ScopedHighResTimer& operator=(ScopedHighResTimer&&)      = delete;
};

} // namespace sfp
