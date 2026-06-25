// test_util.hpp — small shared helpers for the test suites.

#pragma once

#include <filesystem>
#include <string>

namespace sfp {
namespace test {

// A path inside the system temp directory. Portable across Windows
// (%TEMP%) and POSIX (/tmp); replaces the previously hard-coded
// "/tmp/..." literals, which do not exist on Windows and made the
// logger tests fail there before they even ran.
inline std::string temp_path(const std::string& name) {
    namespace fs = std::filesystem;
    return (fs::temp_directory_path() / name).string();
}

} // namespace test
} // namespace sfp
