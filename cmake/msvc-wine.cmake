set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR AMD64)
unset(CMAKE_NINJA_FORCE_RESPONSE_FILE CACHE)
unset(CMAKE_NINJA_FORCE_RESPONSE_FILE)
find_program(CMAKE_CXX_COMPILER NAMES cl REQUIRED)
find_program(CMAKE_LINKER NAMES link REQUIRED)
find_program(CMAKE_AR NAMES lib REQUIRED)
find_program(CMAKE_RC_COMPILER NAMES rc REQUIRED)
find_program(CMAKE_MT NAMES mt REQUIRED)
set(CMAKE_MT "${CMAKE_CURRENT_LIST_DIR}/msvc-mt.sh")
find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
# Use a response file when the CommonLib librarian command exceeds Wine's command-line limit.
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK
    "\"${Python3_EXECUTABLE}\" \"${CMAKE_CURRENT_LIST_DIR}/msvc-link.py\"")
