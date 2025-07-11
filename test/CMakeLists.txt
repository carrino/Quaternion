set (Boost_USE_STATIC_LIBS OFF)
# Attempting to proceed without explicit serialization component for Boost.Math.Quaternion,
# as it might be header-only for the usage in tests.
find_package (Boost REQUIRED COMPONENTS unit_test_framework)
include_directories (${Boost_INCLUDE_DIRS})
include_directories (..)

add_executable (Boost_Tests_run test.cpp)

target_link_libraries(Boost_Tests_run Quaternion)
# Link with Boost libraries. If Boost.Math.Quaternion is header-only for our needs,
# linking only unit_test_framework should be sufficient.
target_link_libraries(Boost_Tests_run Boost::unit_test_framework)

# Add the test to CTest
add_test(NAME QuaternionTests COMMAND Boost_Tests_run)
