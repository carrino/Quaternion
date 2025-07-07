set (Boost_USE_STATIC_LIBS OFF)
# Added serialization component for boost::math::quaternion
find_package (Boost REQUIRED COMPONENTS unit_test_framework serialization)
include_directories (${Boost_INCLUDE_DIRS})
include_directories (..)

add_executable (Boost_Tests_run test.cpp)

target_link_libraries(Boost_Tests_run Quaternion)
# Link with Boost libraries, including quaternion if it's a separate lib (depends on Boost version/build)
# Typically, Boost.Math, including Quaternion, is header-only but linking Boost::boost or specific components is good practice.
target_link_libraries(Boost_Tests_run Boost::unit_test_framework Boost::serialization)

# Add the test to CTest
add_test(NAME QuaternionTests COMMAND Boost_Tests_run)
