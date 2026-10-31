# The tests compare the implementation against boost::math::quaternion
# (header-only) and run under Boost.Test, which needs the compiled
# unit_test_framework library.
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_executable(quaternion_tests
    test_main.cpp
    test_algebra.cpp
    test_rotation.cpp
    test_approx.cpp
)
target_link_libraries(quaternion_tests PRIVATE Quaternion Boost::unit_test_framework)
target_compile_definitions(quaternion_tests PRIVATE BOOST_TEST_DYN_LINK)
target_compile_options(quaternion_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra>
)

add_test(NAME quaternion_tests COMMAND quaternion_tests --log_level=test_suite --report_level=short)
