# Quaternion

This library was originally written to handle rotataions fusing gyro, compass and accelerometer.

This library can handle any rotations in 3 space.

One coordinate system that is particularly useful is NED (x = North, y = East, z = Down).

## Running the tests

The tests check this implementation against `boost::math::quaternion` and run
under Boost.Test, so they need Boost (`libboost-test-dev` on Debian/Ubuntu) and
CMake 3.17 or newer:

```bash
cmake -S . -B build
cmake --build build
cd build && ctest --output-on-failure
```
