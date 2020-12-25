#ifndef HELPER_H
#define HELPER_H

#include <cmath>
#include <string>
#include <vector>

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

#endif