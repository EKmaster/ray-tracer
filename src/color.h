#ifndef COLOR_H
#define COLOR_H

#include "interval.h"
#include "vec3.h"

#include <iostream>

/*
Color output helpers.
- color is an alias of vec3 (RGB in linear space).
- Applies gamma correction before writing.
- Converts to 8-bit PPM-safe channel values.
*/

using color = vec3;


inline double linear_to_gamma(double linear_component) {
    // Gamma 2.0 approximation (sqrt) for display output.
    if (linear_component > 0)
        return std::sqrt(linear_component);
    return 0;
}

inline void write_color(std::ostream& out, const color& pixel_color) {
    auto r = pixel_color.x();
    auto g = pixel_color.y();
    auto b = pixel_color.z();

    r = linear_to_gamma(r);
    g = linear_to_gamma(g);
    b = linear_to_gamma(b);

    // Clamp to [0, 0.999] so 256*x fits into [0,255].
    static const interval intensity(0.000, 0.999);
    int rbyte = int(256 * intensity.clamp(r));
    int gbyte = int(256 * intensity.clamp(g));
    int bbyte = int(256 * intensity.clamp(b));

    out << rbyte << ' ' << gbyte << ' ' << bbyte << '\n';
}

#endif
