#ifndef RTWEEKEND_H
#define RTWEEKEND_H

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <thread>

/*
Shared "prelude" header used across the ray tracer.
- Centralizes common includes, constants, and utility random helpers.
- random_double() is thread-local so multi-threaded rendering stays safe.
- Pulls in core math/types (vec3, interval, ray, color) at the bottom.
*/

using std::make_shared;
using std::shared_ptr;


const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926535897932385;


inline double degrees_to_radians(double degrees) { return degrees * pi / 180.0; }


inline double random_double() {
    // Per-thread generator avoids contention/data races during parallel sampling.
    thread_local std::mt19937 generator = [] {
        std::random_device rd;
        std::mt19937 gen(rd());
        // Nudge each thread's stream so workers do not all start identically.
        gen.discard(static_cast<unsigned>(
            std::hash<std::thread::id>{}(std::this_thread::get_id()) % 100000u));
        return gen;
    }();
    thread_local std::uniform_real_distribution<double> distribution(0.0, 1.0);
    return distribution(generator);
}

inline double random_double(double min, double max) {
    return min + (max - min) * random_double();
}

inline int random_int(int min, int max) { return int(random_double(min, max + 1)); }


#include "vec3.h"
#include "interval.h"
#include "ray.h"
#include "color.h"

#endif
