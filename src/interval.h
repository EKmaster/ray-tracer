#ifndef INTERVAL_H
#define INTERVAL_H

#include <limits>

/*
Utility numeric range type used by ray traversal and clamping.
- Represents [min, max] for valid t-values along rays.
- Supports inclusive/exclusive membership tests.
- Also used to build/merge axis ranges in AABBs.
*/

inline double inf_d() { return std::numeric_limits<double>::infinity(); }


class interval {
  public:
    double min, max;

    // Default-constructed interval is "empty" (no x satisfies min <= x <= max).
    interval() : min(+inf_d()), max(-inf_d()) {}

    interval(double min, double max) : min(min), max(max) {}

    interval(const interval& a, const interval& b) {
        // Smallest interval that contains both input intervals.
        min = a.min <= b.min ? a.min : b.min;
        max = a.max >= b.max ? a.max : b.max;
    }

    double size() const { return max - min; }

    // Inclusive check: [min, max].
    bool contains(double x) const { return min <= x && x <= max; }

    // Strict check: (min, max). Used to avoid self-intersections at boundaries.
    bool surrounds(double x) const { return min < x && x < max; }

    double clamp(double x) const {
        if (x < min) return min;
        if (x > max) return max;
        return x;
    }

    interval expand(double delta) const {
        // Symmetric padding to grow thin intervals (useful for stable AABBs).
        auto padding = delta / 2;
        return interval(min - padding, max + padding);
    }

    static const interval empty, universe;
};

inline const interval interval::empty(interval(+inf_d(), -inf_d()));
inline const interval interval::universe(interval(-inf_d(), +inf_d()));

inline interval operator+(const interval& ival, double displacement) {
    return interval(ival.min + displacement, ival.max + displacement);
}

inline interval operator+(double displacement, const interval& ival) { return ival + displacement; }

#endif
