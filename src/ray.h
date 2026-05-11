#ifndef RAY_H
#define RAY_H

#include "vec3.h"

/*
Core ray primitive used everywhere in the renderer.
- origin + direction define a half-line in 3D.
- time enables motion blur by sampling moving objects at shutter time.
- at(t) evaluates the world-space point reached after traveling t units.
*/
class ray {
  public:
    ray() {}

    // Time is carried with each ray so moving objects can be sampled at shutter time.
    ray(const point3& origin, const vec3& direction, double time)
        : orig(origin), dir(direction), tm(time) {}

    ray(const point3& origin, const vec3& direction) : ray(origin, direction, 0) {}

    const point3& origin() const { return orig; }
    const vec3& direction() const { return dir; }

    double time() const { return tm; }

    // Parametric point along ray: P(t) = origin + t * direction.
    point3 at(double t) const { return orig + t * dir; }

  private:
    point3 orig;
    vec3 dir;
    double tm = 0;
};

#endif
