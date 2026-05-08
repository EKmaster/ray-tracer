#ifndef TEXTURE_H
#define TEXTURE_H

#include "perlin.h"
#include "rtweekend.h"
#include "rtw_stb_image.h"

#include <cmath>

/*
Texture system for material albedo lookups.
- texture is the polymorphic base (value at uv/point).
- solid/checker/noise provide procedural color sources.
- image_texture samples loaded image data through rtw_image.
*/

class texture {
  public:
    virtual ~texture() = default;

    virtual color value(double u, double v, const point3& p) const = 0;
};


class solid_color : public texture {
  public:
    explicit solid_color(const color& albedo) : albedo(albedo) {}

    solid_color(double red, double green, double blue) : solid_color(color(red, green, blue)) {}

    color value(double u, double v, const point3& p) const override { return albedo; }

  private:
    color albedo;
};


class checker_texture : public texture {
  public:
    checker_texture(double scale, shared_ptr<texture> even, shared_ptr<texture> odd)
        : inv_scale(1.0 / scale), even(std::move(even)), odd(std::move(odd)) {}

    checker_texture(double scale, const color& c1, const color& c2)
        : checker_texture(scale, make_shared<solid_color>(c1), make_shared<solid_color>(c2)) {}

    color value(double u, double v, const point3& p) const override {
        // Integer cell parity in 3D selects even/odd sub-texture.
        auto xInteger = int(std::floor(inv_scale * p.x()));
        auto yInteger = int(std::floor(inv_scale * p.y()));
        auto zInteger = int(std::floor(inv_scale * p.z()));

        bool isEven = (xInteger + yInteger + zInteger) % 2 == 0;

        return isEven ? even->value(u, v, p) : odd->value(u, v, p);
    }

  private:
    double inv_scale;
    shared_ptr<texture> even;
    shared_ptr<texture> odd;
};


class image_texture : public texture {
  public:
    explicit image_texture(const char* filename) : image(filename) {}

    color value(double u, double v, const point3& p) const override {
        // Cyan fallback makes missing textures obvious during debugging.
        if (image.height() <= 0) return color(0, 1, 1);

        // Clamp UVs and flip V to match image top-to-bottom layout.
        u = interval(0, 1).clamp(u);
        v = 1.0 - interval(0, 1).clamp(v);

        auto i = int(u * image.width());
        auto j = int(v * image.height());
        auto pixel = image.pixel_data(i, j);

        auto color_scale = 1.0 / 255.0;
        return color(color_scale * pixel[0], color_scale * pixel[1], color_scale * pixel[2]);
    }

  private:
    rtw_image image;
};


class noise_texture : public texture {
  public:
    explicit noise_texture(double scale) : scale(scale) {}

    color value(double u, double v, const point3& p) const override {
        // Marble-like pattern from sine of position warped by Perlin turbulence.
        return color(.5, .5, .5) * (1 + std::sin(scale * p.z() + 10 * noise.turb(p, 7)));
    }

  private:
    perlin noise;
    double scale;
};


#endif
