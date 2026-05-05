#ifndef CAMERA_H
#define CAMERA_H

#include "hittable.h"
#include "material.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

/*
Camera + renderer entry point.
- Builds camera basis and viewport from lookfrom/lookat/vfov.
- Traces stochastic samples per pixel (path tracing).
- Uses multithreaded scanline chunks and reports render throughput.
*/

class camera {
  public:
    double aspect_ratio = 1.0;
    int image_width = 100;
    int samples_per_pixel = 10;
    int max_depth = 10;
    color background = color(0, 0, 0);

    double vfov = 90;
    point3 lookfrom = point3(0, 0, 0);
    point3 lookat = point3(0, 0, -1);
    vec3 vup = vec3(0, 1, 0);

    double defocus_angle = 0;
    double focus_dist = 10;

    void render(const hittable& world) {
        initialize();

        // Render into a framebuffer first so worker threads can write safely,
        // then serialize to PPM once all sampling is complete.
        std::vector<color> framebuffer(static_cast<size_t>(image_width) * image_height);

        std::atomic<int> rows_done{0};
        std::mutex progress_mutex;
        const int progress_stride = std::max(1, image_height / 50);

        // Each worker traces a contiguous row range to reduce sync overhead.
        auto render_row_range = [&](int row_begin, int row_end) {
            for (int j = row_begin; j < row_end; ++j) {
                for (int i = 0; i < image_width; ++i) {
                    color pixel_color(0, 0, 0);
                    for (int sample = 0; sample < samples_per_pixel; ++sample) {
                        ray r = get_ray(i, j);
                        pixel_color += ray_color(r, max_depth, world);
                    }
                    framebuffer[static_cast<size_t>(j) * image_width + i] =
                        pixel_samples_scale * pixel_color;
                }
                int done = rows_done.fetch_add(1, std::memory_order_relaxed) + 1;
                if (done % progress_stride == 0 || done == image_height) {
                    std::lock_guard<std::mutex> lock(progress_mutex);
                    std::clog << "\rScanlines remaining: " << (image_height - done) << ' '
                              << std::flush;
                }
            }
        };

        // Use all available hardware threads for scanline parallelism.
        const unsigned nthreads = std::max(1u, std::thread::hardware_concurrency());
        std::vector<std::thread> threads;
        threads.reserve(nthreads);

        auto t0 = std::chrono::steady_clock::now();

        if (nthreads <= 1 || image_height <= 1) {
            render_row_range(0, image_height);
        } else {
            const int chunk = (image_height + static_cast<int>(nthreads) - 1) / static_cast<int>(nthreads);
            for (unsigned t = 0; t < nthreads; ++t) {
                int rb = static_cast<int>(t) * chunk;
                int re = std::min(rb + chunk, image_height);
                if (rb >= re)
                    break;
                threads.emplace_back(render_row_range, rb, re);
            }
            for (auto& th : threads)
                th.join();
        }

        auto t1 = std::chrono::steady_clock::now();
        const double elapsed_sec =
            std::chrono::duration<double>(t1 - t0).count();
        // Primary-ray throughput is a stable perf metric across same scene/settings.
        const double primary_rays =
            static_cast<double>(image_width) * static_cast<double>(image_height) *
            static_cast<double>(samples_per_pixel);
        const double megaprimary_per_sec = (elapsed_sec > 0.0) ? (primary_rays / elapsed_sec / 1e6) : 0.0;

        std::clog << "\rDone.                 \n";
        std::clog << "Render time: " << elapsed_sec << " s\n";
        std::clog << "Throughput: " << megaprimary_per_sec << " M primary rays/s"
                  << " (" << static_cast<uint64_t>(primary_rays) << " primary rays)\n";

        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = 0; j < image_height; ++j) {
            for (int i = 0; i < image_width; ++i) {
                write_color(std::cout,
                            framebuffer[static_cast<size_t>(j) * image_width + i]);
            }
        }
    }

  private:
    int image_height = 0;
    double pixel_samples_scale = 0;
    point3 center;
    point3 pixel00_loc;
    vec3 pixel_delta_u;
    vec3 pixel_delta_v;
    vec3 u, v, w;
    vec3 defocus_disk_u;
    vec3 defocus_disk_v;

    void initialize() {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        pixel_samples_scale = 1.0 / samples_per_pixel;

        center = lookfrom;

        auto theta = degrees_to_radians(vfov);
        auto h = std::tan(theta / 2);
        auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (double(image_width) / image_height);

        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        vec3 viewport_u = viewport_width * u;
        vec3 viewport_v = viewport_height * -v;

        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Pixel (0,0) starts at the upper-left corner of the camera viewport.
        auto viewport_upper_left = center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j) const {
        auto offset = sample_square();
        auto pixel_sample = pixel00_loc + ((i + offset.x()) * pixel_delta_u)
                            + ((j + offset.y()) * pixel_delta_v);

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;
        auto ray_time = random_double();

        return ray(ray_origin, ray_direction, ray_time);
    }

    vec3 sample_square() const {
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
    }

    point3 defocus_disk_sample() const {
        auto p = random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    color ray_color(const ray& r, int depth, const hittable& world) const {
        if (depth <= 0)
            return color(0, 0, 0);

        hit_record rec;

        if (!world.hit(r, interval(0.001, infinity), rec))
            return background;

        ray scattered;
        color attenuation;
        color color_from_emission = rec.mat->emitted(rec.u, rec.v, rec.p);

        if (!rec.mat->scatter(r, rec, attenuation, scattered))
            return color_from_emission;

        color color_from_scatter = attenuation * ray_color(scattered, depth - 1, world);

        return color_from_emission + color_from_scatter;
    }
};


#endif
