#ifndef HITTABLE_LIST_H
#define HITTABLE_LIST_H

#include "hittable.h"

#include <vector>

/*
Simple aggregate of hittables.
- Provides linear closest-hit traversal.
- Maintains union bounding box for acceleration structures.
- Often wrapped by BVH when scene grows large.
*/

class hittable_list : public hittable {
  public:
    std::vector<shared_ptr<hittable>> objects;

    hittable_list() {}
    explicit hittable_list(shared_ptr<hittable> object) { add(std::move(object)); }

    void clear() {
        objects.clear();
        bbox = aabb::empty;
    }

    void add(shared_ptr<hittable> object) {
        objects.push_back(std::move(object));
        // Incrementally grow world bounds as objects are appended.
        bbox = aabb(bbox, objects.back()->bounding_box());
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        hit_record temp_rec;
        bool hit_anything = false;
        auto closest_so_far = ray_t.max;

        for (const auto& object : objects) {
            // Narrow search interval once a nearer hit is found.
            if (object->hit(r, interval(ray_t.min, closest_so_far), temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }

        return hit_anything;
    }

    aabb bounding_box() const override { return bbox; }

  private:
    aabb bbox;
};


#endif
