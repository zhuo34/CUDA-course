//
// Created by zhuoc on 2021/11/1.
//

#ifndef CUDA_COURSE_BVH_H
#define CUDA_COURSE_BVH_H

#include "tri_contact.h"

#include <set>
#include <utility>

class MyObj;

class BVHNode {
private:
    vec3f a, b, ctr;
    int triIdx = -1;

    static int box_x_cmp(const void *a, const void *b);
    static int box_y_cmp(const void *a, const void *b);
    static int box_z_cmp(const void *a, const void *b);

    friend class MyObj;

public:
    int height = 0;
    int depth = 0;
    BVHNode *l = nullptr;
    BVHNode *r = nullptr;

    BVHNode() = default;

    ~BVHNode() {
        delete l;
        delete r;
    }

    bool isLeaf() {
        return triIdx > -1;
    }

    void setBox(const vec3f &a, const vec3f &b) {
        this->a = a;
        this->b = b;
        ctr.x = (a.x + b.x) / 2;
        ctr.y = (a.y + b.y) / 2;
        ctr.z = (a.z + b.z) / 2;
    }

    void setTriangle(int idx, const vec3f &a, const vec3f &b, const vec3f &c) {
        triIdx = idx;
        vec3f aa(myfmin(a.x, b.x, c.x), myfmin(a.y, b.y, c.y), myfmin(a.z, b.z, c.z));
        vec3f bb(myfmax(a.x, b.x, c.x), myfmax(a.y, b.y, c.y), myfmax(a.z, b.z, c.z));
        this->setBox(aa, bb);
    }

    static BVHNode *build(BVHNode **nodes, int n, int depth=0);
    bool contact(BVHNode *node) const;
    int contact(BVHNode *node, const MyObj *obj, std::set<std::pair<int, int>> &pairs);
    static int box_idx_cmp(const void *a, const void *b);
};


#endif //CUDA_COURSE_BVH_H
