//
// Created by zhuoc on 2021/11/1.
//

#ifndef CUDA_COURSE_BVH_H
#define CUDA_COURSE_BVH_H

#include "tri_contact.h"
#include "myvector.h"

#include <set>
#include <utility>
#include <cuda.h>

class Triangle;
class MyObj;

__host__ __device__
bool boxContact(const vec3f &a1, const vec3f &b1, const vec3f &a2, const vec3f &b2);

class BVHDenseNode {
public:
    vec3f a, b;
    int triIdx = -1;
    int l = -1;
    int r = -1;

    __host__ __device__
    bool isLeaf() const {
        return triIdx > -1;
    }

    __host__ __device__
    bool contact(BVHDenseNode *node) const;
    int contact(BVHDenseNode *bvh, int idx, const MyObj *obj, std::set<std::pair<int, int>> &pairs);
    __host__ __device__
    int contact(BVHDenseNode *bvh, int idx, vec3f *d_vs, Triangle *d_fs, MyVector *d_res);
    __host__ __device__
    int contact_stack(BVHDenseNode *bvh, int height, vec3f *d_vs, Triangle *d_fs, MyVector *d_res) const;
};

class BVHNode {
private:
    vec3f a, b, ctr;
    bool isDensed = false;
    int nodeCnt = 1;

    static int box_x_cmp(const void *a, const void *b);
    static int box_y_cmp(const void *a, const void *b);
    static int box_z_cmp(const void *a, const void *b);

    void _dense(BVHDenseNode *bvh, int cnt);
    void setBox(const vec3f &a, const vec3f &b) {
        this->a = a;
        this->b = b;
        ctr.x = (a.x + b.x) / 2;
        ctr.y = (a.y + b.y) / 2;
        ctr.z = (a.z + b.z) / 2;
    }
    friend class MyObj;

public:
    int triIdx = -1;
    int height = 0;
    int depth = 0;
    int idx = -1;
    BVHNode *l = nullptr;
    BVHNode *r = nullptr;

    BVHNode() = default;

    ~BVHNode() {
        delete l;
        delete r;
    }

    bool isLeaf() const {
        return triIdx > -1;
    }

    void setTriangle(int tidx, const vec3f &a, const vec3f &b, const vec3f &c) {
        triIdx = tidx;
        vec3f aa(myfmin(a.x, b.x, c.x), myfmin(a.y, b.y, c.y), myfmin(a.z, b.z, c.z));
        vec3f bb(myfmax(a.x, b.x, c.x), myfmax(a.y, b.y, c.y), myfmax(a.z, b.z, c.z));
        this->setBox(aa, bb);
    }

    static BVHNode *build(BVHNode **nodes, int n, int depth=0, int idx=0);
    bool contact(BVHNode *node) const;
    int contact(BVHNode *node, const MyObj *obj, std::set<std::pair<int, int>> &pairs);

    int count();
    BVHDenseNode *dense();
    static int box_idx_cmp(const void *a, const void *b);
};

#endif //CUDA_COURSE_BVH_H
