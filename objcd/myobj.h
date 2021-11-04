#pragma once

#include "tri_contact.h"
#include "bvh.h"
#include "myvector.h"

#include <cuda.h>
#include <vector>
#include <utility>
#include <set>

class Triangle {
private:
    int v_id[3];

public:
    Triangle() = default;
    Triangle(int a, int b, int c) {
        v_id[0] = a;
        v_id[1] = b;
        v_id[2] = c;
    }
    __host__ __device__ bool hasSharedWith(const Triangle &t) const;
    __device__ __host__ int operator [] (int i) const {
        return v_id[i % 3];
    }

};

class MyObj {
private:
    std::vector<Triangle> fs;
    std::vector<vec3f> vs;
    std::vector<vec3f> vns;
    BVHNode **leaves = nullptr;
    BVHNode *bvh = nullptr;
    int *leaf_idx = nullptr;
    BVHDenseNode *h_bvh = nullptr;
    int *d_leaves = nullptr;
    BVHDenseNode *d_bvh = nullptr;
    vec3f *d_vs = nullptr;
    Triangle *d_fs = nullptr;
    std::set<std::pair<int, int>> pairs;

    void constructBVH();
    unsigned long long allocObjMem();
    void freeObjMem();

public:
    ~MyObj();
    static MyObj load(const std::string &path);
    bool triContactDetection(int i, int j) const;
    __host__ __device__
    static bool triContactDetectionCUDA(vec3f *d_vs, Triangle *d_fs, int i, int j);
    int selfContactDetection();
    int selfContactDetectionCUDA(int blockSize=128);
    int nFace();
    int nVertex();
};

__global__ void cdOnCUDA(vec3f *d_vs, Triangle *d_fs, int *d_leaves, BVHDenseNode *d_bvh, MyVector *d_vec, int *a, int nFace, int height);
