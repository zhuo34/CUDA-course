#pragma once

#include "tri_contact.h"
#include "cuda.h"

#include <vector>
#include <utility>

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
    __host__ __device__ bool hasSharedWith(const Triangle &t);
    __device__ __host__ int operator [] (int i) const {
        return v_id[i % 3];
    }
};

class MyObj {
private:
    std::vector<Triangle> fs;
    std::vector<vec3f> vs;
    std::vector<vec3f> vns;

    bool triContactDetection(int i, int j);
public:
    static MyObj load(const std::string &path);
    std::vector<std::pair<int, int>> selfContactDetection();
    std::vector<std::pair<int, int>> selfContactDetection(int blockSize, int streamNum);
    int nFace();
    int nVertex();
};

__global__ void cdOnCUDA(vec3f *vs, Triangle *fs, bool *res, int x_faceBegin, int x_faceNum, int y_faceBegin, int y_faceNum, bool isSelf);
