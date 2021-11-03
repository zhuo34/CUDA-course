#include "myobj.h"
#include "book.h"
#include "timer.h"
#include "myvector.h"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <cuda.h>

MyObj MyObj::load(const std::string &path) {
    MyObj obj;
    std::ifstream f;
    f.open(path, std::ios::in);
    std::string s;
    float x, y, z;
    int f_v1, f_v2, f_v3;
    int f_vn1, f_vn2, f_vn3;
    while (f >> s) {
        if (s == "vt") {
            f >> x >> y;
        } else if (s == "v") {
            f >> x >> y >> z;
            obj.vs.emplace_back(x, y, z);
        } else if (s == "ny") {
            f >> x >> y >> z;
            obj.vns.emplace_back(x, y, z);
        } else if (s == "f") {
            f >> f_v1; f.ignore(); f >> f_vn1;
            f >> f_v2; f.ignore(); f >> f_vn2;
            f >> f_v3; f.ignore(); f >> f_vn3;
            obj.fs.emplace_back(f_v1 - 1, f_v2 - 1, f_v3 - 1);
        } else if (s == "td") {
            f >> x;
        }
    }
    return obj;
}

int MyObj::nFace() {
    return (int)fs.size();
}

int MyObj::nVertex() {
    return (int)vs.size();
}

bool MyObj::triContactDetection(int i, int j) const {
    if (fs[i].hasSharedWith(fs[j]))
        return false;
    return tri_contact(vs[fs[i][0]], vs[fs[i][1]], vs[fs[i][2]],
                       vs[fs[j][0]], vs[fs[j][1]], vs[fs[j][2]]);
}

void MyObj::constructBVH() {
    leaves = new BVHNode*[nFace()];
    for (int i = 0; i < nFace(); i++) {
        leaves[i] = new BVHNode;
        leaves[i]->setTriangle(i, vs[fs[i][0]], vs[fs[i][1]], vs[fs[i][2]]);
    }
    bvh = BVHNode::build(leaves, nFace());
}

std::set<std::pair<int, int>> MyObj::selfContactDetection() {
    std::set<std::pair<int, int>> pairs;
    constructBVH();
    int cnt = 0;
    Timer t;
    t.start();
    for (int i = 0; i < nFace(); i++) {
        cnt += leaves[i]->contact(bvh, this, pairs);
    }
    t.stop();
    std::cout << t.now() << " " << cnt << " " << (nFace() * (nFace() - 1)) / 2 << std::endl;
    return pairs;
}

unsigned long long MyObj::allocObjMem() {
    unsigned long long gpuObjMemSize = sizeof(vec3f) * vs.size() + sizeof(Triangle) * fs.size();
    std::cout << "allocating for object, " << (double)gpuObjMemSize / 1024 / 1024 << " MB" << std::endl;
    cudaMalloc(&d_vs, sizeof(vec3f) * nVertex());
    cudaMalloc(&d_fs, sizeof(Triangle) * nFace());
    cudaMemcpy(d_vs, vs.data(), sizeof(vec3f) * nVertex(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_fs, fs.data(), sizeof(Triangle) * nFace(), cudaMemcpyHostToDevice);
    return gpuObjMemSize;
}

void MyObj::freeObjMem() {
    cudaFree(d_vs);
    cudaFree(d_fs);
    d_vs = nullptr;
    d_fs = nullptr;
}

__host__ __device__
bool MyObj::triContactDetectionCUDA(vec3f *d_vs, Triangle *d_fs, int i, int j) {
    if (d_fs[i].hasSharedWith(d_fs[j]))
        return false;
    return tri_contact(d_vs[d_fs[i][0]], d_vs[d_fs[i][1]], d_vs[d_fs[i][2]],
                       d_vs[d_fs[j][0]], d_vs[d_fs[j][1]], d_vs[d_fs[j][2]]);
}

int MyObj::selfContactDetectionCUDA(int blockSize) {
//    std::set<std::pair<int, int>> pairs;
    Timer t;
    t.start();
    std::cout << "Constructing BVH";
    constructBVH();
    h_bvh = bvh->dense();
    double e = t.end();
    printf(", used %.3f s\n", e);

    auto gpuBVHMemSize = sizeof(BVHDenseNode) * bvh->count();
    std::cout << "allocating for BVH, " << (double)gpuBVHMemSize / 1024 / 1024 << " MB" << std::endl;
    HANDLE_ERROR(cudaMalloc(&d_bvh, gpuBVHMemSize));
    HANDLE_ERROR(cudaMemcpy(d_bvh, h_bvh, gpuBVHMemSize, cudaMemcpyHostToDevice));
    std::cout << "copied BVH to GPU" << std::endl;

    auto gpuLeafMemSize = sizeof(int) * nFace();
    leaf_idx = new int[nFace()];
    for (int i = 0; i < nFace(); i++) {
        leaf_idx[i] = leaves[i]->idx;
    }
    std::cout << "allocating memory for leaves, " << (double)gpuLeafMemSize / 1024 / 1024 << " MB" << std::endl;
    HANDLE_ERROR(cudaMalloc(&d_leaves, gpuLeafMemSize));
    HANDLE_ERROR(cudaMemcpy(d_leaves, leaf_idx, gpuLeafMemSize, cudaMemcpyHostToDevice));

    auto gpuObjMemSize = allocObjMem();

    MyVector *h_vec = new MyVector[nFace()];
    MyVector *d_vec;
    auto gpuResMemSize = sizeof(MyVector) * nFace();
    std::cout << "allocating for results, " << (double)gpuResMemSize / 1024 / 1024 << " MB" << std::endl;
    cudaMalloc(&d_vec, gpuResMemSize);
    cudaMemcpy(d_vec, h_vec, gpuResMemSize, cudaMemcpyHostToDevice);

    auto gpuAllMemSize = gpuLeafMemSize + gpuBVHMemSize + gpuObjMemSize + gpuResMemSize;
    std::cout << "Totally allocated " << (double)gpuAllMemSize / 1024 / 1024 << " MB" << std::endl;

    int *h_a = new int[nFace()];
    int *d_a;
    memset(h_a, 0, sizeof(int) * nFace());
    cudaMalloc(&d_a, sizeof(int) * nFace());
    cudaMemcpy(d_a, h_a, sizeof(int) * nFace(), cudaMemcpyHostToDevice);

    cudaEvent_t start, stop;
    HANDLE_ERROR(cudaEventCreate(&start));
    HANDLE_ERROR(cudaEventCreate(&stop));
    HANDLE_ERROR(cudaEventRecord(start, nullptr));
    int blockNum = (nFace() + blockSize - 1) / blockSize;
    printf("Start detecting ...\n");
    cdOnCUDA<<<blockNum, blockSize>>>(d_vs, d_fs, d_leaves, d_bvh, d_vec, d_a, nFace(), bvh->height);
    HANDLE_ERROR(cudaGetLastError());
    HANDLE_ERROR(cudaEventRecord(stop, nullptr));
    HANDLE_ERROR(cudaEventSynchronize(stop));
    float elapsedTime;
    HANDLE_ERROR(cudaEventElapsedTime(&elapsedTime, start, stop));
    printf("Time to detect: %3.1f ms\n", elapsedTime);
    HANDLE_ERROR(cudaEventDestroy(start));
    HANDLE_ERROR(cudaEventDestroy(stop));

    cudaMemcpy(h_vec, d_vec, gpuResMemSize, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_a, d_a, sizeof(int) * nFace(), cudaMemcpyDeviceToHost);

    int cnt = 0;
    for (int i = 0; i < nFace(); i++) {
        cnt += h_a[i];
    }

    cudaFree(d_leaves);
    cudaFree(d_vec);
    delete[] h_a;
    delete[] h_vec;
    freeObjMem();
    return cnt / 2;
}

MyObj::~MyObj() {
    free(leaves);
    delete bvh;
    if (h_bvh) {
        free(h_bvh);
        free(leaf_idx);
    }
}

__host__ __device__ bool Triangle::hasSharedWith(const Triangle &t) const {
    for (auto &v : v_id) {
        for (auto &v2 : t.v_id) {
            if (v == v2) {
                return true;
            }
        }
    }
    return false;
}

__global__ void cdOnCUDA(vec3f *d_vs, Triangle *d_fs, int *d_leaves, BVHDenseNode *d_bvh, MyVector *d_vec, int *a, int nFace, int height) {
    int offset = blockIdx.x * blockDim.x + threadIdx.x;
    if (offset < nFace) {
        a[offset] = d_bvh[d_leaves[offset]].contact_stack(d_bvh, height, d_vs, d_fs, d_vec);
    }
}
