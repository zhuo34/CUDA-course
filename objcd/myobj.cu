#include "myobj.h"
#include "book.h"
#include "cuda.h"
#include "timer.h"

#include <fstream>
#include <iostream>
#include <algorithm>

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
            obj.fs.emplace_back(f_v1, f_v2, f_v3);
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
    return tri_contact(vs[fs[i][0]-1], vs[fs[i][1]-1], vs[fs[i][2]-1],
                       vs[fs[j][0]-1], vs[fs[j][1]-1], vs[fs[j][2]-1]);
}

void MyObj::constructBVH() {
    nodes = new BVHNode*[nFace()];
    for (int i = 0; i < nFace(); i++) {
        nodes[i] = new BVHNode;
        nodes[i]->setTriangle(i, vs[fs[i][0]-1], vs[fs[i][1]-1], vs[fs[i][2]-1]);
    }
    bvh = BVHNode::build(nodes, nFace());
    qsort(nodes, nFace(), sizeof(BVHNode*), BVHNode::box_idx_cmp);
}

std::set<std::pair<int, int>> MyObj::selfContactDetection() {
    std::set<std::pair<int, int>> pairs;
    constructBVH();
    Timer t;
    t.start();
    int cnt = 0;
    for (int i = 0; i < nFace(); i++) {
        cnt += nodes[i]->contact(bvh, this, pairs);
    }
    t.stop();
    std::cout << t.now() << " " << cnt << " " << (nFace() * (nFace() - 1)) / 2 << std::endl;
    return pairs;
}

std::set<std::pair<int, int>> MyObj::selfContactDetection(int blockSize, int streamNum) {
    if (blockSize == 0)
        return selfContactDetection();
    int faceNumPerStream = nFace()/streamNum;
    int faceNumLastStream = nFace() - faceNumPerStream * (streamNum-1);
    int blockNum = (faceNumLastStream + blockSize - 1) / blockSize;
    int gridSize = blockNum * blockSize;
    std::cout << blockNum << " " << gridSize << " " << faceNumPerStream << "/" << faceNumLastStream << "/" << nFace() << std::endl;
    vec3f *d_vs = nullptr;
    Triangle *d_fs = nullptr;
    bool *d_res = nullptr;
    unsigned long long gpuMemSize = sizeof(vec3f) * vs.size() + sizeof(Triangle) * fs.size() + sizeof(bool) * gridSize * gridSize;
    bool *h_res = (bool *)std::malloc(sizeof(bool) * gridSize * gridSize);
    std::cout << "allocating " << (double)gpuMemSize / 1024 / 1024 << " MB" << std::endl;
    cudaMalloc(&d_vs, sizeof(vec3f) * vs.size());
    cudaMalloc(&d_fs, sizeof(Triangle) * fs.size());
    HANDLE_ERROR(cudaMalloc(&d_res, sizeof(bool) * gridSize * gridSize));
    std::cout << "copying data" << std::endl;
    cudaMemcpy(d_vs, vs.data(), sizeof(vec3f) * vs.size(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_fs, fs.data(), sizeof(Triangle) * fs.size(), cudaMemcpyHostToDevice);

    dim3 gSize(blockNum, blockNum);
    dim3 bSize(blockSize, blockSize);
    float totElapsedTime = 0;

    std::set<std::pair<int, int>> pairs;
    cudaEvent_t start, stop;
    for (int i = 0; i < streamNum; i++) {
        int x_faceBegin = i * faceNumPerStream;
        int x_faceNum = faceNumPerStream;
        if (i == streamNum - 1)
            x_faceNum = faceNumLastStream;
        for (int j = i; j < streamNum; j++) {
            int y_faceBegin = j * faceNumPerStream;
            int y_faceNum = faceNumPerStream;
            if (j == streamNum - 1)
                y_faceNum = faceNumLastStream;
            std::cout << "stream (" << i << ", " << j << "): x "
                << x_faceBegin << "/" << x_faceNum << " y "
                << y_faceBegin << "/" << y_faceNum << " all "
                << nFace() << std::endl;
            cudaEventCreate(&start);
            cudaEventCreate(&stop);
            cudaEventRecord(start, nullptr);
            cdOnCUDA<<<gSize, bSize>>>(d_vs, d_fs, d_res, x_faceBegin, x_faceNum, y_faceBegin, y_faceNum, i == j);
            HANDLE_ERROR(cudaGetLastError());
            cudaEventRecord(stop, nullptr);
            cudaEventSynchronize(stop);
            float elapsedTime;
            cudaEventElapsedTime(&elapsedTime, start, stop);
            totElapsedTime += elapsedTime;
            cudaEventDestroy(start);
            cudaEventDestroy(stop);
            printf(" stream time to detect:  %.6f s\n", totElapsedTime/1000);

            HANDLE_ERROR(cudaMemcpy(h_res, d_res, sizeof(bool) * gridSize * gridSize, cudaMemcpyDeviceToHost));
            for (int k = 0; k < gridSize * gridSize; k++) {
                if (h_res[k]) {
                    int y = k / gridSize;
                    int x = k % gridSize;
                    pairs.emplace(x + x_faceBegin, y + y_faceBegin);
                }
            }
        }
    }
    printf( "Time to detect:  %3.1f ms\n", totElapsedTime );

    cudaFree(d_vs);
    cudaFree(d_fs);
    cudaFree(d_res);
    std::free(h_res);
    return pairs;
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

__global__ void cdOnCUDA(vec3f *vs, Triangle *fs, bool *res, int x_faceBegin, int x_faceNum, int y_faceBegin, int y_faceNum, bool isSelf) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int offset = y * (gridDim.x * blockDim.x) + x;
    res[offset] = false;
    if (x >= x_faceNum || y >= y_faceNum)
        return;
    if (isSelf && x >= y)
        return;
    int fid1 = x + x_faceBegin;
    int fid2 = y + y_faceBegin;
    if (fs[fid1].hasSharedWith(fs[fid2]))
        return;
    res[offset] = tri_contact(vs[fs[fid1][0]-1], vs[fs[fid1][1]-1], vs[fs[fid1][2]-1],
                              vs[fs[fid2][0]-1], vs[fs[fid2][1]-1], vs[fs[fid2][2]-1]);
}
