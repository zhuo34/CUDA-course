//
// Created by zhuoc on 2021/11/2.
//

#ifndef CUDA_COURSE_MYVECTOR_H
#define CUDA_COURSE_MYVECTOR_H

#include <cuda.h>

class LinkList {
public:
    int *array = nullptr;
    LinkList *next = nullptr;
    __host__ __device__ LinkList(int size);
    __host__ __device__ ~LinkList();
};

class MyVector {
private:
    LinkList *head = nullptr;
    LinkList *tail = nullptr;
    int mCapacity = 0;
    int mSize = 0;
    int nArray = 0;
    __host__ __device__ int ui(int idx);
    __host__ __device__ int li(int idx);
    __host__ __device__ void add_array();
    __host__ __device__ int *get_array(int idx);
public:
    static const int sizePerArray = 10;
    __host__ __device__ ~MyVector();
    __host__ __device__ void push_back(int v);
    __host__ __device__ int size() const;
    __host__ __device__ int operator[] (int idx);
};


#endif //CUDA_COURSE_MYVECTOR_H
