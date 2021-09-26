/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and 
 * proprietary rights in and to this software and related documentation. 
 * Any use, reproduction, disclosure, or distribution of this software 
 * and related documentation without an express license agreement from
 * NVIDIA Corporation is strictly prohibited.
 *
 * Please refer to the applicable NVIDIA end user license agreement (EULA) 
 * associated with this source code for terms and conditions that govern 
 * your use of this NVIDIA software.
 * 
 */

#include "book.h"
#include "cpu_bitmap.h"
#include "gpu_anim.h"
#include "utilities.h"

#define DIM 1000

struct cuComplex {
    float   r;
    float   i;
    __host__ __device__ cuComplex( float a, float b ) : r(a), i(b)  {}
    __device__ float magnitude2() const {
        return r * r + i * i;
    }
    __device__ cuComplex operator*(const cuComplex& a) const {
        return {r*a.r - i*a.i, i*a.r + r*a.i};
    }
    __device__ cuComplex operator+(const cuComplex& a) const {
        return {r+a.r, i+a.i};
    }
};

__device__ float julia(int x, int y, cuComplex c) {
    const float scale = 2;
    float jx = scale * (float)(x - DIM/2)/(DIM/2);
    float jy = scale * (float)(y - DIM/2)/(DIM/2);

    cuComplex a(jx, jy);

    int i;
    for (i = 0; i < 200; i++) {
        a = a * a + c;
        if (a.magnitude2() > 4)
            return (float)i / 200;
    }

    return 0;
}

__global__ void kernel(unsigned char *ptr, cuComplex c, JuliaSetColor color) {
    // map from blockIdx to pixel position
    int x = blockIdx.x;
    int y = blockIdx.y;
    int offset = x + y * gridDim.x;

    // now calculate the value at that position
    float juliaValue = julia(x, y, c);
    // calculate color for this value
    RGB rgb = color.getColor(juliaValue);

    ptr[offset*4 + 0] = rgb.r * 255;   // R
    ptr[offset*4 + 1] = rgb.g * 255;   // G
    ptr[offset*4 + 2] = rgb.b * 255;   // B
    ptr[offset*4 + 3] = 255;           // A
}

/**
 * Main cardioid of Mandelbrod Set
 * @param theta
 * @return
 */
cuComplex main_cardioid(float theta) {
    float cos_value = cos(theta);
    float sin_value = sin(theta);
    float r = (2 * (1-cos_value) * cos_value + 1) / 4;
    float i = (2 * (1-cos_value) * sin_value) / 4;
    return {r, i};
}

/**
 * Second circle of Mandelbrod Set
 * @param theta
 * @return
 */
cuComplex second_circle(float theta) {
    float cos_value = cos(theta);
    float sin_value = sin(theta);
    float r = cos_value / 4 - 1;
    float i = sin_value / 4;
    return {r, i};
}

/**
 * Callback function for glut
 * @param devPtr
 * @param datablock
 * @param tick
 */
void render(uchar4* devPtr, void *datablock, int tick) {
    dim3    grid(DIM, DIM);
    // calculate animation frame index
    float t = (float)tick/1000;
    // fetch a point from main cardioid in Mandelbrod Set
    auto c = main_cardioid(t * 2 * PI);
    JuliaSetColor color(0.3, 0.87, 0.9);
    kernel<<<grid,1>>>((unsigned char *)devPtr, c, color);
}

int main() {
    GPUAnimBitmap bitmap(DIM, DIM, "Julia Set");

    bitmap.anim_and_exit(render);
    return 0;
}

