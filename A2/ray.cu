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


#include "cuda.h"
#include "book.h"
#include "cpu_bitmap.h"
#include "gpu_anim.h"

#include <random>

#define DIM 1024

#define INF 2e10f

struct Sphere {
    float   r, b, g;
    float   radius;
    float   x, y, z;
    __host__ __device__ Sphere() {}
    __device__ Sphere(const Sphere &s1, const Sphere &s2, float p) {
        r = s1.r;
        g = s1.g;
        b = s1.b;
        x = s1.x * p + s2.x * (1-p);
        y = s1.y * p + s2.y * (1-p);
        z = s1.z * p + s2.z * (1-p);
        radius = s1.radius * p + s2.radius * (1-p);
    }
    __device__ float hit( float ox, float oy, float *n ) const {
        float dx = ox - x;
        float dy = oy - y;
        if (dx*dx + dy*dy < radius*radius) {
            float dz = sqrtf( radius*radius - dx*dx - dy*dy );
            *n = dz / sqrtf( radius * radius );
            return dz + z;
        }
        return -INF;
    }
};

#define SPHERES 500

__constant__ Sphere s_begin[SPHERES];
__constant__ Sphere s_end[SPHERES];

__global__ void kernel(unsigned char *ptr, float p) {
    // map from threadIdx/BlockIdx to pixel position
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    int offset = x + y * blockDim.x * gridDim.x;
    float ox = (x - DIM/2);
    float oy = (y - DIM/2);

    float r=0, g=0, b=0;
    float maxz = -INF;
    p *= 2;
    for (int i = 0; i < SPHERES; i++) {
        Sphere s(s_begin[i], s_end[i], p < 1 ? p : 2-p);
        float n;
        float t = s.hit(ox, oy, &n);
        if (t > maxz) {
            float fscale = n;
            r = s.r * fscale;
            g = s.g * fscale;
            b = s.b * fscale;
            maxz = t;
        }
    }

    ptr[offset*4 + 0] = (int)(r * 255);
    ptr[offset*4 + 1] = (int)(g * 255);
    ptr[offset*4 + 2] = (int)(b * 255);
    ptr[offset*4 + 3] = 255;
}

/**
 * Callback function for glut
 * @param devPtr
 * @param datablock
 * @param tick
 */
void render(uchar4* devPtr, void *datablock, int tick, float state) {
    cudaEvent_t     start, stop;
    HANDLE_ERROR( cudaEventCreate( &start ) );
    HANDLE_ERROR( cudaEventCreate( &stop ) );
    HANDLE_ERROR( cudaEventRecord( start, nullptr ) );

    dim3    grids(DIM/16,DIM/16);
    dim3    threads(16,16);
    kernel<<<grids, threads>>>((unsigned char *)devPtr, state);

    HANDLE_ERROR( cudaEventRecord( stop, nullptr ) );
    HANDLE_ERROR( cudaEventSynchronize( stop ) );
    float   elapsedTime;
    HANDLE_ERROR( cudaEventElapsedTime( &elapsedTime, start, stop ) );
    printf( "Time to generate:  %3.1f ms\n", elapsedTime );

    HANDLE_ERROR( cudaEventDestroy( start ) );
    HANDLE_ERROR( cudaEventDestroy( stop ) );
}

int main() {
    GPUAnimBitmap bitmap(DIM, DIM, "Ray");

    // allocate temp memory, initialize it, copy to constant
    // memory on the GPU, then free our temp memory
    auto temp_s = (Sphere*)malloc( sizeof(Sphere) * SPHERES );
    auto temp_s_end = (Sphere*)malloc( sizeof(Sphere) * SPHERES );
    std::default_random_engine e;
    std::uniform_real_distribution<float> u(0, 1);
    for (int i = 0; i < SPHERES; i++) {
        temp_s[i].r = u(e);
        temp_s[i].g = u(e);
        temp_s[i].b = u(e);
        temp_s[i].x = 1000.0f * u(e) - 500;
        temp_s[i].y = 1000.0f * u(e) - 500;
        temp_s[i].z = 1000.0f * u(e) - 500;
        temp_s[i].radius = 5.0f * u(e) + 10;
        temp_s_end[i].r = temp_s[i].r;
        temp_s_end[i].g = temp_s[i].g;
        temp_s_end[i].b = temp_s[i].b;
        temp_s_end[i].x = 1000.0f * u(e) - 500;
        temp_s_end[i].y = 1000.0f * u(e) - 500;
        temp_s_end[i].z = 1000.0f * u(e) - 500;
        temp_s_end[i].radius = temp_s[i].radius;
    }
    HANDLE_ERROR( cudaMemcpyToSymbol( s_begin, temp_s, sizeof(Sphere) * SPHERES) );
    HANDLE_ERROR( cudaMemcpyToSymbol( s_end, temp_s_end, sizeof(Sphere) * SPHERES) );
    free( temp_s );
    free( temp_s_end );

    // display
    bitmap.anim_and_exit(render);

    return 0;
}
