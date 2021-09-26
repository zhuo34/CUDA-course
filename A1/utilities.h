//
// Created by zhuoc on 2021/9/26.
//

#ifndef JULIA_UTILITIES_H
#define JULIA_UTILITIES_H

#include <cmath>

#define PI acos(-1.f)
#define clamp(x, a, b) (min(max((x), (a)), (b)))

struct RGB {
    float r, g, b;
};

struct HSV {
    float h, s, v;
    __host__ __device__ HSV() : h(0), s(0), v(0)  {}
    __host__ __device__ HSV(float _h, float _s, float _v) : h(_h), s(_s), v(_v)  {}
    __host__ __device__ HSV(const HSV &a) = default;

    __host__ __device__ RGB toRGB() const;
};

struct JuliaSetColor {
    RGB in, out;
    float juliaFactor = 0.1;
    float colorFactor = 20;

    __host__ __device__ JuliaSetColor(const HSV &c);
    __host__ __device__ JuliaSetColor(float h, float s, float v);
    __host__ __device__ RGB getColor(float v) const;
private:
    __host__ __device__ void set_color(const HSV &c);
    static __host__ __device__ float map_color(float v, float a, float b, float factor);
};

#endif //JULIA_UTILITIES_H
