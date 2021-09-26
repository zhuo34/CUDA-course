//
// Created by zhuoc on 2021/9/26.
//

#include "utilities.h"


__host__ __device__ RGB HSV::toRGB() const {
    float r, g, b;

    int i = int(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch(i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }
    return {r, g, b};
}

__host__ __device__ JuliaSetColor::JuliaSetColor(const HSV &c) {
    set_color(c);
}

__host__ __device__ JuliaSetColor::JuliaSetColor(float h, float s, float v) {
    set_color(HSV(h, s, v));
}

__host__ __device__ RGB JuliaSetColor::getColor(float v) const {
    v *= juliaFactor;
    RGB color{};
    color.r = map_color(v, in.r, out.r, colorFactor);
    color.g = map_color(v, in.g, out.g, colorFactor);
    color.b = map_color(v, in.b, out.b, colorFactor);
    return color;
}

__host__ __device__ void JuliaSetColor::set_color(const HSV &c) {
    HSV hsv = c;
    hsv.h = hsv.h;
    hsv.s = hsv.v / 10 + 0.65f * hsv.s;
    hsv.v = 1 - hsv.v;
    this->out = hsv.toRGB();

    hsv.v = (1 - cos(hsv.v * PI)) / 16;
    this->in = hsv.toRGB();
}

__host__ __device__ float JuliaSetColor::map_color(float v, float a, float b, float factor) {
    float ret;
    if (abs(a - b) > 1e-8) {
        ret = ((v - a) / (b - a)) * factor;
        ret = clamp(ret, 0., 1.);
    } else {
        ret = (v > a) ? 1. : 0.;
    }
    return ret;
}
