#pragma once

#include "cuda.h"

#include <ostream>

#define     GLH_ZERO                float(0.0)
#define     GLH_EPSILON          float(10e-6)
#define		GLH_EPSILON_2		float(10e-12)
#define     equivalent(a,b)             (((a < b + GLH_EPSILON) &&\
                                                      (a > b - GLH_EPSILON)) ? true : false)

__device__ __host__ inline float lerp(float a, float b, float t)
{
    return a + t * (b - a);
}

__device__ __host__ inline float myfmax(float a, float b) {
    return (a > b) ? a : b;
}

__device__ __host__ inline float myfmin(float a, float b) {
    return (a < b) ? a : b;
}

__device__ __host__ inline float myfabs(float a) {
    return (a < 0) ? (-a) : a;
}

__device__ __host__ inline bool isEqual(float a, float b, float tol = GLH_EPSILON)
{
    return myfabs(a - b) < tol;
}

#ifndef M_PI
#define M_PI 3.14159f
#endif

#include <cassert>

class vec3f {
public:
    union {
        struct {
            float x, y, z;
        };
        struct {
            float v[3];
        };
    };

    __device__ __host__ inline vec3f()
    {
        x = 0; y = 0; z = 0;
    }

    __device__ __host__ inline vec3f(const vec3f &v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
    }

    __device__ __host__ inline vec3f(const float *v)
    {
        x = v[0];
        y = v[1];
        z = v[2];
    }

    __device__ __host__ inline vec3f(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    __device__ __host__ inline float operator [] (int i) const { return v[i]; }
    __device__ __host__ inline float &operator [] (int i) { return v[i]; }

    __device__ __host__ inline vec3f &operator += (const vec3f &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    __device__ __host__ inline vec3f &operator -= (const vec3f &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    __device__ __host__ inline vec3f &operator *= (float t) {
        x *= t;
        y *= t;
        z *= t;
        return *this;
    }

    __device__ __host__ inline vec3f &operator /= (float t) {
        x /= t;
        y /= t;
        z /= t;
        return *this;
    }

    __device__ __host__ inline void negate() {
        x = -x;
        y = -y;
        z = -z;
    }

    __device__ __host__ inline vec3f operator - () const {
        return vec3f(-x, -y, -z);
    }

    __device__ __host__ inline vec3f oppsite () const {
        return vec3f(-x, -y, -z);
    }

    __device__ __host__ inline vec3f operator+ (const vec3f &v) const
    {
        return vec3f(x + v.x, y + v.y, z + v.z);
    }

    __device__ __host__ inline vec3f operator- (const vec3f &v) const
    {
        return vec3f(x - v.x, y - v.y, z - v.z);
    }

    __device__ __host__ inline vec3f minus (const vec3f &v) const
    {
        return vec3f(x - v.x, y - v.y, z - v.z);
    }

    __device__ __host__ inline vec3f operator *(float t) const
    {
        return vec3f(x*t, y*t, z*t);
    }

    __device__ __host__ inline vec3f operator /(float t) const
    {
        return vec3f(x / t, y / t, z / t);
    }

    // cross product
    __device__ __host__ inline const vec3f cross(const vec3f &vec) const
    {
        return vec3f(y*vec.z - z * vec.y, z*vec.x - x * vec.z, x*vec.y - y * vec.x);
    }

    __device__ __host__ inline float dot(const vec3f &vec) const {
        return x * vec.x + y * vec.y + z * vec.z;
    }

    __device__ __host__ inline void normalize()
    {
        float sum = x * x + y * y + z * z;
        if (sum > GLH_EPSILON_2) {
            float base = float(1.0 / sqrt(sum));
            x *= base;
            y *= base;
            z *= base;
        }
    }

    __device__ __host__ inline float length() const {
        return float(sqrt(x*x + y * y + z * z));
    }

    __device__ __host__ inline vec3f getUnit() const {
        return (*this) / length();
    }

    __host__ __device__ inline bool isUnit() const {
        return isEqual(squareLength(), 1.f);
    }

    //! max(|x|,|y|,|z|)
    __device__ __host__ inline float infinityNorm() const
    {
        return myfmax(myfmax(myfabs(x), myfabs(y)), myfabs(z));
    }

    __device__ __host__ inline vec3f & set_value(const float &vx, const float &vy, const float &vz)
    {
        x = vx; y = vy; z = vz; return *this;
    }

    __device__ __host__ inline bool equal_abs(const vec3f &other) {
        return x == other.x && y == other.y && z == other.z;
    }

    __device__ __host__ inline float squareLength() const {
        return x * x + y * y + z * z;
    }

    __device__ __host__ static vec3f zero() {
        return vec3f(0.f, 0.f, 0.f);
    }

    //! Named constructor: retrieve vector for nth axis
    __device__ __host__ static vec3f axis(int n) {
        assert(n < 3);
        switch (n) {
            case 0: {
                return xAxis();
            }
            case 1: {
                return yAxis();
            }
            case 2: {
                return zAxis();
            }
        }
        return vec3f();
    }

    //! Named constructor: retrieve vector for x axis
    __device__ __host__ static vec3f xAxis() { return vec3f(1.f, 0.f, 0.f); }
    //! Named constructor: retrieve vector for y axis
    __device__ __host__ static vec3f yAxis() { return vec3f(0.f, 1.f, 0.f); }
    //! Named constructor: retrieve vector for z axis
    __device__ __host__ static vec3f zAxis() { return vec3f(0.f, 0.f, 1.f); }

    __host__ __device__ void setMin(const vec3f &v) {
        x = myfmin(x, v.x);
        y = myfmin(y, v.y);
        z = myfmin(z, v.z);
    }
    __host__ __device__ void setMax(const vec3f &v) {
        x = myfmax(x, v.x);
        y = myfmax(y, v.y);
        z = myfmax(z, v.z);
    }
};

__device__ __host__ inline vec3f operator * (float t, const vec3f &v) {
    return vec3f(v.x*t, v.y*t, v.z*t);
}

__device__ __host__ inline vec3f interp(const vec3f &a, const vec3f &b, float t)
{
    return a * (1 - t) + b * t;
}

__device__ __host__ inline vec3f vinterp(const vec3f &a, const vec3f &b, float t)
{
    return a * t + b * (1 - t);
}

__device__ __host__ inline vec3f interp(const vec3f &a, const vec3f &b, const vec3f &c, float u, float v, float w)
{
    return a * u + b * v + c * w;
}

__device__ __host__ inline float clamp(float f, float a, float b)
{
    return myfmax(a, myfmin(f, b));
}

__device__ __host__ inline float vdistance(const vec3f &a, const vec3f &b)
{
    return (a - b).length();
}


__device__ __host__ inline std::ostream& operator<<(std::ostream&os, const vec3f &v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

__device__ __host__ inline void
vmin(vec3f &a, const vec3f &b)
{
    a.set_value(
            myfmin(a[0], b[0]),
            myfmin(a[1], b[1]),
            myfmin(a[2], b[2]));
}

__device__ __host__ inline void
vmax(vec3f &a, const vec3f &b)
{
    a.set_value(
            myfmax(a[0], b[0]),
            myfmax(a[1], b[1]),
            myfmax(a[2], b[2]));
}

__device__ __host__ inline vec3f lerp(const vec3f &a, const vec3f &b, float t)
{
    return a + t * (b - a);
}


__device__ __host__ inline float myfmax(float a, float b, float c)
{
    float t = a;
    if (b > t) t = b;
    if (c > t) t = c;
    return t;
}

__device__ __host__ inline float myfmin(float a, float b, float c)
{
    float t = a;
    if (b < t) t = b;
    if (c < t) t = c;
    return t;
}

__device__ __host__ inline int project3(const vec3f &ax,
                    const vec3f &p1, const vec3f &p2, const vec3f &p3)
{
    float P1 = ax.dot(p1);
    float P2 = ax.dot(p2);
    float P3 = ax.dot(p3);

    float mx1 = myfmax(P1, P2, P3);
    float mn1 = myfmin(P1, P2, P3);

    if (mn1 > 0) return 0;
    if (0 > mx1) return 0;
    return 1;
}

__device__ __host__ inline int project6(vec3f &ax,
                    vec3f &p1, vec3f &p2, vec3f &p3,
                    vec3f &q1, vec3f &q2, vec3f &q3)
{
    float P1 = ax.dot(p1);
    float P2 = ax.dot(p2);
    float P3 = ax.dot(p3);
    float Q1 = ax.dot(q1);
    float Q2 = ax.dot(q2);
    float Q3 = ax.dot(q3);

    float mx1 = myfmax(P1, P2, P3);
    float mn1 = myfmin(P1, P2, P3);
    float mx2 = myfmax(Q1, Q2, Q3);
    float mn2 = myfmin(Q1, Q2, Q3);

    if (mn1 > mx2) return 0;
    if (mn2 > mx1) return 0;
    return 1;
}

__device__ __host__ bool tri_contact(const vec3f &P1, const vec3f &P2, const vec3f &P3, const vec3f &Q1, const vec3f &Q2, const vec3f &Q3);
