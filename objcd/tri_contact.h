#pragma once

#include <cmath>
#include <ostream>

#define GLH_ZERO double(0.0)
#define GLH_EPSILON double(10e-6)
#define GLH_EPSILON_2 double(10e-12)

inline double lerp(double a, double b, float t) {
    return a + t * (b - a);
}

inline double fmax(double a, double b) {
    return (a > b) ? a : b;
}

inline double fmin(double a, double b) {
    return (a < b) ? a : b;
}

inline double fmax(double a, double b, double c) {
    return fmax(a, fmax(b, c));
}

inline double fmin(double a, double b, double c) {
    return fmin(a, fmin(b, c));
}

inline bool feq(double a, double b, double tol = GLH_EPSILON) {
    return fabs(a - b) < tol;
}

#ifndef M_PI
    #define M_PI 3.14159f
#endif

#include <cassert>

class vec3f {
public:
    union {
        struct {
            double x, y, z;
        };
        struct {
            double v[3];
        };
    };

    vec3f() {
        x = 0; y = 0; z = 0;
    }

    vec3f(const vec3f &v) {
        x = v.x;
        y = v.y;
        z = v.z;
    }

    vec3f(const double *v) {
        x = v[0];
        y = v[1];
        z = v[2];
    }

    vec3f(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    double operator [] (int i) const { return v[i]; }
    double &operator [] (int i) { return v[i]; }

    vec3f &operator += (const vec3f &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    vec3f &operator -= (const vec3f &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    vec3f &operator *= (double t) {
        x *= t;
        y *= t;
        z *= t;
        return *this;
    }

    vec3f &operator /= (double t) {
        x /= t;
        y /= t;
        z /= t;
        return *this;
    }

    void negate() {
        x = -x;
        y = -y;
        z = -z;
    }

    vec3f operator - () const {
        return vec3f(-x, -y, -z);
    }

    vec3f operator+ (const vec3f &v) const {
        return vec3f(x + v.x, y + v.y, z + v.z);
    }

    vec3f operator- (const vec3f &v) const {
        return vec3f(x - v.x, y - v.y, z - v.z);
    }

    vec3f operator *(double t) const {
        return vec3f(x*t, y*t, z*t);
    }

    vec3f operator /(double t) const {
        return vec3f(x / t, y / t, z / t);
    }

    // cross product
    const vec3f cross(const vec3f &vec) const {
        return vec3f(y*vec.z - z * vec.y, z*vec.x - x * vec.z, x*vec.y - y * vec.x);
    }

    double dot(const vec3f &vec) const {
        return x * vec.x + y * vec.y + z * vec.z;
    }

    void normalize() {
        double sum = x * x + y * y + z * z;
        if (sum > GLH_EPSILON_2) {
            double base = double(1.0 / sqrt(sum));
            x *= base;
            y *= base;
            z *= base;
        }
    }

    double length() const {
        return double(sqrt(x * x + y * y + z * z));
    }

    vec3f getUnit() const {
        return (*this) / length();
    }

    bool isUnit() const {
        return feq(squareLength(), 1.f);
    }

    //! max(|x|,|y|,|z|)
    double infinityNorm() const {
        return fmax(fabs(x), fabs(y), fabs(z));
    }

    vec3f & set_value(const double &vx, const double &vy, const double &vz) {
        x = vx; y = vy; z = vz; return *this;
    }

    bool equal_abs(const vec3f &other) {
        return x == other.x && y == other.y && z == other.z;
    }

    double squareLength() const {
        return x * x + y * y + z * z;
    }

    static vec3f zero() {
        return vec3f(0.f, 0.f, 0.f);
    }

    //! Named constructor: retrieve vector for nth axis
    static vec3f axis(int n) {
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
    static vec3f xAxis() { return vec3f(1.f, 0.f, 0.f); }
    //! Named constructor: retrieve vector for y axis
    static vec3f yAxis() { return vec3f(0.f, 1.f, 0.f); }
    //! Named constructor: retrieve vector for z axis
    static vec3f zAxis() { return vec3f(0.f, 0.f, 1.f); }

};

inline vec3f operator * (double t, const vec3f &v) {
    return vec3f(v.x*t, v.y*t, v.z*t);
}

inline vec3f interp(const vec3f &a, const vec3f &b, double t) {
    return a * (1 - t) + b * t;
}

inline vec3f interp(const vec3f &a, const vec3f &b, const vec3f &c, double u, double v, double w) {
    return a * u + b * v + c * w;
}

inline double clamp(double f, double a, double b) {
    return fmax(a, fmin(f, b));
}

inline double vdistance(const vec3f &a, const vec3f &b) {
    return (a - b).length();
}


inline std::ostream& operator<<(std::ostream&os, const vec3f &v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
    return os;
}

inline void vmin(vec3f &a, const vec3f &b) {
    a.set_value(
        fmin(a[0], b[0]),
        fmin(a[1], b[1]),
        fmin(a[2], b[2]));
}

inline void vmax(vec3f &a, const vec3f &b) {
    a.set_value(
        fmax(a[0], b[0]),
        fmax(a[1], b[1]),
        fmax(a[2], b[2]));
}

inline vec3f lerp(const vec3f &a, const vec3f &b, float t) {
    return a + t * (b - a);
}


int project3(const vec3f &ax, const vec3f &p1, const vec3f &p2, const vec3f &p3);

int project6(vec3f &ax,
    vec3f &p1, vec3f &p2, vec3f &p3,
    vec3f &q1, vec3f &q2, vec3f &q3);

class Triangle {
public:
    union {
        struct {
            vec3f a, b, c;
        };
        struct {
            vec3f v[3];
        };
    };

    Triangle() = default;
    Triangle(const vec3f &a, const vec3f &b, const vec3f &c) {
        this->a = a;
        this->b = b;
        this->c = c;
    }
    Triangle(const vec3f *v) {
        this->v[0] = v[0];
        this->v[1] = v[1];
        this->v[2] = v[2];
    }
    Triangle(const Triangle &t) {
        this->a = t.a;
        this->b = t.b;
        this->c = t.c;
    }
    bool contact(const Triangle &t);
};
