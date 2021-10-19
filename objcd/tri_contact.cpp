#include "tri_contact.h"


inline int project3(const vec3f &ax, const vec3f &p1, const vec3f &p2, const vec3f &p3) {
    double P1 = ax.dot(p1);
    double P2 = ax.dot(p2);
    double P3 = ax.dot(p3);

    double mx1 = fmax(P1, P2, P3);
    double mn1 = fmin(P1, P2, P3);

    if (mn1 > 0) return 0;
    if (0 > mx1) return 0;
    return 1;
}

inline int project6(vec3f &ax,
    vec3f &p1, vec3f &p2, vec3f &p3,
    vec3f &q1, vec3f &q2, vec3f &q3) {
    double P1 = ax.dot(p1);
    double P2 = ax.dot(p2);
    double P3 = ax.dot(p3);
    double Q1 = ax.dot(q1);
    double Q2 = ax.dot(q2);
    double Q3 = ax.dot(q3);

    double mx1 = fmax(P1, P2, P3);
    double mn1 = fmin(P1, P2, P3);
    double mx2 = fmax(Q1, Q2, Q3);
    double mn2 = fmin(Q1, Q2, Q3);

    if (mn1 > mx2) return 0;
    if (mn2 > mx1) return 0;
    return 1;
}

bool
Triangle::contact(const Triangle &t) {
    vec3f p1;
    vec3f p2 = b - a;
    vec3f p3 = c - a;
    vec3f q1 = t.a - a;
    vec3f q2 = t.b - a;
    vec3f q3 = t.c - a;

    vec3f e1 = p2 - p1;
    vec3f e2 = p3 - p2;
    vec3f e3 = p1 - p3;

    vec3f f1 = q2 - q1;
    vec3f f2 = q3 - q2;
    vec3f f3 = q1 - q3;

    vec3f n1 = e1.cross(e2);
    vec3f m1 = f1.cross(f2);

    vec3f g1 = e1.cross(n1);
    vec3f g2 = e2.cross(n1);
    vec3f g3 = e3.cross(n1);

    vec3f  h1 = f1.cross(m1);
    vec3f h2 = f2.cross(m1);
    vec3f h3 = f3.cross(m1);

    vec3f ef11 = e1.cross(f1);
    vec3f ef12 = e1.cross(f2);
    vec3f ef13 = e1.cross(f3);
    vec3f ef21 = e2.cross(f1);
    vec3f ef22 = e2.cross(f2);
    vec3f ef23 = e2.cross(f3);
    vec3f ef31 = e3.cross(f1);
    vec3f ef32 = e3.cross(f2);
    vec3f ef33 = e3.cross(f3);

    // now begin the series of tests
    if (!project3(n1, q1, q2, q3)) return false;
    if (!project3(m1, -q1, p2 - q1, p3 - q1)) return false;

    if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(g1, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(g2, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(g3, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(h1, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(h2, p1, p2, p3, q1, q2, q3)) return false;
    if (!project6(h3, p1, p2, p3, q1, q2, q3)) return false;

    return true;
}