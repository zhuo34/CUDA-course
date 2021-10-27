#include "tri_contact.h"

bool tri_contact(vec3f &P1, vec3f &P2, vec3f &P3, vec3f &Q1, vec3f &Q2, vec3f &Q3) {
    vec3f p1;
    vec3f p2 = P2 - P1;
    vec3f p3 = P3 - P1;
    vec3f q1 = Q1 - P1;
    vec3f q2 = Q2 - P1;
    vec3f q3 = Q3 - P1;

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