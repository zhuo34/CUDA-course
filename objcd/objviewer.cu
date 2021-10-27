#include "objviewer.h"
#include "cuda.h"

#include <fstream>

MyObj MyObj::load(const std::string &path) {
    MyObj obj;
    std::ifstream f;
    f.open(path, std::ios::in);
    std::string s;
    double x, y, z;
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

bool MyObj::triContactDetection(int i, int j) {
    return tri_contact(vs[fs[i][0]-1], vs[fs[i][1]-1], vs[fs[i][2]-1],
                       vs[fs[j][0]-1], vs[fs[j][1]-1], vs[fs[j][2]-1]);
}

std::vector<std::pair<int, int>> MyObj::selfContactDetection() {
    std::vector<std::pair<int, int>> pairs;
    for (int i = 0; i < 1; i++) {
        for (int j = i+1; j < nFace(); j++) {
            if (fs[i].hasSharedWith(fs[j]))
                continue;
            if (triContactDetection(i, j)) {
                pairs.emplace_back(i, j);
            }
        }
    }
    return pairs;
}

std::vector<std::pair<int, int>> MyObj::selfContactDetection(int blockSize) {
    if (blockSize == 0)
        return selfContactDetection();
    // TODO: CUDA version

    std::vector<std::pair<int, int>> pairs;
    return pairs;
}

bool Triangle::hasSharedWith(const Triangle &t) {
    for (auto &v : v_id) {
        for (auto &v2 : t.v_id) {
            if (v == v2) {
                return true;
            }
        }
    }
    return false;
}