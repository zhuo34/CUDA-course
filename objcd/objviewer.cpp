#include "objviewer.h"

#include <fstream>
#include <iostream>

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
            obj.vs.push_back(vec3f(x, y, z));
        } else if (s == "ny") {
            f >> x >> y >> z;
            obj.vns.push_back(vec3f(x, y, z));
        } else if (s == "f") {
            f >> f_v1; f.ignore(); f >> f_vn1;
            f >> f_v2; f.ignore(); f >> f_vn2;
            f >> f_v3; f.ignore(); f >> f_vn3;
            obj.fs.push_back(Triangle(obj.vs[f_v1], obj.vs[f_v2], obj.vs[f_v3]));
        } else if (s == "td") {
            f >> x;
        }
    }
    // std::cout << obj.vs.size() << std::endl;
    // std::cout << obj.fs.size() << std::endl;
    return obj;
}

int MyObj::nFace() {
    return fs.size();
}

int MyObj::nVertex() {
    return vs.size();
}

std::vector<std::pair<int, int>> MyObj::selfContactDetection() {
    std::vector<std::pair<int, int>> pairs;
    for (int i = 0; i < 1; i++) {
        std::cout << i << std::endl;
        for (int j = i+1; j < fs.size(); j++) {
            if (!fs[i].contact(fs[j])) {
                std::cout << "contact detected" << std::endl;
                pairs.push_back(std::make_pair(i, j));
            }
        }
    }
    return pairs;
}