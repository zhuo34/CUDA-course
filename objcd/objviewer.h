#pragma once

#include "tri_contact.h"

#include <vector>
#include <utility>

class Triangle {
private:
    std::vector<int> v_id;

public:
    Triangle() = default;
    Triangle(int a, int b, int c) {
        v_id.push_back(a);
        v_id.push_back(b);
        v_id.push_back(c);
    }
    bool hasSharedWith(const Triangle &t);
    int operator [] (int i) const {
        return v_id[i % 3];
    }
};

class MyObj {
private:
    std::vector<Triangle> fs;
    std::vector<vec3f> vs;
    std::vector<vec3f> vns;

    bool triContactDetection(int i, int j);
public:
    static MyObj load(const std::string &path);
    std::vector<std::pair<int, int>> selfContactDetection();
    std::vector<std::pair<int, int>> selfContactDetection(int blockSize);
    int nFace();
    int nVertex();
};
