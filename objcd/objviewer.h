#pragma once

#include "tri_contact.h"

#include <vector>
#include <utility>

class MyObj {
private:
    std::vector<Triangle> fs;
    std::vector<vec3f> vs;
    std::vector<vec3f> vns;
public:
    static MyObj load(const std::string &path);
    std::vector<std::pair<int, int>> selfContactDetection();
    int nFace();
    int nVertex();
};
