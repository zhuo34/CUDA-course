/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and 
 * proprietary rights in and to this software and related documentation. 
 * Any use, reproduction, disclosure, or distribution of this software 
 * and related documentation without an express license agreement from
 * NVIDIA Corporation is strictly prohibited.
 *
 * Please refer to the applicable NVIDIA end user license agreement (EULA) 
 * associated with this source code for terms and conditions that govern 
 * your use of this NVIDIA software.
 * 
 */


#include "tri_contact.h"
#include "objviewer.h"
#include <iostream>

int main(int argc, char const *argv[]) {
    // if (argc == 2) {

    // }
    std::string path = "D:/Projects/CUDA-course/objcd/flag-2000-changed/0000_00.obj";
    std::cout << "Loading object ..." << std::endl;
    auto obj = MyObj::load(path);
    std::cout << "Load done with " << obj.nFace() << " face(s)" << std::endl;
    auto pairs = obj.selfContactDetection();
    std::cout << pairs.size() << " contact(s) detected." << std::endl;
    for (auto &p : pairs) {
        std::cout << "\tContact detected at: (" << p.first
            << ", " << p.second << ")." << std::endl;
    }
    return 0;
}
