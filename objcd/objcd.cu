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
#include "myobj.h"
#include "timer.h"
#include "myvector.h"

#include <iostream>
#include <set>

int main(int argc, char const *argv[]) {
//    std::string path = R"(D:\Projects\CUDA-course\objcd\flag-no-cd\0108_00.obj)";
    std::string path = R"(D:\Projects\CUDA-course\objcd\flag-2000-changed\0000_00.obj)";

    std::cout << "Loading object ..." << std::endl;
    Timer t;
    t.start();
    auto obj = MyObj::load(path);
    t.stop();
    std::cout << "Load done with "
        << obj.nFace() << " face(s) "
        << obj.nVertex() << " vertex(s) "
        << t.now() << " s"
        << std::endl;
    t.reset();
    t.start();
    int cnt = obj.selfContactDetectionCUDA(128);
    t.stop();
    std::cout << cnt << " contact(s) detected "
        << t.now() << " s" << std::endl;
    return 0;
}
