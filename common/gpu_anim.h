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


#ifndef __GPU_ANIM_H__
#define __GPU_ANIM_H__

#include "gl_helper.h"

#include "cuda.h"
#include "cuda_gl_interop.h"
#include <iostream>


PFNGLBINDBUFFERARBPROC    glBindBuffer     = nullptr;
PFNGLDELETEBUFFERSARBPROC glDeleteBuffers  = nullptr;
PFNGLGENBUFFERSARBPROC    glGenBuffers     = nullptr;
PFNGLBUFFERDATAARBPROC    glBufferData     = nullptr;


class GPUAnimBitmap {
private:
    GLuint  bufferObj;
    cudaGraphicsResource *resource;
    int     width, height;
    void    *dataBlock;
    void (*fAnim)(uchar4*,void*,int,float);
    void (*animExit)(void*);
    void (*clickDrag)(void*,int,int,int,int);
    int     dragStartX, dragStartY;
    int tick;
    bool state_tick;
    float state;
    float speed;
    static GPUAnimBitmap *gBitmap;
    static const float max_speed;

public:
    GPUAnimBitmap(int w, int h, const std::string &windowName = "Untitled") {
        width = w;
        height = h;
        dataBlock = nullptr;
        clickDrag = nullptr;

        // first, find a CUDA device and set it to graphic interop
        cudaDeviceProp  prop;
        int dev;
        memset( &prop, 0, sizeof( cudaDeviceProp ) );
        prop.major = 1;
        prop.minor = 0;
        HANDLE_ERROR( cudaChooseDevice( &dev, &prop ) );
        cudaGLSetGLDevice( dev );

        // a bug in the Windows GLUT implementation prevents us from
        // passing zero arguments to glutInit()
        int c=1;
        char* dummy = "";
        glutInit( &c, &dummy );
        glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA );
        glutInitWindowSize( width, height );
        glutCreateWindow(windowName.c_str());

        glBindBuffer    = (PFNGLBINDBUFFERARBPROC)GET_PROC_ADDRESS("glBindBuffer");
        glDeleteBuffers = (PFNGLDELETEBUFFERSARBPROC)GET_PROC_ADDRESS("glDeleteBuffers");
        glGenBuffers    = (PFNGLGENBUFFERSARBPROC)GET_PROC_ADDRESS("glGenBuffers");
        glBufferData    = (PFNGLBUFFERDATAARBPROC)GET_PROC_ADDRESS("glBufferData");

        glGenBuffers( 1, &bufferObj );
        glBindBuffer( GL_PIXEL_UNPACK_BUFFER_ARB, bufferObj );
        glBufferData( GL_PIXEL_UNPACK_BUFFER_ARB, width * height * 4,
                      nullptr, GL_DYNAMIC_DRAW_ARB );

        HANDLE_ERROR( cudaGraphicsGLRegisterBuffer( &resource, bufferObj, cudaGraphicsMapFlagsNone ) );
    }

    void anim_and_exit( void (*f)(uchar4*,void*,int,float), void(*e)(void*) = nullptr) {
        gBitmap = this;
        fAnim = f;
        animExit = e;
        init_state();

        glutKeyboardFunc( Key );
        glutDisplayFunc( Draw );
        glutMouseFunc( mouse_func );
        glutIdleFunc( idle_func );
        glutMainLoop();
    }

    ~GPUAnimBitmap() {
        free_resources();
    }

    long image_size() const { return width * height * 4; }

private:
    void free_resources() {
        HANDLE_ERROR( cudaGraphicsUnregisterResource( resource ) );

        glBindBuffer( GL_PIXEL_UNPACK_BUFFER_ARB, 0 );
        glDeleteBuffers( 1, &bufferObj );
    }

    void init_state() {
        tick = 0;
        state = 0;
        state_tick = false;
        speed = 0.001;
    }

//    void click_drag( void (*f)(void*,int,int,int,int)) {
//        clickDrag = f;
//    }

    // static method used for glut callbacks
    static GPUAnimBitmap *get_bitmap_ptr() {
        return gBitmap;
    }

    // static method used for glut callbacks
    static void mouse_func( int button, int state,
                            int mx, int my ) {
        if (button == GLUT_LEFT_BUTTON) {
            GPUAnimBitmap*   bitmap = get_bitmap_ptr();
            if (state == GLUT_DOWN) {
                bitmap->dragStartX = mx;
                bitmap->dragStartY = my;
                bitmap->state_tick = !bitmap->state_tick;
            } else if (state == GLUT_UP) {
                if (bitmap->clickDrag != nullptr)
                    bitmap->clickDrag( bitmap->dataBlock,
                                       bitmap->dragStartX,
                                       bitmap->dragStartY,
                                       mx, my );
            }
        }
    }

    // static method used for glut callbacks
    static void idle_func() {
//        static int ticks = 0;
        GPUAnimBitmap*  bitmap = get_bitmap_ptr();
        uchar4*         devPtr;
        size_t  size;

        HANDLE_ERROR( cudaGraphicsMapResources( 1, &(bitmap->resource), nullptr ) );
        HANDLE_ERROR( cudaGraphicsResourceGetMappedPointer( (void**)&devPtr, &size, bitmap->resource) );

        bitmap->fAnim( devPtr, bitmap->dataBlock, bitmap->tick++, bitmap->state );
        if (bitmap->state_tick) {
            bitmap->state += bitmap->speed;
            if (bitmap->state >= 1) bitmap->state = 0;
        }
        HANDLE_ERROR( cudaGraphicsUnmapResources( 1, &(bitmap->resource), nullptr ) );

        glutPostRedisplay();
    }

    // static method used for glut callbacks
    static void Key(unsigned char key, int x, int y) {
        switch (key) {
            case 27:
                GPUAnimBitmap*   bitmap = get_bitmap_ptr();
                if (bitmap->animExit)
                    bitmap->animExit( bitmap->dataBlock );
                bitmap->free_resources();
                exit(0);
        }
    }

    // static method used for glut callbacks
    static void Draw() {
        GPUAnimBitmap *bitmap = get_bitmap_ptr();
        glClearColor( 0.0, 0.0, 0.0, 1.0 );
        glClear( GL_COLOR_BUFFER_BIT );
        glDrawPixels( bitmap->width, bitmap->height, GL_RGBA,
                      GL_UNSIGNED_BYTE, 0 );
        glutSwapBuffers();
    }
};

GPUAnimBitmap *GPUAnimBitmap::gBitmap = nullptr;
const float GPUAnimBitmap::max_speed = 0.1f;

#endif  // __GPU_ANIM_H__

