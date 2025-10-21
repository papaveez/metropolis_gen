#ifndef CONST_H
#define CONST_H

// #define SPATIAL_TEST

#include "types.h"

#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080


static Box<double> screen_dims(
    {0.0, 0.0},
    {
        static_cast<double>(SCREEN_WIDTH),
        static_cast<double>(SCREEN_HEIGHT)
    }
);


#endif
