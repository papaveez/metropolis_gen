#ifndef CONST_H
#define CONST_H

// #define SPATIAL_TEST

#include "generation/types.h"

#define SCREEN_WIDTH 1700
#define SCREEN_HEIGHT 1000 


static Box<double> screen_dims(
    {0.0, 0.0},
    {
        static_cast<double>(SCREEN_WIDTH),
        static_cast<double>(SCREEN_HEIGHT)
    }
);


#endif
