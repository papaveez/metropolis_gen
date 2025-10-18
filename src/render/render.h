#ifndef RENDER_H
#define RENDER_H

#include <raylib.h>

#include "../generation/types.h"

struct RenderContext {
    int width, height;
    Camera2D camera;
    bool is_drawing;
    bool is_2d_mode;
    DVector2 mouse_world_pos;
    Box<double> viewport;
};


#endif
