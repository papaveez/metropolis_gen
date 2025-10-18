#include <cassert>
#include <limits>
#include <utility>

#include "raylib.h"
#include "raymath.h"

#include "render_field.h"
#include "render.h"



void draw_tensorfield(TensorField *tf, RenderContext& ctx) {
    assert(ctx.is_drawing);
    assert(!ctx.is_2d_mode);


    int granularity = 26; // 30 pixels. hardcoded. not relative. maybe change.

    float scale = 10.0;


    for (float i=0;i<ctx.width;i+=granularity) {
        for (float j=0;j<ctx.height;j+=granularity) {
            // map to world coordinates

            Vector2 world_pos = GetScreenToWorld2D((Vector2) {i, j}, ctx.camera);

            Tensor t = tf->sample(world_pos);
            std::pair<Vector2, Vector2> eigenvectors = t.eigenvectors();


            // draw appropriate cross.

            float l_major = std::hypot(eigenvectors.first.x, eigenvectors.first.y);
            float l_minor = std::hypot(eigenvectors.second.x, eigenvectors.second.y);


            if (l_major) {
                Vector2 world_major_end = Vector2Add(
                    world_pos,
                    Vector2Scale(eigenvectors.first, scale/l_major/ctx.camera.zoom)
                );

                Vector2 world_major_start = Vector2Subtract(
                    world_pos, 
                    Vector2Scale(eigenvectors.first, scale/l_major/ctx.camera.zoom)
                );

                Vector2 pts[2] = {
                    GetWorldToScreen2D(world_major_start, ctx.camera), 
                    GetWorldToScreen2D(world_major_end, ctx.camera)
                };

                DrawSplineLinear(pts, 2, 2.0f, RED);
            }

            if (l_major > std::numeric_limits<float>::epsilon()) {
                Vector2 world_minor_end = Vector2Add(
                    world_pos, 
                    Vector2Scale(eigenvectors.second, scale/l_minor/ctx.camera.zoom)
                );

                Vector2 world_minor_start = Vector2Subtract(
                    world_pos, 
                    Vector2Scale(eigenvectors.second, scale/l_minor/ctx.camera.zoom)
                );

                Vector2 pts[2] = {
                    GetWorldToScreen2D(world_minor_start, ctx.camera),
                    GetWorldToScreen2D(world_minor_end, ctx.camera)
                };

                DrawSplineLinear(pts, 2, 2.0f, DARKBLUE);
            }

            

            DrawCircle(i, j, 1, BLUE);
        }
    } 

}
