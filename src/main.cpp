#include <cassert>

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#define RAYGUI_IMPLEMENTATION

#include "ui.h"

#include "generation/generator.h"
#include "generation/tensor_field.h"

#include "const.h"


static std::unordered_map<RoadType, GeneratorParameters> params = {
    {SideStreet, GeneratorParameters(300, 1970,  20.0,  15.0, 5.0, 1.0,  40.0, 0.1, 0.5, 10.0)},
    {HighStreet, GeneratorParameters(300, 3020, 100.0,  30.0, 8.0, 1.0, 200.0, 0.1, 0.5, 10.0)},
    {Main,       GeneratorParameters(300, 1900, 400.0, 200.0, 10.0, 1.0, 500.0, 0.1, 0.5, 10.0)}
};


int main(int argc, char** argv)
{
    RenderContext ctx = RenderContext {
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        { 0 }, 
        false,
        false,
        {0.0, 0.0},
        Box({0.0, 0.0}, DVector2(SCREEN_WIDTH, SCREEN_HEIGHT))
    };


    TensorField tf;
    std::unique_ptr<NumericalFieldIntegrator> itg =
        std::make_unique<RK4>(&tf);

    RoadNetworkGenerator generator = RoadNetworkGenerator(
            itg,
            params,
            ctx.viewport
    );

    Renderer renderer = Renderer(ctx, &tf, &generator);

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "MapGen");
        ctx.camera.zoom = 1.0f;

        SetTargetFPS(60);
        while (!WindowShouldClose())
        {
            ctx.mouse_world_pos= GetScreenToWorld2D(
                GetMousePosition(), 
                ctx.camera
            );

            ctx.viewport = Box(
                DVector2(GetScreenToWorld2D(screen_dims.min, ctx.camera)),
                DVector2(GetScreenToWorld2D(screen_dims.max, ctx.camera))
            );

            float wheel = GetMouseWheelMove();

            // camera pan
            if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
                Vector2 delta = GetMouseDelta();
                delta = Vector2Scale(delta, -1.0f/ctx.camera.zoom);
                ctx.camera.target = Vector2Add(ctx.camera.target, delta);
            }



            // camera scroll
            if (wheel != 0) {
                ctx.camera.offset = GetMousePosition();
                ctx.camera.target = ctx.mouse_world_pos;

                float scale = 0.2f*wheel;
                ctx.camera.zoom = 
                    Clamp(expf(logf(ctx.camera.zoom)+scale), 0.125f, 64.0f);
            }

            

            
            BeginDrawing(); ctx.is_drawing = true; {
                // Renderer::main loop here
                renderer.main_loop();

                DrawFPS(0, 0);
            } EndDrawing(); ctx.is_drawing = false;
        }

        CloseWindow();

        return 0;
}

