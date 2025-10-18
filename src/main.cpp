#include <cassert>

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#define RAYGUI_IMPLEMENTATION

#include "render/render.h"
#include "render/ui.h"

#include "generation/generator.h"
#include "generation/tensor_field.h"

#define SCREEN_WIDTH 1700
#define SCREEN_HEIGHT 1000 


Box<double> screen_dims(
    {0.0, 0.0},
    {
        static_cast<double>(SCREEN_WIDTH),
        static_cast<double>(SCREEN_HEIGHT)
    }
);




static std::unordered_map<RoadType, GeneratorParameters> params = {
    {SideStreet, GeneratorParameters(300, 1970, 20.0, 15.0, 5.0, 1.0, 40.0, 0.1, 0.5, 0.0, 0.0)},
    {HighStreet, GeneratorParameters(300, 3020, 100.0, 30.0, 5.0, 1.0, 200.0, 0.1, 0.5, 0.0, 0.0)},
    {Main, GeneratorParameters(300, 1900, 400.0, 200.0, 5.0, 1.0, 500.0, 0.1, 0.5, 0.0, 0.0)}
};



void test_draw_spatial(RenderContext& ctx, Spatial& s, qnode_id head_ptr) {
    assert(ctx.is_drawing);
    assert(ctx.is_2d_mode);

    if (head_ptr == QNullNode) return;
    QuadNode node = s.qnodes_[head_ptr];

    if (s.is_leaf(head_ptr)) {
        for (node_id i : node.data) {
            DrawCircleV((*s.all_nodes_)[i].pos, 2, RED);
        }
    }
    else {
        Box<double> bbox = node.bbox;
        DVector2 mid = middle(bbox.min, bbox.max);
        DVector2 hstart = {bbox.min.x, mid.y};
        DVector2 hend = {bbox.max.x, mid.y};

        DVector2 vstart = {mid.x, bbox.min.y};
        DVector2 vend = {mid.x, bbox.max.y};

        DrawLineV(vstart, vend, BLACK);
        DrawLineV(hstart, hend, BLACK);
        for (auto q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
            test_draw_spatial(ctx, s, node.children[q]);
        }
    }
}

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

