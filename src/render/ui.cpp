#include "ui.h"

#include "raylib.h"
#include "raygui.h"

#include "render.h"
#include "render_field.h"
#include "render_roads.h"
#include <memory>

bool Renderer::mouse_in_viewport() {
    return ctx_.viewport.contains(ctx_.mouse_world_pos)
        && !Box<double>(
                { 0, 0 },
                {
                    uiConfig.icon_size + 2*uiConfig.icon_padding, 
                    static_cast<double>(ctx_.height)
                }
            ).contains(ctx_.mouse_world_pos);
}

void Renderer::render_tensorfield() {
    assert(ctx_.is_drawing);
    assert(!ctx_.is_2d_mode);

    if (current_mode_ != FieldEditor) return;

    draw_tensorfield(tf_ptr_, ctx_);
}

void Renderer::editor() {
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if(!mouse_in_viewport()) {
        radial_edit_.initialised = false;
        grid_edit_.initialised = false;
    }

    switch (tool_) {
        case RadialBrush:
            editor_radial();
            break;
        default:
            break;
    }
}

void Renderer::editor_radial() {
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        if (!radial_edit_.initialised) {
            radial_edit_.centre = ctx_.mouse_world_pos;
            radial_edit_.initialised = true;
        }

        DVector2 offset = ctx_.mouse_world_pos - radial_edit_.centre;
        float rad = std::hypot(offset.x, offset.y);

        DrawCircleLinesV(radial_edit_.centre, rad, RED);
    } else if (radial_edit_.initialised) {
        DVector2 offset = ctx_.mouse_world_pos - radial_edit_.centre;
        double rad = std::hypot(offset.x, offset.y);

        tf_ptr_->add_basis_field(std::make_unique<Radial>(
            radial_edit_.centre,
            rad,
            defaultDecay
        ));

        radial_edit_.initialised = false;
    } 
}

void Renderer::test_draw_spatial(qnode_id head_ptr) {
    auto& s = generator_ptr_->spatial_;
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if (head_ptr == QNullNode) return;
    QuadNode node = s.qnodes_[head_ptr];

    if (s.is_leaf(head_ptr)) {
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
            test_draw_spatial(node.children[q]);
        }
    }
}


void Renderer::render_map() {
    if (current_mode_ != Map) return;
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if (!generated_) {
        generator_ptr_->set_viewport(ctx_.viewport);
        generator_ptr_->generate();
        generated_ = true;
    }

    if (mouse_in_viewport()) {
        bool a = generator_ptr_->spatial_.has_nearby_point(ctx_.mouse_world_pos, 10, Major);
        bool b = generator_ptr_->spatial_.has_nearby_point(ctx_.mouse_world_pos, 10, Minor);

        Color c = BLACK;
        if (a && b) {c = PURPLE;}
        else if (a) {c = RED;}
        else if (b) {c = BLUE;}

        DrawCircleV(ctx_.mouse_world_pos, 10, c);
    }
    
    draw_map(ctx_, generator_ptr_);
}

void Renderer::render_hud() {
     DrawRectangle(
            0, 
            0, 
            uiConfig.icon_size + 2*uiConfig.icon_padding,
            ctx_.height, 
            LIGHTGRAY
        );

    for (int i=0; i < FieldEditorToolCount; ++i) {
        Rectangle button = {
            uiConfig.icon_padding, 
            (float) (uiConfig.y +(uiConfig.icon_size+ uiConfig.icon_padding)*i), 
            uiConfig.icon_size, 
            uiConfig.icon_size
        };

        bool clicked = GuiButton(button, NULL);
        if (clicked) {
            tool_ = (FieldEditorTool) i;
        }

        if (i == tool_) {
            DrawRectangleLinesEx(button, 2, RED);
        }

        GuiDrawIcon(
            fieldEditorIcons[i],
            (int) button.x,
            (int) button.y,
            3,
            BLACK
        );
    }

    if (tool_ == GenerateMap) {
        current_mode_ = Map;
        tool_ = (FieldEditorTool) 0;
    }
}

void Renderer::reset_field_editor() {
    tf_ptr_->clear();
    tf_ptr_->add_basis_field(
        std::make_unique<Grid>(0, DVector2{0,0})
    );
}

Renderer::Renderer(
        RenderContext& ctx, 
        TensorField* tf_ptr, 
        RoadNetworkGenerator* gen_ptr
    ) : 
    ctx_(ctx),
    tf_ptr_(tf_ptr),
    generator_ptr_(gen_ptr)
{
    reset_field_editor();
}

void Renderer::main_loop() {
    assert(ctx_.is_drawing);

    ClearBackground(RAYWHITE);
    render_tensorfield();
    BeginMode2D(ctx_.camera); ctx_.is_2d_mode = true; {
        render_map();
        // test_draw_spatial(generator_ptr_->spatial_.root_);
        editor();
    } EndMode2D(); ctx_.is_2d_mode = false;
    render_hud();
}
