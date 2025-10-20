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

#ifdef SPATIAL_TEST
void Renderer::test_spatial() {
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if (IsKeyDown(KEY_SPACE)) {
        Color col;

        bool has_major = 
            generator_ptr_->spatial_.has_nearby_point(ctx_.mouse_world_pos, 100, Major);
        bool has_minor =
            generator_ptr_->spatial_.has_nearby_point(ctx_.mouse_world_pos, 100, Minor);

        if (has_major && has_minor) {
            col = GREEN;
        } else if (has_major) {
            col = RED;
        } else if (has_minor) {
            col= BLUE;
        } else {
            col = BLACK;
        }

        DrawCircleLinesV(ctx_.mouse_world_pos, 100, col);

    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        if (new_nodes_.size()) {
            if (new_nodes_.size() > 1) {
                Vector2* pts = new Vector2[new_nodes_.size()];
                int i = 0;
                for (auto& n : new_nodes_) {
                    pts[i] = n.pos;
                    ++i;
                }
                DrawSplineLinear(pts, new_nodes_.size(), 3.0f, current_dir_==Major ? RED : BLUE);
            }
            StreamlineNode& last_node = new_nodes_.back();
            DVector2 diff = last_node.pos - ctx_.mouse_world_pos;
            if (dot_product(diff, diff) < 1000.0) return;
        }

        current_streamline_.push_back(new_nodes_.size() + generator_ptr_->node_count());
        new_nodes_.push_back(StreamlineNode{
            ctx_.mouse_world_pos,
            0,
            current_dir_
        });


        
    } else if (current_streamline_.size()) {
        generator_ptr_->push_streamline(Main, new_nodes_, current_streamline_, current_dir_);

        current_streamline_ = {};
        new_nodes_ = {};
    } else if (IsKeyPressed(KEY_D)) {
        current_dir_ = flip(current_dir_);
    }
}

void Renderer::test_draw_spatial(qnode_id head_ptr) {
    auto& s = generator_ptr_->spatial_;

    if (head_ptr == QNullNode) return;
    QuadNode node = s.qnodes_[head_ptr];

    Box<double> bbox = node.bbox;
    Vector2 pos = bbox.min;
    Color col = BLACK;
    bool is_major = node.directions_bitmask & (1<<Major);
    bool is_minor = node.directions_bitmask & (1<<Minor);

    if (is_major && is_minor) {
        col = GREEN;
    } else if (is_major) {
        col = RED;
    } else if (is_minor) {
        col = BLUE;
    }
    Vector2 dims = bbox.max - bbox.min;

    Rectangle rect = {pos.x, pos.y, dims.x, dims.y};

    DrawRectangleLinesEx(rect, 2.0f, col);

    if (s.is_leaf(head_ptr)) {
        for (auto id: node.data) {
            std::optional<StreamlineNode> n = generator_ptr_->get_node(id);
            assert(n);
            DrawCircleV(n.value().pos, 1.0, n.value().dir == Major ? RED : BLUE);
        }
    }
    else {
        for (auto q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
            test_draw_spatial(node.children[q]);
        }
    }
}
#endif

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
    #ifdef SPATIAL_TEST
        BeginMode2D(ctx_.camera); ctx_.is_2d_mode = true; {
            test_spatial();
            test_draw_spatial(generator_ptr_->spatial_.root_);
        } EndMode2D(); ctx_.is_2d_mode = false;
        // draw current dir
        
        auto text = current_dir_ == Major ? "MAJOR" : "MINOR";
        Color col = current_dir_ == Major ? RED : BLUE;
        DrawText(text, ctx_.width-200, 10, 30, col);

    #else
    render_tensorfield();
    BeginMode2D(ctx_.camera); ctx_.is_2d_mode = true; {
        render_map();
        editor();
    } EndMode2D(); ctx_.is_2d_mode = false;
    render_hud();
    #endif
}
