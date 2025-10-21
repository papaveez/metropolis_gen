#include "ui.h"

#include <limits>

#include "raylib.h"
#include "raygui.h"
#include "raymath.h"


static constexpr float f_epsilon = std::numeric_limits<float>::epsilon();

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



void 
Renderer::draw_vector_line( 
                 const Vector2& vec, const Vector2& world_pos,
                 Color col) const
{
    float l = std::hypot(vec.x, vec.y);

    if (l <= f_epsilon)
        return;

    Vector2 offset = vec*uiConfig.lineScale/l/ctx_.camera.zoom;

    Vector2 pts[2] = {
        GetWorldToScreen2D(world_pos - offset, ctx_.camera),
        GetWorldToScreen2D(world_pos + offset, ctx_.camera)
    };



    DrawSplineLinear(pts, 2, 2.0f, col);
}



void Renderer::render_tensorfield() const {
    assert(ctx_.is_drawing);
    assert(!ctx_.is_2d_mode);

    for (float i=0;i<ctx_.width;i+=uiConfig.granularity) {
        for (float j=0;j<ctx_.height;j+=uiConfig.granularity) {
            // map to world coordinates

            Vector2 world_pos = GetScreenToWorld2D((Vector2) {i, j}, ctx_.camera);

            Tensor t = tf_ptr_->sample(world_pos);
            DVector2 major_eigen = t.get_major_eigenvector();
            DVector2 minor_eigen = t.get_minor_eigenvector();

            // draw cross
            draw_vector_line(major_eigen, world_pos, RED);
            draw_vector_line(minor_eigen, world_pos, DARKBLUE);

            DrawCircle(i, j, 1, BLUE);
        }
    } 
}

void Renderer::editor() {
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if(!mouse_in_viewport()) {
        radial_edit_.initialised = false;
        grid_edit_.initialised = false;
    }

    EditorTool* edit;
    switch (brush_) {
        case GridBrush:
            edit = &grid_edit_;
            break;
        case RadialBrush:
            edit = &radial_edit_;
            break;
        default:
            return;
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        if (!edit->initialised) {
            edit->centre = ctx_.mouse_world_pos;
            edit->initialised = true;
        }

        DVector2 offset = ctx_.mouse_world_pos - edit->centre;
        float rad = std::hypot(offset.x, offset.y);

        DrawCircleLinesV(edit->centre, rad, RED);
        if (edit->draw_spoke) {
            DrawLineV(edit->centre, ctx_.mouse_world_pos, RED);
        }
    } else if (edit->initialised) {
        handle_brush_release();
        edit->initialised = false;
    } 
}


void Renderer::handle_brush_release() {
    if (brush_ == GridBrush) {
        double theta = vector_angle({1, 0}, ctx_.mouse_world_pos);
        DVector2 diff = ctx_.mouse_world_pos - grid_edit_.centre;
        double rad = std::hypot(diff.x, diff.y);

        tf_ptr_->add_basis_field(std::make_unique<Grid>(
            theta,
            grid_edit_.centre,
            rad,
            grid_edit_.decay
        ));
    } else if (brush_ == RadialBrush) {
        DVector2 diff = ctx_.mouse_world_pos - radial_edit_.centre;
        double rad = std::hypot(diff.x, diff.y);

        tf_ptr_->add_basis_field(std::make_unique<Radial>(
            radial_edit_.centre,
            rad,
            radial_edit_.decay
        ));
    }
}


#ifdef SPATIAL_TEST
void Renderer::test_spatial() {
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if (IsKeyDown(KEY_SPACE)) {
        Color col = BLACK;

        std::list<node_id> majors = 
            generator_ptr_->spatial_.nearby_points(ctx_.mouse_world_pos, 100, Major);
        
        std::list<node_id> minors =
            generator_ptr_->spatial_.nearby_points(ctx_.mouse_world_pos, 100, Minor);

        int a = majors.size();
        int b = minors.size();
        if (a && b ) {
            col = GREEN;
        } else if (a) {
            col = RED;
        } else if (b) {
            col= BLUE;
        }

        DrawCircleLinesV(ctx_.mouse_world_pos, 100, col);

        DrawTextEx(GetFontDefault(), TextFormat("[%i, %i]", a, b),
                ctx_.mouse_world_pos+DVector2{ -44, -24 }, 20, 2, BLACK);
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        if (points_.size()) {
            if (points_.size() > 1) {
                int n = points_.size();
                Vector2* pts = new Vector2[n];

                int i = 0;
                for (const DVector2& v : points_) {
                    pts[i] = v;
                    ++i;
                }

                DrawSplineLinear(pts, n, 3.0f, dir_== Major ? RED : BLUE);
            }

            DVector2 diff = points_.back() - ctx_.mouse_world_pos;
            if (dot_product(diff, diff) < 1000.0) return;
        }

        points_.push_back(ctx_.mouse_world_pos);

        
    } else if (points_.size()) {
        generator_ptr_->push_streamline(Main, points_, dir_);

        points_ = {};
    } else if (IsKeyPressed(KEY_D)) {
        dir_ = flip(dir_);
    }
}

void Renderer::test_draw_spatial(qnode_id head_ptr) {
    auto& s = generator_ptr_->spatial_;

    if (head_ptr == QNullNode) return;
    QuadNode node = s.qnodes_[head_ptr];

    Box<double> bbox = node.bbox;
    Vector2 pos = bbox.min;

    Color col = BLACK;

    if (node.dirs & Major && node.dirs & Minor) {
        col = GREEN;
    } else if (node.dirs & Major) {
        col = RED;
    } else if (node.dirs & Minor) {
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

void Renderer::draw_streamlines(RoadType road, Direction dir) const {
    const std::vector<Streamline>& sls = generator_ptr_->get_streamlines(road, dir);

    const RoadStyle& style = road_styles_.at(road);

    for (const Streamline& sl : sls) {
        assert(sl.size() > 1);

        int extra_edge = 0; // to join circles

        if (sl.front() == sl.back()) {
            assert(sl.size() > 2);
            extra_edge = 1;

        }


        Vector2* positions = new Vector2[sl.size() + extra_edge];

        int i = 0;
        for (const node_id& id: sl) {
            std::optional<StreamlineNode> maybe_node = generator_ptr_->get_node(id);
            assert(maybe_node);
            positions[i] = maybe_node.value().pos;
            i++;
        }

        if (extra_edge) {
            positions[sl.size()] = positions[0];
        }

        DrawSplineLinear(
            positions, 
            sl.size() + extra_edge, 
            style.outline_width+style.width,
            style.outline_colour
        );

        DrawSplineLinear(
            positions,
            sl.size() + extra_edge,
            style.width,
            style.colour
        );

        // for (int i=0; i < sl.size() + extra_edge; ++i) {
        //     DrawCircleV(positions[i], 3, RED);
        // }
        
        delete[] positions;
    }

}

void Renderer::render_map() {
    assert(ctx_.is_drawing);
    assert(ctx_.is_2d_mode);

    if (!generated_) {
        generator_ptr_->set_viewport(ctx_.viewport);
    }
    if (!generated_ && !step_mode_) {
        generator_ptr_->generate();
        generated_ = true;
    } else if (step_mode_ && IsKeyPressed(KEY_SPACE)) {
        if (generator_ptr_->generation_step(Main, dir_))
            dir_ = flip(dir_);
    }

    const std::vector<RoadType>& road_types = generator_ptr_->get_road_types();
    for (int i=road_types.size()-1; i>=0; --i) {
        draw_streamlines(road_types[i], Major);
        draw_streamlines(road_types[i], Minor);
    }
}

void Renderer::render_generating_popup() const {
    Vector2 mid = {ctx_.width/2.0f, ctx_.height/2.0f};

    Rectangle b = {
        mid.x - uiConfig.modalWidth/2.0f,
        mid.y - uiConfig.modalHeight/2.0f,
        uiConfig.modalWidth,
        uiConfig.modalHeight
    };

    GuiButton(b, NULL);
    const char* text = "Generating...";

    int text_width = MeasureText(text, 40);
    DrawText(text, mid.x-text_width/2.0f, mid.y-20, 40, BLACK);

}



void Renderer::render_hud() {
     DrawRectangle(
            0, 
            0, 
            uiConfig.icon_size + 2*uiConfig.icon_padding,
            ctx_.height, 
            LIGHTGRAY
        );

    
     std::list<Tool>& tools = mode_ == FieldEditor ? fieldEditorTools : mapTools;

     int i=0;
     for (Tool t : tools) {
        Rectangle button = {
            uiConfig.icon_padding, 
            (float) (uiConfig.y +(uiConfig.icon_size+ uiConfig.icon_padding)*i), 
            uiConfig.icon_size, 
            uiConfig.icon_size
        };

        bool clicked = GuiButton(button, NULL);
        if (clicked) {
            handle_tool_click(t);
        }

        if (t == brush_) {
            DrawRectangleLinesEx(button, 2, RED);
        }

        GuiDrawIcon(
            toolIcons[t],
            (int) button.x,
            (int) button.y,
            3,
            BLACK
        );

        ++i;
    }
}

void Renderer::handle_tool_click(const Tool& t) {
    bool will_generate;
    if (t <= 1) {
        brush_ = t;
    } else if (t == GenerateMap) {
        step_mode_ = false;
        mode_ = Map;
        will_generate = !generated_;
    } else if (t == StepGen) {
        step_mode_ = true;
        mode_ = Map;
    } else if (t == BackToEditor) {
        mode_ = FieldEditor;
    } else if (t == Regenerate) {
        generated_ = false;
        will_generate = true;
    }

    if (!generated_ && will_generate) {
        render_generating_popup();
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
        
        auto text = dir_ == Major ? "MAJOR" : "MINOR";
        Color col = dir_ == Major ? RED : BLUE;
        DrawText(text, ctx_.width-200, 10, 30, col);

    #else
    if (mode_ == FieldEditor) render_tensorfield();
    BeginMode2D(ctx_.camera); ctx_.is_2d_mode = true; {
        if (mode_ == Map) render_map();
        editor();
    } EndMode2D(); ctx_.is_2d_mode = false;
    render_hud();
    // DrawTextEx(
    //     GetFontDefault(), 
    //     TextFormat(
    //         "[%f, %f]", 
    //         ctx_.mouse_world_pos.x, 
    //         ctx_.mouse_world_pos.y
    //     ), 
    //     GetMousePosition()+(Vector2){ -44, -24 }, 
    //     20,
    //     2,
    //     BLACK
    // );

    #endif
}
