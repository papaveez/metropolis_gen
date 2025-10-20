#ifndef UI_H
#define UI_H

#include <cassert>

#include "raygui.h"

#include "../generation/tensor_field.h"
#include "render.h"
#include "../generation/generator.h"
#include "../const.h"

static constexpr double defaultDecay = 5;

static constexpr struct {
    int icon_size = 48;
    int icon_padding = 6;
    int y = 20;
} uiConfig;

enum UIMode {
    FieldEditor,
    Map
};

enum FieldEditorTool : int {
    GridBrush,
    RadialBrush,
    GenerateMap,
    FieldEditorToolCount
};

static constexpr int fieldEditorIcons[FieldEditorToolCount] = {
    ICON_BOX_GRID,
    ICON_GEAR_EX,
    ICON_ROM
};





class Renderer {
private:
    RenderContext& ctx_;
    TensorField* tf_ptr_;
    RoadNetworkGenerator* generator_ptr_;

    #ifdef SPATIAL_TEST
    Streamline current_streamline_;
    Direction current_dir_ = Major;
    std::vector<StreamlineNode> new_nodes_;

    void test_spatial();
    void test_draw_spatial(qnode_id head_ptr);
    #endif

    UIMode current_mode_  = FieldEditor;
    FieldEditorTool tool_ = GridBrush;

    struct {
        DVector2 centre = {0,0};
        bool initialised = false;
    } radial_edit_;

    struct {
        DVector2 centre = {0,0};
        double theta = 0;
        bool initialised = false;
    } grid_edit_;

    bool generated_;

    void reset_field_editor();

    bool mouse_in_viewport();

    void render_tensorfield();
    void render_map();
    void editor();
    void editor_radial();

    void render_hud();


public:
    Renderer(RenderContext& ctx, TensorField* tf_ptr, RoadNetworkGenerator* gen_ptr);
    void main_loop();
};








#endif

