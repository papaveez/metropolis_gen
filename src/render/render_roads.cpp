#include "render_roads.h"

#include <cassert>

RoadStyle RoadStyle::default_roadstyle(RoadType t) {
    switch (t) {
        case Main:
            return {
                {250, 224, 98, 255},
                {238, 199, 132, 255},
                10.0f,
                2.0f
            };
        case HighStreet:
            return {
                {252,252,224, 255},
                {240,210,152, 255},
                8.0f,
                2.0f
            };
        case SideStreet:
            return {
                {255,255,255, 255},
                {215, 208, 198},
                6.0f,
                1.0f
            };
    }
}

static std::unordered_map<RoadType, RoadStyle> road_styles = {
    {Main, RoadStyle::default_roadstyle(Main)},
    {HighStreet, RoadStyle::default_roadstyle(HighStreet)},
    {SideStreet, RoadStyle::default_roadstyle(SideStreet)}
};



void draw_map(
    RenderContext& ctx, 
    RoadNetworkGenerator* rg
) {
    std::vector<RoadType> road_types = rg->get_road_types();

    for (int i=road_types.size()-1; i>=0; --i) {
        RoadType road = road_types[i];

        draw_streamlines(ctx, rg, road, road_styles[road], Major);
        draw_streamlines(ctx, rg, road, road_styles[road], Minor);
    }

}

void draw_streamlines(
    RenderContext& ctx,
    RoadNetworkGenerator* rg, 
    RoadType road, 
    RoadStyle road_style,
    Direction dir
) {
    assert(ctx.is_drawing);
    assert(ctx.is_2d_mode);


    std::vector<Streamline>& sls = rg->get_streamlines(road, dir);

    for (Streamline& sl : sls) {
        assert(sl.size() > 1);

        int extra_edge = 0; // to join circles

        if (sl.front() == sl.back()) {
            assert(sl.size() > 2);
            extra_edge = 1;

        }


        Vector2* positions = new Vector2[sl.size() + extra_edge];

        int i = 0;
        for (node_id& id: sl) {
            std::optional<StreamlineNode> maybe_node = rg->get_node(id);
            assert(maybe_node);
            positions[i] = maybe_node.value().pos;
            i++;
        }

        if (extra_edge) {
            positions[sl.size()] = positions[0];
        }
        DrawSplineBezierCubic(
            positions, 
            sl.size() + extra_edge, 
            road_style.outline_width+road_style.width,
            road_style.outline_colour
        );

        DrawSplineBezierCubic(
            positions,
            sl.size() + extra_edge,
            road_style.width,
            road_style.colour
        );

        delete[] positions;
    }

}
