#ifndef RENDER_ROADS_H
#define RENDER_ROADS_H

#include <unordered_map>

#include "raylib.h"
#include "render.h"

#include "../generation/generator.h"


struct RoadStyle {
    Color colour;
    Color outline_colour;
    float width;
    float outline_width;

    static RoadStyle default_roadstyle(RoadType t);
};


void draw_map(
    RenderContext& ctx, 
    RoadNetworkGenerator* rg
);

void draw_streamlines(
    RenderContext& ctx,
    RoadNetworkGenerator* rg, 
    RoadType road, 
    RoadStyle road_style,
    Direction dir
);

#endif
