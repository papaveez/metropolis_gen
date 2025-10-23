#ifndef NODE_STORAGE_H
#define NODE_STORAGE_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <vector>

#include "integrator.h"
#include "../types.h"
#include "../const.h"

enum RoadType {
    Main,
    HighStreet,
    SideStreet
};

struct StreamlineNode {
    DVector2 pos;
    int streamline_id;
    Direction dir;
};




// for clarity, type aliases for node_id (in wide node storage), 
using node_id = std::uint32_t;
static constexpr node_id NullNode = -1;

using Streamline = std::list<node_id>;

class Streamlines {
    private:
        std::unordered_map<Direction, std::vector<Streamline>> streamlines_;

    public:
        Streamlines();
        std::vector<Streamline>& get_streamlines(Direction dir);
        void clear();
        void add(Streamline& s, Direction dir);
        int size(Direction dir) const;
};



using qnode_id = node_id;                
constexpr qnode_id QNullNode = node_id(-1); 
                                            
struct QuadNode {
    Box<double> bbox;
    std::list<node_id> data;

    qnode_id children[4] = {QNullNode, QNullNode, QNullNode, QNullNode};

    char dirs; // bit mask saying what direction (Major/Minor) children have
    
    QuadNode(Box<double> bounding_box, char directions) :
        bbox(bounding_box),
        dirs(directions)
    {}
};


class Spatial {
private:
#ifdef SPATIAL_TEST
public:
#endif
    using iter = std::list<node_id>::iterator;

    struct BBoxQuery {
        const char& dirs;
        const bool& gather;
        Box<double> inner_bbox;
        std::list<node_id> harvest;
    };

    struct CircleQuery : BBoxQuery {
        const DVector2& centre;
        const double& radius;
        double radius2;
        Box<double> outer_bbox;
        CircleQuery(const char& dirs, const DVector2& centre, const double& radius, const bool& gather) : 
            BBoxQuery({dirs, gather}),
            centre(centre),
            radius(radius) 
        {
            radius2 = radius*radius;

            DVector2 circumscribed_diag = {radius, radius};
            DVector2 inscribed_diag = circumscribed_diag/M_SQRT2;

            outer_bbox = Box (
                centre - circumscribed_diag,
                centre + circumscribed_diag
            );


            inner_bbox = Box(
                centre - inscribed_diag,
                centre + inscribed_diag 
            );
        }
    };

    const std::vector<StreamlineNode>* all_nodes_;

    Box<double> dimensions_;

    qnode_id root_;
    std::vector<QuadNode> qnodes_;

    int max_depth_;
    int leaf_capacity_;

    const DVector2& node_to_pos(const node_id& id) const;
    char node_to_dir(const node_id& id) const;

    std::array<std::pair<char, std::list<node_id>>, 4> 
        partition(const Box<double>& bbox, std::list<node_id>& s);

    bool is_leaf(const qnode_id& id) const;

    // moves leaf data into subquadrant
    // void promote_leaf_data(qnode_id head_ptr);

    void subdivide(const qnode_id& head_ptr);

    // add leaf data onto existing node, updating bitmask
    void append_leaf_data(const qnode_id& leaf_ptr, const char& dirs, std::list<node_id>& data);

    void insert_rec(
        int depth, 
        const qnode_id& head_ptr,
        const char& dirs,
        std::list<node_id>& list
    );

    bool in_circle_rec(
        const qnode_id& head_ptr,
        CircleQuery& query
    ) const;

    bool in_bbox_rec(
        const qnode_id& head_ptr,
        BBoxQuery& query
    ) const;

public:
    Spatial(const std::vector<StreamlineNode>* all_nodes, Box<double> dims, int depth, int leaf_capacity);

    void insert_streamline(Streamline s, const char& dirs);

    void clear();
    void reset(Box<double> new_dims);
    
    bool has_nearby_point(const DVector2& centre, const double& radius, const char& dirs) const;
    std::list<node_id> nearby_points(const DVector2& centre, const double& radius, const char& dirs) const;
};


#endif
