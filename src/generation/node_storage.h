#ifndef NODE_STORAGE_H
#define NODE_STORAGE_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <vector>

#include "../types.h"
#include "../const.h"
#include "integrator.h"

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
        int size(Direction dir);
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

    std::vector<StreamlineNode>* all_nodes_;

    Box<double> dimensions_;

    qnode_id root_;
    std::vector<QuadNode> qnodes_;

    int max_depth_;
    int leaf_capacity_;

    const DVector2& node_to_pos(const node_id& id) const;
    char node_to_dir(const node_id& id) const;

    std::array<std::pair<char, std::list<node_id>>, 4> 
        partition(const Box<double>& bbox, std::list<node_id>& s);

    bool is_leaf(qnode_id id) const;

    // moves leaf data into subquadrant
    // void promote_leaf_data(qnode_id head_ptr);

    void subdivide(qnode_id head_ptr);

    // add leaf data onto existing node, updating bitmask
    void append_leaf_data(qnode_id leaf_ptr, char dirs, std::list<node_id>& data);

    void insert_rec(
        int depth, 
        qnode_id head_ptr,
        char dirs,
        std::list<node_id>& list
    );

    bool in_circle_rec(
        qnode_id head_ptr, 
        const Box<double>& circumscribed, 
        const Box<double>& inscribed, 
        const char& dirs,
        const DVector2& centre, 
        const double& radius2,
        std::optional<std::list<node_id>>& out 
    ) const;

    bool in_bbox_rec(
        qnode_id head_ptr,
        const Box<double>& bbox,
        const char& dirs,
        std::optional<std::list<node_id>>& out
    ) const;

    bool has_nearby_point(DVector2 centre, double radius, char dirs, std::optional<std::list<node_id>>& out) const;

public:
    Spatial(std::vector<StreamlineNode>* all_nodes, Box<double> dims, int depth, int leaf_capacity);

    void insert_streamline(Streamline s, char dirs);

    void clear();
    void reset(Box<double> new_dims);
    
    bool has_nearby_point(DVector2 centre, double radius, char dirs) const;
    std::list<node_id> nearby_points(DVector2 centre, double radius, char dirs) const;
};

#endif
