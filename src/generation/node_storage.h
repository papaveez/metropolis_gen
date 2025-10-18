#ifndef NODE_STORAGE_H
#define NODE_STORAGE_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <vector>

#include "types.h"
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


// we use as small as an address space as possible for cache optimisation
using addr = std::uint32_t;
static constexpr addr NullAddr = addr(-1); // special id for null.


// for clarity, type aliases for node_id (in wide node storage), 
using node_id = addr;
static constexpr node_id NullNodeId = NullAddr;

using Streamline = std::list<addr>;

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



using qnode_id = addr;                
constexpr qnode_id QNullNode = addr(-1); 
                                            
struct QuadNode {
    Box<double> bbox;
    std::list<node_id> data;

    
    qnode_id parent = QNullNode;
    qnode_id children[4] = {QNullNode, QNullNode, QNullNode, QNullNode};

    char directions_bitmask; // bit mask saying what direction (Major/Minor) children have
    
    QuadNode(Box<double> bounding_box, qnode_id parent_ptr, char directions) :
        bbox(bounding_box),
        parent(parent_ptr),
        directions_bitmask(directions)
    {}
};


class Spatial {
// private:
public: // for now
    using iter = std::list<node_id>::iterator;

    std::vector<StreamlineNode>* all_nodes_;

    Box<double> dimensions_;
    qnode_id root_;
    std::vector<QuadNode> qnodes_;
    int max_depth_;

    DVector2 node_to_pos(const node_id& id) const;

    bool is_leaf(qnode_id id);

    // moves leaf data into subquadrant
    void promote_leaf_data(qnode_id head_ptr);

    // add leaf data onto existing node, updating bitmask
    void append_leaf_data(qnode_id leaf_ptr, Direction dir, std::list<node_id>& list, iter begin, iter end);

    void insert_rec(
        int depth, 
        qnode_id head_ptr,
        Direction dir,
        std::list<node_id>& list, iter begin, iter end
    );

    bool in_circle_rec(
        qnode_id head_ptr, 
        Box<double> bbox, 
        Direction dir,
        DVector2 centre, 
        double radius2
    );

// public:
    Spatial(std::vector<StreamlineNode>* all_nodes, Box<double> dims, int depth);

    void insert(std::list<node_id> list, Direction dir);

    void clear();
    void reset(Box<double> new_dims);
    
    bool has_nearby_point(DVector2 centre, double radius, Direction dir);
};

#endif
