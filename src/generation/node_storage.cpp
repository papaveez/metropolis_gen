#include <iostream>
#include <iterator>
#include <limits>

#include "node_storage.h"
#include "integrator.h"
#include "types.h"

constexpr std::string out[4] = {"TL", "TR", "BL", "BR"};

std::string q_to_str(const Quadrant q) {
    return out[q];
}

//  SECTION: Streamlines

Streamlines::Streamlines () {
     // make space for major, minor so it doesnt resize itself.
     streamlines_.reserve(2);
}

std::vector<Streamline>& Streamlines::get_streamlines(Direction dir) {
    return streamlines_[dir];
}
        
void Streamlines::clear() {
    for (auto & [_, v] : streamlines_) {
        v.clear();
    }
}

void Streamlines::add(Streamline& s, Direction dir) {
    streamlines_[dir].push_back(std::move(s));
}

int Streamlines::size(Direction dir) {
    return streamlines_[dir].size();
}

//  SECTION: Spatial


DVector2 Spatial::node_to_pos(const node_id& id) const {
    return (*all_nodes_)[id].pos;
}

    
bool Spatial::is_leaf(qnode_id id) {
    QuadNode& node = qnodes_[id];

    for (int i=0;i<4;i++) {
        if (node.children[i] != QNullNode) return false;
    }
    return true;
}

   
void Spatial::promote_leaf_data(qnode_id head_ptr) {
    if (qnodes_[head_ptr].data.size() == 0) return;

    // get position of first node in data range to decide which quadrant to promote to 
    DVector2 pos = node_to_pos(*(qnodes_[head_ptr].data.begin()));

    // corresponding quadrant in bounding box
    Quadrant q = qnodes_[head_ptr].bbox.which_quadrant(pos);
    Box<double> sub_bbox = qnodes_[head_ptr].bbox.get_quadrant(q);

    // new leaf node id
    qnode_id child_ptr = qnodes_.size();


    // create new leaf, pointing up to head, with sub_bbox
    qnodes_.emplace_back(sub_bbox, head_ptr, qnodes_[head_ptr].directions_bitmask);
    

    // exchange data 
    qnodes_[child_ptr].data = std::move(qnodes_[head_ptr].data);

    
    // set child pointer of head to the new node
    qnodes_[head_ptr].children[q] = child_ptr;
}


void Spatial::append_leaf_data(qnode_id leaf_ptr, Direction dir, std::list<node_id>& list, iter begin, iter end) {
    qnodes_[leaf_ptr].directions_bitmask |= (1 << dir);

    qnodes_[leaf_ptr].data.splice(qnodes_[leaf_ptr].data.end(), list, begin, end);
}


void Spatial::insert_rec(int depth, qnode_id head_ptr,
    Direction dir,
    std::list<node_id>& list, iter begin, iter end) 
{
    // TODO: append can occur if leaf is known to contain different direction
    if (is_leaf(head_ptr)) {
        auto next = begin;
        std::advance(next, 1);

        // breaking clause: one node left to insert (into empty leaf) or depth exceeds max depth
        // or one node left and 
        if (
            (next == end && (
                qnodes_[head_ptr].data.empty() || 
                !(qnodes_[head_ptr].directions_bitmask & (1<<dir)))
            )|| depth >= max_depth_) {
            append_leaf_data(head_ptr, dir, list, begin, end);
            return;
        }

        // otherwise, promote the leaf data to a subquadrant
        promote_leaf_data(head_ptr);
    }

    Box<double> bbox = qnodes_[head_ptr].bbox;
    DVector2 mid = middle(bbox.max, bbox.min);
    auto is_left = [&mid, this](const node_id& id) {return node_to_pos(id).x < mid.x; };
    auto is_top  = [&mid, this](const node_id& id) {return node_to_pos(id).y < mid.y; };


    // partition iterator into the quadrants: 
    // TL: [Begin, split_x_upper)
    // TR: [split_x_upper, split_y)
    // BL: [split_y, split_x_lower)
    // BR: [split_x_lower, end)
    iter split_y = std::partition(begin, end, is_top);
    iter split_x_upper = std::partition(begin, split_y, is_left);
    iter split_x_lower = std::partition(split_y, end, is_left);


    std::pair<iter, iter> partitions[4] = {
        {begin, split_x_upper},
        {split_x_upper, split_y},
        {split_y, split_x_lower},
        {split_x_lower, end}
    };
    
    ++depth;

    for (Quadrant q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
        auto [part_start, part_end] = partitions[q];

        if (part_start == part_end) continue;
        qnode_id child_ptr = qnodes_[head_ptr].children[q];

        if (child_ptr == QNullNode) {
            child_ptr = qnodes_.size();
            qnodes_.emplace_back(bbox.get_quadrant(q), head_ptr, dir);
            qnodes_[head_ptr].children[q] = child_ptr;
        }
        
        qnodes_[head_ptr].directions_bitmask |= (1<<dir);

        insert_rec(
            depth,
            child_ptr,
            dir,
            list,
            part_start,
            part_end
        );
    }

}


bool 
Spatial::in_circle_rec(qnode_id head_ptr, Box<double> bbox, 
    Direction dir, DVector2 centre, double radius2)
{
    if (is_leaf(head_ptr)) {
        for (node_id id : qnodes_[head_ptr].data) {
            StreamlineNode node = (*all_nodes_)[id];
            if (node.dir != dir) continue;
            DVector2 dist = centre - node.pos;
            if (dot_product(dist, dist) <= radius2) return true;
        }

        return false;
    }
    
    
    for (Quadrant q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
        qnode_id child_ptr = qnodes_[head_ptr].children[q];

        if (child_ptr == QNullNode) continue;
        QuadNode child_node = qnodes_[child_ptr];

        if (
            (child_node.bbox & bbox).is_empty()
            || !(child_node.directions_bitmask & (1<<dir))
        ) continue;

        if (in_circle_rec(child_ptr, bbox, dir, centre, radius2)) return true;
    }


    return false;
}


Spatial::Spatial(std::vector<StreamlineNode>* all_nodes, 
    Box<double> dims, int depth) :
    all_nodes_(all_nodes),
    dimensions_(dims),  
    max_depth_(depth) 
{
    root_ = 0;
    qnodes_.emplace_back(dimensions_, QNullNode, std::numeric_limits<char>::max());
}

void Spatial::clear() {
    qnodes_.clear();
    root_ = 0;
    qnodes_.emplace_back(dimensions_, QNullNode, std::numeric_limits<char>::max());
}

void Spatial::reset(Box<double> new_dims) {
    dimensions_ = new_dims;
    clear();
}

void Spatial::insert(std::list<node_id> list, Direction dir) { // copy list
    if (list.size() == 0) return;

    iter begin = list.begin();
    iter end   = list.end();

    if (*begin == *end) {
        std::advance(end, -1);
    }

    insert_rec(
        0, 
        root_,
        dir,
        list,
        begin,
        end
    );
}

bool Spatial::has_nearby_point(DVector2 centre, double radius, Direction dir) {
    assert(root_ != QNullNode);

    DVector2 circle_bbox_diagonal = {radius, radius};

    Box<double> circle_bbox = Box(
        centre - circle_bbox_diagonal, 
        centre + circle_bbox_diagonal
    ) & dimensions_;

    return 
        (circle_bbox.is_empty() || !(qnodes_[root_].directions_bitmask & (1<<dir)))
            ? false
            : in_circle_rec(
                root_, 
                circle_bbox, 
                dir, 
                centre, 
                radius*radius
            );
}


