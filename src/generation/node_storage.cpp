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


const DVector2& Spatial::node_to_pos(const node_id& id) const {
    return (*all_nodes_)[id].pos;
}

const Direction& Spatial::node_to_dir(const node_id& id) const {
    return (*all_nodes_)[id].dir;
}

// array of 4 intervals representing [TopLeft, TopRight, BottomLeft, BottomRight]
std::array<std::pair<char, std::list<node_id>>, 4> 
Spatial::partition(const Box<double>& bbox, std::list<node_id>& s) {
    DVector2 mid = middle(bbox.min, bbox.max);

    auto is_right  = [&mid, this](const node_id& id) { return node_to_pos(id).x > mid.x; };
    auto is_bottom = [&mid, this](const node_id& id) { return node_to_pos(id).y > mid.y; };

    std::array<std::pair<char, std::list<node_id>>, 4> out;

    // 4 way partition
    for (auto it=s.begin(); it!=s.end();) {
        iter curr = it;
        std::advance(it, 1);
        
        int q = is_right(*curr) + (is_bottom(*curr)<<1);
        out[q].first |= 1<<(node_to_dir(*curr));
        out[q].second.splice(out[q].second.end(), s, curr);
    }

    return out;
};

bool Spatial::is_leaf(qnode_id id) const {
    const QuadNode& root_node = qnodes_[id];
    for (int i=0; i<4;++i) {
        if (root_node.children[i] != QNullNode) return false;
    }
    return true;
}

void Spatial::subdivide(qnode_id head_ptr) {
    const Box<double> bbox = qnodes_[head_ptr].bbox;

    auto parts = partition(bbox, qnodes_[head_ptr].data);

    for (int i=0;i<4;++i) {
        std::list<node_id>& sublist = parts[i].second;
        if (sublist.empty()) continue;

        char& bitmask = parts[i].first;

        Quadrant q = static_cast<Quadrant>(i);
        Box<double> sub_bbox = qnodes_[head_ptr].bbox.get_quadrant(q);

        qnode_id child_ptr = qnodes_.size();
        qnodes_.emplace_back(sub_bbox, head_ptr, bitmask);
        qnodes_[child_ptr].data = std::move(sublist);

        qnodes_[head_ptr].children[q] = child_ptr;
    }
}

void Spatial::append_leaf_data(qnode_id leaf_ptr, char dir_bitmask, 
    std::list<node_id>& data) 
{
    qnodes_[leaf_ptr].directions_bitmask |= dir_bitmask;
    qnodes_[leaf_ptr].data.splice(qnodes_[leaf_ptr].data.end(), data);
}



static constexpr const char* q_strs[4] = {"TL", "TR", "BL", "BR"};

void Spatial::insert_rec(int depth, qnode_id head_ptr,
    char dir_bitmask,
    std::list<node_id>& list) 
{
    const char* t;
    switch (dir_bitmask) {
        case ((1<<Major) + (1<<Minor)):
            t = "Major & Minor";
            break;
        case ((1<<Major)):
            t = "Major";
            break;
        case (1<<Minor):
            t = "Minor";
            break;
        default:
            t = "???";
            break;
    }


    if (depth >= max_depth_) {
        append_leaf_data(head_ptr, dir_bitmask, list);
        return;
    } else if (is_leaf(head_ptr)) {
        if (qnodes_[head_ptr].data.size() + list.size() <= leaf_capacity_) {
            append_leaf_data(head_ptr, dir_bitmask, list);
            return;
        }
        subdivide(head_ptr);
    }

    qnodes_[head_ptr].directions_bitmask |= dir_bitmask;

    Box<double> bbox = qnodes_[head_ptr].bbox;
    auto parts = partition(bbox, list);
    
    ++depth;

    for (Quadrant q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
        auto& [sub_dir_bitmask, sublist] = parts[q];

        if (sublist.empty()) continue;

        qnode_id child_ptr = qnodes_[head_ptr].children[q];

        if (child_ptr == QNullNode) {
            child_ptr = qnodes_.size();
            qnodes_.emplace_back(bbox.get_quadrant(q), head_ptr, 0);
            qnodes_[head_ptr].children[q] = child_ptr;
        }
        

        insert_rec(
            depth,
            child_ptr,
            sub_dir_bitmask,
            sublist
        );
    }

}


bool 
Spatial::in_circle_rec(qnode_id head_ptr,
    const Box<double>& circubscribed, const Box<double>& inscribed,
    const char& dir_bitmask, const DVector2& centre, const double& radius2) const
{
    const QuadNode& qnode = qnodes_[head_ptr];

    if (!(qnode.directions_bitmask & dir_bitmask)) {
        return false;
    }

    if ((qnode.bbox | inscribed) == inscribed) {
        return in_bbox_rec(head_ptr, inscribed, dir_bitmask);
    }

    if (is_leaf(head_ptr)) {
        for (const node_id& id : qnode.data) {
            if (!((1<<node_to_dir(id)) & dir_bitmask)) continue;
            DVector2 dist = centre - node_to_pos(id);
            if (dot_product(dist, dist) <= radius2) return true;
        }
        return false;
    }

    for (Quadrant q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
        const qnode_id& child_ptr = qnodes_[head_ptr].children[q];

        if (child_ptr == QNullNode) continue;
        const QuadNode& child_node = qnodes_[child_ptr];

        if (
            (child_node.bbox & circubscribed).is_empty()
            || !(child_node.directions_bitmask & dir_bitmask)
        ) continue;

        if (in_circle_rec(child_ptr, circubscribed, inscribed, dir_bitmask, centre, radius2)) 
            return true;
    }

    return false;
}

bool 
Spatial::in_bbox_rec(qnode_id head_ptr, const Box<double>& bbox,
        const char& dir_bitmask) const {
    // assumes qnode.dir_bitmask & dir_bitmask != 0
    const QuadNode& qnode = qnodes_[head_ptr];
    // if the current quad's bbox is a subset of the search bbox 
    if ((bbox | qnode.bbox) == bbox) {
        // if there are any points here, return true
        // either the node has children (which in turn must have points)
        // or the node itself has data
        return !is_leaf(head_ptr) || !qnode.data.empty();
    }

    if ((bbox & qnode.bbox).is_empty()) {
        return false; // boxes dont intersect
    }

    // at this stage, the bboxes intersect, but not totally
    // we have to check each node
    if (is_leaf(head_ptr)) {
        for (const node_id& id : qnode.data) {
            if (bbox.contains(node_to_pos(id)) && ((1<<node_to_dir(id)) & dir_bitmask)) {
                return true;
            }
        }

        return false;
    }

    // otherwise, check children
    for (int i=0;i<4;++i) {
        // node is null
        if (qnode.children[i] == QNullNode) continue;
        // node has no children with dir
        if ((qnodes_[qnode.children[i]].directions_bitmask & dir_bitmask) == 0) continue;

        if (in_bbox_rec(qnode.children[i], bbox, dir_bitmask)) return true;
    }

    return false;
}

Spatial::Spatial(std::vector<StreamlineNode>* all_nodes, 
    Box<double> dims, int depth, int leaf_capacity) :
    all_nodes_(all_nodes),
    dimensions_(dims),  
    max_depth_(depth),
    leaf_capacity_(leaf_capacity)
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

void Spatial::insert_streamline(Streamline s, Direction dir) { // copy list
    if (s.size() == 0) return;

    iter begin = s.begin();
    iter end   = s.end();

    iter last_node = end;
    std::advance(last_node, -1);

    if (*begin == *last_node && s.size() > 2) { // if circle
        std::advance(end, -1);
    }

    char dir_bitmask = 1<<dir;
    insert_rec(
        0, 
        root_,
        dir_bitmask,
        s
    );
}

bool Spatial::has_nearby_point(DVector2 centre, double radius, Direction dir) const {
    assert(root_ != QNullNode);

    DVector2 circumscribed_diag = {radius, radius};
    DVector2 inscribed_diag = circumscribed_diag/M_SQRT2;

    Box<double> inscribed = Box(
        centre - circumscribed_diag, 
        centre + circumscribed_diag 
    ) & dimensions_;

    Box<double> circumscribed = Box(
        centre - inscribed_diag,
        centre + inscribed_diag 
    );

    char dir_bitmask = 1<<dir;
    if (circumscribed.is_empty() || !(qnodes_[root_].directions_bitmask & dir_bitmask)) {
        return false;
    }

    return in_circle_rec(root_, circumscribed, inscribed, dir_bitmask, centre, radius*radius);
}


