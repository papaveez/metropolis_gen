#include "node_storage.h"

#include <iterator>


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

char Spatial::node_to_dir(const node_id& id) const {
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
        out[q].first |= node_to_dir(*curr);
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

        char& dirs = parts[i].first;

        Quadrant q = static_cast<Quadrant>(i);
        Box<double> sub_bbox = qnodes_[head_ptr].bbox.get_quadrant(q);

        qnode_id child_ptr = qnodes_.size();
        qnodes_.emplace_back(sub_bbox, dirs);
        qnodes_[child_ptr].data = std::move(sublist);

        qnodes_[head_ptr].children[q] = child_ptr;
    }
}

void Spatial::append_leaf_data(qnode_id leaf_ptr, char dirs, 
    std::list<node_id>& data) 
{
    qnodes_[leaf_ptr].dirs |= dirs;
    qnodes_[leaf_ptr].data.splice(qnodes_[leaf_ptr].data.end(), data);
}

void Spatial::insert_rec(int depth, qnode_id head_ptr,
    char dirs,
    std::list<node_id>& list) 
{
    if (depth >= max_depth_) {
        append_leaf_data(head_ptr, dirs, list);
        return;
    } else if (is_leaf(head_ptr)) {
        if (qnodes_[head_ptr].data.size() + list.size() <= leaf_capacity_) {
            append_leaf_data(head_ptr, dirs, list);
            return;
        }
        subdivide(head_ptr);
    }

    qnodes_[head_ptr].dirs |= dirs;

    Box<double> bbox = qnodes_[head_ptr].bbox;
    auto parts = partition(bbox, list);
    
    ++depth;

    for (Quadrant q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
        auto& [sub_dirs, sublist] = parts[q];

        if (sublist.empty()) continue;

        qnode_id child_ptr = qnodes_[head_ptr].children[q];

        if (child_ptr == QNullNode) {
            child_ptr = qnodes_.size();
            qnodes_.emplace_back(bbox.get_quadrant(q), 0);
            qnodes_[head_ptr].children[q] = child_ptr;
        }
        

        insert_rec(
            depth,
            child_ptr,
            sub_dirs,
            sublist
        );
    }

}


bool 
Spatial::in_circle_rec(qnode_id head_ptr,
    const Box<double>& circubscribed, const Box<double>& inscribed,
    const char& dirs, const DVector2& centre, const double& radius2,
    std::optional<std::list<node_id>>& out) const
{
    const QuadNode& qnode = qnodes_[head_ptr];

    if (!(qnode.dirs & dirs)) {
        return false;
    }

    if ((qnode.bbox | inscribed) == inscribed) {
        return in_bbox_rec(head_ptr, inscribed, dirs, out);
    }

    if (is_leaf(head_ptr)) {
        bool has = false;
        for (const node_id& id : qnode.data) {
            if (!(node_to_dir(id) & dirs)) continue;
            DVector2 dist = centre - node_to_pos(id);
            if (dot_product(dist, dist) > radius2) continue;
            if (out.has_value()) {
                out.value().push_back(id);
                has = true;
            } else {
                return true;
            }
        }
        return has;
    }

    bool flag = false;
    for (Quadrant q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
        const qnode_id& child_ptr = qnodes_[head_ptr].children[q];

        if (child_ptr == QNullNode) continue;
        const QuadNode& child_node = qnodes_[child_ptr];

        if (
            (child_node.bbox & circubscribed).is_empty()
            || !(child_node.dirs & dirs)
        ) continue;

        if (in_circle_rec(child_ptr, circubscribed, inscribed, dirs, centre, radius2, out)) {
            if (out.has_value()) {
                flag = true;
                continue;
            }
            return true;
        }
    }

    return flag;
}

bool 
Spatial::in_bbox_rec(qnode_id head_ptr, const Box<double>& bbox,
        const char& dirs,
        std::optional<std::list<node_id>>& out
) const {
    const QuadNode& qnode = qnodes_[head_ptr];

    if ((bbox | qnode.bbox) == bbox) {
        if (out.has_value()) {
            if (!qnode.data.empty()) {
                for (auto& id : qnode.data) {
                    if (!(node_to_dir(id) & dirs)) continue; 
                    out.value().push_back(id);
                }
                return !is_leaf(head_ptr) || !qnode.data.empty();
            }
        } else {
            return !is_leaf(head_ptr) || !qnode.data.empty();
        }
    }

    if ((bbox & qnode.bbox).is_empty()) {
        return false; // boxes dont intersect
    }

    if (is_leaf(head_ptr)) {
        bool has = false;
        for (const node_id& id : qnode.data) {
            if (bbox.contains(node_to_pos(id)) && (node_to_dir(id) & dirs)) {
                if (out.has_value()) {
                    out.value().push_back(id);
                    has = true;
                } else {
                    return true;
                }
            }
        }
        return has;
    }

    // otherwise, check children
    bool flag = false;
    for (int i=0;i<4;++i) {
        // node is null
        if (qnode.children[i] == QNullNode) continue;
        // node has no children with dir
        if ((qnodes_[qnode.children[i]].dirs & dirs) == 0) continue;

        if (in_bbox_rec(qnode.children[i], bbox, dirs, out)) {
            if (out.has_value()) {
                flag = true;
                continue;
            }
            return true;
        }
    }

    return flag;
}


bool Spatial::has_nearby_point(DVector2 centre, double radius, char dirs, std::optional<std::list<node_id>>& out) const {
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

    if (circumscribed.is_empty() || !(qnodes_[root_].dirs & dirs)) {
        return false;
    }

    return in_circle_rec(root_, circumscribed, inscribed, dirs, centre, radius*radius, out);
}


Spatial::Spatial(std::vector<StreamlineNode>* all_nodes, 
    Box<double> dims, int depth, int leaf_capacity) :
    all_nodes_(all_nodes),
    dimensions_(dims),  
    max_depth_(depth),
    leaf_capacity_(leaf_capacity)
{
    root_ = 0;
    qnodes_.emplace_back(dimensions_, 0);
}

void Spatial::clear() {
    qnodes_.clear();
    root_ = 0;
    qnodes_.emplace_back(dimensions_, std::numeric_limits<char>::max());
}

void Spatial::reset(Box<double> new_dims) {
    dimensions_ = new_dims;
    clear();
}

// copies the list.
void Spatial::insert_streamline(Streamline s, char dir) {
    if (s.size() == 0) return;

    iter begin = s.begin();
    iter end   = s.end();

    iter last_node = end;
    std::advance(last_node, -1);

    // if the streamline is a circle
    if (*begin == *last_node && s.size() > 2) {
        std::advance(end, -1);
    }

    insert_rec(
        0, 
        root_,
        dir,
        s
    );
}



bool Spatial::has_nearby_point(DVector2 centre, double radius, char dirs) const {
    std::optional<std::list<node_id>> out;
    return has_nearby_point(centre, radius, dirs, out);
}

std::list<node_id> Spatial::nearby_points(DVector2 centre, double radius, char dirs) const {
    std::optional<std::list<node_id>> out = std::list<node_id>{};
    has_nearby_point(centre, radius, dirs, out);
    return out.value();
}

