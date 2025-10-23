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

int Streamlines::size(Direction dir) const {
    if (!streamlines_.contains(dir)) return 0;
    return streamlines_.at(dir).size();
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


bool Spatial::is_leaf(const qnode_id& id) const {
    const QuadNode& root_node = qnodes_[id];
    for (int i=0; i<4;++i) {
        if (root_node.children[i] != QNullNode) return false;
    }
    return true;

}

void Spatial::subdivide(const qnode_id& head_ptr) {
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


void Spatial::append_leaf_data(const qnode_id& leaf_ptr, const char& dirs, 
    std::list<node_id>& data) 
{
    qnodes_[leaf_ptr].dirs |= dirs;
    qnodes_[leaf_ptr].data.splice(qnodes_[leaf_ptr].data.end(), data);
}


void Spatial::insert_rec(int depth, const qnode_id& head_ptr,
    const char& dirs,
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

    for (int q=0; q<4;++q) {
        auto& [sub_dirs, sublist] = parts[q];

        if (sublist.empty()) continue;

        qnode_id child_ptr = qnodes_[head_ptr].children[q];

        if (child_ptr == QNullNode) {
            child_ptr = qnodes_.size();
            qnodes_.emplace_back(bbox.get_quadrant((Quadrant) q), 0);
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
Spatial::in_circle_rec(const qnode_id& head_ptr, CircleQuery& query) const {
    const QuadNode& qnode = qnodes_[head_ptr];

    // terminate if bboxes dont intersect, or no dirs to explore
    if (!(qnode.dirs & query.dirs) || (query.outer_bbox & qnode.bbox).is_empty()) {
        return false;
    }

    // if query.inner_bbox ⊆ qnode.bbox
    if ((qnode.bbox | query.inner_bbox) == query.inner_bbox) {
        return in_bbox_rec(head_ptr, query);
    }

    bool flag = false;

    if (is_leaf(head_ptr)) {
        for (const node_id& id : qnode.data) {
            if (!(node_to_dir(id) & query.dirs)) continue;

            DVector2 dist = query.centre - node_to_pos(id);
            if (dot_product(dist, dist) > query.radius2) continue;

            if (query.gather) {
                query.harvest.push_back(id);
                flag = true;
            } else {
                return true;
            }
        }
        return flag;
    }


    for (int q=0; q<4; ++q) {
        const qnode_id& child_ptr = qnode.children[q];

        if (child_ptr == QNullNode) continue;
        const QuadNode& child_node = qnodes_[child_ptr];


        if (in_circle_rec(child_ptr, query)) {
            if (query.gather) {
                flag = true;
            } else {
                return true;
            }
        }
    }

    return flag;
}


bool 
Spatial::in_bbox_rec(const qnode_id& head_ptr, BBoxQuery& query) const {
    const QuadNode& qnode = qnodes_[head_ptr];

    if ((query.inner_bbox & qnode.bbox).is_empty() || !(qnode.dirs & query.dirs)) {
        return false; // boxes dont intersect
    }


    if ((query.inner_bbox | qnode.bbox) == query.inner_bbox) {
        if (query.gather && !qnode.data.empty()) {
            for (auto& id : qnode.data) {
                if (!(node_to_dir(id) & query.dirs)) continue; 

                query.harvest.push_back(id);
            }

            return !is_leaf(head_ptr) || !qnode.data.empty();
        } else if (!query.gather) {
            return !is_leaf(head_ptr) || !qnode.data.empty();
        }
    }


    bool flag = false;

    if (is_leaf(head_ptr)) {
        for (const node_id& id : qnode.data) {
            if (query.inner_bbox.contains(node_to_pos(id)) && (node_to_dir(id) & query.dirs)) {
                if (query.gather) {
                    query.harvest.push_back(id);
                    flag = true;
                } else {
                    return true;
                }
            }
        }
        return flag;
    }

    // otherwise, check children
    for (int i=0;i<4;++i) {
        const qnode_id& child_ptr = qnode.children[i];
        // node is null
        if (child_ptr == QNullNode) continue;

        if (in_bbox_rec(child_ptr, query)) {
            if (query.gather) {
                flag = true;
            } else {
                return true;
            }
        }
    }

    return flag;
}


Spatial::Spatial(const std::vector<StreamlineNode>* all_nodes, 
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
void Spatial::insert_streamline(Streamline s, const char& dir) {
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


bool Spatial::has_nearby_point(const DVector2& centre, const double& radius, const char& dirs) const {
    CircleQuery query(dirs, centre, radius, false);
    return in_circle_rec(root_, query);
}


std::list<node_id> Spatial::nearby_points(const DVector2& centre, const double& radius, const char& dirs) const {
    CircleQuery query(dirs, centre, radius, true);
    in_circle_rec(root_, query);
    return query.harvest;
}
