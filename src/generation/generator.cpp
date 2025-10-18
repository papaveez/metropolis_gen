#include <cassert>
#include <iostream>
#include <list>
#include <memory>
#include <unordered_map>

#include "generator.h"
#include "node_storage.h"
#include "types.h"
#include "integrator.h"


GeneratorParameters::GeneratorParameters(
        int _max_seed_retries,
        int _max_integration_iterations,
        double _d_sep,
        double _d_test,
        double _d_circle,
        double _dl,
        double _d_lookahead,
        double _theta_max,
        double _epsilon,
        double _noise_size,
        double _noise_angle
        ) :
    max_seed_retries(_max_seed_retries),
    max_integration_iterations(_max_integration_iterations),
    d_sep(_d_sep),
    d_sep2(_d_sep*_d_sep),
    d_test(_d_test),
    d_test2(_d_test*_d_test),
    d_circle(_d_circle),
    d_circle2(_d_circle*_d_circle),
    dl(_dl),
    dl2(_dl*_dl),
    d_lookahead(_d_lookahead),
    theta_max(_theta_max),
    epsilon(_epsilon),
    noise_size(_noise_size),
    noise_angle(_noise_angle)
{}


//  SECTION: RoadNetworkGenerator


RoadNetworkGenerator::RoadNetworkGenerator(
        std::unique_ptr<NumericalFieldIntegrator>& integrator,
        std::unordered_map<RoadType, GeneratorParameters> parameters,
        Box<double> viewport
        ) :
    viewport_(viewport),
    integrator_(std::move(integrator)),
    nodes_(std::vector<StreamlineNode>{}),
    spatial_(Spatial(&nodes_, viewport_, kQuadTreeDepth)),
    params_(parameters),
    dist_(0.0, 1.0) 
{
    road_types_.reserve(parameters.size());
    streamlines_.reserve(parameters.size());
    seeds_.reserve(2); // major, minor
    

    for (auto& [key, params] : params_) {
        params.d_test = std::min(params.d_test, params.d_sep);
        road_types_.push_back(key);
        // streamlines_[key] = Streamlines();
    }

}

bool RoadNetworkGenerator::in_bounds(DVector2& p) const {
    return viewport_.contains(p);
}


void RoadNetworkGenerator::set_viewport(Box<double> new_viewport) {
    viewport_ = std::move(new_viewport);
}



std::unordered_map<RoadType, GeneratorParameters> 
RoadNetworkGenerator::get_parameters() const {
    return params_;
}


std::vector<RoadType> 
RoadNetworkGenerator::get_road_types() const {
    return road_types_;
}

std::vector<Streamline>& 
RoadNetworkGenerator::get_streamlines(RoadType road, Direction dir) {
    return streamlines_[road].get_streamlines(dir);
}


std::optional<StreamlineNode> 
RoadNetworkGenerator::get_node(node_id i) const {
    if (0 <= i && i < nodes_.size()) {
        return nodes_[i];
    }
    return {};
}


std::optional<DVector2> 
RoadNetworkGenerator::get_seed(RoadType road, Direction dir) {
    seed_queue candidate_queue = seeds_[dir];

    DVector2 seed;
    while (!candidate_queue.empty()) {
        DVector2 seed = candidate_queue.front();
        candidate_queue.pop();
        if (!spatial_.has_nearby_point(seed, params_.at(road).d_sep, dir)) {
            return seed;
        } 
    }


    for (int count=0; count<params_.at(road).max_seed_retries; count++) {
        seed = DVector2 {
            dist_(gen_)*viewport_.width()  + viewport_.min.x,
            dist_(gen_)*viewport_.height() + viewport_.min.y

        };

        assert(viewport_.contains(seed));


        if (!spatial_.has_nearby_point(seed, params_.at(road).d_sep, dir)) {
            return seed;
        } 
    }
    return {};
}

void RoadNetworkGenerator::add_candidate_seed(node_id id, Direction dir) {
    DVector2 seed = nodes_[id].pos;
    seeds_[dir].push(seed);

}

node_id RoadNetworkGenerator::joining_candidate(RoadType road, node_id id, DVector2 initial_direction) {
    // DVector2 pos = nodes_[node_id].pos;
    // int streamline_id = nodes_[node_id].streamline_id;
    //
    // int best_node_id = -1;
    // double min_dist2 = std::numeric_limits<double>::infinity();
    //
    // for (auto candidate_id : spatial_.points_in_radius(pos, parameters_.at(road).d_lookahead)) {
    //     if (candidate_id == node_id) continue;
    //     // else if (all_nodes[candidate_id].streamline_id == streamline_id) continue;
    //
    //     DVector2 candidate_pos = nodes_[candidate_id].pos;
    //
    //     DVector2 join_vector = candidate_pos - pos;
    //     if (dot_product(join_vector, initial_direction) < 0) {
    //         continue; // vectors are in opposite directions
    //     }
    //
    //     double dist2 = dot_product(join_vector, join_vector);
    //
    //     if (dist2 < 2*parameters_.at(road).dl2) {
    //         best_node_id = candidate_id;
    //         break;
    //     }
    //
    //
    //     double angle = std::abs(vector_angle(join_vector, initial_direction));
    //     if (angle < parameters_.at(road).theta_max && dist2 < min_dist2) {
    //         min_dist2 = dist2;
    //         best_node_id = candidate_id;
    //     }
    // }
    //
    // return best_node_id;
    return -1;
}

void RoadNetworkGenerator::join_streamlines(Direction dir) {
    // for (auto road : road_types_) {
    //     std::vector<Streamline>& sls = streamlines_[road].get_streamlines(dir);
    //
    //     for (auto& streamline : sls) {
    //         if (streamline.front() == streamline.back()) { // streamline not dangling.
    //             continue;
    //         }
    //
    //         DVector2 start_point = nodes_[streamline.front()].pos;
    //         auto it = streamline.begin();
    //         for (int i=0;i<5;i++) it++;
    //         DVector2 start_direction = nodes_[*it].pos - start_point;
    //
    //         DVector2 end_point = nodes_[streamline.back()].pos;
    //         it = streamline.end();
    //         for (int i=0;i<6;i++) it--;
    //
    //
    //         DVector2 end_direction = end_point - nodes_[*it].pos;
    //
    //         int best_start_join = 
    //             joining_candidate(road, streamline.front(), start_direction);
    //
    //         int best_end_join = 
    //             joining_candidate(road, streamline.back(), end_direction);
    //
    //
    //         if (best_start_join != -1) {
    //             streamline.push_front(best_start_join);
    //         }
    //
    //         if (best_end_join != -1) {
    //             streamline.push_back(best_end_join);
    //         }
    //
    //     }
    // }

}


void RoadNetworkGenerator::join_streamlines() {
    join_streamlines(Major);
    join_streamlines(Minor);
}

void RoadNetworkGenerator::douglas_peucker(RoadType road, Streamline& streamline) {
    if (streamline.size() < 3) return;

    auto first = streamline.begin();
    auto last = std::prev(streamline.end());

    DVector2 first_pos = nodes_[*first].pos;
    DVector2 last_pos = nodes_[*last].pos;

    double d_max = 0.0;
    auto index = first;

    for (auto it = std::next(first); it != last; ++it) {
        DVector2 pos = nodes_[*it].pos;
        double d = perpendicular_distance(pos, first_pos, last_pos);

        if (d > d_max) {
            d_max = d;
            index = it;
        }
    }


    if (d_max > params_.at(road).epsilon) {
        Streamline first_half(streamline.begin(), std::next(index));
        Streamline second_half(index, streamline.end());

        douglas_peucker(road, first_half);
        douglas_peucker(road, second_half);

        first_half.pop_back();
        first_half.splice(first_half.end(), second_half);

        streamline.swap(first_half);
    } else {
        auto it = std::next(first);
        streamline.erase(it, last);
    }
}

void RoadNetworkGenerator::simplify_streamlines() {
    std::vector<Direction> dirs = {Major, Minor};

    for (auto road : get_road_types()) {
        for (auto dir : dirs) {
            std::vector<Streamline>& sls = get_streamlines(road, dir);
            for (auto& sl : sls) {
                douglas_peucker(road, sl);
            }
        }
    }

}


void RoadNetworkGenerator::extend_streamline(RoadType road, Integration& i, Direction dir) {
    extend_streamline(road, i, dir, false);
}

void RoadNetworkGenerator::extend_streamline(RoadType road, Integration& i, Direction dir, bool negate) {
    if (i.status != Continue) {
        i.status = Abort;
        return;
    }

    DVector2 delta = integrator_->integrate(
        i.integration_front,
        dir,
        params_.at(road).dl,
        params_.at(road).noise_size,
        params_.at(road).noise_angle
    );

    if (dot_product(delta, delta) < 0.01) {
        i.status = Abort;
        return;
    }

    if (negate || (i.last_delta && dot_product(delta, i.last_delta.value()) < 0)) {
        delta = delta*-1.0;
    }

    DVector2 new_front = i.integration_front + delta;

    if (!in_bounds(new_front)) {
        i.status = Abort;
        return;
    } 

    if (spatial_.has_nearby_point(new_front, params_.at(road).d_test, dir)) {
        i.status = Terminate;
    } 

    i.last_delta = delta;
    i.integration_front = new_front;
}



void RoadNetworkGenerator::push_streamline(RoadType road, std::vector<StreamlineNode>& new_nodes, Streamline& streamline, Direction dir) {
    // store new nodes 
    nodes_.insert(nodes_.end(), new_nodes.begin(), new_nodes.end());

    // store nodes in spatial lookup
    spatial_.insert(streamline, dir);
   
    // if the streamline has dangling endpoints (non cyclical)
    if (streamline.front() != streamline.back()) {
        add_candidate_seed(streamline.front(), flip(dir));
        add_candidate_seed(streamline.back(), flip(dir));
    }

    // push the streamline
    streamlines_[road].add(streamline, dir);
}


bool RoadNetworkGenerator::generate_streamline(RoadType road, DVector2 seed_point, Direction dir) {
    int new_streamline_id = streamlines_[road].get_streamlines(dir).size();
    std::vector<StreamlineNode> new_nodes = {{
        seed_point,
        new_streamline_id,
        dir
    }};

    int offset = nodes_.size();
    node_id curr_node_id = nodes_.size();
    // list of new node ids to add. they are both initialised with the same starting point.
    
    Streamline forward_streamline = std::list<node_id>{curr_node_id};
    Streamline backward_streamline = std::list<node_id>{curr_node_id};

    Integration forward_integration = {{}, Continue, seed_point};
    Integration backward_integration = forward_integration;

    extend_streamline(road, forward_integration, dir);
    extend_streamline(road, backward_integration, dir, true);

    int count = 0;

    // circle logic
    bool points_diverged = false;
    bool join = false; // flag to join endpoints (in a circular road)

    for (int count=0;count<params_.at(road).max_integration_iterations;count++) {
        // integration loigc
        extend_streamline(road, forward_integration, dir);
        extend_streamline(road, backward_integration, dir);

        if (forward_integration.status == Abort && backward_integration.status == Abort) {
            break;
        } 

        if (forward_integration.status != Abort) {
            new_nodes.push_back({forward_integration.integration_front, new_streamline_id, dir});
            ++curr_node_id;
            forward_streamline.push_back(curr_node_id);
        } 

        if (backward_integration.status != Abort) {
            new_nodes.push_back({backward_integration.integration_front, new_streamline_id, dir});
            ++curr_node_id;
            backward_streamline.push_front(curr_node_id);
        }

        DVector2 diff = new_nodes[forward_streamline.back() - offset].pos - new_nodes[backward_streamline.front() - offset].pos;
        double sep2 = dot_product(diff, diff);
        if (points_diverged && sep2 < params_.at(road).d_circle2) {
            join = true;
            forward_integration.status = Abort;
            backward_integration.status = Abort;
            break;
        } else if (!points_diverged && sep2 > params_.at(road).d_circle2) {
            points_diverged = true;
        }
    }

    backward_streamline.pop_back();
    assert(backward_streamline.size() + forward_streamline.size() == new_nodes.size());

    if (join) {
        forward_streamline.push_back(backward_streamline.back());
    }

    backward_streamline.splice(backward_streamline.end(), forward_streamline);

    if (backward_streamline.size() < 5) return false;

    push_streamline(road, new_nodes, backward_streamline, dir);
    return true;
}


int RoadNetworkGenerator::generate_streamlines(RoadType road) {
    Direction dir = Major;

    std::optional<DVector2> seed = get_seed(road, dir);
    int k = 0;
    while (seed) {
        std::cout << "Generating seed: " << k << std::endl;
        std::cout << seed.value() << std::endl;
        if (generate_streamline(road, seed.value(), dir)) {
            k += 1;
            dir = flip(dir);
        }
        seed = get_seed(road, dir);
    };

    return k;
}

void RoadNetworkGenerator::generate() {
    clear();

    spatial_.reset(viewport_);

    std::sort(road_types_.begin(), road_types_.end());

    for (auto r : road_types_) {
        generate_streamlines(r);
    }
}


void RoadNetworkGenerator::clear() {
    // empty everything
    seeds_.clear();
    seeds_.reserve(2);

    nodes_.clear();

    streamlines_.clear();
    spatial_.clear();
}
