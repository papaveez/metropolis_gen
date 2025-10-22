#include "generator.h"

#include <cassert>
#include <iostream>
#include <iterator>
#include <list>


GeneratorParameters::GeneratorParameters(
        int max_seed_retries,
        int max_integration_iterations,
        double d_sep,
        double d_test,
        double d_circle,
        double dl,
        double d_lookahead,
        double theta_max,
        double epsilon,
        double node_sep
        ) :
    max_seed_retries(max_seed_retries),
    max_integration_iterations(max_integration_iterations),
    d_sep(d_sep),
    d_sep2(d_sep*d_sep),
    d_test(d_test),
    d_test2(d_test*d_test),
    d_circle(d_circle),
    d_circle2(d_circle*d_circle),
    dl(dl),
    dl2(dl*dl),
    d_lookahead(d_lookahead),
    theta_max(theta_max),
    epsilon(epsilon),
    node_sep(node_sep),
    node_sep2(node_sep*node_sep)
{}


//  SECTION: RoadNetworkGenerator

bool RoadNetworkGenerator::in_bounds(const DVector2& p) const {
    return viewport_.contains(p);
}


void RoadNetworkGenerator::add_candidate_seed(node_id id, Direction dir) {
    DVector2 seed = nodes_[id].pos;
    seeds_[dir].push(seed);
}


std::optional<DVector2> 
RoadNetworkGenerator::get_seed(RoadType road, Direction dir) {
    seed_queue& candidate_queue = seeds_[dir];

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


        if (!spatial_.has_nearby_point(seed, params_.at(road).d_sep, dir)) {
            return seed;
        } 
    }
    return {};
}


void RoadNetworkGenerator::extend_streamline(
    Integration& res,
    const RoadType& road, 
    const Direction& dir
) const {
    if (res.status != Continue) {
        res.status = Abort;
        return;
    };

    DVector2 delta = integrator_->integrate(
        res.integration_front, 
        dir, 
        params_.at(road).dl
    );

    if (res.negate) 
        delta = delta*-1.0;

    if (res.delta.has_value() && dot_product(res.delta.value(), delta) < 0) {
        delta = delta*-1.0;
    }

    if (dot_product(delta, delta) < 0.01) {
        res.status = Abort;
        return;
    }

    res.integration_front = res.integration_front + delta;
    res.delta = delta;
    if (!in_bounds(res.integration_front)) {
        res.status = Abort;
        return;
    }

    res.status = Continue;
    if (spatial_.has_nearby_point(res.integration_front, params_.at(road).d_test, dir)) {
        res.status = Terminate;
    }
}


std::optional<std::list<DVector2>>
RoadNetworkGenerator::generate_streamline(RoadType road, DVector2 seed_point, Direction dir) {
    Integration forward  (seed_point, false);
    Integration backward (seed_point, true );

    // circle logic
    bool points_diverged = false;
    bool join = false; // flag to join endpoints (in a circular road)

    int count = 0;

    while(count<params_.at(road).max_integration_iterations) {
        extend_streamline(forward,  road, dir);
        extend_streamline(backward, road, dir);

        if (backward.status == Abort && forward.status == Abort)
            break;

        if (forward.status != Abort) {
            forward.points.push_back(forward.integration_front);
            count++;
        }

        if (backward.status != Abort) {
            backward.points.push_front(backward.integration_front);
            count++;
        }


        DVector2 ends_diff = forward.points.back() - backward.points.front();
        double sep2 = dot_product(ends_diff, ends_diff);

        if (points_diverged && sep2 < params_.at(road).d_circle2) {
            join = true;
            break;
        } else if (!points_diverged && sep2 > params_.at(road).d_circle2) {
            points_diverged = true;
        }
    }

    backward.points.pop_back(); // remove shared start point

    if (join) {
        forward.points.push_back(backward.points.back()); // join up streamlines
    }

    std::list<DVector2> result;

    result.splice(result.end(), backward.points);
    result.splice(result.end(), forward.points);

    if (result.size() < 5) return {};

    return result;
}


int RoadNetworkGenerator::generate_streamlines(RoadType road) {
    Direction dir = Major;

    std::optional<DVector2> seed = get_seed(road, dir);
    int k = 0;
    while (seed.has_value()) {
        std::optional<std::list<DVector2>> new_streamline
            = generate_streamline(road, seed.value(), dir);

        if (new_streamline.has_value()) {
            simplify_streamline(road, new_streamline.value());
            push_streamline(road, new_streamline.value(), dir);
            k += 1;
            dir = flip(dir);
        }
        
        seed = get_seed(road, dir);
    };

    return k;
}


void RoadNetworkGenerator::simplify_streamline(RoadType road, std::list<DVector2>& points) const {
    assert(params_.at(road).epsilon > 0.0);
    douglas_peucker(params_.at(road).epsilon, params_.at(road).node_sep2, points, points.begin(), points.end());
}


void RoadNetworkGenerator::douglas_peucker(const double& epsilon, const double& min_sep2, 
        std::list<DVector2>& points,
        std::list<DVector2>::iterator begin, std::list<DVector2>::iterator end) const 
{
    // must be 3> elements 
    int count = 0;
    for (auto it=begin;it!=end && count < 3;++it, ++count);
    if (count < 3) return;

    auto last_elem = std::prev(end);

    const DVector2& first_pos = *begin;
    const DVector2& last_pos  = *last_elem;

    double d_max = 0.0;
    std::list<DVector2>::iterator index;


    for (auto it=std::next(begin); it != last_elem; ++it) {
        double d = perpendicular_distance(*it, first_pos, last_pos);

        if (d > d_max) {
            d_max = d;
            index = it;
        }
    }

    if (d_max > epsilon) {
        douglas_peucker(epsilon, min_sep2, points, begin, std::next(index));
        douglas_peucker(epsilon, min_sep2, points, index, end);
    } else {
        for (auto it=std::next(begin); it!=std::prev(end);) {
            auto next = std::next(it);

            auto prev = std::prev(it);
            DVector2 diff = *it - *prev;
            double dist2 = dot_product(diff, diff);

            if (dist2 < min_sep2) points.erase(it);

            it = next;
        }
    }
}


void RoadNetworkGenerator::push_streamline(RoadType road, std::list<DVector2>& points, Direction dir) {
    int new_streamline_id = streamlines_[road].size(dir);
    int new_node_id = node_count();
    Streamline out;
    for (const DVector2& vec : points) {
        nodes_.push_back(StreamlineNode{
            vec,
            new_streamline_id,
            dir
        });
        out.push_back(new_node_id);
        ++new_node_id;
    }

    spatial_.insert_streamline(out, dir);
   
    if (out.front() != out.back()) {
        add_candidate_seed(out.front(), flip(dir));
        add_candidate_seed(out.back(), flip(dir));
    }

    streamlines_[road].add(out, dir);
}


RoadNetworkGenerator::RoadNetworkGenerator(
        std::unique_ptr<NumericalFieldIntegrator>& integrator,
        std::unordered_map<RoadType, GeneratorParameters> parameters,
        Box<double> viewport
        ) :
    viewport_(viewport),
    integrator_(std::move(integrator)),
    nodes_(std::vector<StreamlineNode>{}),
    params_(parameters),
    dist_(0.0, 1.0),
    spatial_(Spatial(&nodes_, viewport_, kQuadTreeDepth, kQuadTreeLeafCapacity))
{
    road_types_.reserve(parameters.size());
    streamlines_.reserve(parameters.size());
    seeds_.reserve(2); // major, minor
    

    for (auto& [key, params] : params_) {
        params.d_test = std::min(params.d_test, params.d_sep);
        road_types_.push_back(key);
    }
}


const std::vector<RoadType>&
RoadNetworkGenerator::get_road_types() const {
    return road_types_;
}


const std::unordered_map<RoadType, GeneratorParameters>&
RoadNetworkGenerator::get_parameters() const {
    return params_;
}


const StreamlineNode&
RoadNetworkGenerator::get_node(node_id i) const {
    assert(0 <= i && i < nodes_.size());
    return nodes_[i];
}


const std::vector<Streamline>& 
RoadNetworkGenerator::get_streamlines(RoadType road, Direction dir) {
    return streamlines_[road].get_streamlines(dir);
}


int RoadNetworkGenerator::node_count() const {
    return nodes_.size();
}


void RoadNetworkGenerator::set_viewport(Box<double> new_viewport) {
    viewport_ = std::move(new_viewport);
}


bool RoadNetworkGenerator::generation_step(RoadType road, Direction dir) {
    std::optional<DVector2> seed = get_seed(road, dir);
    if (!seed.has_value()) {
        return false;
    }


    std::optional<std::list<DVector2>> new_streamline
        = generate_streamline(road, seed.value(), dir);

    if (!new_streamline.has_value()) {
        return false;
    }

    push_streamline(road, new_streamline.value(), dir);
    return true;
}


void RoadNetworkGenerator::generate() {
    clear();

    spatial_.reset(viewport_);

    std::sort(road_types_.begin(), road_types_.end());

    for (auto r : road_types_) {
        generate_streamlines(r);
    }

    std::cout << "node count: " << node_count() << std::endl;
}


void RoadNetworkGenerator::clear() {
    // empty everything
    seeds_.clear();
    seeds_.reserve(2);

    nodes_.clear();

    streamlines_.clear();
    spatial_.clear();
}
