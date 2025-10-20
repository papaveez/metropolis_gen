#ifndef GENERATOR_H
#define GENERATOR_H

#include <memory>
#include <queue>
#include <random>
#include <unordered_map>
#include <vector>
#include "types.h"
#include "integrator.h"
#include "node_storage.h"

#include "../const.h"


enum FrontCheck {
    Continue,
    Terminate,
    Abort
};

struct Integration {
    std::optional<DVector2> last_delta;
    FrontCheck status;
    DVector2 integration_front;
};


struct GeneratorParameters {
    int max_seed_retries;
    int max_integration_iterations;
    double d_sep;
    double d_sep2;
    double d_test;
    double d_test2;
    double d_circle;
    double d_circle2;
    double dl;
    double dl2;
    double d_lookahead;
    double theta_max; // maximum streamline joining angle
    double epsilon;
    double noise_size;
    double noise_angle;


    GeneratorParameters(
        int max_seed_retries,
        int max_integration_iterations,
        double d_sep,
        double d_test,
        double d_circle,
        double dl,
        double d_lookahead,
        double theta_max,
        double epsilon,
        double noise_size,
        double noise_angle
    );
};


class RoadNetworkGenerator {
    private:
        using seed_queue = std::queue<DVector2>;
        static constexpr int kQuadTreeDepth = 10; // area of 3 pixels at 1920x1080
        static constexpr int kQuadTreeLeafCapacity = 10;
        Box<double> viewport_;


        std::unique_ptr<NumericalFieldIntegrator> integrator_;
        std::vector<RoadType> road_types_;
        std::unordered_map<RoadType, GeneratorParameters> params_;
        std::unordered_map<Direction, seed_queue> seeds_;
        std::default_random_engine gen_;
        std::uniform_real_distribution<double> dist_;
        std::vector<StreamlineNode> nodes_;
        std::unordered_map<RoadType, Streamlines> streamlines_;


        bool in_bounds(DVector2& p) const;

        void add_candidate_seed(node_id id, Direction dir);

        std::optional<DVector2> get_seed(RoadType road, Direction dir);

        void extend_streamline(RoadType road, Integration& i, Direction dir, bool negate);
        void extend_streamline(RoadType road, Integration& i, Direction dir);

#ifdef SPATIAL_TEST
    public:
#endif
        void push_streamline(RoadType road, std::vector<StreamlineNode>& new_nodes, Streamline& streamline, Direction dir);
#ifdef SPATIAL_TEST
    private:
#endif

        node_id joining_candidate(RoadType road, node_id id, DVector2 initial_direction);

        void douglas_peucker(RoadType road, Streamline& streamline);
        // void simplify_streamline(int streamline_id, Direction dir);

    public:
 
        Spatial spatial_;
        RoadNetworkGenerator(
                std::unique_ptr<NumericalFieldIntegrator>& integrator,
                std::unordered_map<RoadType, GeneratorParameters>,
                Box<double> viewport
            );

        void set_viewport(Box<double> new_viewport);

        std::vector<RoadType> get_road_types() const;
        std::vector<Streamline>& get_streamlines(RoadType road, Direction dir);
        std::unordered_map<RoadType, GeneratorParameters> get_parameters() const;

        int generate_streamlines(RoadType road);
        void generate();

        void join_streamlines(Direction dir);
        void join_streamlines();

        void simplify_streamlines();
        void clear();

        std::optional<StreamlineNode> get_node(node_id i) const;
        
        bool generate_streamline(RoadType road, DVector2 seed_point, Direction dir);

        int node_count() const;

};
#endif
