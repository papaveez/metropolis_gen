#ifndef GENERATOR_H
#define GENERATOR_H

#include <memory>
#include <queue>
#include <random>
#include <unordered_map>
#include <vector>

#include "../types.h"
#include "integrator.h"
#include "node_storage.h"

#include "../const.h"


enum IntegrationStatus {
    Continue,
    Terminate,
    Abort
};


struct Integration {
    IntegrationStatus status;
    DVector2 delta;
    DVector2 integration_front;
    bool negate; 
    std::list<DVector2> points;

    Integration(DVector2 seed, bool negate) :
        status(Continue),
        integration_front(seed),
        negate(negate),
        points({seed})
    {}
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


        std::unique_ptr<NumericalFieldIntegrator> integrator_;
        std::vector<RoadType> road_types_;
        std::unordered_map<RoadType, GeneratorParameters> params_;
        std::unordered_map<Direction, seed_queue> seeds_;
        std::default_random_engine gen_;
        std::uniform_real_distribution<double> dist_;
        std::vector<StreamlineNode> nodes_;
        Box<double> viewport_;
#ifdef SPATIAL_TEST
    public:
#endif
        Spatial spatial_;
#ifdef SPATIAL_TEST
    private:
#endif
        std::unordered_map<RoadType, Streamlines> streamlines_;


        bool in_bounds(const DVector2& p) const;


        void add_candidate_seed(node_id id, Direction dir);
        std::optional<DVector2> get_seed(RoadType road, Direction dir);


        void extend_streamline(
            Integration& res,
            const RoadType& road,
            const Direction& dir
        ) const;
        std::optional<std::list<DVector2>>
        generate_streamline(RoadType road, DVector2 seed_point, Direction dir);
        int generate_streamlines(RoadType road);

        
        void simplify_streamline(RoadType road, std::list<DVector2>& points) const;
        void douglas_peucker(
            const double& epsilon,
            const double& min_sep2,
            std::list<DVector2>& points,
            std::list<DVector2>::iterator begin,
            std::list<DVector2>::iterator end
        ) const;


#ifdef SPATIAL_TEST
    public:
#endif
        void push_streamline(RoadType road, std::list<DVector2>& points, Direction dir);


    public:
        RoadNetworkGenerator(
                std::unique_ptr<NumericalFieldIntegrator>& integrator,
                std::unordered_map<RoadType, GeneratorParameters>,
                Box<double> viewport
            );

        // getters
        const std::vector<RoadType>& get_road_types() const;
        const std::unordered_map<RoadType, GeneratorParameters>& get_parameters() const;
        const StreamlineNode& get_node(node_id i) const;
        const std::vector<Streamline>&  get_streamlines(RoadType road, Direction dir);
        int node_count() const;


        void set_viewport(Box<double> new_viewport);


        void generate();
        bool generation_step(RoadType road, Direction dir);


        void clear();
};
#endif
