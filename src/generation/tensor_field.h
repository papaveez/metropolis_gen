#pragma once

#include <memory>
#include <vector>

#include "../SimplexNoise/SimplexNoise.h"

#include "types.h"

class Tensor {
    // 2x2 symmetric, traceless matrix represented as
    // R * | cos(2θ)  sin(2θ) | --> | a  b |
    //     | sin(2θ) -cos(2θ) |     | _  _ |
    public:
        double r;
        double theta;
        double a;
        double b;


        Tensor(double a, double b);
        static Tensor from_r_theta(double r, double theta);
        static Tensor from_xy(DVector2 xy);

        void set_r_theta();

        std::pair<DVector2, DVector2> eigenvectors();

        Tensor rotate(double angle);

        Tensor operator+(const Tensor& other) const;

        // right scalar mult
        Tensor operator*(double right) const;


        // left scalar mult
        friend Tensor operator*(double left, const Tensor& right);
};


class BasisField {
    protected:
        DVector2 centre;
        double size;
        double decay;

    public:
        BasisField(DVector2 _centre);
        BasisField(DVector2 _centre, double _size, double _decay);
        virtual ~BasisField() = default;

        DVector2 get_centre();
        void set_centre(DVector2 _centre);
        void set_size(double _size);
        void set_decay(double _decay);

        virtual Tensor get_tensor(DVector2 pos) = 0;
        virtual bool force_degenerate(DVector2 pos);

        Tensor get_weighted_tensor(DVector2 pos);
        double get_tensor_weight(DVector2 pos);
};


class Grid : public BasisField {
    private:
        double theta;


    public:
        Grid(double _theta, DVector2 _centre);
        Grid(double _theta, DVector2 _centre, double _size, double _decay);

        Tensor get_tensor(DVector2 pos) override;
        void set_theta(double _theta);
};

class Radial : public BasisField {
    public:
        Radial(DVector2 _centre);
        Radial(DVector2 _centre, double _size, double _decay);


        Tensor get_tensor(DVector2 pos) override;
};

class Roundabout : public BasisField {
    private:
        double thickness;
        double size;
        

    public:
        Roundabout(double _thickness, DVector2 _centre, double _size, double _decay);
        bool force_degenerate(DVector2 pos) override;
        Tensor get_tensor(DVector2 pos) override;
};


class TensorField {
    private:
        std::vector<std::unique_ptr<BasisField>> basis_fields;
        SimplexNoise noise;

    public:
        TensorField();
        void clear();
        
        TensorField(std::vector<std::unique_ptr<BasisField>>&& _basis_fields);

        void add_basis_field(std::unique_ptr<BasisField> ptr);
        

        Tensor sample(DVector2 pos);
        Tensor noisy_sample(DVector2 pos, double noise_size, double strength);

        std::vector<DVector2> get_basis_centres();
};


