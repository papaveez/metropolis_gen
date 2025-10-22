#pragma once

#include <limits>

#include "../types.h"

static constexpr double d_epsilon = std::numeric_limits<double>::epsilon();

struct Tensor {
    // 2x2 symmetric, traceless matrix represented as
    // R * | cos(2θ)  sin(2θ) | --> | a  b |
    //     | sin(2θ) -cos(2θ) |     | _  _ |
    double a;
    double b;
    double r;
    double theta;

    static Tensor from_a_b(const double& a, const double& b);
    static Tensor from_r_theta(const double& r, const double& theta);
    static Tensor from_xy(const DVector2& xy);

    void set_r_theta();

    bool is_degenerate() const;
    DVector2 get_major_eigenvector() const;
    DVector2 get_minor_eigenvector() const;

    Tensor rotate(const double& angle) const;

    Tensor operator+(const Tensor& other) const;

    // right scalar mult
    Tensor operator*(const double& right) const;


    // left scalar mult
    friend Tensor operator*(const double& left, const Tensor& right);
};


class BasisField {
    protected:
        DVector2 centre_;
        double size_;
        double decay_;

        virtual Tensor get_tensor(const DVector2& pos) const = 0;
        virtual bool force_degenerate(const DVector2& pos);
        double get_tensor_weight(const DVector2& pos) const;

    public:
        BasisField(DVector2 centre);
        BasisField(DVector2 centre, double size, double decay);
        virtual ~BasisField() = default;

        const DVector2& get_centre() const;
        void set_centre(DVector2 centre);
        void set_size(double size);
        void set_decay(double decay);


        Tensor get_weighted_tensor(const DVector2& pos) const;
};


class Grid : public BasisField {
    private:
        double theta;


    public:
        Grid(double theta, DVector2 centre);
        Grid(double theta, DVector2 centre, double size, double decay);

        Tensor get_tensor(const DVector2& pos) const override;
        void set_theta(double _theta);
};

class Radial : public BasisField {
    public:
        Radial(DVector2 centre);
        Radial(DVector2 centre, double size, double decay);


        Tensor get_tensor(const DVector2& pos) const override;
};


class TensorField {
    private:
        std::vector<std::unique_ptr<BasisField>> basis_fields;

    public:
        TensorField();
        void clear();
        
        TensorField(std::vector<std::unique_ptr<BasisField>>&& _basis_fields);

        void add_basis_field(std::unique_ptr<BasisField> ptr);
        

        Tensor sample(const DVector2& pos) const;
        std::vector<DVector2> get_basis_centres() const;
};


