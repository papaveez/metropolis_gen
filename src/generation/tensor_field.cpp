#include "tensor_field.h"


// ****** Tensor ******

Tensor Tensor::from_a_b(const double& a, const double& b) {
    Tensor out = Tensor {a, b};
    out.set_r_theta();
    return out;
}


void Tensor::set_r_theta() {
    r = std::hypot(a, b);
    if (is_degenerate()) {
        theta = 0;
    }
    else {
        theta = std::atan2(b/r, a/r)/2.0;
    }
}


Tensor Tensor::from_r_theta(const double& r, const double& theta) {
    return Tensor {
        r*std::cos(2*theta),
        r*std::sin(2*theta),
        r,
        theta,
    };
}


Tensor Tensor::from_xy(const DVector2& xy) {
    const double& x = xy.x;
    const double& y = xy.y;

    return from_a_b(y*y - x*x, -2*x*y);
}


bool Tensor::is_degenerate() const {
    return std::abs(r) <= d_epsilon;
}



DVector2 Tensor::get_major_eigenvector() const {
    if (is_degenerate()) return {0.0, 0.0};

    return {
        std::cos(theta),
        std::sin(theta)
    };
}

DVector2 Tensor::get_minor_eigenvector() const {
    if (is_degenerate()) return {0.0, 0.0};
    return {
        std::sin(theta),
        std::cos(theta)*-1.0
    };
}


Tensor Tensor::rotate(const double& angle) const {
    return Tensor::from_r_theta(
        r,
        std::fmodf(theta + angle, 2.0f*M_PI)
    );
}


Tensor Tensor::operator+(const Tensor& other) const {
    return Tensor(a + other.a, b + other.b);
}


Tensor Tensor::operator*(const double& right) const {
    return Tensor(right*a, right*b);
}


Tensor operator*(double left, const Tensor& right) {
    return Tensor(left*right.a, left*right.b);
}



// ****** BasisField ******

BasisField::BasisField(DVector2 centre) 
    : centre_(centre), size_(0), decay_(0) {}


BasisField::BasisField(DVector2 centre, double size, double decay) 
    : centre_(centre), size_(size), decay_(decay) {}


const DVector2& BasisField::get_centre() const {
    return centre_;
}


void BasisField::set_centre(DVector2 centre) {
    centre_ = centre;
}

void BasisField::set_size(double size) {
    size_ = size;
}

void BasisField::set_decay(double decay) {
    decay_ = decay;
}


bool BasisField::force_degenerate(const DVector2& pos) {
    return false;
}


double BasisField::get_tensor_weight(const DVector2& pos) const {
    if (size_ == 0) {
        return 1;
    }

    DVector2 from_centre = pos - centre_;
    double norm_dist_to_centre =
        std::hypot(from_centre.x, from_centre.y) / size_;
    
    if (decay_ == 0 && norm_dist_to_centre >= 1 ) {
        return 0;
    }
    
    double out = std::pow(
        std::max(0.0, 1.0-norm_dist_to_centre),
        decay_
    );

    if (std::abs(out) < d_epsilon) {
        return 0;
    }

    return out;
}


Tensor BasisField::get_weighted_tensor(const DVector2& pos) const {
    return get_tensor(pos)*get_tensor_weight(pos);
}



// ****** BasisField : Grid ******
Grid::Grid(double _theta, DVector2 _centre) 
    : BasisField(_centre), theta(_theta) {}

Grid::Grid(double _theta, DVector2 _centre, double _size, double _decay) 
    : BasisField(_centre, _size, _decay), theta(_theta) {}


void Grid::set_theta(double _theta) {
    theta = _theta;
}


Tensor Grid::get_tensor(const DVector2& pos) const {
    return Tensor::from_r_theta(1, theta);
}



// ****** BasisField : Radial ******

Radial::Radial(DVector2 _centre) 
    : BasisField(_centre) {}

Radial::Radial(DVector2 _centre, double _size, double _decay) 
    : BasisField(_centre, _size, _decay) {}



Tensor Radial::get_tensor(const DVector2& pos) const {
    return Tensor::from_xy(pos - centre_);
}



// ****** TensorField ******


TensorField::TensorField() {}


void TensorField::clear() {
    basis_fields.clear();
}


TensorField::TensorField(std::vector<std::unique_ptr<BasisField>>&& _basis_fields) 
    : basis_fields(std::move(_basis_fields)) {}


void TensorField::add_basis_field(std::unique_ptr<BasisField> bf) {
    basis_fields.push_back(std::move(bf));
}


Tensor TensorField::sample(const DVector2& pos) const {
    Tensor out; // new degenerate tensor

    for (auto& x : basis_fields) {
        Tensor basis_field_tensor = x->get_weighted_tensor(pos);
        out = out + basis_field_tensor;
    }

    out.set_r_theta();

    return out;
}


std::vector<DVector2> TensorField::get_basis_centres() const {
    std::vector<DVector2> out;
    for (auto& basis : basis_fields) {
        out.push_back(basis->get_centre());
    }

    return out;
}
