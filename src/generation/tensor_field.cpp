#include "tensor_field.h"
#include "types.h"
#include <cmath>
#include <limits>
#include <raymath.h>



// ****** Tensor ******

Tensor::Tensor(double a_, double b_) : a(a_), b(b_) {
    set_r_theta();
}

void Tensor::set_r_theta() {
    r = std::hypot(a, b);
    if (std::abs(r) <= std::numeric_limits<float>::epsilon()) {
        theta = 0;
    }
    else {
        theta = std::atan2(b/r, a/r)/2.0;
        // theta = std::atanf(b/a)/2.0f;
    }
}


Tensor Tensor::from_r_theta(double r, double theta) {
    float a = r* std::cos(2*theta);
    float b = r* std::sin(2*theta);

    return Tensor(a, b);
}


Tensor Tensor::from_xy(DVector2 xy) {
    double x = xy.x;
    double y = xy.y;

    return Tensor(y*y - x*x, -2*x*y);
}



std::pair<DVector2, DVector2> Tensor::eigenvectors() {
    
    if (std::abs(r) <= std::numeric_limits<double>::epsilon()) {
        return {{0.0, 0.0}, {0.0, 0.0}};
    }

    DVector2 major =  {
        std::cos(theta), 
        std::sin(theta)
    };

    DVector2 minor = {
        major.y,
        major.x*-1.0
    };

    return {major, minor};
}


Tensor Tensor::rotate(double angle) {
    return Tensor::from_r_theta(r, std::fmodf(theta + angle, 2.0f*M_PI));
}


Tensor Tensor::operator+(const Tensor& other) const {
    return Tensor(a + other.a, b + other.b);
}


Tensor Tensor::operator*(double right) const {
    return Tensor(right*a, right*b);
}


Tensor operator*(double left, const Tensor& right) {
    return Tensor(left*right.a, left*right.b);
}



// ****** BasisField ******

BasisField::BasisField(DVector2 _centre) 
    : centre(_centre), size(0), decay(0) {}


BasisField::BasisField(DVector2 _centre, double _size, double _decay) 
    : centre(_centre), size(_size), decay(_decay) {}


DVector2 BasisField::get_centre() {
    return centre;
}


void BasisField::set_centre(DVector2 _centre) {
   centre = _centre;
}

void BasisField::set_size(double _size) {
    size = _size;
}

void BasisField::set_decay(double _decay) {
    decay = _decay;
}


bool BasisField::force_degenerate(DVector2 pos) {
    return false;
}

double BasisField::get_tensor_weight(DVector2 pos) {
    if (size == 0) {
        return 1;
    }

    DVector2 from_centre = pos - centre;
    double norm_dist_to_centre = std::hypot(from_centre.x, from_centre.y) / size;
    
    if (decay == 0 && norm_dist_to_centre >= 1 ) {
        return 0;
    }
    
    double out = std::pow(std::max(0.0, 1.0-norm_dist_to_centre), decay);
    if (std::abs(out) < std::numeric_limits<double>::epsilon()) {
        return 0;
    }

    return out;
}


Tensor BasisField::get_weighted_tensor(DVector2 pos) {
    return get_tensor_weight(pos)*get_tensor(pos);
}



// ****** BasisField : Grid ******
Grid::Grid(double _theta, DVector2 _centre) 
    : BasisField(_centre), theta(_theta) {}

Grid::Grid(double _theta, DVector2 _centre, double _size, double _decay) 
    : BasisField(_centre, _size, _decay), theta(_theta) {}


void Grid::set_theta(double _theta) {
    theta = _theta;
}


Tensor Grid::get_tensor(DVector2 pos) {
    return Tensor::from_r_theta(1, theta);
}



// ****** BasisField : Radial ******

Radial::Radial(DVector2 _centre) 
    : BasisField(_centre) {}

Radial::Radial(DVector2 _centre, double _size, double _decay) 
    : BasisField(_centre, _size, _decay) {}



Tensor Radial::get_tensor(DVector2 pos) {
    return Tensor::from_xy(pos - centre);
}



Roundabout::Roundabout(double _thickness, DVector2 _centre, double _size, double _decay) :
    BasisField(_centre, _size, _decay),
    thickness(_thickness) {}

bool Roundabout::force_degenerate(DVector2 pos) {
    DVector2 dist = pos - centre;
    if (std::hypot(dist.x, dist.y) < (size - thickness)) {
        return true;
    }
    return false;
}

Tensor Roundabout::get_tensor(DVector2 pos) {
    return Tensor::from_xy(pos - centre);
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


Tensor TensorField::sample(DVector2 pos) {
    Tensor out = Tensor(0.0, 0.0); // new degenerate tensor
                               //
    for (auto& x : basis_fields) {
        Tensor basis_field_tensor = x->get_weighted_tensor(pos);
        out = out + basis_field_tensor;
    }

    return out;
}

Tensor TensorField::noisy_sample(DVector2 pos, double noise_size, double strength) {
    Tensor out = sample(pos);

    double rotation = strength*noise.noise(pos.x/noise_size, pos.y/noise_size)*M_PI;

    return out.rotate(rotation);
}



std::vector<DVector2> TensorField::get_basis_centres() {
    std::vector<DVector2> out;
    for (auto& basis : basis_fields) {
        out.push_back(basis->get_centre());
    }

    return out;
}
