#include "integrator.h"


Direction flip(Direction dir) {
    if (dir == Major) {
        return Minor;
    }

    return Major;
}

NumericalFieldIntegrator::NumericalFieldIntegrator(
        TensorField* field) : field_(field) {}


DVector2 
NumericalFieldIntegrator::get_vector(
    const DVector2& x, const Direction& dir) const {
    Tensor t = field_->sample(x);

    if (dir == Major) {
        return t.get_major_eigenvector();
    } else {
        return t.get_minor_eigenvector();
    }
}


RK4::RK4 (TensorField* field) 
    : NumericalFieldIntegrator(field) {}



DVector2 
RK4::integrate(const DVector2& x, 
    const Direction& dir, const double& dl) const { 
    // return integration delta
    DVector2 dx = {dl, dl};

    DVector2 k1 = get_vector(x, dir);
    DVector2 k2 = get_vector(x + dx/2.0, dir);
    DVector2 k4 = get_vector(x + dx, dir);

    return k1 + k2*4.0 + k4/6.0;
}
