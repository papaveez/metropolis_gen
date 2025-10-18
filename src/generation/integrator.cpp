#include "integrator.h"

#include "tensor_field.h"
#include <utility>
#include <raymath.h>


Direction flip(Direction dir) {
    if (dir == Major) {
        return Minor;
    }

    return Major;
}

NumericalFieldIntegrator::NumericalFieldIntegrator(
        TensorField* _field) : field(_field) {}


DVector2 NumericalFieldIntegrator::get_vector(DVector2 x, Direction dir, double noise_size, double noise_angle) {

    Tensor t = (noise_angle == 0 || noise_size == 0) 
        ? field->sample(x) : field->noisy_sample(x, noise_size, noise_angle);

    std::pair<DVector2, DVector2> eigenvectors = t.eigenvectors();

    if (dir == Major) {
        return eigenvectors.first;
    }
    return eigenvectors.second;
}

RK4::RK4 (TensorField* _field) 
    : NumericalFieldIntegrator(_field) {}



DVector2 RK4::integrate(DVector2 x, Direction dir, double dl, double noise_size, double noise_angle) { // returns delta. 
    DVector2 dx = {dl, dl};

    DVector2 k1 = get_vector(x, dir, noise_size, noise_angle);
    DVector2 k2 = get_vector(x + dx/2.0, dir, noise_size, noise_angle);
    DVector2 k4 = get_vector(x + dx, dir, noise_size, noise_angle);

    return k1 + k2*4.0 + k4/6.0;
}
