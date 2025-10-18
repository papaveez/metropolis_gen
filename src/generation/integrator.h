#pragma once


#include "types.h"
#include "tensor_field.h"


enum Direction : char {
    Minor = 1,
    Major = 2
};

Direction flip(Direction dir);

class NumericalFieldIntegrator {
    private:
        TensorField* field;

    protected:
        DVector2 get_vector(DVector2 x, Direction dir, double noise_size, double noise_angle);

    public:
        NumericalFieldIntegrator(TensorField* _field);
        virtual ~NumericalFieldIntegrator() = default;
        virtual DVector2 integrate(DVector2 x, Direction d, double dl, double noise_size, double noise_angle) = 0;
};


class RK4 : public NumericalFieldIntegrator {
    public:
        RK4(TensorField* _field);

        DVector2 integrate(DVector2 x, Direction dir, double dl, double noise_size, double noise_angle) override;
};
