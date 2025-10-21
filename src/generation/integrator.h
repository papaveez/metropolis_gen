#pragma once


#include "../types.h"
#include "tensor_field.h"


enum Direction : char {
    Minor = 1 << 0,
    Major = 1 << 1
};

Direction flip(Direction dir);

class NumericalFieldIntegrator {
private:
    TensorField* field_;

protected:
    DVector2 get_vector(const DVector2& x, const Direction& dir) const;

public:
    NumericalFieldIntegrator(TensorField* field);
    virtual ~NumericalFieldIntegrator() = default;

    virtual DVector2 
    integrate(
        const DVector2& x, 
        const Direction& d, 
        const double& dl
    ) const = 0;
};


class RK4 : public NumericalFieldIntegrator {
public:
    RK4(TensorField* _field);

    DVector2 
    integrate(
        const DVector2& x, 
        const Direction& d, 
        const double& dl
    ) const override;
};
