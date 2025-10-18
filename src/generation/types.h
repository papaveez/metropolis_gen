#ifndef TYPES_H 
#define TYPES_H 

#include <limits>
#include <ostream>
#include <string>

#include "raylib.h"

template<typename T>
struct TVector2 {
    T x;
    T y;

    TVector2() : x(0.0), y(0.0){}
    TVector2(T _x, T _y) : x(_x), y(_y) {}
    TVector2(const Vector2 v) : 
        x(static_cast<double>(v.x)), 
        y(static_cast<double>(v.y)) 
    {}

    operator Vector2() const {
        return Vector2 {
            static_cast<float>(x),
            static_cast<float>(y)
        };
    }

    bool operator==(const TVector2<T>& other) const {
        return x == other.x && y == other.y;
    }

    TVector2 operator+(const TVector2<T>& other) const {
        return {
            x + other.x,
            y + other.y
        };
    }

    TVector2 operator-(const TVector2<T>& other) const {
        return {
            x - other.x,
            y - other.y
        };
    }

    template<typename U>
    TVector2 operator*(const U& scalar) const {
        return {
            scalar*x,
            scalar*y
        };
    };

    template<typename U>
    TVector2 operator/(const U& scalar) const {
        return {
            x/scalar,
            y/scalar
        };
    }

};


template<typename T>
std::ostream& operator<<(std::ostream& os, const TVector2<T>& v) {
    return os << "(" << v.x << "," << v.y << ")";
}

template<typename T>
T dot_product(const TVector2<T>& a, const TVector2<T>& b) {
    return a.x*b.x + a.y*b.y;
}

template<typename T>
TVector2<T> middle(TVector2<T> const& p1, TVector2<T> const& p2) {
    return (p1 + p2)/2.0;
}

template<typename T>
double vector_angle(const TVector2<T>& a, const TVector2<T>& b) {
    double dot = dot_product(a, b);
    double det = a.x*b.y - a.y*b.x;

    return atan2(det, dot);
}

//  TODO: implement
template<typename T>
double perpendicular_distance(const TVector2<T>& p, const TVector2<T>& x0, const TVector2<T>& x1) {
    return 0.0;
}

enum Quadrant {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight
};



template<typename T> 
struct Box {
    static constexpr T inf = std::numeric_limits<T>::has_infinity
        ? std::numeric_limits<T>::infinity() // for floating point types
        : std::numeric_limits<T>::max(); // for nonfloat types

    TVector2<T> min{inf, inf};
    TVector2<T> max{-inf, -inf};

    Box() = default;
    Box(TVector2<T> _min, TVector2<T> _max) :
        min(_min),
        max(_max)
    {}

    bool is_empty() const {
        return min.x >= max.x
            && min.y >= max.y;
    }

    bool contains(const TVector2<T>& vec) const {
        return 
            min.x <= vec.x 
            && vec.x < max.x
            && min.y <= vec.y
            && vec.y < max.y;
    }

    T width() const {
        return max.x - min.x;
    }

    T height() const {
        return max.y - min.y;
    }

    std::tuple<Box, Box, Box, Box> // TL TR BL BR
    quadrants() {
        TVector2<T> mid = middle(min, max);

        return {
            Box(min, mid), 
            Box({mid.x, min.y}, {max.x, mid.y}), 
            Box({min.x, mid.y}, {mid.x, max.y}), 
            Box(mid, max)
        };
    }


    Box get_quadrant(Quadrant q) {
        TVector2<T> mid = middle(min, max);
        switch (q) {
            case TopLeft:
                return Box(min, mid);
            case TopRight:
                return Box({mid.x, min.y}, {max.x, mid.y});
            case BottomLeft:
                return Box({min.x, mid.y}, {mid.x, max.y});
            case BottomRight:
                return Box(mid, max);
        }
    }

    Quadrant which_quadrant(TVector2<T> pos) {
        TVector2<T> mid = middle(min, max);

        for (Quadrant q : {TopLeft, TopRight, BottomLeft, BottomRight}) {
            if (get_quadrant(q).contains(pos)) return q;
        }

    }

    // box union.
    Box& operator|=(const Box<T>& other) {
        min = {
            std::min(min.x, other.min.x),
            std::min(min.y, other.min.y)
        };
        max = {
            std::max(max.x, other.max.x),
            std::max(max.y, other.max.y)
        };

        return *this;
    }

    Box operator|(const Box<T>& other) const {
        Box<T> out = *this;
        out |= other;
        return out;
    }

    // box union with vector
    Box& operator|=(const TVector2<T>& other) {
        Box<T> boxed = Box(other, other);
        *this |= boxed;
        return *this;
    }

    Box operator|(const TVector2<T>& other) const {
        Box<T> out = *this;
        out |= other;
        return out;
    }


    // box intersection
    Box& operator&=(const Box<T>& other) {
        min = {
            std::max(min.x, other.min.x),
            std::max(min.y, other.min.y)
        };
        max = {
            std::min(max.x, other.max.x),
            std::min(max.y, other.max.y)
        };

        return *this;
    }

    Box operator&(const Box<T>& other) const {
        Box<T> out = *this;
        out &= other;
        return out;
    }
};

// equivelant to foldr (|) boxes or foldr (|) vectors
template<typename T, typename Iterator>
Box<T> bounding_box(Iterator begin, Iterator end) { 
    Box<T> out;
    for (auto it = begin; it != end; ++it) {
        out |= *it;
    }
    return out;
}


// useful type aliases
using DVector2 = TVector2<double>;
using IVector2 = TVector2<int>;

#endif
