#ifndef _462_SPRING_HPP_
#define _462_SPRING_HPP_

#include "p5/body.hpp"

namespace _462 {

class Spring
{
public:
    Spring();
    virtual ~Spring() {};
    void step( real_t dt );

    real_t constant;
    real_t equilibrium;
    real_t damping;
    Body* body1;
    Body* body2;
    Vector3 body1_offset;
    Vector3 body2_offset;

    Vector3 get_force(const Body* body, const State& state) const;
    Vector3 get_offset(const Body* body) const;
    void update_offset(const Body* body, const Vector3& axis, real_t radians);
};

}

#endif
