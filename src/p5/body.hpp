#ifndef _462_PHYSICS_BODY_HPP_
#define _462_PHYSICS_BODY_HPP_

#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include <exception>
#include <iostream>

namespace _462 {

struct State {
	Vector3 position;
	Vector3 velocity;
	Vector3 angular_velocity;
};

struct Derivative {
	Vector3 d_position;
	Vector3 d_velocity;
	Vector3 d_angular_velocity;

	Derivative()
	: d_position(Vector3::Zero()), d_velocity(Vector3::Zero()),
			d_angular_velocity(Vector3::Zero()) { }
};

class Body
{
public:
    int id;
    int type;
    Vector3 position;
    Quaternion orientation;
    Vector3 velocity;
    Vector3 angular_velocity;
    std::vector<Spring*> spring_ptrs;

    virtual ~Body() { }
    virtual Vector3 step_position( real_t dt, real_t motion_damping ) = 0;
    virtual Vector3 step_orientation( real_t dt, real_t motion_damping ) = 0;
    virtual void apply_force( const Vector3& f, const Vector3& offset, const State &state ) = 0;
};

}

#endif
