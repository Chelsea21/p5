#ifndef _462_PHYSICS_BODY_HPP_
#define _462_PHYSICS_BODY_HPP_

#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include <exception>
#include <iostream>
#include <vector>

namespace _462 {

class Spring;

struct State {
	Vector3 position;
	Vector3 velocity;
	Vector3 angular_position;
	Vector3 angular_velocity;
};

struct Derivative {
	Vector3 d_position;
	Vector3 d_velocity;
	Vector3 d_angular_position;
	Vector3 d_angular_velocity;

	Derivative()
	: d_position(Vector3::Zero()), d_velocity(Vector3::Zero()),
	  d_angular_position(Vector3::Zero()), d_angular_velocity(Vector3::Zero()) { }
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

    // Use vector to store the pointers of springs. So we can
    // update offset in spring instances easily.
    std::vector<Spring*> spring_ptrs;

    Body() : spring_ptrs(std::vector<Spring*>()) { }
    virtual ~Body() { }
    virtual Vector3 step_position( real_t dt, real_t motion_damping ) = 0;
    virtual Vector3 step_orientation( real_t dt, real_t motion_damping ) = 0;
    virtual void apply_force( const Vector3& f, const Vector3& offset ) = 0;

    // Clean force before applying force and getting acceleration in every integral step.
    virtual void clean_force() = 0;
};

}

#endif
