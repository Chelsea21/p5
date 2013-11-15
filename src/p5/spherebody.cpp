#include "p5/spherebody.hpp"
#include "math/vector.hpp"
#include "math/matrix.hpp"
#include "scene/sphere.hpp"
#include <iostream>
#include <exception>
#include <algorithm>

namespace _462 {

SphereBody::SphereBody( Sphere* geom )
{
    sphere = geom;
    position = sphere->position;
    radius = sphere->radius;
    orientation = sphere->orientation;
    mass = 0.0;
    velocity = Vector3::Zero();
    angular_velocity = Vector3::Zero();
    force = Vector3::Zero();
    torque = Vector3::Zero();
}

Vector3 SphereBody::step_position( real_t dt, real_t motion_damping )
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // TODO return the delta in position dt in the future

    return force / mass;
}

Vector3 SphereBody::step_orientation( real_t dt, real_t motion_damping )
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // TODO return the delta in orientation dt in the future
    // vec.x = rotation along x axis
    // vec.y = rotation along y axis
    // vec.z = rotation along z axis

    return torque / (2.f / 5 * mass * radius *radius);
}

void SphereBody::apply_force( const Vector3& f, const Vector3& offset, const State &state )
{
    // TODO apply force/torque to sphere
	Vector3 offset_center = Vector3::Zero();
	offset_center = offset - position;
	if (offset_center == Vector3::Zero() || (offset_center = normalize(offset_center)) == normalize(f)) {
		force = f;
		torque = Vector3::Zero();
		return ;
	}

	force = dot(f, offset_center) * offset_center;
	torque = f - force;
}

}
