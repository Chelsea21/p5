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
    (void) dt;
    (void) motion_damping;

    // Compute linear acceleration using f = ma.
    return force / mass;
}

Vector3 SphereBody::step_orientation( real_t dt, real_t motion_damping )
{
	(void) dt;
	(void) motion_damping;

	// Compute angular acceleration using alpha = torque / I; I = (2 / 5) * mr^2.
    return torque / (2.f / 5 * mass * radius *radius);
}

void SphereBody::apply_force( const Vector3& f, const Vector3& offset)
{
	Vector3 offset_normal;

	// Torque equals zeor, if the force is applies to the mass center.
	if (offset == Vector3::Zero() || (offset_normal = normalize(offset)) == normalize(f)) {
		force += f;
		torque += Vector3::Zero();
		return ;
	}

	// Split the force into linear and angular direction.
	Vector3 component = dot(f, offset_normal) * offset_normal;
	force += component;
	// Compute torque.
	torque += cross(offset, f - component);
}

void SphereBody::clean_force() {
	force = Vector3::Zero();
	torque = Vector3::Zero();
}

void SphereBody::update_orientation(const Vector3 rotation) {
	// The rotation vector is computed from angular velocity.
	// The rotation axis is the direction of the angular velocity.
	Quaternion qua( normalize(rotation), length(rotation) );
	orientation = normalize( qua * orientation );

}

}
