#include "math/math.hpp"
#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "p5/spring.hpp"
#include "p5/body.hpp"
#include "p5/spherebody.hpp"
#include <iostream>

namespace _462 {

Spring::Spring()
{
    body1_offset = Vector3::Zero();
    body2_offset = Vector3::Zero();
    damping = 0.0;
}

void Spring::step( real_t dt )
{
    // TODO apply forces to attached bodies
}

/**
 * Compute force for body in given state(position and velocity).
 */
Vector3 Spring::get_force(const Body* body, const State& state) const {
	Vector3 displace;
	if (body == body1)
		displace = body1_offset + state.position -  (body2_offset + body2->position);
	else
		displace = body2_offset + state.position -  (body1_offset + body1->position);
	Vector3 spring_vec = normalize(displace);
	Vector3 equilibrium_vec = equilibrium * spring_vec;
	displace -= equilibrium_vec;

	// F = - constant * x - damping (dx / dt)
	Vector3 force = - constant * displace + (-damping) * dot(spring_vec, state.velocity) * spring_vec;

	return force;
}

Vector3 Spring::get_offset(const Body* body) const {
	return (body == body1) ? body1_offset : body2_offset;
}

/**
 * Rotate given body's offset vector.
 */
void Spring::update_offset(const Body* body, const Vector3& angular_position) {
	Vector3* offset_ptr = (body == body1) ? &body1_offset : &body2_offset;

	Quaternion qua( normalize(angular_position), length(angular_position) );
	Matrix3 mat;
	qua.to_matrix(&mat);

	*offset_ptr = mat * *offset_ptr;
}

}


