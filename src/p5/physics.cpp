#include "p5/physics.hpp"

namespace _462 {

Physics::Physics()
{
    reset();
}

Physics::~Physics()
{
    reset();
}

void Physics::step( real_t dt )
{
    // TODO step the world forward by dt. Need to detect collisions, apply
    // forces, and integrate positions and orientations.
    //
    // Note: put RK4 here, not in any of the physics bodies
    //
    // Must use the functions that you implemented
    //
    // Note, when you change the position/orientation of a physics object,
    // change the position/orientation of the graphical object that represents
    // it

	for (size_t i = 0; i < num_spheres(); i++) {
		for (size_t j = 0; j < num_planes(); j++)
			collides(*spheres[i], *planes[j], collision_damping);
		for (size_t j = 0; j < num_triangles(); j++)
			collides(*spheres[i], *triangles[j], collision_damping);
		for (size_t j = i + 1; j < num_spheres(); j++) {
			collides(*spheres[i], *spheres[j], collision_damping);
		}
	}

	for (size_t i = 0; i < num_springs(); i++) {

	}

	std::vector<State> states(num_spheres());
	for (size_t i = 0; i < num_spheres(); i++) {
		states[i].position = spheres[i]->position;
		states[i].velocity = spheres[i]->velocity;
		states[i].angular_velocity = spheres[i]->angular_velocity;

		//if (i == 0)
			//std::cout << "before: " << state.velocity << std::endl;
		rk4_integrate(states[i], spheres[i], dt);

		//if (i == 0)
			//std::cout << "after: " << state.velocity << std::endl;
	}

	for (size_t i = 0; i < num_spheres(); i++) {
		spheres[i]->position = states[i].position;
		spheres[i]->velocity = states[i].velocity;
		spheres[i]->angular_velocity = states[i].angular_velocity;
		spheres[i]->sphere->position = states[i].position;
		//if (spheres[i]->spring_ptrs->)
	}
}

void Physics::rk4_integrate(State &state, Body *body_ptr, const real_t dt) const {
	Derivative d1 = take_derivative(state, Derivative(), body_ptr, 0);
	Derivative d2 = take_derivative(state, d1, body_ptr, dt * 0.5);
	Derivative d3 = take_derivative(state, d2, body_ptr, dt * 0.5);
	Derivative d4 = take_derivative(state, d3, body_ptr, dt);

	Vector3 dxdt = 1.f / 6 * (d1.d_position + 2 * (d2.d_position + d3.d_position) + d4.d_position);
	Vector3 dvdt = 1.f / 6 * (d1.d_velocity + 2 * (d2.d_velocity + d3.d_velocity) + d4.d_velocity);
	Vector3 davdt = 1.f / 6 * (d1.d_angular_velocity + 2 * (d2.d_angular_velocity + d3.d_angular_velocity)
					+ d4.d_angular_velocity);

	state.position += dxdt * dt;
	state.velocity += dvdt * dt;
	state.angular_velocity += davdt * dt;
}

Derivative Physics::take_derivative(const State &init_state,
		const Derivative &init_derivative, Body *body_ptr, real_t dt) const {
	State update_state;
	update_state.position = init_state.position + init_derivative.d_position * dt;
	update_state.velocity = init_state.velocity + init_derivative.d_velocity * dt;
	update_state.angular_velocity = init_state.angular_velocity +
							init_derivative.d_angular_velocity * dt;

	Derivative result;
	result.d_position = update_state.velocity;
	acceleration(update_state, body_ptr, dt, result);

	return result;
}

void Physics::acceleration(const State &state, Body *body_ptr, const real_t dt, Derivative &a) const {
	(void) dt;
	body_ptr->apply_force(gravity, body_ptr->position, state);
	a.d_velocity = body_ptr->step_position(dt, 0.f);
	a.d_angular_velocity = body_ptr->step_orientation(dt, 0.f);
}

void Physics::add_sphere( SphereBody* b )
{
    spheres.push_back( b );
}

size_t Physics::num_spheres() const
{
    return spheres.size();
}

void Physics::add_plane( PlaneBody* p )
{
    planes.push_back( p );
}

size_t Physics::num_planes() const
{
    return planes.size();
}

void Physics::add_triangle( TriangleBody* t )
{
    triangles.push_back( t );
}

size_t Physics::num_triangles() const
{
    return triangles.size();
}

void Physics::add_spring( Spring* s )
{
    springs.push_back( s );
}

size_t Physics::num_springs() const
{
    return springs.size();
}

void Physics::reset()
{
    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        delete *i;
    }
    for ( PlaneList::iterator i = planes.begin(); i != planes.end(); i++ ) {
        delete *i;
    }
    for ( TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++ ) {
        delete *i;
    }
    for ( SpringList::iterator i = springs.begin(); i != springs.end(); i++ ) {
        delete *i;
    }

    spheres.clear();
    planes.clear();
    triangles.clear();
    springs.clear();
    
    gravity = Vector3::Zero();
	collision_damping = 0.0;
}

}
