#include "p5/collisions.hpp"

namespace _462 {

bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping )
{
	// Velocity of body1 relative to body2.
	Vector3 v1 = body1.velocity - body2.velocity;
	Vector3 distance_vec = body2.position - body1.position;

	// If the velocity is too low or in the opposite direction, they do not
	// collide.
	if (squared_length(v1) > 1e-8 && dot(v1, distance_vec) > 0) {
			real_t distance = length(distance_vec);
			// The distance between them is less than the sum of their radiuses.
			if (distance < (body1.radius + body2.radius)) {
				Vector3 v2 = Vector3::Zero();
				Vector3 d_vec = distance_vec / distance;

				// Compute new velocities.
				v2 = body2.velocity + 2 * d_vec * body1.mass / (body2.mass + body1.mass) * dot(v1, d_vec);
				body1.velocity = body1.velocity + body2.mass / body1.mass * (body2.velocity - v2);
				body2.velocity = v2;

				// Apply damping.
				body1.velocity = (1 - collision_damping) * body1.velocity;
				body2.velocity = (1 - collision_damping) * body2.velocity;

				return true;
			}
		}

    return false;
}

bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping )
{
	// Normal vector of trangle body.
	Vector3 normal = normalize(cross(body2.vertices[1] - body2.vertices[0],
						body2.vertices[2] - body2.vertices[0]));
	real_t distance = dot(normal, body1.position - body2.position);

	// Sphere may come from the opposition direction of normal.
	if (std::abs(distance) <= body1.radius) {
		Vector3 projection = body1.position - distance * normal;
		Vector3 sphere_v = body1.velocity - body2.velocity;

		// If sphere moves toward the triangle, check whether it hits the surface or
		// edges of the triangle.
		if (dot(sphere_v, distance * normal) < 0 && (barycentric(projection, body2) ||
				collides_vertices_axes(body1.position, body1.radius, body2))) {
			// Since the triangle is fixed, use the same formula with plane collision.
			real_t v_projection = dot(body1.velocity, normal);
			body1.velocity = body1.velocity - 2 * v_projection * normal;
			body1.velocity = (1 - collision_damping) * body1.velocity;

			return true;
		}
	}

    return false;
}

/**
 *	Convert the projection of sphere center to barycentric coordinate.
 *	Check whether this point is within the triangle surface.
 */
bool barycentric(const Vector3 projection, const TriangleBody body) {
	Vector3 normal = cross(body.vertices[1] - body.vertices[0],
			body.vertices[2] - body.vertices[0]);
	Vector3 nb = cross(body.vertices[0] - body.vertices[2],
			projection - body.vertices[2]);
	Vector3 nc = cross(body.vertices[1] - body.vertices[0],
			projection - body.vertices[0]);
	real_t beta = dot(normal, nb) / squared_length(normal);
	if (beta < 0 || beta > 1)
		return false;
	real_t gamma = dot(normal, nc) / squared_length(normal);
	if (gamma < 0 || gamma > 1 - beta)
		return false;

	return true;
}

/**
 * Check whether the sphere hits the triangle's vertices or edges.
 */
bool collides_vertices_axes(const Vector3 center, const real_t radius, const TriangleBody body) {
	// Check vertices.
	for (size_t i = 0; i < 3; i++) {
		if (squared_length(center - body.vertices[i]) < radius * radius)
			return true;
	}

	// Check edges.
	for (size_t i = 0; i < 3; i++) {
		Vector3 edge = normalize(body.vertices[(i + 1) % 3] - body.vertices[i]);
		Vector3 projection = dot(center - body.vertices[i], edge) * edge +
									body.vertices[i];
		if (squared_length(center - projection) < radius * radius)
			return true;
	}

	return false;
}

bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping )
{
	real_t v_projection= dot(body1.velocity, body2.normal);
	// Get the normal vector in the direction facing the sphere.
	Vector3 normal_facing_sphere = (dot(body1.position - body2.position, body2.normal) > 0) ?
			body2.normal : -body2.normal;

	// If the velocity is large enough and the sphere is moving toward the plane.
	if (squared_length(body1.velocity) > 1e-8 &&
			(dot(body1.velocity, normal_facing_sphere)) < 0) {
		real_t distance = std::abs(dot(body2.normal, body1.position - body2.position));
		if (distance < body1.radius) {
			// Reverse the velocity component perpendicular to the plane.
			body1.velocity = body1.velocity - 2 * v_projection * body2.normal;
			body1.velocity = (1 - collision_damping) * body1.velocity;

			return true;
		}
	}

	return false;
}

}
