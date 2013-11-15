#ifndef _462_COLLISIONS_HPP_
#define _462_COLLISIONS_HPP_

#include "scene/sphere.hpp"
#include "p5/spherebody.hpp"
#include "p5/trianglebody.hpp"
#include "p5/planebody.hpp"

namespace _462 {

bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping );
bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping );
bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping );
bool barycentric(const Vector3 projection, const TriangleBody body);
bool collides_vertices_axes(const Vector3 center, const real_t radius, const TriangleBody body);

}

#endif
