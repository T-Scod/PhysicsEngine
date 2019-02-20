#include "AABB.h"

AABB::AABB(const glm::vec2 & position, const glm::vec2 & velocity, const float width, const float height, const float mass,
	const glm::vec4 & colour, const float elasticity, const float �s, const float �k) :
	Rigidbody(BOX, position, velocity, 0, mass, colour, elasticity, �s, �k) // passes the relative information into the rigidbody constructor
{
	m_width = width;
	m_height = height;
}
AABB::AABB(const glm::vec2 & position, const float inclination, const float speed, const float width, const float height, const float mass,
	const glm::vec4 & colour, const float elasticity, const float �s, const float �k) :
	Rigidbody(BOX, position, glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0, mass, colour, elasticity, �s, �k) // velocity is worked out using inclination and speed
{
	m_width = width;
	m_height = height;
}
AABB::~AABB()
{
}

void AABB::MakeGizmo()
{
	// uses gizmos to draw a box
	aie::Gizmos::add2DAABB(m_position, GetExtents(), m_colour);
}