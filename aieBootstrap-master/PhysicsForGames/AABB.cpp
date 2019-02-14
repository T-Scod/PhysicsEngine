#include "AABB.h"

AABB::AABB(const glm::vec2 & position, const glm::vec2 & velocity, const float mass, const float width, const float height, const glm::vec4 & colour, const bool collision) :
	Rigidbody(BOX, position, velocity, 0, mass, collision)
{
	m_width = width;
	m_height = height;
	m_colour = colour;
}
AABB::AABB(const glm::vec2 & position, const float inclination, const float speed, const float mass, const float width, const float height, const glm::vec4 & colour, const bool collision) :
	Rigidbody(BOX, position, glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0, mass, collision)
{
	m_width = width;
	m_height = height;
	m_colour = colour;
}
AABB::~AABB()
{
}

void AABB::MakeGizmo()
{
	aie::Gizmos::add2DAABB(m_position, GetExtents(), m_colour);
}