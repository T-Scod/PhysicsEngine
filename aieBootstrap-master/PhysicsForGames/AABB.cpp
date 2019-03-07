#include "AABB.h"

AABB::AABB(const glm::vec2 & position, const glm::vec2 & velocity, const float width, const float height, const float mass,
	const glm::vec4 & colour, const bool kinematic, const bool staticRigidbody,
	const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(BOX, position, velocity, 0.0f, 0.0f, mass, // specified rotation and angular velocity is 0 because the drawn box does not rotate
		colour, kinematic, staticRigidbody, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	m_width = width;
	m_height = height;
	m_moment = 1.0f / 12.0f * mass * width * height;
}
AABB::AABB(const glm::vec2 & position, const float inclination, const float speed, const float width, const float height, const float mass,
	const glm::vec4 & colour, const bool kinematic, const bool staticRigidbody,
	const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(BOX, position, glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0.0f, 0.0f, mass, // specified rotation and angular velocity is 0 because the drawn box does not rotate
		colour, kinematic, staticRigidbody, elasticity, linearDrag, angularDrag, 탎, 탃) // velocity is worked out using inclination and speed
{
	m_width = width;
	m_height = height;
	m_moment = 1.0f / 12.0f * mass * width * height;
}
AABB::~AABB()
{
}

void AABB::MakeGizmo()
{
	// uses gizmos to draw a box
	aie::Gizmos::add2DAABB(m_position, GetExtents(), m_colour);
}

void AABB::SetWidth(const float width)
{
	m_width = width;
}
void AABB::SetHeight(const float height)
{
	m_height = height;
}

std::vector<glm::vec2> AABB::GetCorners() const
{
	std::vector<glm::vec2> corners;
	corners.push_back(GetMin());					// bottom left
	corners.push_back({ GetMin().x, GetMax().y });	// top left
	corners.push_back(GetMax());					// top right
	corners.push_back({ GetMax().x, GetMin().y });	// bottom right
	return corners;
}