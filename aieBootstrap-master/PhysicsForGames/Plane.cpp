#include "Plane.h"
#include <iostream>

Plane::Plane(const glm::vec2 & normal, const float distance, const glm::vec4& colour, const bool kinematic, const float 탎, const float 탃) :
	PhysicsObject(PLANE, colour, kinematic, 탎, 탃)
{
	m_normal = normal;
	m_distanceToOrigin = distance;
}
Plane::Plane(const float inclination, const float distance, const glm::vec4 & colour, const bool kinematic, const float 탎, const float 탃) :
	PhysicsObject(PLANE, colour, kinematic, 탎, 탃)
{
	m_normal = glm::vec2(-sinf(inclination), cosf(inclination));
	m_distanceToOrigin = distance;
}
Plane::~Plane()
{
}

void Plane::Debug()
{
	std::cout << "Shape ID: " << m_shapeID << std::endl;
	std::cout << "Normal: " << m_normal.x << ", " << m_normal.y << std::endl;
	std::cout << "Distance: " << m_distanceToOrigin << std::endl;
	std::cout << "Static Friction: " << m_탎 << std::endl;
	std::cout << "Kinetic Friction: " << m_탃 << std::endl;
}

void Plane::MakeGizmo()
{
	// length of the line
	float lineSegmentLength = 300.0f;
	// location of the center of the line
	glm::vec2 centerPoint = m_normal * m_distanceToOrigin;
	// easy to rotate through 90 degrees around z
	glm::vec2 parallel(m_normal.y, -m_normal.x);
	// uses the direction of the parallel normal for the direction of the line
	glm::vec2 startPos = centerPoint + (parallel * lineSegmentLength);
	glm::vec2 endPos = centerPoint - (parallel * lineSegmentLength);
	// draws the line between the 2 positions and of the specified colour
	aie::Gizmos::add2DLine(startPos, endPos, m_colour);
}