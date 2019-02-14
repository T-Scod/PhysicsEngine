#pragma once

#include "Rigidbody.h"

// circle
class Sphere : public Rigidbody
{
public:
	Sphere(const glm::vec2& position, const glm::vec2& velocity, const float mass, const float radius, const glm::vec4& colour, const bool collision = true);
	Sphere(const glm::vec2& position, const float inclination, const float speed, const float mass, const float radius, const glm::vec4& colour, const bool collision = true);
	~Sphere();

	// draws the circle
	virtual void MakeGizmo();

	float GetRadius() const { return m_radius; }
	glm::vec4 GetColour() const { return m_colour; }

protected:
	// stores the radius of the circle
	float m_radius;
	// stores the colour of the circle
	glm::vec4 m_colour;
};