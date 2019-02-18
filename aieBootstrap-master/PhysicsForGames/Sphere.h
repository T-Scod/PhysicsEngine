#pragma once

#include "Rigidbody.h"

// circle
class Sphere : public Rigidbody
{
public:
	Sphere(const glm::vec2& position, const glm::vec2& velocity, const float radius, const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const float elasticity = 1.0f);
	Sphere(const glm::vec2& position, const float inclination, const float speed, const float radius, const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const float elasticity = 1.0f);
	~Sphere();

	// draws the circle
	virtual void MakeGizmo();

	float GetRadius() const { return m_radius; }

protected:
	// stores the radius of the circle
	float m_radius;
};