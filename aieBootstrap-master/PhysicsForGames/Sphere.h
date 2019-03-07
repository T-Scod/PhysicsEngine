#pragma once

#include "Rigidbody.h"

// circle
class Sphere : public Rigidbody
{
public:
	Sphere(const glm::vec2& position, const glm::vec2& velocity, const float radius, const float mass,
		const glm::vec4& colour = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), const bool kinematic = false, const bool staticRigidbody = false,
		const float elasticity = 1.0f, const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	// determines the velocity of the object using an incline (in radians) and a scalar (the speed)
	Sphere(const glm::vec2& position, const float inclination, const float speed, const float radius, const float mass,
		const glm::vec4& colour = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), const bool kinematic = false, const bool staticRigidbody = false,
		const float elasticity = 1.0f, const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	~Sphere();

	// draws the circle
	virtual void MakeGizmo();

	void SetRadius(const float radius);
	float GetRadius() const { return m_radius; }

protected:
	// stores the radius of the circle
	float m_radius;
};