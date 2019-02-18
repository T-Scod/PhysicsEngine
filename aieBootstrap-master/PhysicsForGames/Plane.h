#pragma once

#include "PhysicsObject.h"
#include "Rigidbody.h"

class Plane : public PhysicsObject
{
public:
	Plane();
	// given values for initialisation
	Plane(const glm::vec2& normal, const float distance, const glm::vec4& colour = glm::vec4(1, 1, 1, 1));
	~Plane();

	// does nothing because planes don't move
	virtual void FixedUpdate(const glm::vec2& gravity, const float timeStep) {}
	// prints the object values
	virtual void Debug();
	// draws the line
	virtual void MakeGizmo();
	virtual void ResetPosition() {}

	glm::vec2 GetNormal() const { return m_normal; }
	float GetDistance() const { return m_distanceToOrigin; }

protected:
	// normalised vector perpendicular to the plane
	glm::vec2 m_normal;
	// distance between the plane and the origin
	float m_distanceToOrigin;
};