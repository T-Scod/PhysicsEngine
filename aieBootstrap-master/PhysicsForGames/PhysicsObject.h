#pragma once

#include <glm\ext.hpp>
#include <Gizmos.h>

enum ShapeType
{
	PLANE = 0, // line
	SPHERE, // circle
	BOX // square
};

// the amount of shapes being delt with
const unsigned int SHAPE_COUNT = BOX + 1;

// abstract class
class PhysicsObject
{
protected:
	PhysicsObject(const ShapeType& a_shapeID, const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const float 탎 = 0.0f, const float 탃 = 0.0f) :
		m_shapeID(a_shapeID), m_colour(colour), m_탎(탎), m_탃(탃) {}

public:
	// updates with a fixed time step
	virtual void FixedUpdate(const glm::vec2& gravity, const float timeStep) = 0;
	// used to check the variable values
	virtual void Debug() = 0;
	// draws the object
	virtual void MakeGizmo() = 0;
	// used to reset the position of the object
	virtual void ResetPosition() {}

	ShapeType GetShapeType() const { return m_shapeID; }
	float GetStaticFriction() const { return m_탎; }
	float GetKineticFriction() const { return m_탃; }
	glm::vec4 GetColour() const { return m_colour; }

protected:
	// stores the type of shape
	ShapeType m_shapeID;
	// coefficient of static friction
	float m_탎;
	// coefficient of kinetic friction
	float m_탃;
	// stores the colour of the object
	glm::vec4 m_colour;
};