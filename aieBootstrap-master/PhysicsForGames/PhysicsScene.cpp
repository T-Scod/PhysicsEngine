#include "PhysicsScene.h"
#include <list>
#include <iostream>
#include <limits>
#include "Plane.h"
#include "Sphere.h"
#include "AABB.h"
#include "Poly.h"

// function pointer array for doing collisions
typedef bool(*fn)(PhysicsObject*, PhysicsObject*, const glm::vec2&, const float);

// collection of different collision check functions
static fn collisionFunctionArray[SHAPE_COUNT][SHAPE_COUNT] =
{
	{PhysicsScene::Plane2Plane, PhysicsScene::Plane2Sphere, PhysicsScene::Plane2Box, PhysicsScene::Plane2Poly},
	{PhysicsScene::Sphere2Plane, PhysicsScene::Sphere2Sphere, PhysicsScene::Sphere2Box, PhysicsScene::Sphere2Poly},
	{PhysicsScene::Box2Plane, PhysicsScene::Box2Sphere, PhysicsScene::Box2Box, PhysicsScene::Box2Poly},
	{PhysicsScene::Poly2Plane, PhysicsScene::Poly2Sphere, PhysicsScene::Poly2Box, PhysicsScene::Poly2Poly}
};

PhysicsScene::PhysicsScene()
{
	m_timeStep = 0.01f;
	m_gravity = glm::vec2(0.0f, 0.0f);
}
PhysicsScene::PhysicsScene(const glm::vec2& gravity, const float timeStep)
{
	m_gravity = gravity;
	m_timeStep = timeStep;
}
PhysicsScene::~PhysicsScene()
{
	// deallocates the actors
	for (auto pActor : m_actors)
	{
		if (pActor != nullptr)
		{
			delete pActor;
			pActor = nullptr;
		}
	}
	m_actors.clear();
}

void PhysicsScene::AddActor(PhysicsObject * actor)
{
	m_actors.push_back(actor);
}
bool PhysicsScene::RemoveActor(PhysicsObject * actor)
{
	// searches through the vector for the object
	for (unsigned int i = 0; i < m_actors.size(); i++)
	{
		// checks if the object was found
		if (m_actors[i] == actor)
		{
			// increments an iterator that begins at the start of the vector by the current index
			// removes the object at the index
			m_actors.erase(m_actors.begin() + i);
			return true;
		}
	}

	return false;
}

// update physics at a fixed time step
void PhysicsScene::Update(const float dt)
{
	static float accumulatedTime = 0.0f;
	accumulatedTime += dt;

	while (accumulatedTime >= m_timeStep)
	{
		// calls fixed update on all actors
		for (auto pActor : m_actors)
		{
			pActor->FixedUpdate(m_gravity, m_timeStep);
		}

		// check for collisions
		CheckForCollision();

		accumulatedTime -= m_timeStep;
	}
}
// draws all the actors
void PhysicsScene::UpdateGizmos()
{
	for (auto pActor : m_actors)
	{
		pActor->MakeGizmo();
	}
}
// calls the debug function of each actor
void PhysicsScene::DebugScene()
{
	int count = 0;
	for (auto pActor : m_actors)
	{
		std::cout << count << " : ";
		pActor->Debug();
		count++;
	}
}

// checks for collision between all actors in the scene
void PhysicsScene::CheckForCollision()
{	
	int actorCount = m_actors.size();

	// need to check for collision against all objects except this one
	for (int outer = 0; outer < actorCount - 1; outer++)
	{
		for (int inner = outer + 1; inner < actorCount; inner++)
		{
			PhysicsObject* object1 = m_actors[outer];
			PhysicsObject* object2 = m_actors[inner];
			int shapeID1 = object1->GetShapeType();
			int shapeID2 = object2->GetShapeType();

			fn collisionFunctionPtr = collisionFunctionArray[shapeID1][shapeID2];
			if (collisionFunctionPtr != nullptr)
			{
				// did a collision occur
				collisionFunctionPtr(object1, object2, m_gravity, m_timeStep);
			}
		}
	}
}

#pragma region Plane Collision
// does nothing because planes don't collide
bool PhysicsScene::Plane2Plane(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity, const float timeStep)
{
	return false;
}
// swaps the order of the objects and passes them into the opposite function
bool PhysicsScene::Plane2Sphere(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity, const float timeStep)
{
	return Sphere2Plane(obj2, obj1, gravity, timeStep);
}
// swaps the order of the objects and passes them into the opposite function
bool PhysicsScene::Plane2Box(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity, const float timeStep)
{
	return Box2Plane(obj2, obj1, gravity, timeStep);
}
// swaps the order of the objects and passes them into the opposite function
bool PhysicsScene::Plane2Poly(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2 & gravity, const float timeStep)
{
	return Poly2Plane(obj2, obj1, gravity, timeStep);
}
#pragma endregion

#pragma region Sphere Collision
bool PhysicsScene::Sphere2Plane(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity, const float timeStep)
{
	// try to cast objects to sphere and plane
	Sphere* sphere = dynamic_cast<Sphere*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);
	// if successful then test for collision
	if (sphere != nullptr && plane != nullptr)
	{
		// uses the plane's normal as the collision normal
		glm::vec2 normal = plane->GetNormal();
		// determines the distance between the circle and the plane
		float sphereToPlane = glm::dot(sphere->GetPosition(), normal) - plane->GetDistance();

		// the amount the circle overlaps the plane
		float overlap = sphere->GetRadius() - sphereToPlane;
		// if the overlap amount is positive then a collision occured
		if (overlap >= 0)
		{
			// uses the circle's velocity as the relative velocity
			glm::vec2 relativeVelocity = sphere->GetVelocity();

			// sphere to plane restitution
			if (glm::dot(relativeVelocity + (gravity * timeStep), normal) > 0.0f)
			{
				ApplyResitiution(sphere, relativeVelocity + (gravity * timeStep), normal, overlap);
			}
			else
			{
				ApplyResitiution(sphere, -(relativeVelocity + (gravity * timeStep)), normal, overlap);
			}

			// uses the elasticity of the circle
			float elasticity = sphere->GetElasticity();
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// for planes the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m)
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / sphere->GetMass()));

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;

			glm::vec2 contact = sphere->GetPosition() + (normal * -sphere->GetRadius());

			// applys the friction force to the object
			ApplyFriction(sphere, force, contact, gravity, timeStep, plane->GetStaticFriction(), plane->GetKineticFriction());

			return true;
		}
	}

	return false;
}
bool PhysicsScene::Sphere2Sphere(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity, const float timeStep)
{
	// try to cast objects to spheres
	Sphere* sphere1 = dynamic_cast<Sphere*>(obj1);
	Sphere* sphere2 = dynamic_cast<Sphere*>(obj2);
	// if successful then test for collision
	if (sphere1 != nullptr && sphere2 != nullptr)
	{
		// determines the distance between the two circles
		float distance = glm::distance(sphere1->GetPosition(), sphere2->GetPosition());
		// sum of the two radii
		float radii = sphere1->GetRadius() + sphere2->GetRadius();

		// checks if the distance is less than the sum of the two radii
		if (distance <= radii)
		{
			// the collision normal is the vector between the two circle's positions
			glm::vec2 normal = glm::normalize(sphere2->GetPosition() - sphere1->GetPosition());
			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = sphere2->GetVelocity() - sphere1->GetVelocity();

			float p1 = sphere1->GetMass() * glm::length(sphere1->GetVelocity());
			float p2 = sphere2->GetMass() * glm::length(sphere2->GetVelocity());
			// the sum of the two masses
			float momentum = p1 + p2;
			// the amount the two circles overlap
			float overlap = radii - distance;
			// seperates the overlap based on the ratio of the masses of the two circles
			float overlap1 = overlap * (p2 / momentum);
			float overlap2 = overlap * (p1 / momentum);

			// sphere to sphere restitution
			if (glm::dot(sphere1->GetVelocity() + (gravity * timeStep), -normal) > 0.0f)
			{
				ApplyResitiution(sphere1, sphere1->GetVelocity() + (gravity * timeStep), -normal, overlap1);
			}
			else
			{
				ApplyResitiution(sphere1, -(sphere1->GetVelocity() + (gravity * timeStep)), -normal, overlap1);
			}
			if (glm::dot(sphere2->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
			{
				ApplyResitiution(sphere2, sphere2->GetVelocity() + (gravity * timeStep), normal, overlap2);
			}
			else
			{
				ApplyResitiution(sphere2, -(sphere2->GetVelocity() + (gravity * timeStep)), normal, overlap2);
			}

			// uses the average elasticity of the two circles
			float elasticity = (sphere1->GetElasticity() + sphere2->GetElasticity()) / 2.0f;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / sphere1->GetMass()) + (1 / sphere2->GetMass())));

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;

			glm::vec2 contact = 0.5f * (sphere1->GetPosition() + sphere2->GetPosition());

			// applys the friction force on each object
			ApplyFriction(sphere1, -force, contact, gravity, timeStep, sphere2->GetStaticFriction(), sphere2->GetKineticFriction());
			ApplyFriction(sphere2, force, contact, gravity, timeStep, sphere1->GetStaticFriction(), sphere1->GetKineticFriction());

			return true;
		}
	}

	return false;
}
bool PhysicsScene::Sphere2Box(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity, const float timeStep)
{
	// try to cast objects to sphere and box
	Sphere* sphere = dynamic_cast<Sphere*>(obj1);
	AABB* box = dynamic_cast<AABB*>(obj2);
	// if successful then test for collision
	if (sphere != nullptr && box != nullptr)
	{
		// clamps the circle's position to the box's bounds, i.e. the closest point on the box relative to the circle
		glm::vec2 clamp = glm::clamp(sphere->GetPosition(), box->GetMin(), box->GetMax());
		// if the distance between the closest point and the circle's is less than the circle's radius then a collision occured
		if (glm::distance(clamp, sphere->GetPosition()) < sphere->GetRadius())
		{
			glm::vec2 normal;
			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = box->GetVelocity() - sphere->GetVelocity();
			if (clamp == sphere->GetPosition())
			{

			}
			else
			{
				// the collision normal between the circle and the box will be perpendicular to one of the box's sides
				normal = glm::normalize(clamp - sphere->GetPosition());
			}

			// the amount the circle and box overlap
			float overlap = 0.0f;
			// checks if the circle's position is inside the box
			if (glm::distance(clamp, sphere->GetPosition()) < 0.01f)
			{
				// checks if the collision normal is horizontal
				if (normal.x != 0)
				{
					// calculates the overlap amount using the distance between the edge of the circle and the box's x position
					overlap = box->GetExtents().x - (box->GetPosition().x - clamp.x) + sphere->GetRadius();
				}
				// checks if the collision normal is vertical
				else if (normal.y != 0)
				{
					// calculates the overlap amount using the distance between the edge of the circle and the box's y position
					overlap = box->GetExtents().y - (box->GetPosition().y - clamp.y) + sphere->GetRadius();
				}
			}
			else // circle's center is not in the box
			{
				// scales the vector between the clamped position and the circle's center by the radius
				glm::vec2 vectorViaClamp = normal * sphere->GetRadius();
				// the overlap amount is the distance between the clamped position and the scaled clamped position
				overlap = glm::distance(clamp, vectorViaClamp + sphere->GetPosition());
			}

			float p1 = sphere->GetMass() * glm::length(sphere->GetVelocity());
			float p2 = box->GetMass() * glm::length(box->GetVelocity());
			// the sum of the two masses
			float momentum = sphere->GetMass() + box->GetMass();
			// seperates the overlap based on the ratio of the masses of the two objects
			float overlap1 = overlap * (p2 / momentum);
			float overlap2 = overlap * (p1 / momentum);

			// box to sphere restitution
			if (glm::dot(sphere->GetVelocity() + (gravity * timeStep), -normal) > 0.0f)
			{
				ApplyResitiution(sphere, sphere->GetVelocity() + (gravity * timeStep), -normal, overlap1);
			}
			else
			{
				ApplyResitiution(sphere, -(sphere->GetVelocity() + (gravity * timeStep)), -normal, overlap1);
			}
			if (glm::dot(box->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
			{
				ApplyResitiution(box, box->GetVelocity() + (gravity * timeStep), normal, overlap2);
			}
			else
			{
				ApplyResitiution(box, -(box->GetVelocity() + (gravity * timeStep)), normal, overlap2);
			}

			// uses the average elasticity of the two objects
			float elasticity = (box->GetElasticity() + sphere->GetElasticity()) / 2.0f;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / box->GetMass()) + (1 / sphere->GetMass())));

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;

			// applys the friction force on each object
			ApplyFriction(sphere, -force, clamp, gravity, timeStep, box->GetStaticFriction(), box->GetKineticFriction());
			ApplyFriction(box, force, clamp, gravity, timeStep, sphere->GetStaticFriction(), sphere->GetKineticFriction());

			return true;
		}
	}

	return false;
}
// swaps the order of the objects and passes them into the opposite function
bool PhysicsScene::Sphere2Poly(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2 & gravity, const float timeStep)
{
	return Poly2Sphere(obj2, obj1, gravity, timeStep);
}
#pragma endregion

#pragma region Box Collision
bool PhysicsScene::Box2Plane(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity, const float timeStep)
{
	// try to cast objects to box and plane
	AABB* box = dynamic_cast<AABB*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);
	// if successful then test for collision
	if (plane != nullptr && box != nullptr)
	{
		std::vector<glm::vec2> corners = box->GetCorners();
		bool intersections[4] = { false };

		for (unsigned int i = 0; i < 4; i++)
		{
			// checks if the corner is behind the plane
			if (glm::dot(plane->GetNormal(), corners[i]) - plane->GetDistance() <= 0)
			{
				// this corner intersects with the plane
				intersections[i] = true;
			}
		}

		// if any of the corners intersect with the plane then a collision occured
		if (intersections[0] || intersections[1] || intersections[2] || intersections[3])
		{
			// uses the plane's normal as the collision normal
			glm::vec2 normal = plane->GetNormal();
			// uses the box's velocity as the relative velocity
			glm::vec2 relativeVelocity = box->GetVelocity();

			// the amount the box overlaps the plane
			float overlap = 0;

			// out of the intersecting corners, finds which one is furthest from the plane to determine the overlap amount
			for (int i = 0; i < 4; i++)
			{
				// checks if the corner is behind the plane
				if (intersections[i])
				{
					// gets the corner's distance from the plane
					float distance = glm::dot(corners[i], normal) - plane->GetDistance();
					// checks if the distance is greater the currently stored overlap amount
					if (fabsf(overlap) < fabsf(distance))
					{
						// stores the distance as the new overlap
						overlap = distance;
					}
				}
			}

			// box to plane restitution
			if (glm::dot(box->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
			{
				ApplyResitiution(box, box->GetVelocity() + (gravity * timeStep), normal, overlap);
			}
			else
			{
				ApplyResitiution(box, -(box->GetVelocity() + (gravity * timeStep)), normal, overlap);
			}

			// uses the box's elasticity
			float elasticity = box->GetElasticity();
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// for planes the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m)
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / box->GetMass()));

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;
			
			glm::vec2 contact = box->GetPosition() + (-normal * overlap);

			// applys the friction force on the object
			ApplyFriction(box, force, contact, gravity, timeStep, plane->GetStaticFriction(), plane->GetKineticFriction());

			return true;
		}
	}

	return false;
}
// swaps the order of the objects and passes them into the opposite function
bool PhysicsScene::Box2Sphere(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity, const float timeStep)
{
	return Sphere2Box(obj2, obj1, gravity, timeStep);
}
bool PhysicsScene::Box2Box(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity, const float timeStep)
{
	// try to cast objects to boxes
	AABB* box1 = dynamic_cast<AABB*>(obj1);
	AABB* box2 = dynamic_cast<AABB*>(obj2);
	// if successful then test for collision
	if (box1 != nullptr && box2 != nullptr)
	{
		// checks if there is a gap between any of the axis of the boxes
		if (box1->GetMin().x > box2->GetMax().x ||
			box1->GetMin().y > box2->GetMax().y ||
			box1->GetMax().x < box2->GetMin().x ||
			box1->GetMax().y < box2->GetMin().y)
		{
			// no collision
			return false;
		}
		else // collision occured
		{
			glm::vec2 clamp = glm::clamp(box1->GetPosition(), box2->GetMin(), box2->GetMax());
			// the collision normal between the boxes will be perpendicular to one of the box's sides
			glm::vec2 normal = glm::normalize(clamp - box1->GetPosition());
			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = box2->GetVelocity() - box1->GetVelocity();

			float min1 = std::numeric_limits<float>::max();
			float max1 = std::numeric_limits<float>::lowest();
			std::vector<glm::vec2> corners = box1->GetCorners();
			for each (glm::vec2 corner in corners)
			{
				float dot = glm::dot(corner, normal);
				if (dot < min1)
				{
					min1 = dot;
				}
				if (dot > max1)
				{
					max1 = dot;
				}
			}
			float min2 = std::numeric_limits<float>::max();
			float max2 = std::numeric_limits<float>::lowest();
			corners = box2->GetCorners();
			for each (glm::vec2 corner in corners)
			{
				float dot = glm::dot(corner, normal);
				if (dot < min2)
				{
					min2 = dot;
				}
				if (dot > max2)
				{
					max2 = dot;
				}
			}

			// the amount the boxes overlap
			float overlap = 0.0f;

			// finds the smaller overlap between the boxes and stores it as the overlap amount
			if ((max1 - min2) < (max2 - min1))
			{
				overlap = max1 - min2;
			}
			else
			{
				overlap = max2 - min1;
			}
			
			float p1 = box1->GetMass() * glm::length(box1->GetVelocity());
			float p2 = box2->GetMass() * glm::length(box2->GetVelocity());
			// sum of the two masses
			float momentum = p1 + p2;
			// seperates the overlap based on the ratio of the masses of the two objects
			float overlap1 = overlap * (p2 / momentum);
			float overlap2 = overlap * (p1 / momentum);

			// box to box restitution
			if (glm::dot(box1->GetVelocity() + (gravity * timeStep), -normal) > 0.0f)
			{
				ApplyResitiution(box1, box1->GetVelocity() + (gravity * timeStep), -normal, overlap1);
			}
			else
			{
				ApplyResitiution(box1, -(box1->GetVelocity() + (gravity * timeStep)), -normal, overlap1);
			}
			if (glm::dot(box2->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
			{
				ApplyResitiution(box2, box2->GetVelocity() + (gravity * timeStep), normal, overlap2);
			}
			else
			{
				ApplyResitiution(box2, -(box2->GetVelocity() + (gravity * timeStep)), normal, overlap2);
			}

			// uses the average elasticity of the two objects
			float elasticity = (box1->GetElasticity() + box2->GetElasticity()) / 2.0f;;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / box1->GetMass()) + (1 / box2->GetMass())));

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;

			glm::vec2 contact = box1->GetPosition() + (normal * box1->GetExtents());

			// applys the friction force on each object
			ApplyFriction(box1, -force, contact, gravity, timeStep, box2->GetStaticFriction(), box2->GetKineticFriction());
			ApplyFriction(box2, force, contact, gravity, timeStep, box1->GetStaticFriction(), box1->GetKineticFriction());

			return true;
		}
	}

	return false;
}
// swaps the order of the objects and passes them into the opposite function
bool PhysicsScene::Box2Poly(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2 & gravity, const float timeStep)
{
	return Poly2Box(obj2, obj1, gravity, timeStep);
}
#pragma endregion

#pragma region Poly Collision
bool PhysicsScene::Poly2Plane(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2 & gravity, const float timeStep)
{
	return false;
}
bool PhysicsScene::Poly2Sphere(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2 & gravity, const float timeStep)
{
	return false;
}
bool PhysicsScene::Poly2Box(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2 & gravity, const float timeStep)
{
	return false;
}
bool PhysicsScene::Poly2Poly(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2 & gravity, const float timeStep)
{
	Poly* poly1 = dynamic_cast<Poly*>(obj1);
	Poly* poly2 = dynamic_cast<Poly*>(obj2);
	if (poly1 != nullptr && poly2 != nullptr)
	{
		if (glm::distance(poly1->GetPosition(), poly2->GetPosition()) <= poly1->GetRadius() + poly2->GetRadius())
		{
			float overlap = std::numeric_limits<float>::max();
			glm::vec2 normal = glm::vec2(0.0f, 0.0f);
			std::vector<glm::vec2> axis1 = poly1->GetAxis();
			std::vector<glm::vec2> axis2 = poly2->GetAxis();
			
			for each (glm::vec2 axis in axis1)
			{
				glm::vec2 proj1 = poly1->Project(axis);
				glm::vec2 proj2 = poly2->Project(axis);

				if (!poly1->Overlap(proj1, proj2))
				{
					return false;
				}
				else
				{
					float o = poly1->GetOverlap(proj1, proj2);
					if (o < overlap)
					{
						overlap = o;
						normal = axis;
					}
				}
			}

			for each (glm::vec2 axis in axis2)
			{
				glm::vec2 proj1 = poly1->Project(axis);
				glm::vec2 proj2 = poly2->Project(axis);

				if (!poly1->Overlap(proj1, proj2))
				{
					return false;
				}
				else
				{
					float o = poly1->GetOverlap(proj1, proj2);
					if (o < overlap)
					{
						overlap = o;
						normal = axis;
					}
				}
			}

			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = poly2->GetVelocity() - poly1->GetVelocity();
			float p1 = poly1->GetMass() * glm::length(poly1->GetVelocity());
			float p2 = poly2->GetMass() * glm::length(poly2->GetVelocity());
			// sum of the two masses
			float momentum = p1 + p2;
			// seperates the overlap based on the ratio of the masses of the two objects
			float overlap1 = overlap * (p2 / momentum);
			float overlap2 = overlap * (p1 / momentum);

			// box to box restitution
			if (glm::dot(poly1->GetVelocity() + (gravity * timeStep), -normal) > 0.0f)
			{
				ApplyResitiution(poly1, poly1->GetVelocity() + (gravity * timeStep), -normal, overlap1);
			}
			else
			{
				ApplyResitiution(poly1, -(poly1->GetVelocity() + (gravity * timeStep)), -normal, overlap1);
			}
			if (glm::dot(poly2->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
			{
				ApplyResitiution(poly2, poly2->GetVelocity() + (gravity * timeStep), normal, overlap2);
			}
			else
			{
				ApplyResitiution(poly2, -(poly2->GetVelocity() + (gravity * timeStep)), normal, overlap2);
			}

			// uses the average elasticity of the two objects
			float elasticity = (poly1->GetElasticity() + poly2->GetElasticity()) / 2.0f;;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / poly1->GetMass()) + (1 / poly2->GetMass())));

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;

			std::vector<glm::vec2> contactPoints = poly1->ContactPoints(poly2, normal);
			glm::vec2 contact = glm::vec2(0.0f, 0.0f);

			if (contactPoints.size() <= 0)
			{
				return false;
			}

			for each (glm::vec2 point in contactPoints)
			{
				contact += point;
			}
			contact /= contactPoints.size();

			// applys the friction force on each object
			ApplyFriction(poly1, -force, contact, gravity, timeStep, poly2->GetStaticFriction(), poly2->GetKineticFriction());
			ApplyFriction(poly2, force, contact, gravity, timeStep, poly1->GetStaticFriction(), poly1->GetKineticFriction());
		}
	}

	return false;
}
#pragma endregion

void PhysicsScene::ApplyFriction(Rigidbody * obj, const glm::vec2 & force, const glm::vec2& contact, const glm::vec2 gravity, const float timeStep, const float µs, const float µk)
{
	// the velocity after collision
	glm::vec2 velocity = obj->GetVelocity() + (force / obj->GetMass()) + (gravity * timeStep);
	// the collision normal
	glm::vec2 normal = glm::normalize(force);
	// the force due to friction is perpendicular to the normal force
	glm::vec2 frictionForce = glm::vec2(normal.y, -normal.x);
	// projects the resolution force on the friction force
	float frictionDirection = glm::dot(frictionForce, velocity);
	// if the result is positive then the friction force is in the wrong direction
	// the friction force is always against the resolution force
	if (frictionDirection > 0.0f)
	{
		frictionForce *= -1.0f;
	}

	// checks if the object is not moving
	if ((obj->GetVelocity() - (gravity * timeStep)) == glm::vec2(0.0f, 0.0f))
	{
		// multiplies the friction force by the static friction coefficient
		frictionForce *= (obj->GetStaticFriction() + µs) / 2.0f;
	}
	else
	{
		// multiplies the friction force by the kinetic friction coefficient
		frictionForce *= (obj->GetKineticFriction() + µk) / 2.0f;
	}

	// adds the friction force to the velocity after the collision
	velocity += frictionForce;
	// projects the velocity on the friction force to see if the force overcame the friction
	frictionDirection = glm::dot(frictionForce, velocity);
	// if the projection is negative then the force overcame the friction
	if (frictionDirection <= 0.0f)
	{
		// applies the force only on the circle because the plane is static
		obj->ApplyForce(force + frictionForce, contact - obj->GetPosition());
	}
	else // did not overcome friction
	{
		// stops the object
		obj->SetVelocity(glm::vec2(0.0f, 0.0f));
	}
}

void PhysicsScene::ApplyResitiution(Rigidbody * obj, const glm::vec2 & velocity, const glm::vec2 & normal, const float overlap)
{
	// the angle between the velocity and collision normal
	float theta = acosf(glm::dot(glm::normalize(velocity), normal));
	// the amount the box needs to move perpendicular to the collision normal
	float perpendicular = tanf(theta) * overlap;
	// multiplies the velocity in the opposite direction by the magnitude of the vector at ("perpendicular", "overlap")
	glm::vec2 restitution = glm::length(glm::vec2(perpendicular, overlap)) * glm::normalize(velocity);
	// adds the restitution to the current object's position
	obj->SetPosition(obj->GetPosition() + restitution);
}