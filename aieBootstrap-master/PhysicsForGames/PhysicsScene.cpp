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

			// gets the function based on which 2 objects are being checked against
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
			// if either object is kinematic then there is no collision resolution
			// if the circle is static then there is no collision resolution because both objects will not move
			if (sphere->GetKinematic() || plane->GetKinematic() || sphere->GetStatic())
			{
				return true;
			}

			// uses the circle's velocity as the relative velocity
			glm::vec2 relativeVelocity = sphere->GetVelocity();

			// sphere to plane restitution
			// determines which direction of velocity is away from the other object
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

			// the contact point is found by scaling the normal by the radius and taking that vector from the position
			glm::vec2 contact = sphere->GetPosition() - (normal * sphere->GetRadius());

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
			// if either object is kinematic then there is no collision resolution
			// if both objects are static then there will be no collision resolution because neither object will move
			if (sphere1->GetKinematic() || sphere2->GetKinematic() || (sphere1->GetStatic() && sphere2->GetStatic()))
			{
				return true;
			}

			// the collision normal is the vector between the circles' positions
			glm::vec2 normal = glm::normalize(sphere2->GetPosition() - sphere1->GetPosition());
			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = sphere2->GetVelocity() - sphere1->GetVelocity();

			// the individual momentums of the circles
			float p1 = sphere1->GetMass() * glm::length(sphere1->GetVelocity());
			float p2 = sphere2->GetMass() * glm::length(sphere2->GetVelocity());
			// the sum of the two momentums
			float momentum = p1 + p2;
			// the amount the two circles overlap
			float overlap = radii - distance;
			// seperates the overlap based on the ratio of the momentums of the two circles
			float overlap1 = overlap * (p2 / momentum);
			float overlap2 = overlap * (p1 / momentum);

			// uses the average elasticity of the two circles
			float elasticity;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			float j;

			// sphere to sphere restitution
			// checks if both objects are not static
			if (!sphere1->GetStatic() && !sphere2->GetStatic())
			{
				// determines which direction of velocity is away from the other object
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

				elasticity = (sphere1->GetElasticity() + sphere2->GetElasticity()) / 2.0f;
				// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / sphere1->GetMass()) + (1 / sphere2->GetMass())));
			}
			// checks if only sphere1 is not static
			else if (!sphere1->GetStatic() && sphere2->GetStatic())
			{
				// gives the object the full overlap amount because the other object is static
				if (glm::dot(sphere1->GetVelocity() + (gravity * timeStep), -normal) > 0.0f)
				{
					ApplyResitiution(sphere1, sphere1->GetVelocity() + (gravity * timeStep), -normal, overlap);
				}
				else
				{
					ApplyResitiution(sphere1, -(sphere1->GetVelocity() + (gravity * timeStep)), -normal, overlap);
				}

				// uses the elasticity of sphere1
				elasticity = sphere1->GetElasticity();
				// the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m), because sphere2 has infinite mass when static
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / sphere1->GetMass()));
			}
			// checks if only sphere 2 is not static
			else if (sphere1->GetStatic() && !sphere2->GetStatic())
			{
				// gives the object the full overlap amount because the other object is static
				if (glm::dot(sphere2->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
				{
					ApplyResitiution(sphere2, sphere2->GetVelocity() + (gravity * timeStep), normal, overlap1);
				}
				else
				{
					ApplyResitiution(sphere2, -(sphere2->GetVelocity() + (gravity * timeStep)), normal, overlap1);
				}

				// uses the elasticity of sphere2
				elasticity = sphere2->GetElasticity();
				// the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m), because sphere1 has infinite mass when static
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / sphere2->GetMass()));
			}

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;

			glm::vec2 contact = 0.5f * (sphere1->GetPosition() + sphere2->GetPosition());

			// applys the friction force on each object if they are not static
			if (!sphere1->GetStatic())
			{
				ApplyFriction(sphere1, -force, contact, gravity, timeStep, sphere2->GetStaticFriction(), sphere2->GetKineticFriction());
			}
			if (!sphere2->GetStatic())
			{
				ApplyFriction(sphere2, force, contact, gravity, timeStep, sphere1->GetStaticFriction(), sphere1->GetKineticFriction());
			}

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
		// if the distance between the closest point and the circle is less than the circle's radius then a collision occured
		if (glm::distance(clamp, sphere->GetPosition()) < sphere->GetRadius())
		{
			// if either object is kinematic then there is no collision resolution
			// if both objects are static then there will be no collision resolution because neither object will move
			if (sphere->GetKinematic() || box->GetKinematic() || (sphere->GetStatic() && box->GetStatic()))
			{
				return true;
			}

			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = box->GetVelocity() - sphere->GetVelocity();
			// the collision normal
			glm::vec2 normal = glm::vec2(0.0f, 0.0f);

			// collection of the corners of the AABB
			std::vector<glm::vec2> corners = box->GetCorners();
			// stores the corner closest to the circle
			glm::vec2 closestCorner = normal;
			// checks if the circle hit the corner of the box
			for (glm::vec2 corner : corners)
			{
				// vector between the corner and the box
				glm::vec2 v1 = glm::normalize(corner - box->GetPosition());
				// vector between the circle and the box
				glm::vec2 v2 = glm::normalize(sphere->GetPosition() - box->GetPosition());
				// if the 2 vectors are very similar then use the vector from the circle to the box as the collision normal
				if (glm::distance(v1, v2) < 0.05f)
				{
					normal = v2;
					closestCorner = corner;
					break;
				}
			}
			// the circle did not hit the corner of the box, therefore the collision normal will be horizontal or vertical
			if (normal == glm::vec2(0.0f, 0.0f))
			{
				// the collision normal between the circle and the box will be perpendicular to one of the box's sides
				normal = glm::normalize(sphere->GetPosition() - clamp);
			}

			// the amount the circle and box overlap
			float overlap = 0.0f;
			// checks if the circle's position is inside the box
			if (glm::distance(clamp, sphere->GetPosition()) < 0.01f)
			{
				// checks if the collision normal is vertical or horizontal
				if (normal.y == 0 || normal.x == 0)
				{
					// the absolute values of the collision normal
					glm::vec2 absNormal = glm::vec2(fabsf(normal.x), fabsf(normal.y));
					// gets the distance between the box's max position and the circle's min position
					float o1 = glm::dot(absNormal, box->GetMax()) - glm::dot(absNormal, sphere->GetPosition() - (absNormal * sphere->GetRadius()));
					// gets the distance between the box's min position and the circle's max position
					float o2 = glm::dot(absNormal, sphere->GetPosition() + (absNormal * sphere->GetRadius())) - glm::dot(absNormal, box->GetMin());
					// the smaller of the 2 distances is the overlap amount
					overlap = (o1 < o2) ? o1 : o2;
				}
				// checks if the circle hit a corner
				else if (closestCorner != glm::vec2(0.0f, 0.0f))
				{
					// flip the normal and scale it by the radius then add the vector to the circle's position to get the furthest point from the corner
					glm::vec2 furthestPoint = sphere->GetPosition() - (normal * sphere->GetRadius());
					// the distance between that point and the closest corner is the overlap amount
					overlap = glm::distance(furthestPoint, closestCorner);
				}
			}
			else // circle's center is not in the box
			{
				// scales the vector between the clamped position and the circle's center by the radius
				glm::vec2 vectorViaClamp = normal * sphere->GetRadius();
				// the overlap amount is the distance between the clamped position and the scaled clamped position
				overlap = glm::distance(clamp, sphere->GetPosition() - vectorViaClamp);
			}

			// the individual momentums of the objects
			float p1 = sphere->GetMass() * glm::length(sphere->GetVelocity());
			float p2 = box->GetMass() * glm::length(box->GetVelocity());
			// the sum of the two momentums
			float momentum = p1 + p2;
			// seperates the overlap based on the ratio of the momentums of the two objects
			float overlap1 = overlap * (p2 / momentum);
			float overlap2 = overlap * (p1 / momentum);

			// uses the average elasticity of the two objects
			float elasticity;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			float j;

			// box to sphere restitution
			// checks if both objects are not static
			if (!sphere->GetStatic() && !box->GetStatic())
			{
				// determines which direction of velocity is away from the other object
				if (glm::dot(sphere->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
				{
					ApplyResitiution(sphere, sphere->GetVelocity() + (gravity * timeStep), normal, overlap1);
				}
				else
				{
					ApplyResitiution(sphere, -(sphere->GetVelocity() + (gravity * timeStep)), normal, overlap1);
				}
				if (glm::dot(box->GetVelocity() + (gravity * timeStep), -normal) > 0.0f)
				{
					ApplyResitiution(box, box->GetVelocity() + (gravity * timeStep), -normal, overlap2);
				}
				else
				{
					ApplyResitiution(box, -(box->GetVelocity() + (gravity * timeStep)), -normal, overlap2);
				}

				elasticity = (box->GetElasticity() + sphere->GetElasticity()) / 2.0f;
				// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / box->GetMass()) + (1 / sphere->GetMass())));
			}
			// checks if only the sphere is not static
			else if (!sphere->GetStatic())
			{
				// gives the object the full overlap amount because the other object is static
				if (glm::dot(sphere->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
				{
					ApplyResitiution(sphere, sphere->GetVelocity() + (gravity * timeStep), normal, overlap);
				}
				else
				{
					ApplyResitiution(sphere, -(sphere->GetVelocity() + (gravity * timeStep)), normal, overlap);
				}

				// uses the elasticity of the sphere
				elasticity = sphere->GetElasticity();
				// the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m), because the box has infinite mass when static
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / sphere->GetMass()));
			}
			// checks if only the box is not static
			else if (!box->GetStatic())
			{
				// gives the object the full overlap amount because the other object is static
				if (glm::dot(box->GetVelocity() + (gravity * timeStep), -normal) > 0.0f)
				{
					ApplyResitiution(box, box->GetVelocity() + (gravity * timeStep), -normal, overlap);
				}
				else
				{
					ApplyResitiution(box, -(box->GetVelocity() + (gravity * timeStep)), -normal, overlap);
				}

				// uses the elasticity of the box
				elasticity = box->GetElasticity();
				// the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m), because the sphere has infinite mass when static
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / box->GetMass()));
			}

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;

			// applys the friction force on each object if the object is not static
			if (!sphere->GetStatic())
			{
				ApplyFriction(sphere, -force, clamp, gravity, timeStep, box->GetStaticFriction(), box->GetKineticFriction());
			}
			if (!box->GetStatic())
			{
				ApplyFriction(box, force, clamp, gravity, timeStep, sphere->GetStaticFriction(), sphere->GetKineticFriction());
			}

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
		// collection of the corners of the box
		std::vector<glm::vec2> corners = box->GetCorners();
		// used to determine if the corner intersected with the plane
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
			// if the either object is kinematic then there is no collision resolution
			// if the box is static then there is no collision resolution because both objects will not move
			if (box->GetKinematic() || plane->GetKinematic() || box->GetStatic())
			{
				return true;
			}

			// uses the plane's normal as the collision normal
			glm::vec2 normal = plane->GetNormal();
			// uses the box's velocity as the relative velocity
			glm::vec2 relativeVelocity = box->GetVelocity();

			// the amount the box overlaps the plane
			float overlap = 0;
			// from the intersecting corners, finds which one is furthest from the plane to determine the overlap amount
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
			// determines which direction of velocity is away from the other object
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
			// the contact point is the point on the box furthest down the normal
			glm::vec2 contact = box->GetPosition() - (normal * overlap);

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
			// if either object is kinematic then there is no collision resolution
			// if both objects are static then there will be no collision resolution because neither object will move
			if (box1->GetKinematic() || box2->GetKinematic() || (box1->GetStatic() && box2->GetStatic()))
			{
				return true;
			}

			// the closest point on box2 to box1
			glm::vec2 clamp = glm::clamp(box1->GetPosition(), box2->GetMin(), box2->GetMax());
			glm::vec2 normal = glm::vec2(0.0f, 0.0f);
			// checks if box1 is not inside box2
			if (clamp != box1->GetPosition())
			{
				// the collision normal between the boxes will be the vector between box1 and the clamped point
				normal = glm::normalize(clamp - box1->GetPosition());
				// checks if the normal is not perpendicular to one of the box's sides because a perpendicular normal is favoured in box to box resolution
				if (normal.x != 0.0f && normal.y != 0.0f)
				{
					// the closest point on box1 to box2
					clamp = glm::clamp(box2->GetPosition(), box1->GetMin(), box1->GetMax());
					// checks if box2 is not inside box1
					if (clamp != box2->GetPosition())
					{
						// the collision normal between the boxes will be the vector between box2 and the clamped point
						normal = glm::normalize(box2->GetPosition() - clamp);
					}
					else // box2 is inside box1
					{
						// the normal becomes the vector between the boxes
						normal = glm::normalize(box2->GetPosition() - box1->GetPosition());
					}
				}
			}
			else // box1 is inside box2
			{
				// the normal becomes the vector between the boxes
				normal = glm::normalize(box2->GetPosition() - box1->GetPosition());
			}

			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = box2->GetVelocity() - box1->GetVelocity();

			// uses SAT to determine the overlap amount
			// the smallest projection of box1 onto the normal
			float min1 = std::numeric_limits<float>::max();
			// the largest projection of box1 onto the normal
			float max1 = std::numeric_limits<float>::lowest();
			// the corners of box1
			std::vector<glm::vec2> corners = box1->GetCorners();
			for (glm::vec2 corner : corners)
			{
				// projects the corner onto the normal
				float dot = glm::dot(corner, normal);
				// determines if projection is the new min or max
				if (dot < min1)
				{
					min1 = dot;
				}
				if (dot > max1)
				{
					max1 = dot;
				}
			}
			// the smallest projection of box2 onto the normal
			float min2 = std::numeric_limits<float>::max();
			// the largest projection of box2 onto the normal
			float max2 = std::numeric_limits<float>::lowest();
			// the corners of box2
			corners = box2->GetCorners();
			for (glm::vec2 corner : corners)
			{
				// projects the corner onto the normal
				float dot = glm::dot(corner, normal);
				// determines if projection is the new min or max
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
			
			// the individual momentums of the boxes
			float p1 = box1->GetMass() * glm::length(box1->GetVelocity());
			float p2 = box2->GetMass() * glm::length(box2->GetVelocity());
			// sum of the two momentums
			float momentum = p1 + p2;
			// seperates the overlap based on the ratio of the momentums of the two objects
			float overlap1 = overlap * (p2 / momentum);
			float overlap2 = overlap * (p1 / momentum);

			// uses the average elasticity of the two objects
			float elasticity;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			float j;

			// box to box restitution
			// checks if both objects are not static
			if (!box1->GetStatic() && !box2->GetStatic())
			{
				// determines which direction of velocity is away from the other object
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
				elasticity = (box1->GetElasticity() + box2->GetElasticity()) / 2.0f;
				// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / box1->GetMass()) + (1 / box2->GetMass())));
			}
			// checks if only box1 is not static
			else if (!box1->GetStatic())
			{
				// gives the object the full overlap amount because the other object is static
				if (glm::dot(box1->GetVelocity() + (gravity * timeStep), -normal) > 0.0f)
				{
					ApplyResitiution(box1, box1->GetVelocity() + (gravity * timeStep), -normal, overlap);
				}
				else
				{
					ApplyResitiution(box1, -(box1->GetVelocity() + (gravity * timeStep)), -normal, overlap);
				}

				// uses the elasticity of box1
				elasticity = box1->GetElasticity();
				// the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m), because box2 has infinite mass when static
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / box1->GetMass()));
			}
			// checks if only box2 is not static
			else if (!box2->GetStatic())
			{
				// gives the object the full overlap amount because the other object is static
				if (glm::dot(box2->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
				{
					ApplyResitiution(box2, box2->GetVelocity() + (gravity * timeStep), normal, overlap);
				}
				else
				{
					ApplyResitiution(box2, -(box2->GetVelocity() + (gravity * timeStep)), normal, overlap);
				}

				// uses the elasticity of box2
				elasticity = box2->GetElasticity();
				// the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m), because box1 has infinite mass when static
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / box2->GetMass()));
			}

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;
			// the contact point is the point furthest away from box1 in the direction of box2
			glm::vec2 contact = box1->GetPosition() - (normal * overlap);

			// applys the friction force on each object if the object is not static
			if (!box1->GetStatic())
			{
				ApplyFriction(box1, -force, contact, gravity, timeStep, box2->GetStaticFriction(), box2->GetKineticFriction());
			}
			if (!box2->GetStatic())
			{
				ApplyFriction(box2, force, contact, gravity, timeStep, box1->GetStaticFriction(), box1->GetKineticFriction());
			}

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
	// try to cast objects to poly and plane
	Poly* poly = dynamic_cast<Poly*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);
	// if successful then test for collision
	if (poly != nullptr && plane != nullptr)
	{
		// use the normal of the plane as the collision normal
		glm::vec2 normal = plane->GetNormal();
		// does a broad check to see if it's worth checking the poly properly
		if (poly->GetRadius() - (glm::dot(poly->GetPosition(), normal) - plane->GetDistance()) >= 0.0f)
		{
			// projects the poly onto the normal
			glm::vec2 projection = poly->Project(normal);
			// checks if the minimum projection is behind the plane
			float overlap = projection.x - plane->GetDistance();
			if (overlap <= 0.0f)
			{
				overlap = -overlap;

				// if the either object is kinematic then there is no collision resolution
				// if the poly is static then there is no collision resolution because both objects will not move
				if (poly->GetKinematic() || plane->GetKinematic() || poly->GetStatic())
				{
					return true;
				}

				// uses the poly's velocity as the relative velocity
				glm::vec2 relativeVelocity = poly->GetVelocity();

				// poly to plane restitution
				// determines which direction of velocity is away from the other object
				if (glm::dot(relativeVelocity + (gravity * timeStep), normal) > 0.0f)
				{
					ApplyResitiution(poly, relativeVelocity + (gravity * timeStep), normal, overlap);
				}
				else
				{
					ApplyResitiution(poly, -(relativeVelocity + (gravity * timeStep)), normal, overlap);
				}

				// uses the elasticity of the poly
				float elasticity = poly->GetElasticity();
				// "j" is the magnitude of the force vector that needs to be applied to the objects
				// for planes the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m)
				float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / poly->GetMass()));

				// scales the normal by the impulse magnitude to get the resolution force
				glm::vec2 force = normal * j;
				glm::vec2 contact = glm::vec2(0.0f, 0.0f);

				// finds the point that was moveed to the edge of the plane
				for (glm::vec2 vertex : poly->GetVertices())
				{
					if (glm::dot(vertex + poly->GetPosition(), normal) - plane->GetDistance() == 0.0f)
					{
						contact = vertex + poly->GetPosition();
						break;
					}
				}

				// applys the friction force to the object
				ApplyFriction(poly, force, contact, gravity, timeStep, plane->GetStaticFriction(), plane->GetKineticFriction());

				return true;
			}
		}
	}

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
	// try to cast objects to poly and poly
	Poly* poly1 = dynamic_cast<Poly*>(obj1);
	Poly* poly2 = dynamic_cast<Poly*>(obj2);
	// if successful then test for collision
	if (poly1 != nullptr && poly2 != nullptr)
	{
		// performs a circle to circle collision check using the distance of the poly's furthest vertex as a radius
		// this is to check if the two objects are worth checking for collision
		if (glm::distance(poly1->GetPosition(), poly2->GetPosition()) <= poly1->GetRadius() + poly2->GetRadius())
		{
			// the overlap amount between the two objects
			float overlap = std::numeric_limits<float>::max();
			// collision normal
			glm::vec2 normal = glm::vec2(0.0f, 0.0f);
			// gets a collection of potential collision normals from both objects
			std::vector<glm::vec2> axis1 = poly1->GetAxis();
			std::vector<glm::vec2> axis2 = poly2->GetAxis();
			
			for (glm::vec2 axis : axis1)
			{
				// projects both objects onto the axis
				glm::vec2 proj1 = poly1->Project(axis);
				glm::vec2 proj2 = poly2->Project(axis);

				// checks if the two projections do not overlap
				if (!poly1->Overlap(proj1, proj2))
				{
					// there is no collision
					return false;
				}
				else
				{
					// gets the overlap amount and stores it if it is less than the current overlap amount
					float o = poly1->GetOverlap(proj1, proj2);
					if (o < overlap)
					{
						overlap = o;
						// stores the axis as the current collision normal
						normal = axis;
					}
				}
			}

			for (glm::vec2 axis : axis2)
			{
				// projects both objects onto the axis
				glm::vec2 proj1 = poly1->Project(axis);
				glm::vec2 proj2 = poly2->Project(axis);

				// checks if the two projections do not overlap
				if (!poly1->Overlap(proj1, proj2))
				{
					// there is no collision
					return false;
				}
				else
				{
					// gets the overlap amount and stores it if it is less than the current overlap amount
					float o = poly1->GetOverlap(proj1, proj2);
					if (o < overlap)
					{
						overlap = o;
						// stores the axis as the current collision normal
						normal = axis;
					}
				}
			}

			// if either object is kinematic then there is no collision resolution
			// if both objects are static then there will be no collision resolution because neither object will move
			if (poly1->GetKinematic() || poly2->GetKinematic() || (poly1->GetStatic() && poly2->GetStatic()))
			{
				return true;
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

			// uses the average elasticity of the two objects
			float elasticity;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			float j;

			// box to box restitution
			// checks if both objects are not static
			if (!poly1->GetStatic() && !poly2->GetStatic())
			{
				// determines which direction of velocity is away from the other object
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

				elasticity = (poly1->GetElasticity() + poly2->GetElasticity()) / 2.0f;
				// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / poly1->GetMass()) + (1 / poly2->GetMass())));
			}
			// checks if only poly1 is not static
			else if (!poly1->GetStatic())
			{
				// gives the object the full overlap amount because the other object is static
				if (glm::dot(poly1->GetVelocity() + (gravity * timeStep), -normal) > 0.0f)
				{
					ApplyResitiution(poly1, poly1->GetVelocity() + (gravity * timeStep), -normal, overlap);
				}
				else
				{
					ApplyResitiution(poly1, -(poly1->GetVelocity() + (gravity * timeStep)), -normal, overlap);
				}

				// uses the elasticity of poly1
				elasticity = poly1->GetElasticity();
				// the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m), because poly2 has infinite mass when static
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / poly1->GetMass()));
			}
			// checks if only poly2 is not static
			else if (!poly2->GetStatic())
			{
				// gives the object the full overlap amount because the other object is static
				if (glm::dot(poly2->GetVelocity() + (gravity * timeStep), normal) > 0.0f)
				{
					ApplyResitiution(poly2, poly2->GetVelocity() + (gravity * timeStep), normal, overlap);
				}
				else
				{
					ApplyResitiution(poly2, -(poly2->GetVelocity() + (gravity * timeStep)), normal, overlap);
				}

				// uses the elasticity of poly2
				elasticity = poly2->GetElasticity();
				// the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m), because poly1 has infinite mass when static
				j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * (1 / poly2->GetMass()));
			}

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;

			// gets the contact manifold
			std::vector<glm::vec2> contactPoints = poly1->ContactPoints(poly2, normal);
			glm::vec2 contact = glm::vec2(0.0f, 0.0f);
			// checks if there is no contact point given
			if (contactPoints.size() == 0)
			{
				return false;
			}

			// sets the contact point as the average point of the contact manifold
			for each (glm::vec2 point in contactPoints)
			{
				contact += point;
			}
			contact /= contactPoints.size();

			// applys the friction force on each object if the object is not static
			if (!poly1->GetStatic())
			{
				ApplyFriction(poly1, -force, contact, gravity, timeStep, poly2->GetStaticFriction(), poly2->GetKineticFriction());
			}
			if (!poly2->GetStatic())
			{
				ApplyFriction(poly2, force, contact, gravity, timeStep, poly1->GetStaticFriction(), poly1->GetKineticFriction());
			}

			return true;
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

	//// adds the friction force to the velocity after the collision
	//velocity += frictionForce;
	//// projects the velocity on the friction force to see if the force overcame the friction
	//frictionDirection = glm::dot(frictionForce, velocity);
	//// if the projection is negative then the force overcame the friction
	//if (frictionDirection <= 0.0f)
	//{
	//	obj->ApplyForce(force + frictionForce, contact - obj->GetPosition());
	//}
	// checks if the magnitude of the friction force is less than or equal to the magnitude of the velocity
	if (glm::length(frictionForce) <= glm::length(velocity))
	{
		obj->ApplyForce(force + frictionForce, contact - obj->GetPosition());
	}
	else // did not overcome friction
	{
		// stops the object
		obj->SetVelocity(glm::vec2(0.0f, 0.0f));
		// don't apply gravity this frame
		obj->ApplyForce(-gravity * timeStep, contact - obj->GetPosition());
	}
}

void PhysicsScene::ApplyResitiution(Rigidbody * obj, const glm::vec2 & velocity, const glm::vec2 & normal, const float overlap)
{
	const float HALF_PI = acosf(0.0f);
	const float tolerance = 0.000001f;

	float theta = 0.0f;
	// checks if the object is moving
	if (glm::length(velocity) != 0.0f)
	{
		// the angle between the velocity and collision normal
		theta = acosf(fminf(glm::dot(glm::normalize(velocity), normal), 1.0f));
		// the amount the box needs to move perpendicular to the collision normal
		float perpendicular = 0.0f;
		// ensures that the value is not 90 or 270 degrees to the normal
		if ((theta > HALF_PI + tolerance && theta < (HALF_PI * 3.0f) - tolerance) || theta > (HALF_PI * 3.0f) + tolerance || theta <= HALF_PI - tolerance)
		{
			perpendicular = tanf(theta) * overlap;
		}
		// multiplies the velocity in the opposite direction by the magnitude of the vector at ("perpendicular", "overlap")
		glm::vec2 restitution = glm::length(glm::vec2(perpendicular, overlap)) * glm::normalize(velocity);
		// adds the restitution to the current object's position
		obj->SetPosition(obj->GetPosition() + restitution);
	}
	else // the object is not moving
	{
		// the amount the box needs to move perpendicular to the collision normal
		float perpendicular = overlap;
		// multiplies the velocity in the opposite direction by the magnitude of the vector at ("perpendicular", "overlap")
		// because the object is stationary the normal will be used to determine restitution direction
		glm::vec2 restitution = glm::length(glm::vec2(perpendicular, overlap)) * glm::normalize(normal);
		// adds the restitution to the current object's position
		obj->SetPosition(obj->GetPosition() + restitution);
	}
}