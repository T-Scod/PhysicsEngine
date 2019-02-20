#include "PhysicsScene.h"
#include "Rigidbody.h"
#include <list>
#include <iostream>
#include "Sphere.h"
#include "Plane.h"
#include "AABB.h"

// function pointer array for doing collisions
typedef bool(*fn)(PhysicsObject*, PhysicsObject*, const glm::vec2&);

// collection of different collision check functions
static fn collisionFunctionArray[] =
{
	PhysicsScene::Plane2Plane, PhysicsScene::Plane2Sphere, PhysicsScene::Plane2Box,
	PhysicsScene::Sphere2Plane, PhysicsScene::Sphere2Sphere, PhysicsScene::Sphere2Box,
	PhysicsScene::Box2Plane, PhysicsScene::Box2Sphere, PhysicsScene::Box2Box
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

			// finds the index of the collision function required
			int functionID = (shapeID1 * SHAPE_COUNT) + shapeID2;
			fn collisionFunctionPtr = collisionFunctionArray[functionID];
			if (collisionFunctionPtr != nullptr)
			{
				// did a collision occur
				collisionFunctionPtr(object1, object2, m_gravity);
			}
		}
	}
}

// does nothing because planes don't collide
bool PhysicsScene::Plane2Plane(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity)
{
	return false;
}
// swaps the order of the objects and passes them into the opposite function
bool PhysicsScene::Plane2Sphere(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity)
{
	return Sphere2Plane(obj2, obj1, gravity);
}
// swaps the order of the objects and passes them into the opposite function
bool PhysicsScene::Plane2Box(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity)
{
	return Box2Plane(obj2, obj1, gravity);
}

bool PhysicsScene::Sphere2Plane(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity)
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
		if (overlap > 0)
		{
			// uses the circle's velocity as the relative velocity
			glm::vec2 relativeVelocity = sphere->GetVelocity();

			// sphere to plane restitution
			// the angle between the negative velocity and collision normal
			float theta = acosf(glm::dot(-glm::normalize(relativeVelocity), normal));
			// the amount the circle needs to move perpendicular to the collision normal
			float perpendicular = tanf(theta) * overlap;
			// multiplies the velocity in the opposite direction by the magnitude of the vector at ("perpendicular", "overlap")
			glm::vec2 restitution = glm::distance(glm::vec2(perpendicular, overlap), glm::vec2(0, 0)) * -glm::normalize(relativeVelocity);
			// adds the restitution to the current circle position
			sphere->SetPosition(sphere->GetPosition() + restitution);

			// uses the elasticity of the circle
			float elasticity = sphere->GetElasticity();
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// for planes the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m)
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / (1 / sphere->GetMass());

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;

			// the velocity after collision
			glm::vec2 velocity = sphere->GetVelocity() + (force / sphere->GetMass()) + gravity;
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
			if (sphere->GetVelocity() == glm::vec2(0.0f, 0.0f))
			{
				// multiplies the friction force by the static friction coefficient
				frictionForce *= (sphere->GetStaticFriction() + plane->GetStaticFriction()) / 2.0f;
			}
			else
			{
				// multiplies the friction force by the kinetic friction coefficient
				frictionForce *= (sphere->GetKineticFriction() + plane->GetKineticFriction()) / 2.0f;
			}

			// adds the friction force to the resolution force
			velocity += frictionForce;
			// projects the resolution force on the friction force to see if the force overcame the friction
			frictionDirection = glm::dot(frictionForce, velocity);
			// if the projection is negative then the force overcame the friction
			if (frictionDirection <= 0.0f)
			{
				// applies the force only on the circle because the plane is static
				sphere->ApplyForce(force + frictionForce);
			}
			//sphere->ApplyForce(force + frictionForce);

			return true;
		}
	}

	return false;
}
bool PhysicsScene::Sphere2Sphere(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity)
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
		if (distance < radii)
		{
			// the collision normal is the vector between the two circle's positions
			glm::vec2 normal = glm::normalize(sphere2->GetPosition() - sphere1->GetPosition());
			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = sphere2->GetVelocity() - sphere1->GetVelocity();

			// the sum of the two masses
			float mass = sphere1->GetMass() + sphere2->GetMass();
			// the amount the two circles overlap
			float overlap = radii - distance;
			// seperates the overlap based on the ratio of the masses of the two circles
			float overlap1 = overlap * (sphere2->GetMass() / mass);
			float overlap2 = overlap * (sphere1->GetMass() / mass);

			// sphere to sphere restitution
			// the angle between the velocity and collision normal
			float theta = acosf(glm::dot(glm::normalize(relativeVelocity), -normal));
			// the amount the circle needs to move perpendicular to the collision normal
			float perpendicular = tanf(theta) * overlap1;
			// multiplies the velocity by the magnitude of the vector at ("perpendicular", "overlap")
			glm::vec2 restitution = glm::distance(glm::vec2(perpendicular, overlap1), glm::vec2(0, 0)) * glm::normalize(relativeVelocity);
			// adds the restitution to the current circle position
			sphere1->SetPosition(sphere1->GetPosition() + restitution);

			// the angle between the negative velocity and collision normal
			theta = acosf(glm::dot(-glm::normalize(relativeVelocity), -normal));
			// the amount the circle needs to move perpendicular to the collision normal
			perpendicular = tanf(theta) * overlap2;
			// multiplies the velocity in the opposite direction by the magnitude of the vector at ("perpendicular", "overlap")
			restitution = glm::distance(glm::vec2(perpendicular, overlap2), glm::vec2(0, 0)) * -glm::normalize(relativeVelocity);
			// adds the restitution to the current circle position
			sphere2->SetPosition(sphere2->GetPosition() + restitution);

			// uses the average elasticity of the two circles
			float elasticity = (sphere1->GetElasticity() + sphere2->GetElasticity()) / 2.0f;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / sphere1->GetMass()) + (1 / sphere2->GetMass())));

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;
			// applies the force on both objects in opposite directions
			sphere1->ApplyForceToActor(sphere2, force);

			return true;
		}
	}

	return false;
}
bool PhysicsScene::Sphere2Box(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity)
{
	// try to cast objects to sphere and box
	Sphere* sphere = dynamic_cast<Sphere*>(obj1);
	AABB* box = dynamic_cast<AABB*>(obj2);
	// if successful then test for collision
	if (sphere != nullptr && box != nullptr)
	{
		// clamps the circle's position to the box's bounds, i.e. the closest point on the box relative to the circle
		glm::vec2 clamp = glm::vec2(fmaxf(fminf(sphere->GetPosition().x, box->GetMax().x), box->GetMin().x),
									fmaxf(fminf(sphere->GetPosition().y, box->GetMax().y), box->GetMin().y));
		// if the distance between the closest point and the circle's is less than the circle's radius then a collision occured
		if (glm::distance(clamp, sphere->GetPosition()) < sphere->GetRadius())
		{
			// the collision normal between the circle and the box will be perpendicular to one of the box's sides
			glm::vec2 normal = glm::normalize(box->GetPosition() - sphere->GetPosition());
			// sets the smaller of the two coordinates to 0
			(fabsf(normal.x) > fabsf(normal.y)) ? normal.y = 0.0f : normal.x = 0.0f;
			// ensures that the larger coordinate equals 1
			normal = glm::normalize(normal);
			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = box->GetVelocity() - sphere->GetVelocity();

			// the amount the circle and box overlap
			float overlap = 0.0f;
			// checks if the circle's position is inside the box
			if (clamp == sphere->GetPosition())
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
				glm::vec2 vectorViaClamp = glm::normalize(clamp - sphere->GetPosition()) * sphere->GetRadius();
				// the overlap amount is the distance between the clamped position and the scaled clamped position
				overlap = glm::distance(clamp, vectorViaClamp);
			}

			// the sum of the two masses
			float mass = sphere->GetMass() + box->GetMass();
			// seperates the overlap based on the ratio of the masses of the two objects
			float overlap1 = overlap * (sphere->GetMass() / mass);
			float overlap2 = overlap * (box->GetMass() / mass);

			// box to sphere restitution
			// the angle between the negative velocity and collision normal
			float theta = acosf(glm::dot(-glm::normalize(relativeVelocity), -normal));
			// the amount the box needs to move perpendicular to the collision normal
			float perpendicular = tanf(theta) * overlap1;
			// multiplies the velocity in the opposite direction by the magnitude of the vector at ("perpendicular", "overlap")
			glm::vec2 restitution = glm::distance(glm::vec2(perpendicular, overlap1), glm::vec2(0, 0)) * -glm::normalize(relativeVelocity);
			// adds the restitution to the current box's position
			box->SetPosition(box->GetPosition() + restitution);

			// the angle between the velocity and collision normal
			theta = acosf(glm::dot(glm::normalize(relativeVelocity), -normal));
			// the amount the circle needs to move perpendicular to the collision normal
			perpendicular = tanf(theta) * overlap2;
			// multiplies the velocity by the magnitude of the vector at ("perpendicular", "overlap")
			restitution = glm::distance(glm::vec2(perpendicular, overlap2), glm::vec2(0, 0)) * glm::normalize(relativeVelocity);
			// adds the restitution to the current circle position
			sphere->SetPosition(sphere->GetPosition() + restitution);

			// uses the average elasticity of the two objects
			float elasticity = (box->GetElasticity() + sphere->GetElasticity()) / 2.0f;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / box->GetMass()) + (1 / sphere->GetMass())));

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;
			// applies the force on both objects in opposite directions
			sphere->ApplyForceToActor(box, force);

			return true;
		}
	}

	return false;
}

bool PhysicsScene::Box2Plane(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity)
{
	// try to cast objects to box and plane
	AABB* box = dynamic_cast<AABB*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);
	// if successful then test for collision
	if (plane != nullptr && box != nullptr)
	{
		// contains the position of the corner and if it intersects the plane
		struct Corner
		{
			glm::vec2 position;
			bool intersect;
		};
		// collection of the corners
		Corner corners[4];
		corners[0] = { box->GetMin(), false };									// bottom left
		corners[1] = {	glm::vec2(box->GetMin().x, box->GetMax().y), false };	// top left
		corners[2] = { box->GetMax(), false };									// top right
		corners[3] = {	glm::vec2(box->GetMax().x, box->GetMin().y), false };	// bottom right

		for (unsigned int i = 0; i < 4; i++)
		{
			// checks if the corner is behind the plane
			if (glm::dot(plane->GetNormal(), corners[i].position) - plane->GetDistance() <= 0)
			{
				// this corner intersects with the plane
				corners[i].intersect = true;
			}
		}

		// if any of the corners intersect with the plane then a collision occured
		if (corners[0].intersect || corners[1].intersect || corners[2].intersect || corners[3].intersect)
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
				if (corners[i].intersect)
				{
					// gets the corner's distance from the plane
					float distance = glm::dot(corners[i].position, normal) - plane->GetDistance();
					// checks if the distance is greater the currently stored overlap amount
					if (fabsf(overlap) < fabsf(distance))
					{
						// stores the distance as the new overlap
						overlap = distance;
					}
				}
			}

			// brute force method
			/*if (normal.x == 0.0f || normal.y == 0.0f)
			{
				if (normal.x == 1.0f || normal.y == 1.0f)
				{
					overlap = glm::dot(box->GetMin(), normal) - plane->GetDistance();
				}
				else if (normal.x == -1.0f || normal.y == -1.0f)
				{
					overlap = glm::dot(box->GetMax(), normal) - plane->GetDistance();
				}
			}
			else if (normal.x > 0.0f && normal.y > 0.0f)
			{
				overlap = glm::dot(box->GetMin(), normal) - plane->GetDistance();
			}
			else if (normal.x > 0.0f && normal.y < 0.0f)
			{
				overlap = glm::dot(glm::vec2(box->GetMin().x, box->GetMax().y), normal) - plane->GetDistance();
			}
			else if (normal.x < 0.0f && normal.y < 0.0f)
			{
				overlap = glm::dot(box->GetMax(), normal) - plane->GetDistance();
			}
			else if (normal.x < 0.0f && normal.y > 0.0f)
			{
				overlap = glm::dot(glm::vec2(box->GetMax().x, box->GetMin().y), normal) - plane->GetDistance();
			}*/

			// box to plane restitution
			// the angle between the negative velocity and collision normal
			float theta = acosf(glm::dot(-glm::normalize(relativeVelocity), normal));
			// the amount the box needs to move perpendicular to the collision normal
			float perpendicular = tanf(theta) * overlap;
			// multiplies the velocity in the opposite direction by the magnitude of the vector at ("perpendicular", "overlap")
			glm::vec2 restitution = glm::distance(glm::vec2(perpendicular, overlap), glm::vec2(0, 0)) * -glm::normalize(relativeVelocity);
			// adds the restitution to the current box's position
			box->SetPosition(box->GetPosition() + restitution);

			// uses the box's elasticity
			float elasticity = box->GetElasticity();
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// for planes the formula is: (j = (-(1 + e)v.rel)·n) / (1 / m)
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / (1 / box->GetMass());

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;
			// applies the force only on the box because the plane is static
			box->ApplyForce(force);

			return true;
		}
	}

	return false;
}
// swaps the order of the objects and passes them into the opposite function
bool PhysicsScene::Box2Sphere(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity)
{
	return Sphere2Box(obj2, obj1, gravity);
}
bool PhysicsScene::Box2Box(PhysicsObject * obj1, PhysicsObject * obj2, const glm::vec2& gravity)
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
			// the collision normal between the boxes will be perpendicular to one of the box's sides
			glm::vec2 normal = glm::normalize(box2->GetPosition() - box1->GetPosition());
			// sets the smaller of the two coordinates to 0
			(fabsf(normal.x) > fabsf(normal.y)) ? normal.y = 0.0f : normal.x = 0.0f;
			// ensures that the larger coordinate equals 1
			normal = glm::normalize(normal);
			// the difference between the velocities is the relative velocity
			glm::vec2 relativeVelocity = box2->GetVelocity() - box1->GetVelocity();

			// projects each of the perpendicular sides onto the normal
			float min1Dot = glm::dot(box1->GetMin(), normal);
			float max1Dot = glm::dot(box1->GetMax(), normal);
			float min2Dot = glm::dot(box2->GetMin(), normal);
			float max2Dot = glm::dot(box2->GetMax(), normal);

			// the amount the boxes overlap
			float overlap = 0.0f;

			// finds the smaller overlap between the boxes and stores it as the overlap amount
			if ((max1Dot - min2Dot) < (max2Dot - min1Dot))
			{
				overlap = max1Dot - min2Dot;
			}
			else
			{
				overlap = max2Dot - min1Dot;
			}
			
			// sum of the two masses
			float mass = box1->GetMass() + box2->GetMass();
			// seperates the overlap based on the ratio of the masses of the two objects
			float overlap1 = overlap * (box2->GetMass() / mass);
			float overlap2 = overlap * (box1->GetMass() / mass);

			// box to box restitution
			// the angle between the velocity and collision normal
			float theta = acosf(glm::dot(glm::normalize(relativeVelocity), -normal));
			// the amount the box needs to move perpendicular to the collision normal
			float perpendicular = tanf(theta) * overlap1;
			// multiplies the velocity by the magnitude of the vector at ("perpendicular", "overlap")
			glm::vec2 restitution = glm::distance(glm::vec2(perpendicular, overlap1), glm::vec2(0, 0)) * glm::normalize(relativeVelocity);
			// adds the restitution to the current box's position
			box1->SetPosition(box1->GetPosition() + restitution);

			// the angle between the negative velocity and collision normal
			theta = acosf(glm::dot(-glm::normalize(relativeVelocity), -normal));
			// the amount the box needs to move perpendicular to the collision normal
			perpendicular = tanf(theta) * overlap2;
			// multiplies the velocity in the opposite direction by the magnitude of the vector at ("perpendicular", "overlap")
			restitution = glm::distance(glm::vec2(perpendicular, overlap2), glm::vec2(0, 0)) * -glm::normalize(relativeVelocity);
			// adds the restitution to the current box's position
			box2->SetPosition(box2->GetPosition() + restitution);

			// uses the average elasticity of the two objects
			float elasticity = (box1->GetElasticity() + box2->GetElasticity()) / 2.0f;;
			// "j" is the magnitude of the force vector that needs to be applied to the objects
			// the formula is: (j = (-(1 + e)v.rel)·n) / n·(n((1 / m.1) + (1 / m.2)))
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / box1->GetMass()) + (1 / box2->GetMass())));

			// scales the normal by the impulse magnitude to get the resolution force
			glm::vec2 force = normal * j;
			// applies the force on both objects in opposite directions
			box1->ApplyForceToActor(box2, force);

			return true;
		}
	}

	return false;
}