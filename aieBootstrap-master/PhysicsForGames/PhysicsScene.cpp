#include "PhysicsScene.h"
#include "Rigidbody.h"
#include <list>
#include <iostream>
#include "Sphere.h"
#include "Plane.h"
#include "AABB.h"

// function pointer array for doing our collisions
typedef bool(*fn)(PhysicsObject*, PhysicsObject*);

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

void PhysicsScene::UpdateGizmos()
{
	for (auto pActor : m_actors)
	{
		pActor->MakeGizmo();
	}
}

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

			// using function pointers
			int functionID = (shapeID1 * SHAPE_COUNT) + shapeID2;
			fn collisionFunctionPtr = collisionFunctionArray[functionID];
			if (collisionFunctionPtr != nullptr)
			{
				// did a collision occur
				collisionFunctionPtr(object1, object2);
			}
		}
	}
}

bool PhysicsScene::Plane2Plane(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return false;
}
bool PhysicsScene::Plane2Sphere(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return Sphere2Plane(obj2, obj1);
}
bool PhysicsScene::Plane2Box(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return Box2Plane(obj2, obj1);
}

bool PhysicsScene::Sphere2Plane(PhysicsObject * obj1, PhysicsObject * obj2)
{
	// try to cast objects to sphere and sphere
	Sphere* sphere = dynamic_cast<Sphere*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);
	// if we are successful then test for collision
	if (sphere != nullptr && plane != nullptr)
	{
		glm::vec2 normal = plane->GetNormal();
		float sphereToPlane = glm::dot(sphere->GetPosition(), normal) - plane->GetDistance();

		float overlap = sphere->GetRadius() - sphereToPlane;
		if (overlap > 0)
		{
			glm::vec2 relativeVelocity = sphere->GetVelocity();

			// seperate the sphere from the plane
			float theta = acosf(glm::dot(-glm::normalize(relativeVelocity), normal));
			float perpendicular = tanf(theta) * overlap;
			glm::vec2 restitution = glm::distance(glm::vec2(perpendicular, overlap), glm::vec2(0, 0)) * -glm::normalize(relativeVelocity);
			sphere->SetPosition(sphere->GetPosition() + restitution);

			float elasticity = 1;
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / (1 / sphere->GetMass());

			glm::vec2 force = normal * j;

			sphere->ApplyForce(force);

			return true;
		}
	}

	return false;
}
bool PhysicsScene::Sphere2Sphere(PhysicsObject * obj1, PhysicsObject * obj2)
{
	// try to cast objects to sphere and sphere
	Sphere* sphere1 = dynamic_cast<Sphere*>(obj1);
	Sphere* sphere2 = dynamic_cast<Sphere*>(obj2);
	// if we are successful then test for collision
	if (sphere1 != nullptr && sphere2 != nullptr)
	{
		// determines the distance between the two spheres
		float distance = glm::distance(sphere1->GetPosition(), sphere2->GetPosition());
		float radii = sphere1->GetRadius() + sphere2->GetRadius();

		// checks if the distance is less than the sum of the two radii
		if (distance < radii)
		{
			glm::vec2 normal = glm::normalize(sphere2->GetPosition() - sphere1->GetPosition());

			float mass = sphere1->GetMass() + sphere2->GetMass();
			float overlap = radii - distance;
			float overlap1 = overlap * (sphere1->GetMass() / mass);
			float overlap2 = overlap * (sphere2->GetMass() / mass);

			float theta = acosf(glm::dot(-glm::normalize(sphere1->GetVelocity()), -normal));
			float perpendicular = tanf(theta) * overlap1;
			glm::vec2 restitution = glm::distance(glm::vec2(perpendicular, overlap1), glm::vec2(0, 0)) * -glm::normalize(sphere1->GetVelocity());
			sphere1->SetPosition(sphere1->GetPosition() + restitution);

			theta = acosf(glm::dot(-glm::normalize(sphere2->GetVelocity()), -normal));
			perpendicular = tanf(theta) * overlap2;
			restitution = glm::distance(glm::vec2(perpendicular, overlap2), glm::vec2(0, 0)) * -glm::normalize(sphere2->GetVelocity());
			sphere2->SetPosition(sphere2->GetPosition() + restitution);

			glm::vec2 relativeVelocity = sphere2->GetVelocity() - sphere1->GetVelocity();
			float elasticity = 1;
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / sphere1->GetMass()) + (1 / sphere2->GetMass())));

			glm::vec2 force = normal * j;

			sphere1->ApplyForceToActor(sphere2, force);

			return true;
		}
	}

	return false;
}
bool PhysicsScene::Sphere2Box(PhysicsObject * obj1, PhysicsObject * obj2)
{
	// try to cast objects to sphere and sphere
	Sphere* sphere = dynamic_cast<Sphere*>(obj1);
	AABB* box = dynamic_cast<AABB*>(obj2);
	// if we are successful then test for collision
	if (sphere != nullptr && box != nullptr)
	{
		glm::vec2 clamp = glm::vec2(fmaxf(fminf(sphere->GetPosition().x, box->GetMax().x), box->GetMin().x),
									fmaxf(fminf(sphere->GetPosition().y, box->GetMax().y), box->GetMin().y));
		if (glm::distance(clamp, sphere->GetPosition()) < sphere->GetRadius())
		{
			glm::vec2 normal = glm::normalize(box->GetPosition() - sphere->GetPosition());
			(fabsf(normal.x) > fabsf(normal.y)) ? normal.y = 0.0f : normal.x = 0.0f;
			normal = glm::normalize(normal);

			float overlap = 0.0f;

			if (clamp == sphere->GetPosition())
			{
				if (normal.x != 0)
				{
					overlap = box->GetExtents().x - (box->GetPosition().x - clamp.x) + sphere->GetRadius();
				}
				else if (normal.y != 0)
				{
					overlap = box->GetExtents().y - (box->GetPosition().y - clamp.y) + sphere->GetRadius();
				}
			}
			else
			{
				glm::vec2 vectorViaClamp = glm::normalize(clamp - sphere->GetPosition()) * sphere->GetRadius();
				overlap = glm::distance(clamp, vectorViaClamp);
			}

			float mass = sphere->GetMass() + box->GetMass();
			float overlap1 = overlap * (box->GetMass() / mass);
			float overlap2 = overlap * (sphere->GetMass() / mass);

			float theta = acosf(glm::dot(-glm::normalize(box->GetVelocity()), -normal));
			float perpendicular = tanf(theta) * overlap1;
			glm::vec2 restitution = glm::distance(glm::vec2(perpendicular, overlap1), glm::vec2(0, 0)) * -glm::normalize(box->GetVelocity());
			box->SetPosition(box->GetPosition() + restitution);

			theta = acosf(glm::dot(-glm::normalize(sphere->GetVelocity()), -normal));
			perpendicular = tanf(theta) * overlap2;
			restitution = glm::distance(glm::vec2(perpendicular, overlap2), glm::vec2(0, 0)) * -glm::normalize(sphere->GetVelocity());
			sphere->SetPosition(sphere->GetPosition() + restitution);

			glm::vec2 relativeVelocity = box->GetVelocity() - sphere->GetVelocity();
			float elasticity = 1;
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / box->GetMass()) + (1 / sphere->GetMass())));

			glm::vec2 force = normal * j;

			sphere->ApplyForceToActor(box, force);

			return true;
		}
	}

	return false;
}

bool PhysicsScene::Box2Plane(PhysicsObject * obj1, PhysicsObject * obj2)
{
	// try to cast objects to sphere and sphere
	AABB* box = dynamic_cast<AABB*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);

	// if we are successful then test for collision
	if (plane != nullptr && box != nullptr)
	{
		struct Corner
		{
			glm::vec2 position;
			bool intersect;
		};
		Corner corners[4];
		corners[0] = { box->GetMin(), false };									// bottom left
		corners[1] = {	glm::vec2(box->GetMin().x, box->GetMax().y), false };	// top left
		corners[2] = { box->GetMax(), false };									// top right
		corners[3] = {	glm::vec2(box->GetMax().x, box->GetMin().y), false };	// bottom right

		for (unsigned int i = 0; i < 4; i++)
		{
			if (glm::dot(plane->GetNormal(), corners[i].position) - plane->GetDistance() <= 0)
			{
				corners[i].intersect = true;
			}
		}

		if (corners[0].intersect || corners[1].intersect || corners[2].intersect || corners[3].intersect)
		{
			glm::vec2 normal = plane->GetNormal();
			glm::vec2 relativeVelocity = box->GetVelocity();

			float overlap = 0;

			for (int i = 0; i < 4; i++)
			{
				if (corners[i].intersect)
				{
					float distance = glm::dot(corners[i].position, normal) - plane->GetDistance();
					if (fabsf(overlap) < fabsf(distance))
					{
						overlap = distance;
					}
				}
			}

			//if (normal.x == 0.0f || normal.y == 0.0f)
			//{
			//	if (normal.x == 1.0f || normal.y == 1.0f)
			//	{
			//		overlap = glm::dot(box->GetMin(), normal) - plane->GetDistance();
			//	}
			//	else if (normal.x == -1.0f || normal.y == -1.0f)
			//	{
			//		overlap = glm::dot(box->GetMax(), normal) - plane->GetDistance();
			//	}
			//}
			//else if (normal.x > 0.0f && normal.y > 0.0f)
			//{
			//	overlap = glm::dot(box->GetMin(), normal) - plane->GetDistance();
			//}
			//else if (normal.x > 0.0f && normal.y < 0.0f)
			//{
			//	overlap = glm::dot(glm::vec2(box->GetMin().x, box->GetMax().y), normal) - plane->GetDistance();
			//}
			//else if (normal.x < 0.0f && normal.y < 0.0f)
			//{
			//	overlap = glm::dot(box->GetMax(), normal) - plane->GetDistance();
			//}
			//else if (normal.x < 0.0f && normal.y > 0.0f)
			//{
			//	overlap = glm::dot(glm::vec2(box->GetMax().x, box->GetMin().y), normal) - plane->GetDistance();
			//}

			// seperate the box from the plane
			float theta = acosf(glm::dot(-glm::normalize(relativeVelocity), normal));
			float perpendicular = tanf(theta) * overlap;
			glm::vec2 restitution = glm::distance(glm::vec2(perpendicular, overlap), glm::vec2(0, 0)) * -glm::normalize(relativeVelocity);
			box->SetPosition(box->GetPosition() + restitution);

			float elasticity = 1;
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / (1 / box->GetMass());

			glm::vec2 force = normal * j;

			box->ApplyForce(force);

			return true;
		}
	}

	return false;
}
bool PhysicsScene::Box2Sphere(PhysicsObject * obj1, PhysicsObject * obj2)
{
	return Sphere2Box(obj2, obj1);
}
bool PhysicsScene::Box2Box(PhysicsObject * obj1, PhysicsObject * obj2)
{
	// try to cast objects to sphere and sphere
	AABB* box1 = dynamic_cast<AABB*>(obj1);
	AABB* box2 = dynamic_cast<AABB*>(obj2);
	// if we are successful then test for collision
	if (box1 != nullptr && box2 != nullptr)
	{
		if (box1->GetMin().x > box2->GetMax().x ||
			box1->GetMin().y > box2->GetMax().y ||
			box1->GetMax().x < box2->GetMin().x ||
			box1->GetMax().y < box2->GetMin().y)
		{
			return false;
		}
		else
		{
			glm::vec2 normal = glm::normalize(box2->GetPosition() - box1->GetPosition());
			(fabsf(normal.x) > fabsf(normal.y)) ? normal.y = 0.0f : normal.x = 0.0f;
			normal = glm::normalize(normal);



			glm::vec2 relativeVelocity = box2->GetVelocity() - box1->GetVelocity();
			float elasticity = 1;
			float j = glm::dot(-(1 + elasticity) * (relativeVelocity), normal) / glm::dot(normal, normal * ((1 / box1->GetMass()) + (1 / box2->GetMass())));

			glm::vec2 force = normal * j;

			box1->ApplyForceToActor(box2, force);

			return true;
		}
	}

	return false;
}