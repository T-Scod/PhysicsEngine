#include "CollisionApp.h"
#include "Texture.h"
#include "Font.h"
#include "Input.h"
#include <iostream>
#include "AABB.h"
#include "Sphere.h"
#include "Plane.h"

CollisionApp::CollisionApp()
{
}

CollisionApp::~CollisionApp()
{
}

bool CollisionApp::startup()
{
	// increase the 2D line count to maximise the number of objects we can draw
	aie::Gizmos::create(255U, 255U, 65535U, 65535U);

	m_2dRenderer = new aie::Renderer2D();
	m_font = new aie::Font("../bin/font/consolas.ttf", 32);

	m_physicsScene = new PhysicsScene();
	m_physicsScene->SetGravity(glm::vec2(0, 0)); // gravity off
	m_physicsScene->SetTimeStep(0.2f);

	glm::vec2 startPos1 = glm::vec2(-10, 0);
	glm::vec2 startPos2 = glm::vec2(10, 0);
	glm::vec2 velocity = glm::vec2(0, 0);
	glm::vec2 force1 = glm::vec2(10, -0);
	force1 = glm::normalize(force1) * 4.0f;
	glm::vec2 force2 = glm::vec2(-15, -7);
	float mass = 1.0f;
	float mass1 = 10000.7f;
	float mass2 = 1.6f;
	glm::vec4 red = glm::vec4(1, 0, 0, 1);
	glm::vec4 green = glm::vec4(0, 1, 0, 1);

	AABB* box1 = new AABB(startPos1, velocity, 5, 5, mass, red);
	m_physicsScene->AddActor(box1);
	box1->ApplyForce(force1);
	AABB* box2 = new AABB(startPos2, velocity, 5, 5, mass1, green);
	m_physicsScene->AddActor(box2);
	//box2->ApplyForce(force2);
	//Sphere* ball1 = new Sphere(startPos1, -0.785398f, 10, 4, mass, red);
	//m_physicsScene->AddActor(ball1);
	//ball1->ApplyForce(force1);
	//Sphere* ball2 = new Sphere(startPos2, velocity, 5, mass2, green);
	//m_physicsScene->AddActor(ball2);
	//ball2->ApplyForce(force2);
	//Plane* plane1 = new Plane(glm::vec2(-0.707f, 0.707f), 10.0f);
	//m_physicsScene->AddActor(plane1);
	/*Plane* plane2 = new Plane(glm::vec2(0.707f, 0.707f), 10.0f);
	m_physicsScene->AddActor(plane2);*/
	/*Plane* plane3 = new Plane(glm::vec2(0, 1), 0);
	m_physicsScene->AddActor(plane3);*/

	return true;
}

void CollisionApp::shutdown()
{
	delete m_font;
	delete m_2dRenderer;

	aie::Gizmos::destroy();

	delete m_physicsScene;
	m_physicsScene = nullptr;
}

void CollisionApp::update(float deltaTime)
{
	// input example
	aie::Input* input = aie::Input::getInstance();

	aie::Gizmos::clear();

	m_physicsScene->Update(deltaTime);
	m_physicsScene->UpdateGizmos();

	// exit the application
	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		quit();
}

void CollisionApp::draw()
{
	// wipe the screen to the background colour
	clearScreen();

	// begin drawing sprites
	m_2dRenderer->begin();

	// draw your stuff here!
	static float aspectRatio = 16.0f / 9.0f;
	aie::Gizmos::draw2D(glm::ortho<float>(-100.0f, 100.0f, -100.0f / aspectRatio, 100.0f / aspectRatio, -1.0f, 1.0f));

	// output some text, uses the last used colour
	m_2dRenderer->drawText(m_font, "Press ESC to quit", 0, 0);

	// done drawing sprites
	m_2dRenderer->end();
}