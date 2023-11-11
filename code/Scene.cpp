//
//  Scene.cpp
//
#include "Scene.h"
#include <thread>
#include "..\Intersection.h"
#include "../Shape.h"
#include "../Broadphase.h"

Scene::~Scene() 
{
	for (int i = 0; i < bodies.size(); i++) 
	{
		delete bodies[i].shape;
	}
	bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() 
{
	for (int i = 0; i < bodies.size(); i++) 
	{
		delete bodies[i].shape;
	}
	bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() 
{
	srand(time(NULL));

	for (int i = 1; i < 100; i++)
	{
		/* Sphere */
		Body ball;
		ball.position = Vec3((rand() % 10) / 10.f, (rand() % 10) / 10.f, 5 * i);
		ball.orientation = Quat(0, 0, 0, 1);
		ball.shape = new ShapeSphere(1.0f);
		ball.inverseMass = 1.f;
		ball.friction = .9f;
		ball.elasticity = .5f;
		ball.linearVelocity = Vec3(0, 0, 0);
		bodies.push_back(ball);
	}

	/* Earth */
	Body earth;
	earth.position = Vec3(0, 0, -1000);
	earth.orientation = Quat(0, 0, 0, 1);
	earth.shape = new ShapeSphere(1000.0f);
	earth.inverseMass = 0.0f;
	earth.friction = .9f;
	earth.elasticity = 1.0f;
	bodies.push_back(earth);

	Body body;
	float incrementalAngle = 0;
	float radiusArena = 5;
	float gap = 9;
	float n_balls = 30;

	for (int i = 0; i < n_balls; i++)
	{
		body.position = Vec3(cos(incrementalAngle) * radiusArena * gap, sin(incrementalAngle) * radiusArena * gap, 0);
		body.orientation = Quat(0, 0, 0, 1);
		body.shape = new ShapeSphere(radiusArena);
		body.inverseMass = 0.00f;
		body.elasticity = 0.5f;
		body.friction = 0.05f;
		body.linearVelocity = Vec3(0, 0, 0);
		incrementalAngle += 2 * 3.14159265 / n_balls;
		bodies.push_back(body);
	}
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update(const float dt_sec)
{
	// Gravity
	for (int i = 0; i < bodies.size(); ++i)
	{
		Body& body = bodies[i];
		float mass = 1.0f / body.inverseMass;
		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt
		// <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body.ApplyImpulseLinear(impulseGravity);
	}

	// Broadphase
	std::vector<CollisionPair> collisionPairs;
	BroadPhase(bodies.data(), bodies.size(), collisionPairs, dt_sec);

	// Collision checks (Narrow phase)
	int numContacts = 0;
	const int maxContacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)malloc(sizeof(Contact) * maxContacts);
	for (int i = 0; i < collisionPairs.size(); ++i)
	{
		const CollisionPair& pair = collisionPairs[i];
		Body& bodyA = bodies[pair.a];
		Body& bodyB = bodies[pair.b];
		if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f)
			continue;
		Contact contact;
		if (Intersection::Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			++numContacts;
		}
	}
	// Sort times of impact
	if (numContacts > 1) 
	{
		qsort(contacts, numContacts, sizeof(Contact),
			Contact::CompareContact);
	}
	// Contact resolve in order
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; ++i)
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;
		Body* bodyA = contact.a;
		Body* bodyB = contact.b;

		// Skip body par with infinite mass
		if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f)
			continue;
		// Position update
		for (int j = 0; j < bodies.size(); ++j) 
		{
			bodies[j].Update(dt);
		}
		Contact::ResolveContact(contact);
		accumulatedTime += dt;
	}

	free(contacts);

	// Other physics behavirous, outside collisions.
	// Update the positions for the rest of this frame's time.
	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < bodies.size(); ++i) 
		{
			bodies[i].Update(timeRemaining);
		}
	}
}