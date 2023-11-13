//
//  Scene.h
//
#pragma once
#include <vector>

#include "../Body.h"

/*
====================================================
Scene
====================================================
*/
class Scene 
{
public:
	Scene() { bodies.reserve(128);}
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	

	std::vector<Body> bodies;
	void ThrowLaBouleDePetanque(Vec3 positionP, Vec3 cameraRotation);

	void SetPower(float amountP);
private:
	Body cochonnet;
	int round{ 0 };
	float powerBourrinage{ 1 };
};

