#include "PhysicsEngine.h"

#include <map>
#include <numeric>

#include "Application.h"
#include "Camera.h"
#include "Force.h"

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void SymplecticEuler(RigidBody& rb, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	rb.SetVelocity(rb.Velocity() + ((1 / rb.Mass()) * rb.AccumulatedForce() * dt) + (rb.AccumulatedImpulse() / rb.Mass()));
	rb.SetPosition(rb.Position() + (rb.Velocity() * dt));
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void Integrate(RigidBody& rb, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	auto newRot = rb.AngularVelocity() + (dt * rb.AngularVelocity());
	auto angVelSkew = glm::matrixCross3(newRot);
	auto R = glm::mat3(rb.Orientation());
	R += dt * angVelSkew * R;
	R = glm::orthonormalize(R);
	rb.SetOrientation(glm::mat4(R));
}


void CollisionImpulse(RigidBody& rb, int elasticity, int y_level)
{
	for (auto x : rb.GetMesh()->Data().positions.data)
	{
		mat4 test = rb.ModelMatrix();
		auto ws_coord = (rb.ModelMatrix()) * vec4(x, 1);
		if (ws_coord.y < y_level)
		{
			auto delta = y_level - ws_coord.y;
			rb.SetPosition(vec3(rb.Position().x, rb.Position().y + delta, rb.Position().z));
			vec3 impulse = vec3(0);
			vec3 normal = vec3(0, 1, 0);
			auto v_close = dot(rb.Velocity(), normal);

			impulse = -(1 + elasticity) * rb.Mass() * v_close * normal;
			rb.ApplyImpulse(impulse);
			printf("impulse = %f, %f, %f\n", impulse.x, impulse.y, impulse.z);
		}
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate collision impulse
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto groundMesh = meshDb.Get("plane");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	// TODO: Get the mesh and shader for rigidy body
	camera = Camera(vec3(0, 5, 10));
	Task1Init(defaultShader, meshDb.Get("cube"), vec3(0,10,0), vec3(1,1,1), vec3(0), vec3(1,1,1));
	
	for (auto x : ground.GetMesh()->Data().positions.data)
	{
		printf("Ground Normal - %f, %f, %f\n", x.x, x.y, x.z);
	}
}

void PhysicsEngine::Task1Init(const Shader* rbShader, const Mesh* rbMesh, vec3 pos, vec3 scale, vec3 initVel, vec3 initRotVel)
{
	// Initialise the rigid body, setting parameterised members first
	rbody1.SetShader(rbShader);
	rbody1.SetMesh(rbMesh);
	rbody1.SetPosition(pos);
	rbody1.SetScale(scale);
	rbody1.SetVelocity(initVel);
	rbody1.SetAngularVelocity(initRotVel);

	// Hard values as they don't *really* matter, not going to change
	rbody1.SetMass(1.0f);
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	rbody1.ClearForcesImpulses();
	Force::Gravity(rbody1);
	//SymplecticEuler(rbody1, deltaTime);
	CollisionImpulse(rbody1, 1.0f , ground.Position().y);
	SymplecticEuler(rbody1, deltaTime);
	Integrate(rbody1, deltaTime);
}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Time step code and task updates
	double timeStep = 0.004167;
	float alpha = 0;
	double physAcca = 0.0;

	if (deltaTime > 0.25)
	{
		deltaTime = 0.25;
	}
	physAcca += deltaTime;
	while (physAcca >= timeStep)
	{
		if (toggleSim)
		{
			switch (simMode)
			{
			case 0:
				break;
			case 1:
				Task1Update(timeStep, totalTime);
				break;
			case 2:
				//TaskClothSim(timeStep, totalTime);
				break;
			}
		}
		totalTime += timeStep;
		physAcca -= timeStep;
	}
	alpha = physAcca / timeStep;
	/*for (int x = 0; x < prt_len; x++)
	{
		Update_TimestepAlphaEval(particles[x], x, p_arr, v_arr, alpha);
	}*/
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	ground.Draw(viewMatrix, projMatrix);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Anything else to draw here
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	rbody1.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// TODO: Add any task swapping keys here
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	case 32:
		if (pressed)
			toggleSim = !toggleSim;
		break;
	default:
		if (pressed)
			printf("key pressed = %d\n", keyCode);
		break;
	}
}