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
	auto test = rb.Velocity() + ((1 / rb.Mass()) * rb.AccumulatedForce() * dt);
	rb.SetVelocity(rb.Velocity() + ((1 / rb.Mass()) * rb.AccumulatedForce() * dt));// + (rb.AccumulatedImpulse() / rb.Mass()));
	auto test2 = rb.Position() + (rb.Velocity() * dt);
	rb.SetPosition(rb.Position() + (rb.Velocity() * dt));
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void TOTAL_Integration(RigidBody& rb, float dt)
{
	auto torque = glm::cross(rb.r(), rb.AccumulatedForce());
	rb.AddTorque(torque);
	auto velocity = rb.Velocity() + ((1.0f/rb.Mass()) * rb.AccumulatedForce() * dt);
	auto position = rb.Position() + (velocity * dt);

	rb.SetVelocity(velocity);
	rb.SetPosition(position);

 	auto R = glm::mat3(rb.Orientation());

	auto pre_momentum = rb.GetInertia() * rb.AngularVelocity();
	rb.SetAngularMomentum(pre_momentum);
	auto momentum = rb.AngularMomentum() + rb.GetTorque() * dt;
	auto angular_velocity = rb.GetInverseInertia() * momentum * dt;
	auto angVelSkew = glm::matrixCross3(angular_velocity);
	auto rotation = glm::orthonormalize(R + (angVelSkew * R));

	auto test = position - rb.Position();
	rb.Translate(position - rb.Position());
	rb.SetOrientation(rotation);

}

void Integrate(RigidBody& rb, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	/*auto newRot = rb.AngularVelocity() + (dt * rb.GetAngularMomentum());
	auto angVelSkew = glm::matrixCross3(newRot);
	auto R = glm::mat3(rb.Orientation());
	R += dt * angVelSkew * R;
	R = glm::orthonormalize(R);
	rb.SetOrientation(glm::mat4(R));*/
	
	auto calc_vel = rb.Velocity() + glm::cross(rb.AngularVelocity(), rb.r());
	auto momentum = (float)(rb.Mass() * pow(glm::length(rb.r()), 2)) * rb.AngularVelocity();

	/*
	rb.SetAngularVelocity(rb.GetInverseInertia() * momentum + glm::cross(rb.r(), rb.GetRotationalImpulse());
	auto R = glm::mat3(rb.Orientation());
	R = glm::orthonormalize(R + (glm::matrixCross3(rb.AngularVelocity()) * R));
	rb.SetOrientation(glm::mat4(R));*/
	rb.SetAngularMomentum(momentum + (rb.GetTorque() * dt));
	auto bfore = rb.GetInverseInertia() * rb.AngularMomentum();
	rb.SetAngularVelocity(rb.GetInverseInertia() * rb.AngularMomentum());
	auto R = glm::mat3(rb.Orientation());
	R += (glm::matrixCross3(rb.AngularVelocity()) * R) * dt;
	R = glm::orthonormalize(R);
	rb.SetOrientation(R);
}

double RigidCollision(RigidBody& rb, float elasticity, vec3 CollisionCoords, vec3 CollisionNormal)
{
	/*rb.SetRotationalApplicationVector(vec3(ws_coord) - rb.Position());
	auto j_floor = impulse / (1.0 / rb.Mass()) + glm::dot(normal, glm::cross(rb.GetInverseInertia() * glm::cross(rb.GetRotationalApplicationVector(), normal), rb.GetRotationalApplicationVector()));
	rb.ApplyRotationalImpulse(j_floor);*/
	rb.Set_r(CollisionCoords - rb.Position());
	auto numerator = -(1.0 + elasticity) * glm::dot(rb.Velocity() + glm::cross(rb.AngularVelocity(), rb.r()), CollisionNormal);
	auto r_x_Nhat = glm::cross(rb.r(), CollisionNormal);
	auto inertia_calc = rb.GetInverseInertia() * r_x_Nhat;
	auto iner_x_r = glm::cross(inertia_calc, rb.r());
	auto denominator = (1.0 / rb.Mass()) + glm::dot(CollisionNormal, iner_x_r);
	auto rotImpulse = numerator / denominator;
	printf("impulse - %f\n", rotImpulse);
	if (rotImpulse > 99)
	{
		printf("impulse - %f\n", rotImpulse);
	}
	return rotImpulse;
}


void CollisionImpulse(RigidBody& rb, float elasticity, int y_level)
{
	for (auto x : rb.GetMesh()->Data().positions.data)
	{
		auto ws_coord = (rb.ModelMatrix()) * vec4(x, 1);
		if (ws_coord.y < y_level)
		{
			auto delta = y_level - ws_coord.y;
			rb.SetPosition(vec3(rb.Position().x, rb.Position().y + delta, rb.Position().z));
			vec3 normal = vec3(0, 1, 0);
			/*auto v_close = dot(rb.Velocity(), normal);

			impulse = -(1 + elasticity) * rb.Mass() * v_close * normal;
			rb.ApplyImpulse(impulse);*/
			//printf("impulse = %f, %f, %f\n", impulse.x, impulse.y, impulse.z);
			printf("world coord = %f, %f, %f\n", ws_coord.x, ws_coord.y, ws_coord.z);
			// use the above output to calculate the r vec for applying angular forces. since mesh is box, CoM is just the location, derive stuff from there
			float jr = RigidCollision(rb, elasticity, vec3(ws_coord), normal);
			auto calc_vel = rb.Velocity() + glm::cross(rb.AngularVelocity(), rb.r());
			rb.SetVelocity(calc_vel + (jr / rb.Mass()) * normal);
			auto test = (rb.AngularVelocity() - (jr * (rb.GetInverseInertia() * glm::cross(rb.Position(), normal))));
			rb.SetAngularVelocity(rb.AngularVelocity() + (jr * (rb.GetInverseInertia() * glm::cross(rb.Position(), normal))));
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
	Task1Init(defaultShader, meshDb.Get("cube"), vec3(0, 10, 0), vec3(1, 3, 1), vec3(0), vec3(1, 0, 0));

	for (auto x : ground.GetMesh()->Data().positions.data)
	{
		printf("Ground Positions - %f, %f, %f\n", x.x, x.y, x.z);
	}

}

void PhysicsEngine::Task1Init(const Shader* rbShader, const Mesh* rbMesh, vec3 pos, vec3 scale, vec3 initVel, vec3 initRotVel)
{
	// Initialise the rigid body
	rbody1.SetShader(rbShader);
	rbody1.SetMesh(rbMesh);
	rbody1.SetPosition(pos);
	rbody1.SetScale(scale);
	rbody1.SetMass(2.0f);
	rbody1.SetVelocity(initVel);
	rbody1.SetAngularVelocity(initRotVel);
	rbody1.SetInverseInertia(vec3(scale.x * 2, scale.y * 2, scale.z * 2));
	rbody1.SetInertia(vec3(scale.x * 2, scale.y * 2, scale.z * 2));
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	rbody1.ClearForcesImpulses();
	rbody1.ClearRotationalForces();
	Force::Gravity(rbody1);
	//SymplecticEuler(rbody1, deltaTime);
	//SymplecticEuler(rbody1, deltaTime);
	//Integrate(rbody1, deltaTime);
	TOTAL_Integration(rbody1, deltaTime);
	CollisionImpulse(rbody1, 0.9f, ground.Position().y);
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