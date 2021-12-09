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
	/*auto torque = glm::cross(rb.r(), rb.AccumulatedForce());
	rb.AddTorque(torque);*/
	auto velocity = rb.Velocity() + ((1.0f / rb.Mass()) * rb.AccumulatedForce() * dt) + rb.AccumulatedImpulse();
	auto position = rb.Position() + (velocity * dt);

	rb.SetVelocity(velocity);
	rb.SetPosition(position);

 	auto R = glm::mat3(rb.Orientation());

	auto pre_momentum = rb.GetInertia() * rb.AngularVelocity();
	rb.SetAngularMomentum(pre_momentum);
	auto momentum = rb.AngularMomentum() + rb.GetTorque() * dt;
	auto angular_velocity = rb.GetInverseInertia() * momentum;
	rb.SetAngularVelocity(angular_velocity);
	auto angVelSkew = glm::matrixCross3(angular_velocity);
	auto rotation = glm::orthonormalize(R + (angVelSkew * R));

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
	R += (glm::matrixCross3(rb.AngularVelocity()) * R);
	R = glm::orthonormalize(R);
	rb.SetOrientation(R);
}

double Impulse_RigidCollision(RigidBody& rb, float elasticity, vec3 CollisionCoords, vec3 CollisionNormal)
{
	rb.Set_r(CollisionCoords - rb.Position());
	auto test1 = glm::cross(rb.AngularVelocity(), rb.r());
	auto test2 = glm::dot(rb.Velocity() + glm::cross(rb.AngularVelocity(), rb.r()), CollisionNormal);
	auto numerator = -(1.0 + elasticity) * glm::dot(rb.Velocity() + glm::cross(rb.AngularVelocity(), rb.r()), CollisionNormal);
	auto r_x_Nhat = glm::cross(rb.r(), CollisionNormal);
	auto inertia_calc = rb.GetInverseInertia() * r_x_Nhat;
	auto iner_x_r = glm::cross(inertia_calc, rb.r());
	auto denominator = (1.0 / rb.Mass()) + glm::dot(CollisionNormal, iner_x_r);
	auto rotImpulse = numerator / denominator;
	
	if (rotImpulse < 0)
	{
		printf("Break Point\n");
	}

	return rotImpulse;
}

double Impulse_RelativeRigidCollision(RigidBody& rb, RigidBody& rb2, float elasticity, vec3 CollisionCoords, vec3 CollisionNormal)
{
	rb.Set_r(CollisionCoords - rb.Position());
	rb2.Set_r(CollisionCoords - rb2.Position());

	auto numerator = -(1.0 + elasticity) * glm::dot((rb.Velocity() - rb2.Velocity()) + glm::cross(rb.AngularVelocity(), rb.r()), CollisionNormal);

	//rb1
	auto R1_r_x_Nhat = glm::cross(rb.r(), CollisionNormal);
	auto R1_inertia_calc = rb.GetInverseInertia() * R1_r_x_Nhat;
	auto R1_iner_x_r = glm::cross(R1_inertia_calc, rb.r());
	
	//rb2
	auto R2_r_x_Nhat = glm::cross(rb2.r(), CollisionNormal);
	auto R2_inertia_calc = rb2.GetInverseInertia() * R2_r_x_Nhat;
	auto R2_iner_x_r = glm::cross(R2_inertia_calc, rb2.r());

	auto denominator = (1.0 / rb.Mass()) + (1.0 / rb2.Mass()) + glm::dot(CollisionNormal, (R1_iner_x_r + R2_iner_x_r));
	auto rotImpulse = numerator / denominator;
	printf("impulse - %f\n", rotImpulse);
	if (rotImpulse > 99)
	{
		printf("broken impulse - %f\n", rotImpulse);
	}
	return rotImpulse;
}

vec3 Friction(RigidBody& rb, float jr_impulse, vec3 rel_vel, float friction_val, vec3 normal, vec3 ext_forces)
{
	vec3 jt = vec3(0);
	vec3 tangent = vec3(0);
	if (glm::dot(rel_vel, normal) == 0)
	{
		if (glm::dot(ext_forces, normal) == 0)
		{
			return vec3(0);
		}
		tangent = ext_forces - (glm::dot(ext_forces, normal) * normal);
	}
	else
	{
		auto stage_1 = glm::dot(rel_vel, normal);
		auto stage_2 = (glm::dot(rel_vel, normal) * normal);
		tangent = rel_vel - (glm::dot(rel_vel, normal) * normal);
	}
	auto unit_vec = normalize(tangent);
	jt = -friction_val * jr_impulse * unit_vec;
	auto test_statement = dot(rel_vel, tangent);
	vec3 jf = vec3(0);
	if (test_statement == 0 && rb.Mass() * test_statement < (friction_val * jr_impulse))
	{
		jf = -(rb.Mass() * dot(rel_vel, tangent)) * tangent;
	}
	else
	{
		jf = -(friction_val * jr_impulse) * tangent;
	}
	return jf;
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
			auto nHat = normalize(normal);
			/*auto v_close = dot(rb.Velocity(), normal);

			impulse = -(1 + elasticity) * rb.Mass() * v_close * normal;
			rb.ApplyImpulse(impulse);*/
			//printf("impulse = %f, %f, %f\n", impulse.x, impulse.y, impulse.z);
			printf("world coord of vertex that has collided = %f, %f, %f\n", ws_coord.x, ws_coord.y, ws_coord.z);
			// use the above output to calculate the r vec for applying angular forces. since mesh is box, CoM is just the location, derive stuff from there
			float jr = Impulse_RigidCollision(rb, elasticity, vec3(ws_coord), nHat);
			auto final_force = Friction(rb, jr, rb.Velocity(), 0.6, nHat, vec3(0)); //--Eventually get friction in
			rb.ApplyForce(final_force);
			rb.SetVelocity(rb.Velocity() + (jr / rb.Mass()) * nHat);
			auto test = (rb.AngularVelocity() - (jr * (rb.GetInverseInertia() * glm::cross(rb.Position(), nHat))));
			rb.SetAngularVelocity(rb.AngularVelocity() + (jr * (rb.GetInverseInertia() * glm::cross(rb.Position(), nHat))));
		}
	}
}

std::tuple<vec3, vec3> StS_Collision_noRot(RigidBody& rb1, RigidBody& rb2, float elasticity)
{
	auto rb1_normal = rb2.Position() - rb1.Position();
	auto rb2_normal = rb1.Position() - rb2.Position();

	auto rb1_nHat = normalize(rb1_normal);
	auto rb2_nHat = normalize(rb2_normal);

	auto rb1_hitpt = rb1.Position() + (rb1.GetRadius() * rb1_nHat);
	auto rb2_hitpt = rb2.Position() + (rb2.GetRadius() * rb1_nHat);

	float rb1_jr = Impulse_RelativeRigidCollision(rb1, rb2, elasticity, rb1_hitpt, rb1_nHat);
	float rb2_jr = Impulse_RelativeRigidCollision(rb2, rb1, elasticity, rb2_hitpt, rb2_nHat);

	auto rb1_retval = (rb1.Velocity() + (rb1_jr / rb1.Mass()) * rb1_nHat);
	auto rb2_retval = (rb2.Velocity() + (rb2_jr / rb2.Mass()) * rb2_nHat);
 	return std::tuple<vec3, vec3>(rb1_retval, rb2_retval);
}

void StS_ColDetection(RigidBody& rb1, RigidBody& rb2, float elasticityVal)
{
	auto posVec = rb2.Position() - rb1.Position();
	auto rb1_normal = normalize(posVec);
	auto rb2_normal = -rb1_normal;
	auto distance = length(posVec);
	if (distance <= rb1.GetRadius() + rb2.GetRadius())
	{
		auto sinkMargin = -((rb1.GetRadius() + rb2.GetRadius()) - distance)/2;
		rb1.Translate(sinkMargin * rb1_normal);
		rb2.Translate(sinkMargin * rb2_normal);
		auto retVal = StS_Collision_noRot(rb1, rb2, elasticityVal);
		rb1.SetVelocity(std::get<0>(retVal));
		rb2.SetVelocity(std::get<1>(retVal));
	}
}

void Walls_CollisionDetection(RigidBody& rb, PhysicsBody& ground, float elasticityVal)
{
	float impulse = 0;
	if (rb.Position().x >= ground.Position().x + ground.Scale().x || rb.Position().x <= ground.Position().x - ground.Scale().x)
	{
		auto sinkMargin = rb.Position().x >= ground.Position().x + ground.Scale().x ? ground.Position().x + ground.Scale().x - rb.Position() : ground.Position() - ground.Scale().x - rb.Position();
		auto normal = rb.Position().x >= ground.Position().x + ground.Scale().x - rb.GetRadius() ? vec3(-1, 0, 0) : vec3(1, 0, 0);
		auto hitPoint = rb.Position() + (rb.GetRadius() * normal);
		rb.Translate(sinkMargin * normal);
		//rb.SetPosition(vec3(ground.Position().x + (ground.Scale().x * normal.x), rb.Position().y, rb.Position().z));
		impulse = abs(Impulse_RigidCollision(rb, elasticityVal, hitPoint, normal));
		rb.SetVelocity(rb.Velocity() + ((impulse / rb.Mass()) * normal));
		auto print = rb.Velocity();
	}
	if (rb.Position().z >= ground.Position().z + ground.Scale().z || rb.Position().z <= ground.Position().z - ground.Scale().z)
	{
		auto sinkMargin = rb.Position().z >= ground.Position().z + ground.Scale().z ? ground.Position().z + ground.Scale().z - rb.Position() : rb.Position() - ground.Position().z - ground.Scale().z;
		auto normal = rb.Position().z >= ground.Position().z + ground.Scale().z ? vec3(0, 0, -1) : vec3(0, 0, 1);
		auto hitPoint = rb.Position() + (rb.GetRadius() * normal);
		rb.Translate(sinkMargin * normal);
		impulse = abs(Impulse_RigidCollision(rb, elasticityVal, hitPoint, normal));
		rb.SetVelocity(rb.Velocity() + (impulse / rb.Mass()) * normal);
	}
	if (rb.Position().y <= ground.Position().y + (2*rb.GetRadius()))
	{
		auto sinkMargin = ground.Position().y + 2*rb.GetRadius() - rb.Position();
		auto normal = vec3(0, 1, 0);
		auto hitPoint = rb.Position() + (rb.GetRadius() * normal);
		rb.Translate(sinkMargin * normal);
		impulse = abs(Impulse_RigidCollision(rb, elasticityVal, hitPoint, normal));
		rb.SetVelocity(rb.Velocity() + ((impulse / rb.Mass()) * normal));
	}
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	// Initialise ground
	auto groundMesh = meshDb.Get("cube");

	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(20.f, 1.0f, 20.f));

	camera = Camera(vec3(0, 10, 30));
	//RigidBodyInit(defaultShader, meshDb.Get("cube"), vec3(0, 5, 0), vec3(1, 3, 1), vec3(5, 0, 0), vec3(0, 0, 0));
	for (int x = 0; x <= ballCount-1; x++)
	{
		auto sphereMesh = meshDb.Get("sphere");
		vec3 randomPoint = vec3(rand() % 38 - 19, 8, rand() % 38 - 19);
		RigidBody temp = SpheresInit(defaultShader, sphereMesh, randomPoint/*vec3(x*4, 10, 0)*/, vec3(1), vec3(0), vec3(0));
		Balls.push_back(temp);
	}
	for (auto x : ground.GetMesh()->Data().positions.data)
	{
		printf("Local Ground Positions - %f, %f, %f\n", x.x, x.y, x.z);
		auto ws_groundpts = ground.ModelMatrix() * vec4(x.x, x.y, x.z, 1);
		printf("World Space Ground Position - %f, %f, %f\n\n", ws_groundpts.x, ws_groundpts.y, ws_groundpts.z);
	}
	//Balls[1].SetVelocity(vec3(-10, 0, 0));
}

void PhysicsEngine::RigidBodyInit(const Shader* rbShader, const Mesh* rbMesh, vec3 pos, vec3 scale, vec3 initVel, vec3 initRotVel)
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

RigidBody PhysicsEngine::SpheresInit(const Shader* rbShader, const Mesh* rbMesh, vec3 pos, vec3 scale, vec3 initVel, vec3 initRotVel)
{
	// Initialise the rigid body
	RigidBody sphere;
	sphere.SetShader(rbShader);
	sphere.SetMesh(rbMesh);
	sphere.SetPosition(pos);
	sphere.SetScale(scale);
	sphere.SetMass(1.0f);
	sphere.SetVelocity(initVel);
	sphere.SetAngularVelocity(initRotVel);
	sphere.SetInverseInertia_Sphere(sphere.GetRadius());
	sphere.SetInertia_Sphere(sphere.GetRadius());
	return sphere;
}

void PhysicsEngine::Pooling()
{
	for (auto&& ball : Balls)
	{
		if (ball.Position().x >= 0)
		{
			if (ball.Position().z >= 0)
			{
				if (!ball.CheckChunk(3))
				{
					ball.SetUniqueChunk(3);
				}
			}
			else
			{
				if (!ball.CheckChunk(2))
				{
					ball.SetUniqueChunk(2);
				}
			}
		}
		else
		{
			if (ball.Position().z >= 0)
			{
				if (!ball.CheckChunk(4))
				{
					ball.SetUniqueChunk(4);
				}
			}
			else
			{
				if (!ball.CheckChunk(1))
				{
					ball.SetUniqueChunk(1);
				}
			}
		}
	}
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	for (int i = 0; i <= ballCount-1; i++)
	{
		Balls[i].ClearForcesImpulses();
		Balls[i].ApplyForce(GRAVITY);
		TOTAL_Integration(Balls[i], deltaTime);
		Walls_CollisionDetection(Balls[i], ground, wallElasticity);
	}
	Pooling();
	StS_ColDetection(Balls[0], Balls[1], ballElasticity);
	//printf("ball 1 speed = %f, %f, %f\n", Balls[1].Velocity().x, Balls[1].Velocity().y, Balls[1].Velocity().z);
	//CollisionImpulse(rbody1, 0.7f, ground.Position().y);
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
	//rbody1.Draw(viewMatrix, projMatrix);
	for (auto x : Balls)
	{
		x.Draw(viewMatrix, projMatrix);
	}
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