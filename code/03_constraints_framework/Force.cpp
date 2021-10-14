#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"
#include <glm/gtx/string_cast.hpp>

using namespace glm;

void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Should apply the aerodynamic drag force
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd)
{
	float curLength = glm::distance(p1.Position(), p2.Position());
	//printf("Current length = %f\n", curLength);
	if (curLength <= 0)
	{
		curLength = 0.00001f;
	}
	auto unitVector = (p2.Position() - p1.Position()) / curLength;
	auto p1_vel1D = glm::dot(unitVector, p1.Velocity());
	auto p2_vel1D = glm::dot(unitVector, p2.Velocity());
	auto forceSD_p1 = -(ks * (restLength - curLength)) - (kd * p1_vel1D);
	auto forceSD_p2 = -(ks * (restLength - curLength)) - (kd * p2_vel1D);
	auto force_p1 = forceSD_p1 * unitVector;
	auto force_p2 = -forceSD_p2 * unitVector;
	if (p1.IsFixed())
	{
		p1.ApplyForce(vec3(0));
	}
	else
	{
		p1.ApplyForce(force_p1);
	}

	if (p2.IsFixed())
	{
		p2.ApplyForce(vec3(0));
	}
	else
	{
		p2.ApplyForce(force_p2);
	}
	//printf("Force P1 = %s\n", glm::to_string(force_p1).c_str());
	//printf("Force P2 = %s\n", glm::to_string(force_p2).c_str());
}