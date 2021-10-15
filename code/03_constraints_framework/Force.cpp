#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"
#include <glm/gtx/string_cast.hpp>

using namespace glm;

bool ConicalCalculation(vec3 particlePos, double base, double tip, double radius)
{
	double coneHeight = base + tip;
	double coneRatio = coneHeight / radius;
	if (particlePos.y >= particlePos.x * coneRatio && particlePos.y >= particlePos.z * coneRatio && particlePos.y < coneHeight && particlePos.y > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

vec3 BlowDryerForce(const vec3& particlePosition, float cone_y_base, float cone_y_tip, float cone_r_base, float max_force = 100)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate blow dryer force
	vec3 force = { 0,0,5 };
	float posArr[3] = { particlePosition.x, particlePosition.y, particlePosition.z };
	float linearMultiplier[sizeof(posArr)] = { 0 };
	for (int x = 0; x < sizeof(posArr); x++)
	{
		if (posArr[x] <= -1)
		{
			linearMultiplier[x] = 1 / posArr[x];
		}
		if (posArr[x] < 0 && posArr[x] > -1)
		{
			linearMultiplier[x] = 1 * posArr[x];
		}
		if (posArr[x] == 0)
		{
			linearMultiplier[x] = 1;
		}
		else if (posArr[x] < 1 && posArr[x] > 0)
		{
			linearMultiplier[x] = 1 * posArr[x];
		}
		else if (posArr[x] > 1)
		{
			linearMultiplier[x] = 1 / posArr[x];
		}
	}
	if (ConicalCalculation(particlePosition, cone_y_base, cone_y_tip, cone_r_base))
	{
		force = vec3(force.x * linearMultiplier[0], force.y * linearMultiplier[1], force.z * linearMultiplier[2]);
		return force;
	}
	else
	{
		return vec3(0);
	}
}

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