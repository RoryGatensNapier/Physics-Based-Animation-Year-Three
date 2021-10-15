#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"
#include <glm/gtx/string_cast.hpp>

using namespace glm;

bool ConicalCalculation(vec3 particlePos, double base, double tip, double radius)
{
	double coneHeight = base + tip;
	double coneRatio = coneHeight / radius;
	if (particlePos.z >= particlePos.x * coneRatio && particlePos.z >= particlePos.y * coneRatio && particlePos.z < coneHeight && particlePos.z > base)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Force::BlowDryer(Particle& particle, float cone_z_base, float cone_z_tip, float cone_r_base, float max_force = 100)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate blow dryer force
	vec3 force = { 0,0,-5 };
	float posArr[3] = { particle.Position().x, particle.Position().y, particle.Position().z };
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
	if (ConicalCalculation(particle.Position(), cone_z_base, cone_z_tip, cone_r_base))
	{
		force = vec3(force.x * linearMultiplier[0], force.y * linearMultiplier[1], force.z * linearMultiplier[2]);
		particle.ApplyForce(force);
	}
	else
	{
		particle.ApplyForce(vec3(0));
	}
}

void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p, Particle& p2, Particle& p3, vec3 vel_Air, float airDensity, float drag)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Should apply the aerodynamic drag force
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	vec3 v_surface = (p.Velocity() + p2.Velocity() + p3.Velocity());
	v_surface = vec3(v_surface.x / 3, v_surface.y / 3, v_surface.z / 3);
	auto v_computed = v_surface - vel_Air;
	auto normal_top = glm::cross((p2.Position() - p.Position()), (p3.Position() - p.Position()));
	auto normal = vec3(normal_top.x / normal_top.length(), normal_top.y / normal_top.length(), normal_top.z / normal_top.length());
	auto area = 0.5f * (normal_top.length());
	auto exposed_area = area * (glm::dot(v_computed, normal) / v_computed.length());

	auto force_pt1 = 0.5f * (airDensity) * (pow(v_computed.length(), 2)) * drag * (exposed_area);
	auto force = vec3(normal.x * force_pt1, normal.y * force_pt1, normal.z * force_pt1);
	force = vec3(force.x / 3, force.y / 3, force.z / 3);
	p.ApplyForce(force);
	p2.ApplyForce(force);
	p3.ApplyForce(force);
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