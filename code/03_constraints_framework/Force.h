#pragma once

#include <glm/glm.hpp>

class Particle;

class Force
{
public:
	static void Gravity(Particle& p);
	static void Drag(Particle& p, Particle& p2, Particle& p3, glm::vec3 vel_Air, float airDensity, float drag);
	static void Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd);
	static void BlowDryer(Particle& particle, float cone_y_base, float cone_y_tip, float cone_r_base, float max_force);
private:
};