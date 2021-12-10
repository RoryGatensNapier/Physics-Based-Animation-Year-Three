#pragma once

#include <vector>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Shader;
class Mesh;
class Force;

// A generic renderable object, such as a particle or the ground, that we can render, move, rotate, etc
class PhysicsBody
{
public:

	// If we're going to derive from this class, create a virtual destructor that does nothing
	virtual ~PhysicsBody() {}

	void Draw(const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) const;

	// gets the position
	const glm::vec3& Position() const
	{
		return m_position;
	}

	const glm::vec3& Scale() const
	{
		return m_scale;
	}

	const glm::mat4& Orientation() const
	{
		return m_orientation;
	}

	const Mesh* GetMesh() const
	{
		return m_mesh;
	}

	// we must initialise it with a mesh and a shader
	void SetMesh(const Mesh* mesh)
	{
		m_mesh = mesh;
	}

	void SetShader(const Shader* shader)
	{
		m_shader = shader;
	}

	void SetColor(const glm::vec4& c)
	{
		m_color = c;
	}

	void SetPosition(const glm::vec3& position)
	{
		m_position = position;
	}

	void SetScale(const glm::vec3& scale)
	{
		m_scale = scale;
	}

	void SetOrientation(const glm::mat4& m) {
		m_orientation = m;
	}

	// translate mesh by a vector
	void Translate(const glm::vec3& offset)
	{
		m_position += offset;
	}

	// rotate mesh by an axis,angle pair
	void Rotate(const float angleInRads, const glm::vec3& axis)
	{
		m_orientation = glm::rotate(m_orientation, angleInRads, axis);
	}

	// getModel computes the model matrix any time it is required
	const glm::mat4 ModelMatrix() const
	{
		auto translateMatrix = glm::translate(glm::mat4(1.0f), m_position);
		auto scaleMatrix = glm::scale(glm::mat4(1.0f), m_scale);
		return translateMatrix * m_orientation * scaleMatrix;
	}

private:

	// A pointer to a const shader. Many particles can share the same shader, and we never modify it
	const Shader* m_shader = nullptr;

	// A pointer to a const mesh. Many particles can share the same mesh, and we never modify it
	const Mesh* m_mesh = nullptr;

	// color
	glm::vec4 m_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f);

	// Transformation data
	glm::vec3 m_position = glm::vec3(0.0f);
	glm::vec3 m_scale = glm::vec3(1.0f);
	glm::mat4 m_orientation = glm::mat4(1.0f);
};

// A particle is a physics body without shape/size. 
class Particle : public PhysicsBody
{
public:

	void SetCoefficientOfRestitution(float cor) { m_cor = cor; }
	void SetMass(float mass) { m_mass = mass; }
	void SetVelocity(const glm::vec3& velocity) { m_velocity = velocity; }

	// Call this at the beginning of a simulation step
	void ClearForcesImpulses() { m_accumulatedForce = glm::vec3(0.0f);  m_accumulatedImpulse = glm::vec3(0.0f); }
	// Adds to the sum of forces
	void ApplyForce(const glm::vec3& force) { m_accumulatedForce += force; }
	// Adds to the sum of impulses
	void ApplyImpulse(const glm::vec3& impulse) { m_accumulatedImpulse += impulse; }


	float Mass() const { return m_mass; }
	const glm::vec3& Velocity() const { return m_velocity; }
	const glm::vec3& AccumulatedForce() { return m_accumulatedForce; }
	const glm::vec3& AccumulatedImpulse() { return m_accumulatedImpulse; }
	float CoefficientOfRestitution() { return m_cor; }

private:
	float m_cor = 0.9f;									// Coefficient of restitution
	float m_mass = 1.0f;								// Particle mass, in kg
	glm::vec3 m_velocity = glm::vec3(0.0f);				// Velocity, in m/s. Important! Must initialise (like this here), otherwise starting value would be undefined
	glm::vec3 m_accumulatedForce = glm::vec3(0.0f);		// Accumulated force in a single simulation step
	glm::vec3 m_accumulatedImpulse = glm::vec3(0.0f);	// Accumulated impulse in a single simulation step
};

class RigidBody : public Particle
{
public:

	void SetAngularVelocity(const glm::vec3& angVel) { m_angularVelocity = angVel; }
	//void SetAngularAcceleration(const glm::vec3& angAccel) { m_angularAcceleration = angAccel; }

	const glm::vec3& AngularVelocity() const { return m_angularVelocity; }
	//const glm::vec3& AngularAcceleration() const { return m_angularAcceleration; }

	const glm::vec3& GetTorque() const { return m_torque; }
	void AddTorque(glm::vec3& newTorque) { m_torque = newTorque; }

	const glm::mat3& GetInertia() const { return m_Inertia; }
	void SetInertia(glm::vec3& dimensions) { m_Inertia = Inertia(dimensions); }

	const glm::mat3& GetInverseInertia() const { return m_Inv_Inertia; }
	void SetInverseInertia(glm::vec3& dimensions) { m_Inv_Inertia = InverseInertia(dimensions); }

	const glm::mat3& GetInertia_Sphere() const { return m_Inertia; }
	void SetInertia_Sphere(float radius) { m_Inertia = Inertia_Sphere(radius); }

	const glm::mat3& GetInverseInertia_Sphere() const { return m_Inv_Inertia; }
	void SetInverseInertia_Sphere(float radius) { m_Inv_Inertia = InverseInertia_Sphere(radius); }


	const glm::vec3& AngularMomentum() const { return m_AngularMomentum; }
	void SetAngularMomentum(glm::vec3& newMomentum) { m_AngularMomentum = newMomentum; }

	const glm::vec3& r() const { return m_rotAppVec; }
	void Set_r(glm::vec3& newRotAppVec) { m_rotAppVec = newRotAppVec; }

	void ClearRotationalForces() { m_torque = glm::vec3(0); }

	void SetChunk(int chunk_id) { _chunks.push_back(chunk_id); }
	void SetUniqueChunk(int chunk_id)
	{
		for (auto&& i : _chunks)
		{
			if (i == chunk_id)
			{
				printf("chunk already present!");
				return;
			}
		}
		_chunks.push_back(chunk_id);
	}
	void UnsetChunk(int chunk_id)
	{ 
		for (int i = 0; i < _chunks.size()-1; i++)
		{
			if (_chunks[i] == chunk_id)
			{
				auto iter = find(_chunks.begin(), _chunks.end(), chunk_id);
				if (iter != _chunks.end())
				{
					_chunks.erase(iter);
				}
				return;
			}
		}
	}
	std::vector<int> GetAllChunks() { return _chunks; }
	bool CheckChunk(int chunk_id)
	{
		if (_chunks.empty())
		{
			return false;
		}
		else
		{
			std::vector<int>::iterator it;
			it = find(_chunks.begin(), _chunks.end(), chunk_id);
			if (it != _chunks.end())
			{
				
				return true;
			}
			else
			{
				return false;
			}
		}
	}

	float GetRadius() { return _radius; }
	void SetRadius(float radius) { _radius = radius; }

private:
	glm::mat3 Inertia(glm::vec3 dimensions);
	glm::mat3 InverseInertia(glm::vec3 dimensions);

	glm::mat3 Inertia_Sphere(float radius);
	glm::mat3 InverseInertia_Sphere(float radius);

	glm::mat3 m_Inertia = glm::mat3(0);
	glm::mat3 m_Inv_Inertia = glm::mat3(0);

	glm::vec3 m_angularVelocity = glm::vec3(0.0f);
	glm::vec3 m_angularAcceleration = glm::vec3(0.0f);
	glm::vec3 m_AngularMomentum = glm::vec3(0);

	glm::vec3 m_torque = glm::vec3(0);
	glm::vec3 m_rotAppVec = glm::vec3(0);

	std::vector<int> _chunks;
	float _radius;
};