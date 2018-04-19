/**
* A class for a self-animated simulation object. 
* It will hold ground truth values for that particular object.
*/


#include "renderer/quad.h"

#include <glm/mat4x4.hpp>				// glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/component_wise.hpp>

#include <iostream>

#include "renderer\renderer.h"

class SimulatedWorldObject
{
public:
	/** Object motion path enumerator. */
	enum class MotionType { skewed_8 = 0, linear_no_acc, linear_const_acc, sin_with_acc, circle, stationary, Count };

	/** Create an animated object. Creates the visualization for the object. */
	SimulatedWorldObject(const unsigned int shader_id, const MotionType motion_type = MotionType::skewed_8, const glm::dvec3& position = glm::dvec3(0), const double time_multiplier = 1., const double velocity_multiplier = 1.):
		cosine_shift(2.7), 
		radius(3), 
		velocity(0, 0, 0), 
		acceleration(0, 0, 0),		
		animate(true) ,
		motion_type(motion_type),
		position(position),
		initial_position(position),
		time_multiplier(time_multiplier),
		velocity_multiplier(velocity_multiplier)
	{ 
		quad_id = ::renderer::quad::create(shader_id, true); 
	};

	/** Destroy the object. */
	~SimulatedWorldObject() { ::renderer::quad::kill(quad_id); };
	
	/** Render the object. */
	void Render(const unsigned int shader_id)
	{
		const auto model_matrix{ glm::translate(glm::mat4(1), {position.x, position.y, position.z}) * glm::scale(glm::mat4(1.0f), glm::vec3(0.25f)) };
		SetModelMatrix(model_matrix, shader_id);
		::renderer::quad::render(quad_id); 
	};

	/** Update the object location, stepping the simulation to a new time. */
	void Update(const double& global_t, const double& delta_t)
	{
		if (!animate) return;
		if (global_t < 0.1) return;	// Don't run the simulation for a time.

		// Previous velocity
		const auto v0 = velocity * velocity_multiplier;// (std::sin(global_t));

		// Motion equations
		switch (motion_type)
		{
		case MotionType::skewed_8:
			// Skewed 8
			velocity = glm::dvec3({ -std::cos(global_t * time_multiplier + cosine_shift), -std::sin(global_t * time_multiplier * 2.) * 0.5, std::sin(global_t * time_multiplier) * .15 });
			break;
		case MotionType::linear_no_acc:
			// Linear without acceleration
			velocity = glm::dvec3({ 1, 0.5, 0.3 });
			break;
		case MotionType::linear_const_acc:
			// Linear with constant acceleration
			velocity = glm::dvec3({ global_t*0.1, global_t*0.1, global_t*0.1 });
			break;
		case MotionType::sin_with_acc:
			// Sinusoidal with acceleration
			velocity = glm::dvec3({ 1, sin(global_t * 2.0), 0.0 });
			break;
		case MotionType::circle:
			// Circle
			velocity = glm::dvec3({ sin(global_t), cos(global_t), -cos(global_t) });
			break;
		case MotionType::stationary:
			// Static
			velocity = glm::dvec3({ 0., 0., 0. });
			break;
			
		}

		// Computed properties
		acceleration = (velocity - v0) / delta_t;		
		position += ((v0 + velocity) * 0.5 * delta_t) * radius;  // or equivalently === position += ((v0 * delta_t) + 0.5 * acceleration * delta_t * delta_t) * radius;

		// Sanity check
		const auto pred_vec_mag{ glm::compAdd(glm::abs(velocity - (v0 + acceleration * delta_t)) ) };
		assert(pred_vec_mag < 0.00001);
	}

	/** Change the object position. */
	void SetPosition(const glm::dvec3& position_in)
	{
		position = position_in;
	}

	/** Reset the object position to initial values. */
	void Reset()
	{
		position = initial_position;
		velocity = { 0, 0, 0 };
		acceleration = { 0, 0, 0 };
	}

	/** Getter functions. */
	glm::dvec3 GetPosition() const { return position; };
	glm::dvec3 GetVelocity() const { return velocity; };
	glm::dvec3 GetAcceleration() const { return acceleration; };

private:
	/** Change the object model matrix for rendering. */
	void SetModelMatrix(const glm::mat4x4& model_transform, const unsigned int shader_id) const
	{
		// get shader uniform locations
		const auto u_model_matrix{ glGetUniformLocation(shader_id, "u_model_matrix") };
		// update the shader uniforms
		if (u_model_matrix != -1) glUniformMatrix4fv(u_model_matrix, 1, GL_FALSE, glm::value_ptr(model_transform));
	}

private:
	glm::dvec3 position, velocity, acceleration;
	unsigned int quad_id;			// For rendering purposes.

	const glm::dvec3 initial_position;
	const MotionType motion_type;
	const bool animate;
	const double cosine_shift;		// A value added to the cos input. this will create deviation from the circular shape.
	const double radius;				// The radius of the path the object will take, when the shape is left as a circle
	const double velocity_multiplier; // The adjustment to the speed of the object.
	const double time_multiplier;	// By how much we will displace the global time for animation purposes
};