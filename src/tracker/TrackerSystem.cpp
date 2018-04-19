#include <iostream>
#include <stdexcept>
#include <vector>
#include <random>
#include <cmath>

#include "TrackerSystem.h"
#include "TrackerSystemInputHandler.h"
#include "renderer\renderer.h"
#include "util\timer.h"



/** Constructor */
TrackerSystem::TrackerSystem() :
	filter_update_interval_t(KALMAN_FILTER_FREQUENCY),

#ifdef DATA_3D
	object_X_filter_pos_xyz(filter_update_interval_t),
	object_X_filter_pos_acc_xyz(filter_update_interval_t),
#else
	object_X_filter_xy(filter_update_interval_t),
#endif
	continuous_graph_data_capture(false),
	stop_graph_data_capture(false),
	next_graph_bake_update_t(0),
	object_X(nullptr),
	object_C(nullptr),
	object_X_position_sensor(0, .2),
	object_X_acceleration_sensor(0, 0.1),
	object_C_acceleration_sensor(0, 0.1),
	future_position{ 0., 0., 0. },
	future_resolution_time{ FUTURE_PREDICTION_TIME },
	last_future_prediction_error{ 0. }
{
}

/** Destructor */
TrackerSystem::~TrackerSystem() {
	delete object_X;
	delete object_C;
	::renderer::window::kill();
}

/** Initialize the simulator. */
void TrackerSystem::Init() {
	InitGL();
	CreateWorldObjects();
	TrackerSystemInputHandler::Init(::renderer::window::get_window());
	PrintLegend();
};

/** Create the world objects. */
void TrackerSystem::CreateWorldObjects()
{
	glUseProgram(quad_shader_id);
	object_X = new SimulatedWorldObject(quad_shader_id, OBJECT_MOVEMENT_TYPE, OBJECT_START_POSITION, OBJECT_SIMULATION_TIME_MULTIPLIER, OBJECT_SIMULATION_VELOCITY_MULTIPLIER);
	object_C = new SimulatedWorldObject(quad_shader_id, CAMERA_MOVEMENT_TYPE, CAMERA_START_POSITION, CAMERA_SIMULATION_TIME_MULTIPLIER, CAMERA_SIMULATION_VELOCITY_MULTIPLIER);

	object_X_position_sensor.LaunchSensorThread(CAMERA_SENSOR_FREQUENCY);
	object_X_acceleration_sensor.LaunchSensorThread(ACCELEROMETER_X_SENSOR_FREQUENCY);
	object_C_acceleration_sensor.LaunchSensorThread(ACCELEROMETER_C_SENSOR_FREQUENCY);
}


////////////////////////////
// GL

/** Creates a window and initializes OpenGL */
void TrackerSystem::InitGL()
{
	// Create OpenGL window and context
	::renderer::window::create();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_COLOR, GL_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glLineWidth(2.f);
	glPointSize(2.f);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	// Load the shader to be used for rendering
	pointcloud_shader_id = ::renderer::shader::load_program({ "../src/shaders/pointcloud.glsl" });
	quad_shader_id = ::renderer::shader::load_program({ "../src/shaders/quad.glsl" });
}

/** Sets the view and projection matrices. */
void TrackerSystem::SetViewProjectionMatrix(const glm::vec3& camera_pos, const glm::vec3& camera_target, unsigned int shader_id) const
{
	// Change the view transformation matrix
	::renderer::space::update_view_matrix(camera_pos, camera_target);

	// get shader uniform locations
	const auto u_projection_matrix{ glGetUniformLocation(shader_id, "u_projection_matrix") };
	const auto u_view_matrix{ glGetUniformLocation(shader_id, "u_view_matrix") };

	// update the shader uniforms
	if (u_projection_matrix != -1) glUniformMatrix4fv(u_projection_matrix, 1, GL_FALSE, glm::value_ptr(::renderer::space::get_projection_matrix()));
	if (u_view_matrix != -1) glUniformMatrix4fv(u_view_matrix, 1, GL_FALSE, glm::value_ptr(::renderer::space::get_view_matrix()));
}

/** Renders the world. */
void TrackerSystem::Render() const
{
	// Get the user controlled camera properties.
	const auto camera_tracking{ TrackerSystemInputHandler::GetCameraTracking() };

	// Display
	glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
		
	glUseProgram(pointcloud_shader_id);
	SetViewProjectionMatrix(camera_tracking.position, camera_tracking.target, pointcloud_shader_id);
	analysis.Render(pointcloud_shader_id);

	glUseProgram(quad_shader_id);
	SetViewProjectionMatrix(camera_tracking.position, camera_tracking.target, quad_shader_id);
	object_X->Render(quad_shader_id);
	object_C->Render(quad_shader_id);
}


////////////////////////////
// FLOW

/** Start to continuously capture the data, respecting the senspor update interval. */
void TrackerSystem::StartContinousCapture()
{
	continuous_graph_data_capture = true;
	stop_graph_data_capture = false;
}

/** Stop continuously capturing data, and bake the already captured data. */
void TrackerSystem::StopContinuousCapture()
{
	continuous_graph_data_capture = false;
	stop_graph_data_capture = true;
}

////////////////////////////
// INFO

/** Prints the meanings of colours. */
void TrackerSystem::PrintLegend()
{
	std::cout << std::endl << "INFO:" << std::endl;
	std::cout << std::endl << "State estimation using noisy data and sensor fusion via a kalman filter." << std::endl;
	std::cout << "Filter Inputs: camera acceleration vector, target object acceleration vector, target location, in camera space." << std::endl;
	std::cout << "Filter Outputs: target object position, velocity, and acceleration, in camera space." << std::endl;

	std::cout << std::endl << "LEGEND (left):" << std::endl;
	std::cout << "\twhite - Camera path ground truth." << std::endl;
	std::cout << "\tred - Tracked object path ground truth." << std::endl;
	std::cout << "\tblue - Sensor readings." << std::endl;
	std::cout << "\tgreen - Kalman filter estimate." << std::endl;
	
	std::cout << std::endl << "LEGEND (right):" << std::endl;
	std::cout << "\tGraphs are differences in position between the ground truth and:" << std::endl;
	std::cout << "\twhite - Ground truth distance between the camera and the target." << std::endl;
	std::cout << "\tblue - The Sensor readings." << std::endl;
	std::cout << "\tgreen - The Kalman filter estimate." << std::endl;
	//std::cout << "\tyellow - Predicted location 0.2 seconds in the future." << std::endl;

	std::cout << std::endl;
}