//#define CLOTHSIMULATION_VERBOSE
#include "Simulator.hpp"

int main()
{
	
	Simulator::Simulator simulator;

	auto& cloth_handle = simulator.add_cloth<Simulator::RectangleDeformalbleModel>(20, 10);
	cloth_handle->translate(0, 2, -0.2);
	cloth_handle->set_color(1.0f, 0.0f, 0.0f);
	auto& rigid_handle = simulator.add_rigid<Simulator::Quad>();
	rigid_handle->set_color(0.0f, 0.0f, 1.0f);
	//rigid_handle->translate(0, 1.5, -1);
	//rigid_handle->scale(0.5, 0.5, 0.5);

	simulator.init_main_loop();

	glEnable(GL_DEPTH_TEST); /* enable depth-testing */
	glEnable(GL_MULTISAMPLE);/* enable multi-sampling */


	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPointSize(5);
	
	simulator.add_shader("main.vs", "main.fs");
	simulator.set_camera(-5, 4, 0, 0.f, -30.f);
	
	double lastTime = glfwGetTime();
	const double timeStep = 1.0 / 60.0;
	double accumulator = timeStep;
	bool needRedisplay = true;

	unsigned i = 0;

	while (!simulator.window_should_close())
	{
		double currTime = glfwGetTime();
		double deltaTime = currTime - lastTime;
		lastTime = currTime;
		accumulator += deltaTime;

		simulator.process_input(deltaTime);

		if (needRedisplay)
		{
			needRedisplay = false;
			glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			//simulator.use_and_set_shader("main");

			simulator.draw_all_model("main");

			simulator.main_loop_end();
		}

		if (accumulator >= timeStep)
		{
			/*if (i % 10 == 0) {
				simulator.clear_hashtable();
			}*/
			simulator.pre_step_physics();
			cloth_handle->set_translation_degree(19, 0);
			//cloth_handle->set_translation_degree(199, 0);
			simulator.step_physics(timeStep);
			accumulator -= timeStep;
			i++;
		}

		needRedisplay = true;
	}
	glfwTerminate();

	return 0;
}