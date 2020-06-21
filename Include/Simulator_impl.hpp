#include "Simulator.hpp"

using namespace Eigen;
const float KPenalty = 2000;

template<typename ClothType, typename... Val>
Simulator::Simulator::cloth_handle&
Simulator::Simulator::add_cloth(Val&&... val)
{
	_cloths.emplace_back(std::make_unique<ClothType>(val...));

	return _cloths.back();
}

template<typename RigidType, typename... Val>
Simulator::Simulator::rigid_handle&
Simulator::Simulator::add_rigid(Val&&... val)
{
	_rigids.emplace_back(std::make_unique<RigidType>(val...));

	return _rigids.back();
}

Simulator::Simulator::cloth_handle&
Simulator::Simulator::cloth(unsigned i)
{
	return _cloths[i];
}

Simulator::Simulator::rigid_handle&
Simulator::Simulator::rigid(unsigned i)
{
	return _rigids[i];
}

Shader
Simulator::Simulator::add_shader(const char* vShaderFile, const char* fShaderFile, const char* gShaderFile, const char* name)
{
	return _shaders.LoadShader(vShaderFile, fShaderFile, gShaderFile, name);
}

Shader
Simulator::Simulator::shader(const char* name)
{
	return _shaders.GetShader(name);
}

/*
void
Simulator::Simulator::limit_particle(unsigned cloth, unsigned element, int degree,
											float pX, float pY, float pZ, float qX, float qY, float qZ)
{
	auto& particle = *(_cloths[cloth]->vertices_begin() + element);

	particle.set_translation_degree(degree);
	if (degree == 2) {
		Vector3f p(pX, pY, pZ);
		particle.set_p(p);
	} else if (degree == 1) {
		Vector3f p(pX, pY, pZ);
		Vector3f q(qX, qY, qZ);
		particle.set_p(p);
		particle.set_q(q);
	}
}

void
Simulator::Simulator::unlimit_particle(unsigned cloth, unsigned element)
{
	auto& particle = *(_cloths[cloth]->vertices_begin() + element);
	particle.set_translation_degree(3);
}
*/

void
Simulator::Simulator::process_input(float deltaTime)
{
	if (glfwGetKey(_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(_window, true);
	else if (glfwGetKey(_window, GLFW_KEY_W) == GLFW_PRESS)
		_camera.process_keyboard(FORWARD, deltaTime);
	else if (glfwGetKey(_window, GLFW_KEY_S) == GLFW_PRESS)
		_camera.process_keyboard(BACKWARD, deltaTime);
	else if (glfwGetKey(_window, GLFW_KEY_A) == GLFW_PRESS)
		_camera.process_keyboard(LEFT, deltaTime);
	else if (glfwGetKey(_window, GLFW_KEY_D) == GLFW_PRESS)
		_camera.process_keyboard(RIGHT, deltaTime);
	else if (glfwGetKey(_window, GLFW_KEY_Z) == GLFW_PRESS)
		_camera.process_keyboard(UP, deltaTime);
	else if (glfwGetKey(_window, GLFW_KEY_X) == GLFW_PRESS)
		_camera.process_keyboard(DOWN, deltaTime);
	else if (glfwGetKey(_window, GLFW_KEY_Q) == GLFW_PRESS)
		_camera.process_keyboard(TURNLEFT, deltaTime);
	else if (glfwGetKey(_window, GLFW_KEY_E) == GLFW_PRESS)
		_camera.process_keyboard(TURNRIGHT, deltaTime);
	else if (glfwGetKey(_window, GLFW_KEY_R) == GLFW_PRESS)
		_camera.process_keyboard(TURNUP, deltaTime);
	else if (glfwGetKey(_window, GLFW_KEY_F) == GLFW_PRESS)
		_camera.process_keyboard(TURNDOWN, deltaTime);
}

void
Simulator::Simulator::init_main_loop()
{
	if (!glfwInit()) {
		std::cerr << "GL initialization failed" << std::endl;
		exit(1);
	}
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif
	_window = glfwCreateWindow(_scr_width, _scr_height, "LearnOpenGL", NULL, NULL);
	if (_window == nullptr)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(-1);
	}
	glfwMakeContextCurrent(_window);
	
	gladLoadGL();

	for (rigid_handle& pRigid : _rigids)
		pRigid->init_for_drawing();

	for (cloth_handle& pCloth : _cloths)
		pCloth->init_for_drawing();

	init_hashtable();
}

inline void
Simulator::Simulator::set_camera(float posX, float posY, float posZ, float yaw, float pitch, float upX, float upY, float upZ)
{
	_camera = Camera(posX, posY, posZ, upX, upY, upZ, yaw, pitch);
}


bool
Simulator::Simulator::window_should_close()
{
	return glfwWindowShouldClose(_window);
}

void
Simulator::Simulator::use_and_set_shader(const char* name)
{
	Shader objShader = _shaders.GetShader(name);
	objShader.Use();

	glm::mat4 projection =
		glm::perspective(
			glm::radians(_camera.fov()),
			(float)_scr_width / (float)_scr_height,
			0.1f, 1000.0f
			);
	
	objShader.SetMatrix4("view", _camera.view_matrix());
	objShader.SetMatrix4("projection", projection);
}

void
Simulator::Simulator::main_loop_end()
{
	glfwSwapBuffers(_window);
	glfwPollEvents();
}
/*
void
Simulator::Simulator::main_loop()
{
	double lastTime = glfwGetTime();
	const double timeStep = 1.0 / 60.0;
	double accumulator = timeStep;
	bool needRedisplay = true;

	unsigned i = 0;
	
	while (!glfwWindowShouldClose(_window))
	{
		double currTime = glfwGetTime();
		double deltaTime = currTime - lastTime;
		lastTime = currTime;
		accumulator += deltaTime;
		
		process_input(deltaTime);

		if (needRedisplay)
		{
			needRedisplay = false;
			glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			glm::mat4 projection =
				glm::perspective(
					glm::radians(_camera.fov()),
					(float)_scr_width / (float)_scr_height,
					0.1f, 1000.0f
					);

			Shader objShader = _shaders.GetShader("main");
			objShader.Use();

			objShader.SetMatrix4("view", _camera.view_matrix());
			objShader.SetMatrix4("projection", projection);

			draw_all_model();

			glfwSwapBuffers(_window);
			glfwPollEvents();
		}

		if (accumulator >= timeStep)
		{
			if (i % 10 == 0) {
				_hash_table->clear();
			}
			step_physics(timeStep);
			accumulator -= timeStep;
			i++;
		}

		needRedisplay = true;
	}
	glfwTerminate();
}*/

void
Simulator::Simulator::pre_step_physics()
{
	for (cloth_handle& pCloth : _cloths)
		pCloth->pre_step_physics();
}

void
Simulator::Simulator::step_physics(float step)
{
#ifdef CLOTHSIMULATION_VERBOSE
	std::cout << std::setprecision(6);
	double currTime = glfwGetTime();
#endif
	
	_hash_table->pre_next_step();
	collision_process();

#ifdef CLOTHSIMULATION_VERBOSE
	double deltaTime = glfwGetTime() - currTime;
	std::cout << "Collision Handling: " << deltaTime << " seconds" << std::endl;
	currTime = glfwGetTime();
#endif
	
	for (cloth_handle& pCloth : _cloths)
		pCloth->step_physics(step);

#ifdef CLOTHSIMULATION_VERBOSE
	deltaTime = glfwGetTime() - currTime;
	std::cout << "Step forward: " << deltaTime << " seconds" << std::endl;
#endif
}

void
Simulator::Simulator::draw_all_model(const char* shader)
{
	Shader objShader = _shaders.GetShader(shader);
	objShader.Use();

	glm::mat4 projection =
		glm::perspective(
			glm::radians(_camera.fov()),
			(float)_scr_width / (float)_scr_height,
			0.1f, 1000.0f
			);

	objShader.SetMatrix4("view", _camera.view_matrix());
	objShader.SetMatrix4("projection", projection);
	
	for (rigid_handle& pRigid : _rigids)
	{
		objShader.SetVector3f("color", pRigid->color().x(), pRigid->color().y(), pRigid->color().z());
		pRigid->draw();
	}

	for (cloth_handle& pCloth : _cloths)
	{
		objShader.SetVector3f("color", pCloth->color().x(), pCloth->color().y(), pCloth->color().z());
		pCloth->draw();
	}
}

inline void
Simulator::Simulator::clear_hashtable()
{
	_hash_table->clear();
}


void Simulator::Simulator::collision_process()
{
	float cellSize = 0.0f;
	unsigned int nEdges = 0;

	for (rigid_handle& pRigid :_rigids) {
		cellSize += pRigid->avg_edge_length() * pRigid->size_edges();
		nEdges += pRigid->size_edges();
	}
#ifdef CLOTHSIMULATION_VERBOSE
	std::cout << "\tNumber of cloth edges: " << nEdges << std::endl;
#endif
	cellSize /= nEdges;

#ifdef CLOTHSIMULATION_VERBOSE
	double deltaTime, currTime = glfwGetTime();
	unsigned totalRigidsInsertingCells = 0;
#endif
	

	// Build grids
	for (unsigned int i = 0; i < _cloths.size(); ++i) {
		cloth_handle& pCloth = _cloths[i];
		unsigned element = 0;
		
		for (auto it = pCloth->vertices_begin(); it != pCloth->vertices_end(); ++it, ++element) {
			Vector3i cell = (it->coord() / cellSize).array().floor().cast<int>();
			_hash_table->insert(cell, HashTableEntry::CLOTH, i, element);
		}
	}

#ifdef CLOTHSIMULATION_VERBOSE
	deltaTime = glfwGetTime() - currTime;
	std::cout << "Insert cloths into hashtable: " << deltaTime << " seconds" << std::endl;
	currTime = glfwGetTime();

	double totalAABBTime = 0;
	double totalInsertingTime = 0;
	double currSubTime = 0;
#endif

	
	for (unsigned int i = 0; i < _rigids.size(); ++i) {
		rigid_handle& pRigid = _rigids[i];
		RigidModel::point_iterator pointsBegin = pRigid->points_begin();
		unsigned element = 0;

		for (auto it = pRigid->tetrahedrons_begin(); it != pRigid->tetrahedrons_end(); ++it, ++element) {
#ifdef CLOTHSIMULATION_VERBOSE
			currSubTime = glfwGetTime();
#endif
			auto& indices = (*it);
			Matrix<float, 3, 4> points;
			RigidModel::Point& p1 = *(pointsBegin + indices[0]);
			RigidModel::Point& p2 = *(pointsBegin + indices[1]);
			RigidModel::Point& p3 = *(pointsBegin + indices[2]);
			RigidModel::Point& p4 = *(pointsBegin + indices[3]);
			points.col(0) = Vector3f(p1.x(), p1.y(), p1.z());
			points.col(1) = Vector3f(p2.x(), p2.y(), p2.z());
			points.col(2) = Vector3f(p3.x(), p3.y(), p3.z());
			points.col(3) = Vector3f(p4.x(), p4.y(), p4.z());
			Vector3f minCoord = points.rowwise().minCoeff();
			Vector3f maxCoord = points.rowwise().maxCoeff();
			minCoord = (minCoord / cellSize).array().floor();
			maxCoord = (maxCoord / cellSize).array().floor();

#ifdef CLOTHSIMULATION_VERBOSE
			double AABBTime = glfwGetTime() - currSubTime;
			totalAABBTime += AABBTime;
			currSubTime = glfwGetTime();
			totalRigidsInsertingCells += ((maxCoord - minCoord) + Vector3f(1, 1, 1)).prod();
#endif
			_hash_table->insert_range(minCoord.x(), maxCoord.x(), minCoord.y(), maxCoord.y(), minCoord.z(), maxCoord.z(),
				HashTableEntry::RIGID, i, element);
#ifdef CLOTHSIMULATION_VERBOSE
			double insertingTime = glfwGetTime() - currSubTime;
			totalInsertingTime += insertingTime;
#endif
		}
	}

#ifdef CLOTHSIMULATION_VERBOSE
	deltaTime = glfwGetTime() - currTime;
	std::cout << "\tInsert rigids into hashtable: " << deltaTime << " seconds" << std::endl;
	std::cout << "\tCalculate AABB: " << totalAABBTime << " seconds" << std::endl;
	std::cout << "\tInsering rigids: " << totalInsertingTime << " seconds" << std::endl;
	std::cout << "\tTotal inserted cells of rigids: " << totalRigidsInsertingCells << std::endl;
	std::cout << "\tCell size " << cellSize << std::endl;
	currTime = glfwGetTime();
#endif

	for (unsigned int i = 0; i < _cloths.size(); ++i) {
		cloth_handle& pCloth = _cloths[i];
		unsigned element = 0;

		for (auto it = pCloth->vertices_begin(); it != pCloth->vertices_end(); ++it, ++element) {
			std::vector<HashTableEntry> allPossibleCollisions;
			
			Vector3i cell = (it->coord() / cellSize).array().floor().cast<int>();
			_hash_table->find_in_cell(cell, std::back_inserter(allPossibleCollisions));
			for (const HashTableEntry& possibleCollision: allPossibleCollisions)
			{
				if (possibleCollision.type == HashTableEntry::RIGID)
				{
					const rigid_handle& pRigid = _rigids[possibleCollision.object];
					const RigidModel::Tetrahedron& tedrahedron = *(pRigid->tetrahedrons_begin() + possibleCollision.element);
					RigidModel::Point& p1 = *(pRigid->points_begin() + tedrahedron[0]);
					RigidModel::Point& p2 = *(pRigid->points_begin() + tedrahedron[1]);
					RigidModel::Point& p3 = *(pRigid->points_begin() + tedrahedron[2]);
					RigidModel::Point& p4 = *(pRigid->points_begin() + tedrahedron[3]);
					const Vector3f x0 = Vector3f(p1.x(), p1.y(), p1.z());
					const Vector3f x1 = Vector3f(p2.x(), p2.y(), p2.z());
					const Vector3f x2 = Vector3f(p3.x(), p3.y(), p3.z());
					const Vector3f x3 = Vector3f(p4.x(), p4.y(), p4.z());
					const Vector3f& p = it->coord();
					Matrix3f A;
					A << x1 - x0, x2 - x0, x3 - x0;
					Vector3f beta = A.inverse() * (p - x0);
					Vector4f::Index minIndex;
					
					if ((beta.array() >= 0).all() && beta.sum() <= 1)
					{
						typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
						typedef K::Plane_3 Plane;
						typedef K::Line_3 Line;
						typedef K::Ray_3 Ray;
						typedef K::Point_3 Point;
						typedef K::Vector_3 Vector;
						Vector penaltyForce;
						Point points[4] = { p1, p2, p3, p4 };
						Plane planes[4] = {
								Plane(points[0], points[1], points[2]),
								Plane(points[0], points[1], points[3]),
								Plane(points[0], points[2], points[3]),
								Plane(points[1], points[2], points[3])
						};
						Point penetrated = Point(p.x(), p.y(), p.z());
						
						if (true/*it->velocity().isZero()*/)
						{
							Vector4f distances(
								CGAL::squared_distance(penetrated, planes[0]),
								CGAL::squared_distance(penetrated, planes[1]),
								CGAL::squared_distance(penetrated, planes[2]),
								CGAL::squared_distance(penetrated, planes[3])
								);
							float minDistance = distances.minCoeff(&minIndex);
							
						}
						else
						{
							Vector velocity(it->velocity().x(), it->velocity().y(), it->velocity().z());
							Ray raysAlongVelocity(penetrated, -velocity);
							Vector4f distances;
							for (unsigned i = 0; i < 4; ++i)
							{
								auto intersection = CGAL::intersection(raysAlongVelocity, planes[i]);
								if (intersection)
								{
									const Point* pSurface = boost::get<Point>(&*intersection);
									distances(i) = (*pSurface - penetrated).squared_length();
								}
								else
									distances(i) = INFINITY;
							}
							float minDistance = distances.minCoeff(&minIndex);
						}
						auto perpendicularLine = planes[minIndex].perpendicular_line(penetrated);
						auto intersection = CGAL::intersection(perpendicularLine, planes[minIndex]);
						const Point* p2 = boost::get<Point>(&*intersection);
						penaltyForce = KPenalty * (*p2 - penetrated);// +Vector(Gravity.x(), Gravity.y(), Gravity.z());
						
						//pCloth->set_delta_v_constrained(element, penaltyForce.x(), penaltyForce.y(), penaltyForce.z());
						//pCloth->set_translation_degree(element, 0);
						pCloth->add_force(element, penaltyForce.x(), penaltyForce.y(), penaltyForce.z());
						pCloth->set_velocity(element, 0.0f, 0.0f, 0.0f);
					}
				}
			}
		}
	}

#ifdef CLOTHSIMULATION_VERBOSE
	deltaTime = glfwGetTime() - currTime;
	std::cout << "Collision Response: " << deltaTime << " seconds" << std::endl;
	currTime = glfwGetTime();
#endif
}

void Simulator::Simulator::init_hashtable()
{
	unsigned int nVertices = 0;

	for (cloth_handle& pCloth : _cloths)
		nVertices += pCloth->size_vertices();
	for (rigid_handle& pRigid : _rigids)
		nVertices += pRigid->size_vertices();

	_hash_table = std::make_unique<SpatialHashing>(nVertices);
}
