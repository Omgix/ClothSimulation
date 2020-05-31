#ifndef CLOTHSIMULATION_CLOTHSIMULATOR
#define CLOTHSIMULATION_CLOTHSIMULATOR

#include "model.hpp"
#include "SpatialHashing.hpp"

#include <vector>
#include <memory>

#include "camera.hpp"
#include "resource_manager.hpp"

namespace Simulator {
class Simulator {
public:
	typedef std::unique_ptr<DeformableModel> cloth_handle;
	typedef std::unique_ptr<RigidModel> rigid_handle;
	
	template<typename ClothType, typename... Val>
	cloth_handle& add_cloth(Val&&... val);
	template<typename RigidType, typename... Val>
	rigid_handle& add_rigid(Val&&... val);
	cloth_handle& cloth(unsigned i);
	rigid_handle& rigid(unsigned i);
	Shader add_shader(const char* vShaderFile, const char* fShaderFile, const char* gShaderFile = nullptr, const char* name = "main");
	Shader shader(const char* name);
	void process_input(float deltaTime);
	void init_main_loop();
	void set_camera(float posX, float posY, float posZ, float yaw = YAW, float pitch = PITCH, float upX = 0, float upY = 1, float upZ = 0);
	bool window_should_close();
	void use_and_set_shader(const char *name);
	void main_loop_end();
	//void main_loop();
	void pre_step_physics();
	void step_physics(float step);
	void draw_all_model(const char* shader);
	void clear_hashtable();
private:
	typedef std::unique_ptr<SpatialHashing> hashtable_pointer;
	
	std::vector<cloth_handle> _cloths;
	std::vector<rigid_handle> _rigids;
	hashtable_pointer _hash_table = nullptr;
	ResourceManager _shaders;
	Camera _camera = Camera(0, 0, 0);
	GLFWwindow* _window = nullptr;
	int _scr_width = 800;
	int _scr_height = 600;

	void collision_process();
	void init_hashtable();
};
}

#include "Simulator_impl.hpp"

#endif // !CLOTHSIMULATION_CLOTHSIMULATOR
