/*#ifndef CLOTHSIMULATION_CLOTHSIMULATION
#define CLOTHSIMULATION_CLOTHSIMULATION

#include <glad/glad.h>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/ext/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale
#include <glm/ext/matrix_clip_space.hpp> // glm::perspective

#include <list>
#include <memory>
#include <vector>

class Simulator {
	struct Display {
		GLfloat *vertices;  // Array of fixed size
		GLuint *indices;	// Array of fixed size
		GLuint size_vertices;
		GLuint size_indices;
		~Display();
		GLfloat avg_edge_length();
	};

	struct Cloth {
		struct Spring {
			GLuint p1;  // Index of the particle
			GLuint p2;  // Index of the particle
			GLfloat l0; // Rest length
			GLfloat Ks;
			GLfloat Kd;
		};

		struct Particle {
			glm::vec3 position;
			glm::vec3 velocity;
			glm::vec3 acceleration;
		};

		struct Model {
			std::vector<Particle> particles;
			std::vector<Spring> springs;
			GLfloat mass;
		};

		Display display;
		Model model;
		Cloth(const char* filename);
	};

	struct RigidBody {
		struct Model {
			std::vector<glm::vec3> vertices;
			std::vector<glm::uvec4> tetrahedron_indices;
		};

		Display display;
		Model model;
		void init_tetrahedra();
	public:
		RigidBody(const char* filename);
	};

	struct HashTableEntry {
		enum Type {
			CLOTH,
			RIGIDBODY,
		};
		glm::ivec3 cell;
		Type type;
		GLuint object;
		GLuint element;
	};

	class SpatialHashing {
		typedef HashTableEntry value_type;

		GLuint _buckle_size;
		std::vector<std::list <value_type> >_buckles; // Fixed size array
		std::vector<GLuint> _size_current;
		inline GLuint hash(int ix, int iy, int iz) {
			return ((ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791)) % _buckle_size;
		}
	public:
		SpatialHashing(GLuint buckle_size);
		void preNextStep();
		void clear();
		void insert(const glm::ivec3 &cell, HashTableEntry::Type type, GLuint object, GLuint element);
		void insert(int i, int j, int k, HashTableEntry::Type type, GLuint object, GLuint element);
		void insert(int minx, int maxx, int miny, int maxy, int minz, int maxz, HashTableEntry::Type type, GLuint object, GLuint element);
		template< class InputIt >
		void insert(InputIt first, InputIt last);
		template < class OutputIt >
		OutputIt find_in_cell(const glm::ivec3 &cell, OutputIt &iterator);
		template < class OutputIt >
		OutputIt find_in_cell(int i, int j, int k, OutputIt& iterator);
	};

	std::vector<Cloth> _clothes;
	std::vector<RigidBody> _rigids;
	std::unique_ptr<SpatialHashing> _hash_table;
	GLfloat _step;
	GLuint _n_vertices;
	GLfloat _avg_edge_length;
	char _log[512];
	void init_hashtable();
public:
	explicit Simulator(GLfloat step = 0.01);
	void add_cloth(const char *filename, const glm::vec3 position, const glm::vec4 rotation);
	void add_rigid(const char *filename, const glm::vec3 position, const glm::vec4 rotation);
	void compute_force();
	void reslove_collision();
};

#endif // !CLOTHSIMULATION_CLOTHSIMULATION*/
