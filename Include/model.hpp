#ifndef CLOTHSIMULATION_CLOTHSIMULATOR_MODEL
#define CLOTHSIMULATION_CLOTHSIMULATOR_MODEL

#include "base.hpp"
#include "SparseMatrix3Xf.hpp"

#include <array>
#include <vector>

#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

//const float	KsStruct = 0.75f, KdStruct = -0.25f;
//const float	KsShear = 0.75f, KdShear = -0.25f;
//const float	KsBend = 0.95f, KdBend = -0.25f;
const float	KsStruct = 5.5e2f, KdStruct = 1.5e0f;
const float	KsShear = 5.5e2f, KdShear = 1.5e0f;
const float	KsBend = 1.5e2f, KdBend = 1.5e0f;
const float Mass = 1.f;

namespace Simulator {
class base_vertex;
class base_face;
class deformable_vertex;
class deformable_face;
class spring;

template <typename VertexType, typename FaceType>
class BaseModel {
public:
	typedef VertexType vertex_type;
	typedef FaceType face_type;
	typedef typename std::vector<VertexType>::iterator vertex_iterator;
	typedef typename std::vector<FaceType>::iterator face_iterator;
	
	BaseModel();
	template <typename VertexIter, typename FaceIter>
	BaseModel(VertexIter vertexBegin, VertexIter vertexEnd, FaceIter faceBegin, FaceIter faceEnd);
	BaseModel(std::vector<VertexType>&& vertices, std::vector<FaceType>&& faces);

	void set_color(float r, float g, float b);
	const Eigen::Vector3f& color();
	float avg_edge_length();
	inline unsigned int size_vertices();
	inline unsigned int size_edges();
	void translate(float tx, float ty, float tz);
	void rotate(float angle, float axisx, float axisy, float axisz);
	void scale(float scalex, float scaley, float scalez);
	inline vertex_iterator vertices_begin();
	inline vertex_iterator vertices_end();
	inline face_iterator face_begin();
	inline face_iterator face_end();
	void init_for_drawing();
	void draw();
protected:
	std::vector<VertexType> _vertices;
	std::vector<FaceType> _faces;
	Eigen::Vector3f _color;
private:
	unsigned int VAO = 0;
	unsigned int VBO = 0;
	unsigned int EBO = 0;
};

class DeformableModel: public BaseModel<deformable_vertex, deformable_face> {
public:
	explicit DeformableModel(unsigned nVertices, float pMass = Mass, float ksStruct = KsStruct, float kdStruct = KdStruct, 
							    float ksShear = KsShear, float kdShear = KdShear, float ksBend = KsBend, float kdBend = KdBend) :
		_dfdx_precomputed(nVertices), _dfdv_precomputed(nVertices), _p_mass(pMass),_ks_struct(ksStruct), _kd_struct(kdStruct), _ks_shear(ksShear), _kd_shear(kdShear),
		_ks_bend(ksBend), _kd_bend(kdBend) {}
	//void set(std::vector<deformable_vertex>& vertices, std::vector<deformable_face>& faces, std::vector<spring>& springs);
	//void set(std::vector<deformable_vertex>&& vertices, std::vector<deformable_face>&& faces, std::vector<spring>&& springs);
	void pre_step_physics();
	void step_physics(float step);
	inline void set_p(unsigned int index, Eigen::Vector3f& p) { _vertices[index].set_p(p); }
	inline void set_q(unsigned int index, Eigen::Vector3f& q) { _vertices[index].set_q(q); }
	inline void set_translation_degree(unsigned int index, int degree) { _vertices[index].set_translation_degree(degree); }
	inline void set_delta_v_constrained(unsigned int index, float zX, float zY, float zZ) { _vertices[index].set_delta_v_constrained(zX, zY, zZ); }
	inline void set_delta_v_constrained(unsigned int index, const Eigen::Vector3f& z) {_vertices[index].set_delta_v_constrained(z); }
	inline void reset_delta_v_forced(unsigned int index) { _vertices[index].reset_delta_v_forced(); }
	inline void add_force(unsigned int index, float zX, float zY, float zZ) { _vertices[index]._force += Eigen::Vector3f(zX, zY, zZ);  }
	inline void set_velocity(unsigned int index, float zX, float zY, float zZ) { _vertices[index]._velocity = Eigen::Vector3f(zX, zY, zZ); }
protected:
	std::vector<spring> _springs;
	const float _p_mass;
	const float _ks_struct;
	const float _kd_struct;
	const float _ks_shear;
	const float _kd_shear;
	const float _ks_bend;
	const float _kd_bend;
	SparseMatrix3Xf _dfdx_precomputed;
	SparseMatrix3Xf _dfdv_precomputed;
private:
	void update_force();
	void update_position(float step);
	void apply_inverse_dynamic();
	Eigen::VectorXf force_vector() const;
	Eigen::VectorXf velocity_vector() const;
	Eigen::VectorXf delta_v_constrained_vector() const;
	SparseMatrix3Xf mass_inverse_matrix() const;
	//SparseMatrix3Xf mass_matrix() const;
	SparseMatrix3Xf jacobian_dfdx() const;
	SparseMatrix3Xf jacobian_dfdv() const;
};

class RigidModel : public BaseModel<base_vertex, base_face> {
public:
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef CGAL::Delaunay_triangulation_3<K>      Triangulation;
	typedef K::Aff_transformation_3  Transformation;
	typedef Triangulation::Point          Point;
	typedef Triangulation::Vertex_handle  Vertex_handle;
	typedef K::Vector_3 Vector;
	typedef std::array<unsigned int, 4> Tetrahedron;
	typedef std::vector<Tetrahedron>::iterator tetrahedron_iterator;
	typedef std::vector<Point>::iterator point_iterator;

	RigidModel() = default;
	RigidModel(const char* objFile);
	template <typename VertexIter, typename FaceIter>
	RigidModel(VertexIter vertexBegin, VertexIter vertexEnd, FaceIter faceBegin, FaceIter faceEnd);
	RigidModel(std::vector<base_vertex>&& vertices, std::vector<base_face>&& faces);

	void translate(float tx, float ty, float tz);
	void rotate(float angle, float axisx, float axisy, float axisz);
	void scale(float scalex, float scaley, float scalez);
	inline bool is_fixed() { return _fixed;  }
	inline tetrahedron_iterator tetrahedrons_begin();
	inline tetrahedron_iterator tetrahedrons_end();
	inline point_iterator points_begin();
	inline point_iterator points_end();
protected:
	std::vector<Point> _points;
	std::vector<Tetrahedron> _tetrahedrons;
	bool _fixed = true;
	
	void init_tetrahedra();
};

class Quad: public RigidModel
{
public:
	Quad();
};

class Sphere : public RigidModel
{
public:
	Sphere();
};

class RectangleDeformableModel : public DeformableModel {
public:
	explicit RectangleDeformableModel(unsigned int width, unsigned int height, float pMass = Mass,
		float ksStruct = KsStruct, float kdStruct = KdStruct,
		float ksShear = KsShear, float kdShear = KdShear,
		float ksBend = KsBend, float kdBend = KdBend);
private:
	unsigned int _width;
	unsigned int _height;
	
	inline unsigned int index(unsigned int i, unsigned int j) {
		return i * _width + j;
	}
	void add_spring(unsigned v1, unsigned v2, float ks, float kd, float l0);
};
}

#include "model_impl.hpp"

#endif // !CLOTHSIMULATION_CLOTHSIMULATOR_MODEL
