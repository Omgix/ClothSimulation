#include "model.hpp"
#include "SpatialHashing.hpp"

#include <algorithm>
#include <Eigen/Dense> 
#include <Eigen/Geometry> 
#include "glad/glad.h"
#include "GLFW/glfw3.h"

//Eigen::Vector3f Gravity (0.0f, -0.00981f, 0.0f);
Eigen::Vector3f Gravity(0.0f, -9.81e-1f, 0.0f);
const float Length = 0.1f;
const float DiagonalCoeff = sqrt(2);

static float verticesBufferData[8192];
static unsigned int indicesBufferData[8192];

using namespace Eigen;

namespace Simulator {

template <typename VertexType, typename FaceType>
BaseModel<VertexType, FaceType>::BaseModel() {}

template <typename VertexType, typename FaceType>
template <typename VertexIter, typename FaceIter>
BaseModel<VertexType, FaceType>::BaseModel(VertexIter vertexBegin, VertexIter vertexEnd, FaceIter faceBegin, FaceIter faceEnd):
	_vertices(vertexBegin, vertexEnd), _faces(faceBegin, faceEnd) {}

template <typename VertexType, typename FaceType>
BaseModel<VertexType, FaceType>::BaseModel(std::vector<VertexType>&& vertices, std::vector<FaceType>&& faces):
	_vertices(vertices), _faces(faces) {}

template <typename VertexType, typename FaceType>
void
BaseModel<VertexType, FaceType>::set_color(float r, float g, float b)
{
	_color = Eigen::Vector3f(r, g, b);
}

template <typename VertexType, typename FaceType>
const Eigen::Vector3f&
BaseModel<VertexType, FaceType>::color()
{
	return _color;
}
	
template <typename VertexType, typename FaceType>
float
BaseModel<VertexType, FaceType>::avg_edge_length()
{
	float avgLength = 0;

	for (face_type& face : _faces) {
		const unsigned int p1 = face.indices()[0];
		const unsigned int p2 = face.indices()[1];
		const unsigned int p3 = face.indices()[2];
		Vector3f e1 = _vertices[p1].coord() - _vertices[p2].coord();
		Vector3f e2 = _vertices[p1].coord() - _vertices[p3].coord();
		Vector3f e3 = _vertices[p3].coord() - _vertices[p2].coord();
		avgLength += e1.norm();
		avgLength += e2.norm();
		avgLength += e3.norm();
	}

	avgLength /= _faces.size() * 3;
	return avgLength;
}

template <typename VertexType, typename FaceType>
inline unsigned int
BaseModel<VertexType, FaceType>::size_vertices()
{
	return _vertices.size();
}

template <typename VertexType, typename FaceType>
inline unsigned int
BaseModel<VertexType, FaceType>::size_edges()
{
	return _faces.size() * 3;
}

template <typename VertexType, typename FaceType>
void
BaseModel<VertexType, FaceType>::translate(float tx, float ty, float tz)
{
	Transform<float, 3, Affine> t(Translation<float, 3>(tx, ty, tz));
	Eigen::Matrix3f normalMatrix = t.linear().inverse().transpose();

	for (vertex_type& vertex : _vertices) {
		vertex._coord = t * vertex._coord;
		vertex._normal = normalMatrix * vertex._normal;
	}
}

template <typename VertexType, typename FaceType>
void
BaseModel<VertexType, FaceType>::rotate(float angle, float axisx, float axisy, float axisz)
{
	Transform<float, 3, Affine> t(AngleAxis<float>(angle, Vector3f(axisx, axisy, axisz).normalized()));
	Eigen::Matrix3f normalMatrix = t.linear().inverse().transpose();

	for (vertex_type& vertex : _vertices) {
		vertex._coord = t * vertex._coord;
		vertex._normal = normalMatrix * vertex._normal;
	}
}

template <typename VertexType, typename FaceType>
void
BaseModel<VertexType, FaceType>::scale(float scalex, float scaley, float scalez)
{
	Transform<float, 3, Affine> t(Scaling(scalex, scaley, scalez));
	Eigen::Matrix3f normalMatrix = t.linear().inverse().transpose();

	for (vertex_type& vertex : _vertices) {
		vertex._coord = t * vertex._coord;
		vertex._normal = normalMatrix * vertex._normal;
	}
}

template <typename VertexType, typename FaceType>
inline typename BaseModel<VertexType, FaceType>::vertex_iterator
BaseModel<VertexType, FaceType>::vertices_begin()
{
	return _vertices.begin();
}

template <typename VertexType, typename FaceType>
inline typename BaseModel<VertexType, FaceType>::vertex_iterator
BaseModel<VertexType, FaceType>::vertices_end()
{
	return _vertices.end();
}

template <typename VertexType, typename FaceType>
inline typename BaseModel<VertexType, FaceType>::face_iterator
BaseModel<VertexType, FaceType>::face_begin()
{
	return _faces.begin();
}

template <typename VertexType, typename FaceType>
inline typename BaseModel<VertexType, FaceType>::face_iterator
BaseModel<VertexType, FaceType>::face_end()
{
	return _faces.end();
}

template <typename VertexType, typename FaceType>
void
BaseModel<VertexType, FaceType>::draw()
{
	

	for (unsigned i = 0; i < _vertices.size(); ++i)
	{
		verticesBufferData[i * 8 + 0] = _vertices[i].coord().x();
		verticesBufferData[i * 8 + 1] = _vertices[i].coord().y();
		verticesBufferData[i * 8 + 2] = _vertices[i].coord().z();
		verticesBufferData[i * 8 + 3] = _vertices[i].normal().x();
		verticesBufferData[i * 8 + 4] = _vertices[i].normal().y();
		verticesBufferData[i * 8 + 5] = _vertices[i].normal().z();
		verticesBufferData[i * 8 + 6] = _vertices[i].tex_coord().x();
		verticesBufferData[i * 8 + 7] = _vertices[i].tex_coord().y();
	}

	for (unsigned i = 0; i < _faces.size(); ++i)
	{
		indicesBufferData[i * 3 + 0] = _faces[i].indices()[0];
		indicesBufferData[i * 3 + 1] = _faces[i].indices()[1];
		indicesBufferData[i * 3 + 2] = _faces[i].indices()[2];
	}

	glBindVertexArray(VAO);
	
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * _vertices.size() * 8, verticesBufferData, GL_DYNAMIC_DRAW);
	/*glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);*/

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * _faces.size() * 3, indicesBufferData, GL_DYNAMIC_DRAW);
	
	glDrawElements(GL_TRIANGLES, _faces.size() * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

template <typename VertexType, typename FaceType>
void
BaseModel<VertexType, FaceType>::init_for_drawing()
{
	if (VAO != 0)
		return;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	
	glBindVertexArray(VAO);
	
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * _vertices.size() * 8, verticesBufferData, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * _faces.size(), indicesBufferData, GL_DYNAMIC_DRAW);

	glBindVertexArray(0);
}

void
DeformableModel::pre_step_physics()
{
	for (vertex_type& v : _vertices)
	{
		v.set_translation_degree(3);
		v.reset_delta_v_forced();
		v._force.setZero();
	}
}

void
DeformableModel::step_physics(float step)
{
	update_force();
	update_position(step);
}

void
DeformableModel::update_force()
{
	// Add gravity
	for (deformable_vertex& vertex : _vertices)
		vertex._force += Gravity * _p_mass;

	// Add spring forces
	for (spring& s : _springs) {
		Vector3f springForce2p1 = s.forces2p1(_vertices.begin());
		_vertices[s.p1]._force += springForce2p1;
		_vertices[s.p2]._force += -springForce2p1;
	}
}

void
DeformableModel::update_position(float step)
{
#ifdef CLOTHSIMULATION_VERBOSE
	std::cout << std::setprecision(6);
	double currTime = glfwGetTime();
#endif	

	const SparseMatrix3Xf I = SparseMatrix3Xf::Identity(_vertices.size());

#ifdef CLOTHSIMULATION_VERBOSE
	double deltaTime = glfwGetTime() - currTime;
	std::cout << "Prepare for Linear System 1.1.1: " << deltaTime << " seconds" << std::endl;
	currTime = glfwGetTime();
#endif
	
	const SparseMatrix3Xf dfdx = jacobian_dfdx();
	const SparseMatrix3Xf dfdv = jacobian_dfdv();

#ifdef CLOTHSIMULATION_VERBOSE
	deltaTime = glfwGetTime() - currTime;
	std::cout << "Prepare for Linear System 1.1.2: " << deltaTime << " seconds" << std::endl;
	currTime = glfwGetTime();
#endif
	
	SparseMatrix3Xf jacobians1 = dfdx * step * step + dfdv * step;
	for (unsigned i = 0; i < dfdx.size(); ++i)
		jacobians1.multi_sub3_rows_matrix3f(i, _vertices[i].mass_inverse_matrix());
	SparseMatrix3Xf A = I - jacobians1;

#ifdef CLOTHSIMULATION_VERBOSE
	deltaTime = glfwGetTime() - currTime;
	std::cout << "Prepare for Linear System 1.2: " << deltaTime << " seconds" << std::endl;
	currTime = glfwGetTime();
#endif
	
	VectorXf v = velocity_vector();
	VectorXf f = force_vector();
	VectorXf z = delta_v_constrained_vector();
	VectorXf b = f + dfdx.multi(v) * step;
	for (unsigned i = 0; i < dfdx.size(); ++i)
		b.segment<3>(i * 3) = _vertices[i].mass_inverse_matrix() * b.segment<3>(i * 3);
	b = b * step + z;

#ifdef CLOTHSIMULATION_VERBOSE
	deltaTime = glfwGetTime() - currTime;
	std::cout << "Prepare for Linear System 2: " << deltaTime << " seconds" << std::endl;
	currTime = glfwGetTime();
#endif
	
	Eigen::BiCGSTAB<SparseMatrix3Xf, Eigen::IdentityPreconditioner> bicg;
	bicg.compute(A);
	VectorXf deltaV = bicg.solve(b);

#ifdef CLOTHSIMULATION_VERBOSE
	deltaTime = glfwGetTime() - currTime;
	std::cout << "Solve System: " << deltaTime << " seconds" << std::endl;
#endif

	for (unsigned int i = 0; i < _vertices.size(); ++i)
	{
		_vertices[i]._velocity += deltaV.segment<3>(i * 3);
		_vertices[i]._coord += step * _vertices[i]._velocity;
	}
}

Eigen::VectorXf
DeformableModel::force_vector() const
{
	VectorXf result (_vertices.size() * 3);

	for (unsigned int i = 0; i < _vertices.size(); i++)
		result.segment<3>(i * 3) = _vertices[i].force();

	return result;
}

Eigen::VectorXf
DeformableModel::velocity_vector() const
{
	VectorXf result(_vertices.size() * 3);

	for (unsigned int i = 0; i < _vertices.size(); i++)
		result.segment<3>(i * 3) = _vertices[i].velocity();

	return result;
}

Eigen::VectorXf
DeformableModel::delta_v_constrained_vector() const
{
	VectorXf result(_vertices.size() * 3);

	for (unsigned int i = 0; i < _vertices.size(); i++)
		result.segment<3>(i * 3) = _vertices[i].delta_v_constrained();

	return result;
}

SparseMatrix3Xf
DeformableModel::mass_inverse_matrix() const
{
	SparseMatrix3Xf result(_vertices.size());

	for (unsigned int i = 0; i < _vertices.size(); i++)
		result.sub_matrix(i, i) = _vertices[i].mass_inverse_matrix();

	return result;
}
/*
SparseMatrix3Xf
DeformableModel::mass_matrix() const
{
	SparseMatrix3Xf result(_vertices.size());

	for (unsigned int i = 0; i < _vertices.size(); i++)
		result.sub_matrix(i, i) = _vertices[i].mass_matrix();

	return result;
}*/

SparseMatrix3Xf
DeformableModel::jacobian_dfdx() const
{
	SparseMatrix3Xf result(_vertices.size());

	for (const spring& s : _springs) {
		unsigned int i = s.p1;
		unsigned int j = s.p2;
		Matrix3f value = s.df1dx1(_vertices.begin());
		result.sub_matrix(i, i) += value;
		result.sub_matrix(j, j) += value;
		result.sub_matrix(i, j) += -value;
		result.sub_matrix(j, i) += -value;
	}

	return result;
}

SparseMatrix3Xf
DeformableModel::jacobian_dfdv() const
{
	SparseMatrix3Xf result(_vertices.size());

	for (const spring& s : _springs) {
		unsigned int i = s.p1;
		unsigned int j = s.p2;
		Matrix3f value = s.df1dv1(_vertices.begin());
		result.sub_matrix(i, i) += value;
		result.sub_matrix(j, j) += value;
		result.sub_matrix(i, j) += -value;
		result.sub_matrix(j, i) += -value;
	}

	return result;
}

template <typename VertexIter, typename FaceIter>
RigidModel::RigidModel(VertexIter vertexBegin, VertexIter vertexEnd, FaceIter faceBegin, FaceIter faceEnd):
	BaseModel<vertex_type, face_type>(vertexBegin, vertexEnd, faceBegin, faceEnd) {}

RigidModel::RigidModel(std::vector<base_vertex>&& vertices, std::vector<base_face>&& faces) :
	BaseModel<vertex_type, face_type>(std::move(vertices), std::move(faces)) {}

void
RigidModel::translate(float tx, float ty, float tz)
{
	Transform<float, 3, Affine> t(Translation<float, 3>(tx, ty, tz));
	Eigen::Matrix3f normalMatrix = t.linear().inverse().transpose();
	Transformation transformation_TRANSLATION(0.0f, 0.0f, 0.0f, tx, 0.0f, 0.0f, 0.0f, ty, 0.0f, 0.0f, 0.0f, tz);

	for (vertex_type& vertex : _vertices) {
		vertex._coord = t * vertex._coord;
		vertex._normal = normalMatrix * vertex._normal;
	}

	for (Point& p : _points)
		p = transformation_TRANSLATION(p);
}

void
RigidModel::rotate(float angle, float axisx, float axisy, float axisz)
{
	Transform<float, 3, Affine> t(AngleAxis<float>(angle, Vector3f(axisx, axisy, axisz).normalized()));
	Eigen::Matrix3f normalMatrix = t.linear().inverse().transpose();

	for (vertex_type& vertex : _vertices) {
		vertex._coord = t * vertex._coord;
		vertex._normal = normalMatrix * vertex._normal;
	}
}

void
RigidModel::scale(float scalex, float scaley, float scalez)
{
	Transform<float, 3, Affine> t(Scaling(scalex, scaley, scalez));
	Eigen::Matrix3f normalMatrix = t.linear().inverse().transpose();
	Transformation transformation_SCALING(scalex, 0.0f, 0.0f, 0.0f, 0.0f, scaley, 0.0f, 0.0f, 0.0f, 0.0f, scalez, 0.0f);

	for (vertex_type& vertex : _vertices) {
		vertex._coord = t * vertex._coord;
		vertex._normal = normalMatrix * vertex._normal;
	}

	for (Point & p : _points)
		p = transformation_SCALING(p);
}


inline RigidModel::tetrahedron_iterator
RigidModel::tetrahedrons_begin()
{
	return _tetrahedrons.begin();
}

inline RigidModel::tetrahedron_iterator
RigidModel::tetrahedrons_end()
{
	return _tetrahedrons.end();
}

inline RigidModel::point_iterator
RigidModel::points_begin()
{
	return _points.begin();
}

inline RigidModel::point_iterator
RigidModel::points_end()
{
	return _points.end();
}

void
RigidModel::init_tetrahedra()
{
	SpatialHashing hashtable(_vertices.size());
	std::vector<Vector3f> triangleNorms;
	float cellSize = avg_edge_length();
	Vector3i maxAllCoord (-INFINITY, -INFINITY, -INFINITY);
	Vector3i minAllCoord (INFINITY, INFINITY, INFINITY);

	for (unsigned int i = 0; i < _faces.size(); ++i) {
		Vector3f p1 = _vertices[_faces[i].indices()[0]].coord();
		Vector3f p2 = _vertices[_faces[i].indices()[1]].coord();
		Vector3f p3 = _vertices[_faces[i].indices()[2]].coord();
		Matrix3f points;
		points << p1, p2, p3;

		triangleNorms.push_back((p1 - p2).cross(p2 - p3));
		Vector3f minCoord = points.rowwise().minCoeff();
		Vector3f maxCoord = points.rowwise().maxCoeff();

		minCoord = (minCoord / cellSize).array().floor();
		maxCoord = (maxCoord / cellSize).array().floor();
		hashtable.insert_range(minCoord.x(), minCoord.x(), minCoord.y(), minCoord.y(), minCoord.z(), minCoord.z(), 
							HashTableEntry::RIGID, 0, i);

		minAllCoord = minAllCoord.cwiseMin(minCoord.cast<int>());
		maxAllCoord = maxAllCoord.cwiseMin(maxCoord.cast<int>());
	}

	for (int ix = minAllCoord.x(); ix <= maxAllCoord.x(); ix++)
		for (int iy = minAllCoord.y(); iy <= maxAllCoord.y(); iy++) {
			std::vector<HashTableEntry> possibleCollision;
			for (int iz = minAllCoord.z(); iz <= maxAllCoord.z(); iz++) {
				Vector3i cell(ix, iy, iz);
				hashtable.find_in_cell(cell, std::back_inserter(possibleCollision));
			}

			std::vector<int> prefixSum(maxAllCoord.z() - minAllCoord.z() + 1);
			Vector3f P(ix, iy, minAllCoord.z());
			for (HashTableEntry& possibleCollidingTriangle : possibleCollision) {
				unsigned int triangleIndex = possibleCollidingTriangle.element;
				const unsigned int* indice = _faces[triangleIndex].indices();
				const Vector3f& A = _vertices[indice[0]].coord();
				const Vector3f& B = _vertices[indice[1]].coord();
				const Vector3f& C = _vertices[indice[2]].coord();
				const Vector3f normal = (B - A).cross(C - A);
				const float d = -normal.z();
				if (d == 0)
					continue;
				const Vector3f PA = (P - A);
				const Vector3f e(PA.y(), PA.x(), 0.0f);
				const float v = (C - A).dot(e) / d;
				const float w = -(B - A).dot(e) / d;
				if (v < 0 || w < 0 || v + w > 1)
					continue;
				const Vector3f intersection = (1 - v - w) * A + v * B + w * C;
				const Vector3i intersectionCell = (intersection / cellSize).array().floor().cast<int>();
				if (intersectionCell.x() == ix && intersectionCell.y() == iy)
					prefixSum[intersectionCell.z() - minAllCoord.z()]++;
			}

			for (unsigned int i = 1; i < maxAllCoord.z() - minAllCoord.z() + 1; i++)
				prefixSum[i] += prefixSum[i - 1];

			for (int iz = minAllCoord.z(); iz <= maxAllCoord.z(); iz++)
				if (prefixSum[iz - minAllCoord.z()] % 2 == 1 && prefixSum[iz - minAllCoord.z()] - prefixSum[iz - minAllCoord.z() - 1] != 3)
					_points.emplace_back(ix * cellSize, iy * cellSize, iz * cellSize);
		}

	std::map<Point, unsigned int> VI;

	for (unsigned int i = 0; i < _points.size(); i++) {
		VI[_points[i]] = i;
	}

	Triangulation T(_points.begin(), _points.end());
	for (auto it = T.all_cells_begin(); it != T.all_cells_end(); it++) {
		Point p0 = it->vertex(0)->point();
		Point p1 = it->vertex(1)->point();
		Point p2 = it->vertex(2)->point();
		Point p3 = it->vertex(3)->point();
		_tetrahedrons.push_back({ VI[p0], VI[p1], VI[p2], VI[p3] });
	}
}

Quad::Quad()
{
	_vertices.emplace_back(0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f);
	_vertices.emplace_back(sqrt(6) / 3.0f, -1.0f / 3.0f, -sqrt(2) / 3.0f, sqrt(6) / 3.0f, -1.0f / 3.0f, -sqrt(2) / 3.0f);
	_vertices.emplace_back(-sqrt(6) / 3.0f, -1.0f / 3.0f, -sqrt(2) / 3.0f, -sqrt(6) / 3.0f, -1.0f / 3.0f, -sqrt(2) / 3.0f);
	_vertices.emplace_back(0.0f, -1.0f / 3.0f, sqrt(2) * 2 / 3.0f, 0.0f, -1.0f / 3.0f, sqrt(2) * 2 / 3.0f);
	_faces.emplace_back(0, 1, 2);
	_faces.emplace_back(0, 1, 3);
	_faces.emplace_back(0, 2, 3);
	_faces.emplace_back(1, 2, 3);
	_points.emplace_back(0.0f, 1.0f, 0.0f);
	_points.emplace_back(sqrt(6) / 3.0f, -1.0f / 3.0f, -sqrt(2) / 3.0f);
	_points.emplace_back(-sqrt(6) / 3.0f, -1.0f / 3.0f, -sqrt(2) / 3.0f);
	_points.emplace_back(0.0f, -1.0f / 3.0f, sqrt(2) * 2 / 3.0f);
	_tetrahedrons.push_back({ 0, 1, 2, 3 });
}

RectangleDeformalbleModel::RectangleDeformalbleModel(unsigned int width, unsigned int height,
																float pMass , float ksStruct , float kdStruct ,
																float ksShear , float kdShear , float ksBend , float kdBend) :
	DeformableModel(pMass, ksStruct, kdStruct, ksShear, kdShear, ksBend, kdBend), _width(width), _height(height)
{
	const float midWidth = _width * Length / 2;
	const float midHeight = _height * Length / 2;

	for (unsigned int i = 0; i < _height; i++)
		for (unsigned int j = 0; j < _width; j++) {
			// Add vertices
			_vertices.emplace_back(_p_mass, i * Length - midHeight, 0.0f, j * Length - midWidth, 0.0f, 1.0f, 0.0f);

			// Add faces
			if (i != _height - 1 && j != _width - 1)
				_faces.emplace_back(index(i, j), index(i, j + 1), index(i + 1, j), 0.0f, 1.0f, 0.0f);
			if (i != 0 && j != 0)
				_faces.emplace_back(index(i - 1, j), index(i, j), index(i, j - 1), 0.0f, 1.0f, 0.0f);

			// Add springs
			if (j != _width - 1)
				_springs.emplace_back(index(i, j), index(i, j + 1), _ks_struct, _kd_struct, Length);
			if (i != _height - 1)
				_springs.emplace_back(index(i, j), index(i + 1, j), _ks_struct, _kd_struct, Length);
			if (i != _height - 1 && j != _width - 1)
				_springs.emplace_back(index(i, j), index(i + 1, j + 1), _ks_shear, _kd_shear, DiagonalCoeff * Length);
			if (i != 0 && j != _width - 1)
				_springs.emplace_back(index(i, j), index(i - 1, j + 1), _ks_shear, _kd_shear, DiagonalCoeff * Length);
			if (j < _width - 2)
				_springs.emplace_back(index(i, j), index(i, j + 2), _ks_bend, _kd_bend, 2 * Length);
			if (i < _height - 2)
				_springs.emplace_back(index(i, j), index(i + 2, j), _ks_bend, _kd_bend, 2 * Length);
		}

	_vertices.shrink_to_fit();
	_faces.shrink_to_fit();
	_springs.shrink_to_fit();
}
};
