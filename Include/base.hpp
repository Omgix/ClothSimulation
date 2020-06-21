#ifndef CLOTHSIMULATION_CLOTHSIMULATOR_BASE
#define CLOTHSIMULATION_CLOTHSIMULATOR_BASE

#include <Eigen/Dense>

#include <memory>

namespace Simulator {
class base_vertex {
	template<typename, typename> friend class BaseModel;
	friend class RigidModel;
protected:
	Eigen::Vector3f _coord;
	Eigen::Vector3f _normal;
	Eigen::Vector2f _tex_coord;
public:
	base_vertex(const Eigen::Vector3f& coord, const Eigen::Vector3f& normal, 
				const Eigen::Vector2f& tex_coord) :
		_coord(coord), _normal(normal), _tex_coord(tex_coord) {}
	base_vertex(float coordX, float coordY, float coordZ, float normalX = 0.0f, float normalY = 0.0f, float normalZ = 0.0f, 
				float texCoordU = 0.0f, float texCoordV = 0.0f) :
		_coord(Eigen::Vector3f(coordX, coordY, coordZ)), _normal(Eigen::Vector3f(normalX, normalY, normalZ)), 
		_tex_coord(Eigen::Vector2f(texCoordU, texCoordV)) {}
	void update_coord(Eigen::Vector3f& deltaV, float step) { _coord += deltaV * step;  }
	const Eigen::Vector3f& coord() const { return _coord; }
	const Eigen::Vector3f& normal() const { return _normal; }
	const Eigen::Vector2f& tex_coord() const { return _tex_coord; }
};

class base_face {
protected:
	unsigned int _indices [3];
	Eigen::Vector3f _normal;
public:
	base_face(unsigned int index1, unsigned int index2, unsigned int index3, const Eigen::Vector3f& normal) :
		_indices{ index1, index2, index3 }, _normal(normal) {}
	base_face(unsigned int index1, unsigned int index2, unsigned int index3,
				float normalX = 0, float normalY = 0, float normalZ = 0) :
		_indices{ index1, index2, index3 }, _normal(Eigen::Vector3f(normalX, normalY, normalZ)) {}
	const unsigned int* indices() const { return _indices; }
	const Eigen::Vector3f& normal() const { return _normal; }
};

class deformable_vertex : public base_vertex {
	friend class DeformableModel;
public:
	deformable_vertex(float mass, float coordX, float coordY, float coordZ, float normalX, float normalY, float normalZ,
						float texCoordU = 0.0f, float texCoordV = 0.0f) :
		base_vertex(coordX, coordY, coordZ, normalX, normalY, normalZ, texCoordU, texCoordV), _mass(mass) {}
	const Eigen::Vector3f& velocity() const { return _velocity; }
	const Eigen::Vector3f& force() const { return _force; }
	float mass() const { return _mass; }
	const Eigen::Vector3f& delta_v_constrained() const { return _delta_v_constrained; }
	Eigen::Matrix3f mass_inverse_matrix() const {
		if (_degree == 0)
			return Eigen::Matrix3f::Zero();
		else if (_degree == 1)
			return (Eigen::Matrix3f::Identity() - *_p * _p->transpose() - *_q * _q->transpose()) / _mass;
		else if (_degree == 2)
			return (Eigen::Matrix3f::Identity() - *_p * _p->transpose()) / _mass;
		else
			return Eigen::Matrix3f::Identity() / _mass;
	}

	/*Eigen::Matrix3f mass_matrix() const {
		return Eigen::Matrix3f::Identity() * _mass;
	}*/

	inline void set_p(Eigen::Vector3f& p) {
		if (_p != nullptr)
			*_p = p;
		else
			_p = std::make_unique<Eigen::Vector3f>(p);
	}

	inline void set_q(Eigen::Vector3f& q) {
		if (_q != nullptr)
			*_q = q;
		else
			_q = std::make_unique<Eigen::Vector3f>(q);
	}

	inline void set_translation_degree(int degree) {
		_degree = degree;
		if (degree >= 2 || degree == 0)
			_p.reset(nullptr);
		else if (degree >= 1 || degree == 0)
			_q.reset(nullptr);
	}

	inline void set_delta_v_constrained(float zX, float zY, float zZ) {
		_delta_v_constrained = Eigen::Vector3f(zX, zY, zZ);
	}

	inline void set_delta_v_constrained(const Eigen::Vector3f& z) {
		_delta_v_constrained = z;
	}

	inline void reset_delta_v_forced() {
		_delta_v_constrained = Eigen::Vector3f::Zero();
	}

	inline void reset_constraints() {
		set_translation_degree(3);
		reset_delta_v_forced();
	}
protected:
	Eigen::Vector3f _velocity = Eigen::Vector3f::Zero();
	Eigen::Vector3f _force = Eigen::Vector3f::Zero();
	float _mass;
	Eigen::Vector3f _delta_v_constrained = Eigen::Vector3f::Zero();
	int _degree = 3;
	std::unique_ptr<Eigen::Vector3f> _p;
	std::unique_ptr<Eigen::Vector3f> _q;
};

class deformable_face : public base_face {
public:
	deformable_face(unsigned int index1, unsigned int index2, unsigned int index3, const Eigen::Vector3f& normal,
					float delta_u1 = 0, float delta_u2 = 0, float delta_v1 = 0, float delta_v2 = 0) :
		base_face(index1, index2, index3, normal), _delta_u1(delta_u1), _delta_u2(delta_u2),
		_delta_v1(delta_v1), _delta_v2(delta_v2) {}
	deformable_face(unsigned int index1, unsigned int index2, unsigned int index3, 
			        float normalX, float normalY, float normalZ,
			        float delta_u1 = 0, float delta_u2 = 0, float delta_v1 = 0, float delta_v2 = 0) :
		base_face(index1, index2, index3, normalX, normalY, normalZ), _delta_u1(delta_u1), _delta_u2(delta_u2),
		_delta_v1(delta_v1), _delta_v2(delta_v2) {}
private:
	const float _delta_u1;
	const float _delta_u2;
	const float _delta_v1;
	const float _delta_v2;
};

class spring {
public:
	const unsigned int p1;
	const unsigned int p2;
	const float l0;
	spring(unsigned int P1, unsigned int P2, float Ks, float Kd, float L0) :
		p1(P1), p2(P2), _Ks(Ks), _Kd(Kd), l0(L0) {}

	template <class InputIter>
	Eigen::Vector3f forces2p1(InputIter particles) const {
		deformable_vertex& vertex1 = *(particles + p1);
		deformable_vertex& vertex2 = *(particles + p2);
		Eigen::Vector3f forceDirection = (vertex2.coord() - vertex1.coord());
		forceDirection.normalize();
			
		float distance = (vertex1.coord() - vertex2.coord()).norm();
		distance = std::max(0.9f * l0, distance);
		Eigen::Vector3f springForce = _Ks * (distance - l0) * forceDirection;
		//Eigen::Vector3f dampingForce = _Kd * (vertex2.velocity() - vertex1.velocity()).dot(forceDirection) * forceDirection;
		Eigen::Vector3f dampingForce = _Kd * (vertex2.velocity() - vertex1.velocity());

		return springForce + dampingForce;
	}

	template <class InputIter>
	Eigen::Matrix3f df1dx1(InputIter particles) const {
		const deformable_vertex& vertex1 = *(particles + p1);
		const deformable_vertex& vertex2 = *(particles + p2);

		float distance = (vertex1.coord() - vertex2.coord()).norm();
		if (distance >= 0.9 * l0)
		{
			Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
			Eigen::Vector3f xij = vertex2.coord() - vertex1.coord();
			Eigen::Vector3f xijUnit = xij.normalized();
			Eigen::Vector3f vij = vertex2.velocity() - vertex1.velocity();
			float l = xij.norm();
			Eigen::Matrix3f dxijUnit_dxi = -1 / l * (I - 1 / (l * l) * xij * xij.transpose());

			return _Ks * (-I - l0 * dxijUnit_dxi);// _Kd* (vij.dot(xijUnit) * I - xijUnit * vij.transpose())* dxijUnit_dxi;
		}
		else
			return Eigen::Matrix3f::Zero();
	}

	template <class InputIter>
	Eigen::Matrix3f df1dx1_nonlinear_part(InputIter particles) const {
		const deformable_vertex& vertex1 = *(particles + p1);
		const deformable_vertex& vertex2 = *(particles + p2);

		float distance = (vertex1.coord() - vertex2.coord()).norm();
		if (distance >= 0.9 * l0)
		{
			Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
			Eigen::Vector3f xij = vertex2.coord() - vertex1.coord();
			Eigen::Vector3f xijUnit = xij.normalized();
			Eigen::Vector3f vij = vertex2.velocity() - vertex1.velocity();
			float l = xij.norm();
			Eigen::Matrix3f dxijUnit_dxi = -1 / l * (I - 1 / (l * l) * xij * xij.transpose());

			return _Ks * (l0 * dxijUnit_dxi);// _Kd* (vij.dot(xijUnit) * I - xijUnit * vij.transpose())* dxijUnit_dxi;
		}
		else
			return Eigen::Matrix3f::Zero();
	}

	template <class InputIter>
	Eigen::Matrix3f df1dv1(InputIter particles) const {
		const deformable_vertex& vertex1 = *(particles + p1);
		const deformable_vertex& vertex2 = *(particles + p2);
		
		Eigen::Vector3f xijUnit = (vertex2.coord() - vertex1.coord()).normalized();

		return -_Kd * Eigen::Matrix3f::Identity();// *xijUnit* xijUnit.transpose();
	}
	//template <class InputIter>
	//Eigen::Matrix3f dfdv1(InputIter particles);
private:
	float _Ks;
	float _Kd;
};
}

#endif // !CLOTHSIMULATION_CLOTHSIMULATOR_BASE
