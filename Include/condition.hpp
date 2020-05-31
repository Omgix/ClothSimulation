#ifndef CLOTHSIMULATION_CONDITION
#define CLOTHSIMULATION_CONDITION

#include <Eigen/Dense>

#include <utility>

const float DIFF = 0.000125f;

using namespace Eigen;
typedef Matrix<float, 3, 2> Matrix3x2f;

Matrix3x2f 
wuvMatrix(const Vector3f& xi, const Vector3f& xj, const Vector3f& xk, const Matrix2f& uv) {
	Matrix3x2f xMatrix;
	xMatrix.col(0) = xj - xi;
	xMatrix.col(1) = xk - xi;
	return xMatrix * uv.inverse();
}

Vector2f 
stretchCondition(const Matrix3x2f& wuv, float a, float bu, float bv) {
	Vector3f wu = wuv.col(0);
	Vector3f wv = wuv.col(1);
	return a * Vector2f(wu.norm() - bu, wv.norm() - bv);
}

float 
shearCondition(const Matrix3x2f& wuv, float a) {
	Vector3f wu = wuv.col(0);
	Vector3f wv = wuv.col(1);
	return a * wu.dot(wv);
}

float 
bendCondition(const Vector3f& n1, const Vector3f& n2) {
	float sinTheta = (n1.cross(n2)).norm();
	float cosTheta = n1.dot(n2);
	return atan2f(sinTheta, cosTheta);
}

std::pair<Matrix3x2f, Vector3f> 
stretchAndShearConditionPatialDiff(const Vector3f& x1, const Vector3f& x2, const Vector3f& x3, const Matrix2f& uv, Vector3f& xi, float bu, float bv) {
	float area = uv.determinant();
	Matrix3x2f CstretchPatialDiff;
	Vector3f CshearPatialDiff;

	for (int i = 0; i < 3; i++) {
		xi(i) += DIFF;
		Matrix3x2f wuvForward = wuvMatrix(x1, x2, x3, uv);
		Vector2f CstretchForward = stretchCondition(wuvForward, area, bu, bv);
		float CshearForward = shearCondition(wuvForward, area);
		xi(i) -= 2 * DIFF;
		Matrix3x2f wuvBackward = wuvMatrix(x1, x2, x3, uv);
		Vector2f CstretchBackward = stretchCondition(wuvBackward, area, bu, bv);
		float CshearBackward = shearCondition(wuvBackward, area);
		xi(i) += DIFF;
		Vector2f CstretchDiff = CstretchForward - CstretchBackward;
		float CshearDiff = CshearForward - CshearBackward;
		CstretchPatialDiff.row(i) = CstretchDiff / (2 * DIFF);
		CshearPatialDiff(i) = CshearDiff / (2 * DIFF);
	}

	return std::make_pair(CstretchPatialDiff, CshearPatialDiff);
}

// Note: Two triagle face has common edge x2 - x3.
Vector3f 
bendConditionPatialDiff(const Vector3f& x1, const Vector3f& x2, const Vector3f& x3, const Vector3f& x4, Vector3f& xi) {
	Vector3f CbendPatialDiff;

	for (int i = 0; i < 3; i++) {
		xi(i) += DIFF;
		Vector3f n1 = (x2 - x1).cross(x3 - x1);
		Vector3f n2 = (x2 - x4).cross(x3 - x4);
		n1.normalize();
		n2.normalize();
		float CbendForward = bendCondition(n1, n2);
		xi(i) -= 2 * DIFF;
		n1 = (x2 - x1).cross(x3 - x1);
		n2 = (x2 - x4).cross(x3 - x4);
		n1.normalize();
		n2.normalize();
		float CbendBackward = bendCondition(n1, n2);
		xi(i) += DIFF;
		float CbendDiff = CbendForward - CbendBackward;
		CbendPatialDiff(i) = CbendDiff / (2 * DIFF);
	}

	return CbendPatialDiff;
}

/*std::pair<Matrix3x2f, Vector3f>
stretchAndShear2ndConditionPatialDiff(Vector3f& x1, Vector3f& x2, Vector3f& x3, Matrix2f& uv, Vector3f& xi, Vector3f& xj, float bu, float bv) {

}*/

#endif // !CLOTHSIMULATION_CONDITION
