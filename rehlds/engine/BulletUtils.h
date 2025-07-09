#pragma once

#include <btBulletDynamicsCommon.h>

#define UNIT_SCALE (1.0f / 32.0f)

inline void EulerToMatrix(const btVector3& eulerDegrees, btMatrix3x3& outMatrix) {
	btVector3 angles = eulerDegrees * SIMD_RADS_PER_DEG;

	btScalar c1 = btCos(angles[0]);
	btScalar c2 = btCos(angles[1]);
	btScalar c3 = btCos(angles[2]);
	btScalar s1 = btSin(angles[0]);
	btScalar s2 = btSin(angles[1]);
	btScalar s3 = btSin(angles[2]);

	outMatrix.setValue(
		c1 * c2, -c3 * s2 - s1 * s3 * c2, s3 * s2 - s1 * c3 * c2,
		c1 * s2, c3 * c2 - s1 * s3 * s2, -s3 * c2 - s1 * c3 * s2,
		s1, c1 * s3, c1 * c3
	);
}

inline void MatrixToEuler(const btMatrix3x3& inMatrix, btVector3& outEulerDegrees) {
	outEulerDegrees[0] = btAsin(inMatrix[2][0]);

	if (btFabs(inMatrix[2][0]) < (1.0f - 0.001f)) {
		outEulerDegrees[1] = btAtan2(inMatrix[1][0], inMatrix[0][0]);
		outEulerDegrees[2] = btAtan2(inMatrix[2][1], inMatrix[2][2]);
	}
	else {
		outEulerDegrees[1] = btAtan2(inMatrix[1][2], inMatrix[1][1]);
		outEulerDegrees[2] = 0.0f;
	}

	outEulerDegrees *= SIMD_DEGS_PER_RAD;
}

inline bool getBulletShape(const char* model, btCollisionShape** shape, float* mass) {
	float x, y, z;
	if (sscanf(model, "box/%f/%f/%f/%f", &x, &y, &z, mass) == 4) {
		*shape = new btBoxShape(btVector3(x, y, z));
	}
	else if (sscanf(model, "capsule/%f/%f/%f", &x, &y, mass) == 3) {
		*shape = new btCapsuleShapeZ(x, y);
	}
	else if (sscanf(model, "cone/%f/%f/%f", &x, &y, mass) == 3) {
		*shape = new btConeShapeZ(x, y);
	}
	else if (sscanf(model, "cylinder/%f/%f/%f/%f", &x, &y, &z, mass) == 4) {
		*shape = new btCylinderShapeZ(btVector3(x, y, z));
	}
	else if (sscanf(model, "sphere/%f/%f", &x, mass) == 2) {
		*shape = new btSphereShape(x);
	}
	else {
		return false;
	}
	return true;
}
