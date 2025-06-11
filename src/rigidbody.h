// cheeky fix to avoid including rigidbody.h twice in the same project
#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include <raylib.h>

typedef struct Matrix3 {
  float m0, m3, m6;
  float m1, m4, m7;
  float m2, m5, m8;
} Matrix3;

typedef struct RigidBody {

  float density;
  Mesh *mesh;
  Vector3 position;
  Quaternion rotation;

  Matrix transform;

  // float volume;
  float invMass; // 0 for staticbodies
  Vector3 centerOfMass;

  Vector3 linearVelocity; // from center of mass
  // direction the rb "wants" to turn around
  Vector3 angularMomentum;

  // Vector3 invInertiaMatrix;
  Matrix3 invInertiaMatrix; // 0 for staticbodies
  float restitution;
  float friction;
} RigidBody;

Matrix Matrix3ToMatrix(Matrix3);
Matrix3 StripMatrixToMatrix3(Matrix);
Matrix ComputeWorldInertia(Matrix, Quaternion);

Vector3 MultiplyMatrixVector3(Matrix3, Vector3);

Vector3 ComputeAngularVelocity(Matrix3, Quaternion, Vector3);

float ComputeInertiaMoment(Vector3[3], unsigned int);
float ComputeInertiaProduct(Vector3[3], unsigned int, unsigned int);
RigidBody CreateRB(Mesh *, float, Vector3);

Matrix TransformToMatrix(Vector3, Quaternion);
// update rb based on timestep
void UpdateRB(RigidBody *, float);
void MeshComputeInertiaMatrix(Mesh *, float, Matrix3 *, Vector3 *, float *);
void DeinitRB(RigidBody *);
#endif
