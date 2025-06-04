#include "rididbody.h"
#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <stdlib.h>

Vector3 ComputeInvInertiaMatrix(Vector3 dims) {

  float volume = dims.x * dims.y * dims.z;
  float x_sqr = dims.x * dims.x;
  float y_sqr = dims.y * dims.y;
  float z_sqr = dims.z * dims.z;

  // applies only to cuboids
  Vector3 invInertiaMatrix = (Vector3){
      12.0 / (volume * (y_sqr + z_sqr)),
      12.0 / (volume * (x_sqr + z_sqr)),
      12.0 / (volume * (x_sqr + y_sqr)),
  };
  printf("(%f), (%f), (%f)\n", 1.0 / invInertiaMatrix.x,
         1.0 / invInertiaMatrix.y, 1.0 / invInertiaMatrix.z);

  return invInertiaMatrix;
}

CuboidRigidBody CreateRB(float mass, Vector3 pos, Vector3 dims) {
  Mesh *mesh = malloc(sizeof(Mesh));
  *mesh = GenMeshCube(dims.x, dims.y, dims.z);
  Vector3 invIner = ComputeInvInertiaMatrix(dims);

  return (CuboidRigidBody){mass,
                           mesh,
                           pos,
                           QuaternionIdentity(),
                           MatrixTranslate(pos.x, pos.y, pos.z),
                           dims,
                           {0.0f, 0.0f, 0.0f}, // linVel,
                           {0.0f, 0.0f, 0.0f}, // angMomentum,
                           invIner};
}

Matrix TransformToMatrix(Vector3 pos, Quaternion rot) {
  Matrix res = QuaternionToMatrix(rot);
  res.m12 = pos.x;
  res.m13 = pos.y;
  res.m14 = pos.z;
  return res;
}

// TODO: angular momentum w/ inertia matrix
void ApplyAngularVelocity(Quaternion *rot, Vector3 angularVelocity,
                          float deltaTime) {
  Quaternion w = {angularVelocity.x, angularVelocity.y, angularVelocity.z,
                  0.0f};

  // dq/dt = 0.5 * q * w
  Quaternion dq = QuaternionMultiply(*rot, w);
  dq.x *= 0.5f * deltaTime;
  dq.y *= 0.5f * deltaTime;
  dq.z *= 0.5f * deltaTime;
  dq.w *= 0.5f * deltaTime;

  // q += dq
  rot->x += dq.x;
  rot->y += dq.y;
  rot->z += dq.z;
  rot->w += dq.w;

  // Normalize to prevent drift
  *rot = QuaternionNormalize(*rot);
}

void ApplyLinearVelocity(Vector3 *pos, Vector3 linearVelocity,
                         float deltaTime) {

  Vector3 deltaMove = Vector3Scale(linearVelocity, deltaTime);
  *pos = Vector3Add(*pos, deltaMove);
}

Vector3 ComputeAngularVelocity(Vector3 invInertiaMatrix, Quaternion rot,
                               Vector3 angMomentum) {
  // Step 1: Rotate angular momentum into local space
  Quaternion invRot = QuaternionInvert(rot);
  Vector3 localAngMom = Vector3RotateByQuaternion(angMomentum, invRot);

  // Step 2: Multiply by I^-1 (element-wise, since diagonal)
  Vector3 localAngVel = {invInertiaMatrix.x * localAngMom.x,
                         invInertiaMatrix.y * localAngMom.y,
                         invInertiaMatrix.z * localAngMom.z};

  // Step 3: Rotate result back to world space
  Vector3 angVel = Vector3RotateByQuaternion(localAngVel, rot);

  return angVel;
}

void UpdateRB(CuboidRigidBody *rb, float deltaTime, Vector3 *angularVelocity) {
  // steps:
  // update linear_momentum and angular_momentum if needed
  // calculate quat and pos based on them
  // normalize quat
  // calculate transform based on that

  // first: gravity
  // rb->linearVelocity.y -= deltaTime * 9.81;
  // move based on linear_momentum
  ApplyLinearVelocity(&rb->position, rb->linearVelocity, deltaTime);
  Vector3 angVel = ComputeAngularVelocity(rb->invInertiaMatrix, rb->rotation,
                                          rb->angularMomentum);
  ApplyAngularVelocity(&rb->rotation, angVel, deltaTime);
  // apply changes to transform matrix
  rb->transform = TransformToMatrix(rb->position, rb->rotation);
  *angularVelocity = angVel;
}

void DeinitRB(CuboidRigidBody *rb) { free(rb->mesh); }
