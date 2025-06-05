#include "cuboid_rb.h"
#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>

Matrix3 CuboidComputeInvInertiaMatrix(Vector3 dims, float density) {
  float volume = dims.x * dims.y * dims.z;
  float x_sqr = dims.x * dims.x;
  float y_sqr = dims.y * dims.y;
  float z_sqr = dims.z * dims.z;
  float mass = volume * density;

  // applies only to cuboids
  Matrix3 invInertiaMatrix = {
      (12.0f) / (mass * (y_sqr + z_sqr)), 0.0f, 0.0f, 0.0f,
      (12.0f) / (mass * (x_sqr + z_sqr)), 0.0f, 0.0f, 0.0f,
      (12.0f) / (mass * (x_sqr + y_sqr)),
  };
  return invInertiaMatrix;
}

RigidBody CreateCuboidRB(float density, Vector3 pos, Vector3 dims) {
  Mesh *mesh = malloc(sizeof(Mesh));
  *mesh = GenMeshCube(dims.x, dims.y, dims.z);
  Matrix3 invIner = CuboidComputeInvInertiaMatrix(dims, density);

  return (RigidBody){density,
                     mesh,
                     pos,
                     QuaternionIdentity(),
                     MatrixTranslate(pos.x, pos.y, pos.z),
                     (dims.x * dims.y * dims.z),
                     {0.0f, 0.0f, 0.0f}, // centerofmass compared to mesh origin

                     {0.0f, 0.0f, 0.0f}, // linVel,
                     {0.0f, 0.0f, 0.0f}, // angMomentum,
                     invIner};
}
