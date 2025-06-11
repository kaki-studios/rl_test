#include "staticbody.h"
#include "rigidbody.h"
#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>

RigidBody CreateCuboidSB(Vector3 position, Quaternion rotation,
                         Vector3 dimensions) {
  Mesh *mesh = malloc(sizeof(Mesh));
  *mesh = GenMeshCube(dimensions.x, dimensions.y, dimensions.z);
  Matrix transform =
      MatrixMultiply(MatrixTranslate(position.x, position.y, position.z),
                     QuaternionToMatrix(rotation));

  // Matrix transform = MatrixTranslate(position.x, position.y, position.z);
  return (RigidBody){
      .mesh = mesh,
      .position = position,
      .rotation = rotation,
      .transform = transform,
      .restitution = 1.0f,
      .friction = 1.0f,
      .invMass = 0.0f,
      .invInertiaMatrix = {0},
  };
}
