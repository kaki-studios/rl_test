#include "staticbody.h"
#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>

CuboidStaticBody CreateCuboidSB(Vector3 position, Quaternion rotation,
                                Vector3 dimensions) {
  Mesh *mesh = malloc(sizeof(Mesh));
  *mesh = GenMeshCube(dimensions.x, dimensions.y, dimensions.z);
  // Matrix transform =
  //     MatrixMultiply(MatrixTranslate(dimensions.x, dimensions.y,
  //     dimensions.z),
  //                    QuaternionToMatrix(rotation));

  Matrix transform = MatrixTranslate(position.x, position.y, position.z);
  return (CuboidStaticBody){
      .mesh = mesh,
      .position = position,
      .rotation = rotation,
      .dimensions = dimensions,
      .transform = transform,
      .restitution = 1.0f,
      .friction = 1.0f,
  };
}
