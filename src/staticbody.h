
#include <raylib.h>

typedef struct CuboidStaticBody {
  Mesh *mesh;

  Vector3 position;
  Quaternion rotation;
  Vector3 dimensions;

  Matrix transform;
} CuboidStaticBody;

CuboidStaticBody CreateCuboidSB(Vector3 position, Quaternion rotation,
                                Vector3 dimensions);
