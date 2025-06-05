#include "cuboid_rb.h"
#include <math.h>
#include <raylib.h>
#include <raymath.h>

#define CAMERA_RADIUS 15.0f

Vector3 NewCamPos(Vector2, float);
void UpdateCamPos(Camera3D *, Vector2 *);

int main(void) {
  const int width = 800;
  const int height = 400;
  InitWindow(width, height, "Raylib Test");
  SetTargetFPS(60);
  // camera setup, rn just orbits center
  Camera3D camera = {0};
  Vector2 sphereCoords = {0.0f, 0.5f};
  camera.position = NewCamPos(sphereCoords, CAMERA_RADIUS);

  camera.target = (Vector3){0.0f, 0.0f, 0.0f}; // Looking at the origin
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};     // Up vector
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;
  Vector3 dims = {3.0f, 6.0f, .5f};
  RigidBody rb = CreateCuboidRB(0.2f, (Vector3){0.0f, 0.0f, 0.0f}, dims);
  Mesh torus = GenMeshTorus(10.0f, 1.0f, 15, 20);
  RigidBody rb2 = CreateRB(&torus, 2.0f, (Vector3){10.0f, 0.0f, 0.0f});
  // rb2.linearVelocity = (Vector3){1.0f, 0.0f, 0.0f};
  rb.angularMomentum = (Vector3){10.0f, 0.01f, 0.01f};
  rb2.angularMomentum = (Vector3){0.f, 10.0f, 0.01f};
  Material mat = LoadMaterialDefault();
  Material mat2 = LoadMaterialDefault();
  mat2.maps[0].color = BLUE;
  double timeAcc = 0.0f;
  Vector3 angVel = {0.f, 0.f, 0.f};

  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // input
    UpdateCamPos(&camera, &sphereCoords);

    // Update
    // avoid too long deltatimes:
    double H = 0.001f;
    timeAcc += GetFrameTime();
    while (timeAcc >= H) {
      UpdateRB(&rb, H); // deltaTime is now constant
      UpdateRB(&rb2, H);

      timeAcc -= H;
    }

    // Draw

    BeginDrawing();
    ClearBackground(GRAY);
    DrawFPS(50, 50);

    BeginMode3D(camera);

    DrawMesh(*rb.mesh, mat, rb.transform);
    DrawMesh(*rb2.mesh, mat2, rb2.transform);
    // debug
    DrawLine3D(rb.position, Vector3Add(rb.position, rb.angularMomentum), GREEN);
    DrawLine3D(rb.position, Vector3Add(rb.position, rb.linearVelocity), RED);
    DrawLine3D(rb2.position, Vector3Add(rb2.position, rb2.angularMomentum),
               GREEN);
    DrawLine3D(rb2.position, Vector3Add(rb2.position, rb2.linearVelocity), RED);
    // DrawCylinderEx(rb.position, Vector3Add(rb.position, rb.angularMomentum),
    //                0.1f, 0.1f, 20, GREEN);
    // DrawCylinderEx(rb.position, Vector3Add(rb.position, rb.linearVelocity),
    //                0.1f, 0.1f, 20, RED);
    //
    // DrawCylinderEx(rb2.position, Vector3Add(rb2.position,
    // rb2.angularMomentum),
    //                0.1f, 0.1f, 20, GREEN);
    // DrawCylinderEx(rb2.position, Vector3Add(rb2.position,
    // rb2.linearVelocity),
    //                0.1f, 0.1f, 20, RED);

    DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){5.0f, 0.0f, 0.0f},
                   0.05f, 0.05f, 20, RED);
    DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){0.0f, 5.0f, 0.0f},
                   0.05f, 0.05f, 20, YELLOW);
    DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){0.0f, 0.0f, 5.0f},
                   0.05f, 0.05f, 20, BLUE);

    DrawGrid(50, 1.0f);

    EndMode3D();
    EndDrawing();
  }

  // De-initialization
  CloseWindow(); // Close window and OpenGL context

  return 0;
}

Vector3 NewCamPos(Vector2 sphereCoords, float length) {
  // find a circle at a latitude, determined by sphereCoords.y
  float height = length * sinf(sphereCoords.y);

  float radius = length * cosf(sphereCoords.y);
  // find coords within this circle
  float x = radius * sinf(sphereCoords.x);
  float y = radius * cosf(sphereCoords.x);
  // now final pos is x,height,y

  return (Vector3){x, height, y};
}

void UpdateCamPos(Camera3D *camera, Vector2 *sphereCoords) {
  if (IsKeyDown(KEY_UP)) {
    sphereCoords->y += 1 * GetFrameTime();
  }
  if (IsKeyDown(KEY_DOWN)) {
    sphereCoords->y -= 1 * GetFrameTime();
  }
  if (IsKeyDown(KEY_RIGHT)) {
    sphereCoords->x += 1 * GetFrameTime();
  }
  if (IsKeyDown(KEY_LEFT)) {
    sphereCoords->x -= 1 * GetFrameTime();
  }
  sphereCoords->y = Clamp(sphereCoords->y, -PI / 2, PI / 2);
  // instead of clamping x we can wrap instead:
  sphereCoords->x = Wrap(sphereCoords->x, -PI, PI);
  camera->position = NewCamPos(*sphereCoords, CAMERA_RADIUS);
}
