#include "light.h"
#include <raylib.h>

Light CreateLight(int type, Vector3 position, Vector3 target, Color color,
                  float intensity, Shader shader) {
  Light light = {0};

  if (lightCount < MAX_LIGHTS) {
    light.enabled = 1;
    light.type = type;
    light.position = position;
    light.target = target;
    light.color[0] = (float)color.r / 255.0f;
    light.color[1] = (float)color.g / 255.0f;
    light.color[2] = (float)color.b / 255.0f;
    light.color[3] = (float)color.a / 255.0f;
    light.intensity = intensity;

    // NOTE: Shader parameters names for lights must match the requested ones
    light.enabledLoc =
        GetShaderLocation(shader, TextFormat("lights[%i].enabled", lightCount));
    light.typeLoc =
        GetShaderLocation(shader, TextFormat("lights[%i].type", lightCount));
    light.positionLoc = GetShaderLocation(
        shader, TextFormat("lights[%i].position", lightCount));
    light.targetLoc =
        GetShaderLocation(shader, TextFormat("lights[%i].target", lightCount));
    light.colorLoc =
        GetShaderLocation(shader, TextFormat("lights[%i].color", lightCount));
    light.intensityLoc = GetShaderLocation(
        shader, TextFormat("lights[%i].intensity", lightCount));

    UpdateLight(shader, light);

    lightCount++;
  }

  return light;
}

void UpdateLight(Shader shader, Light light) {
  SetShaderValue(shader, light.enabledLoc, &light.enabled, SHADER_UNIFORM_INT);
  SetShaderValue(shader, light.typeLoc, &light.type, SHADER_UNIFORM_INT);

  // Send to shader light position values
  float position[3] = {light.position.x, light.position.y, light.position.z};
  SetShaderValue(shader, light.positionLoc, position, SHADER_UNIFORM_VEC3);

  // Send to shader light target position values
  float target[3] = {light.target.x, light.target.y, light.target.z};
  SetShaderValue(shader, light.targetLoc, target, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader, light.colorLoc, light.color, SHADER_UNIFORM_VEC4);
  SetShaderValue(shader, light.intensityLoc, &light.intensity,
                 SHADER_UNIFORM_FLOAT);
}
