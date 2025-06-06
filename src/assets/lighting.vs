#version 330

in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;

uniform mat4 mvp;
uniform mat4 matModel;

out vec2 fragTexCoord;
out vec3 fragNormal;
out vec3 fragPosition;

void main()
{
    fragTexCoord = vertexTexCoord;

    vec4 worldPos = matModel * vec4(vertexPosition, 1.0);
    fragPosition = worldPos.xyz;

    // Normal matrix simplified: just transform normal by model (no transpose/inverse)
    fragNormal = normalize(matModel * vec4(vertexNormal, 0.0)).xyz;

    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
