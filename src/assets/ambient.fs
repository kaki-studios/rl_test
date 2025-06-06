#version 330

in vec2 fragTexCoord;

uniform vec4 ambientColor; // ambient light color (RGBA)
uniform sampler2D texture0; // model texture

out vec4 finalColor;

void main()
{
    vec4 texColor = texture(texture0, fragTexCoord);
    finalColor = texColor * ambientColor;
}
