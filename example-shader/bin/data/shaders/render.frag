#version 150

// Custom attributes.

uniform sampler2DRect uColorTex; // Sampler for the color registered data

in vec2 gTexCoord;

out vec4 fragColor;

void main()
{
    fragColor = texture(uColorTex, gTexCoord);
}