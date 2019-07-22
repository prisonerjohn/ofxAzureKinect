#version 150

// Custom attributes.

uniform sampler2DRect uColorTex; // Sampler for the color registered data

in vec2 gTexCoord;
flat in int gValid;

out vec4 fragColor;

void main()
{
    if (gValid == 0)
    {
        discard;
    }

    fragColor = texture(uColorTex, gTexCoord);
}