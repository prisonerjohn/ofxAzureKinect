#version 150

layout (points) in;
layout (triangle_strip) out;
layout (max_vertices = 4) out;

// OF handled uniforms and attributes.
uniform mat4 projectionMatrix;

// App specific uniforms and attributes.
uniform float uSpriteSize;

in vec4 vPosition[];
in vec2 vTexCoord[];
flat in int vValid[];

out vec2 gTexCoord;

void main()
{
    if (vValid[0] == 0) return;

    gTexCoord = vTexCoord[0];

    for (int i = 0; i < gl_in.length(); ++i)
    {
        gl_Position = projectionMatrix * (vPosition[i] + vec4(1.0, -1.0, 0.0, 0.0) * uSpriteSize);
        EmitVertex();

        gl_Position = projectionMatrix * (vPosition[i] + vec4(1.0, 1.0, 0.0, 0.0) * uSpriteSize);
        EmitVertex();

        gl_Position = projectionMatrix * (vPosition[i] + vec4(-1.0, -1.0, 0.0, 0.0) * uSpriteSize);
        EmitVertex();

        gl_Position = projectionMatrix * (vPosition[i] + vec4(-1.0, 1.0, 0.0, 0.0) * uSpriteSize);
        EmitVertex();

        EndPrimitive();
    }
}
