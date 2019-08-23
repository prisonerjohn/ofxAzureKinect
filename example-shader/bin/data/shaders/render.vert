#version 150

// OF built-in attributes.

uniform mat4 modelViewMatrix;

// Custom attributes.

uniform sampler2DRect uDepthTex; // Sampler for the depth space data
uniform sampler2DRect uWorldTex; // Transformation from kinect depth/color space to kinect world space

uniform ivec2 uFrameSize;

out vec4 vPosition;
out vec2 vTexCoord;
flat out int vValid;

void main()
{
    vTexCoord = vec2(gl_InstanceID % uFrameSize.x, gl_InstanceID / uFrameSize.x);

    float depth = texture(uDepthTex, vTexCoord).x;
    vec4 ray = texture(uWorldTex, vTexCoord);

    vValid = (depth != 0 && ray.x != 0 && ray.y != 0) ? 1 : 0;

    vec4 posWorld = vec4(1);
    posWorld.z = depth * 65535.0; // Remap to float range.
    posWorld.x = ray.x * posWorld.z;
    posWorld.y = ray.y * posWorld.z;

    // Flip X as OpenGL and K4A have different conventions on which direction is positive.
    posWorld.x *= -1;

    vPosition = modelViewMatrix * posWorld;
}