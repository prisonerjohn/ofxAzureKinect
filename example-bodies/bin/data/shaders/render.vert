#version 150

// OF built-in attributes.

uniform mat4 modelViewProjectionMatrix;

// Custom attributes.

#define BODY_INDEX_MAP_BACKGROUND 255

const vec4[6] COLORS = vec4[]
(
    vec4(211 / 255.0, 248 / 255.0, 226 / 255.0, 1.0),
    vec4(228 / 255.0, 193 / 255.0, 249 / 255.0, 1.0),
    vec4(237 / 255.0, 231 / 255.0, 177 / 255.0, 1.0),
    vec4(246 / 255.0, 148 / 255.0, 193 / 255.0, 1.0),
    vec4(169 / 255.0, 222 / 255.0, 249 / 255.0, 1.0),
    vec4(255 / 255.0, 135 / 255.0, 111 / 255.0, 1.0)
);

uniform sampler2DRect uDepthTex; // Sampler for the depth space data
uniform sampler2DRect uBodyIndexTex; // Sampler for the body index data
uniform sampler2DRect uWorldTex; // Transformation from kinect depth space to kinect world space

uniform ivec2 uFrameSize;

uniform int[6] uBodyIDs;

out vec4 vColor;

void main()
{
    vec2 texCoord = vec2(gl_InstanceID % uFrameSize.x, gl_InstanceID / uFrameSize.x);

    float depth = texture(uDepthTex, texCoord).x;
    int bodyIndex = int(texture(uBodyIndexTex, texCoord).x * 255);
    vec4 ray = texture(uWorldTex, texCoord);

    if (depth != 0 && 
        bodyIndex != BODY_INDEX_MAP_BACKGROUND && 
        ray.x != 0 && ray.y != 0)
    {
        int bodyID = uBodyIDs[bodyIndex];
        vColor = COLORS[bodyID % 6];
    }
    else
    {
        vColor = vec4(0.0);
    }

    vec4 posWorld = vec4(1);
    posWorld.z = depth * 65535.0; // Remap to float range.
    posWorld.x = ray.x * posWorld.z;
    posWorld.y = ray.y * posWorld.z;

    gl_Position = modelViewProjectionMatrix * posWorld;
}