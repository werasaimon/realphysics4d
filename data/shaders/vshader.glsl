

#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif


attribute vec4 a_vertex;
attribute vec2 a_texcoord;
attribute vec4 a_normal;

uniform mat4 modelToWorldMatrix;
uniform mat4 worldToViewMatrix;
uniform mat4 projectionMatrix;

varying vec2 v_texcoord;


varying	vec3 L;
varying	vec3 V;
varying	vec3 N;
varying vec3 H;


//varying vec4 ShadowMapTexCoord;         // Texture shadow coordinates 4d
//uniform mat4 ShadowMatrix;


uniform vec3 cameraWorldPosition;           // World position of the camera
uniform vec3 lightWorldPosition;            // World position of the light


varying vec3 Position_worldspace;
varying vec3 Normal_cameraspace;
varying vec3 EyeDirection_cameraspace;
varying vec3 LightDirection_cameraspace;


void main()
{

    vec4 Position = modelToWorldMatrix * a_vertex;


    //--------------------------------------//
    vec4 n = (a_normal * modelToWorldMatrix);
    N =  n.xyz;                                   // vector normal face
    L = ( lightWorldPosition  - Position.xyz );   // vector to light source
    V = ( cameraWorldPosition - Position.xyz );   // vector to the eye
    H = L + V;



    // Position of the vertex, in worldspace : M * position
    Position_worldspace = Position.xyz;

    // Vector that goes from the vertex to the camera, in camera space.
    // In camera space, the camera is at the origin (0,0,0).
    //EyeDirection_cameraspace = vec3(0,0,0) - (worldToViewMatrix * modelToWorldMatrix * a_vertex).xyz;


    // Vector that goes from the vertex to the light, in camera space
    //LightDirection_cameraspace = (worldToViewMatrix * vec4(lightWorldPosition,0)).xyz;


    // Normal of the the vertex, in camera space
    //Normal_cameraspace =  (worldToViewMatrix * modelToWorldMatrix * vec4(a_normal, 0.0)).xyz; // Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.

    //--------------------------------------//

    //ShadowMapTexCoord = Position * ShadowMatrix;

    //--------------------------------------//

    v_texcoord = a_texcoord;
    //v_texcoord = gl_MultiTexCoord0.xy;


    gl_Position = projectionMatrix * worldToViewMatrix * Position;
    //gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * Position;
}

