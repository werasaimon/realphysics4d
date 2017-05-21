 

// Uniform variables
uniform mat4 modelToWorldMatrix;      // Model too world coordinates matrix
uniform mat4 worldToViewMatrix;       // World to camera coordinates matrix
uniform mat4 projectionMatrix;        // Projection matrix

uniform mat4 worldMatrixShadow;       // Shadow to coordinates matrix



// Varying variables
varying vec3 worldPosition;             // World position of the vertex
varying vec3 worldNormal;               // World surface normalWorld


varying vec4   ShadowMapTexCoord;       // Texture shadow coordinates 4d
varying vec2   texCoords;    



void main() 
{
    // Compute the vertex position
    vec4 worldPos = modelToWorldMatrix * gl_Vertex;

    // World position vec3
    worldPosition = worldPos.xyz;

    // Compute the world surface normal
    worldNormal = (modelToWorldMatrix * vec4(gl_Normal, 0.0)).xyz;

    // Get the texture coordinates
    texCoords = gl_MultiTexCoord0.xy;

    // Compute the shadow
    ShadowMapTexCoord = worldMatrixShadow * worldPos;

    // Compute the clip-space vertex coordinates
    gl_Position = projectionMatrix * worldToViewMatrix * worldPos;
}