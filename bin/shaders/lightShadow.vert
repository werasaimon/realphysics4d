

// Uniform variables
uniform mat4x4 modelToWorldMatrix;        // Model too world coordinates matrix
uniform mat4x4 worldToViewMatrix;         // World to camera coordinates matrix
uniform mat4x4 projectionMatrix;          // Projection matrix


uniform mat4x4 ShadowMatrix;              // Shadow to coordinates matrix

varying vec4   ShadowMapTexCoord;         // Texture shadow coordinates 4d
varying vec2   texCoords;                 // Texture coordinates 2d

uniform vec3   cameraWorldPosition;       // World position of the camera
uniform vec3   lightWorldPosition;        // World position of the light
uniform vec3   vertexWorldPosition;       // World position of the Vertex

varying	vec3 L;
varying	vec3 V;
varying	vec3 N;
varying vec3 H;


void main()
{

	// Compute the vertex position
	vec4 Position = modelToWorldMatrix * gl_Vertex;
	vertexWorldPosition = Position.xyz;


	// Compute the vector direction
	N = ( vec4(gl_Normal, 0.0) * modelToWorldMatrix).xyz;
	L = ( lightWorldPosition  - vertexWorldPosition );
	V = ( cameraWorldPosition - vertexWorldPosition );
    H = ( L + V );


    // Compute the shadow
    ShadowMapTexCoord = ShadowMatrix  * Position;
    texCoords     = gl_MultiTexCoord0.xy;


   	//gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * Position;
      gl_Position = projectionMatrix * worldToViewMatrix * Position;
}
