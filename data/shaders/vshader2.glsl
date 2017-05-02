

#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif


attribute vec4 a_vertex;
attribute vec2 a_texcoord;
attribute vec4 a_normal;

uniform mat4 ModelMatrix;
uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;

varying vec2 v_texcoord;




void main()
{

    vec4 Position = ModelMatrix * a_vertex;

    v_texcoord = a_texcoord;
    //v_texcoord = gl_MultiTexCoord0.xy;

    gl_Position = ProjectionMatrix * ViewMatrix * Position;
    //gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * Position;
}

