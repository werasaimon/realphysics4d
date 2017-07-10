/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2013 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/


// Uniform variables
uniform vec3 cameraWorldPosition;           // World position of the camera
uniform vec3 lightWorldPosition;            // World position of the light
uniform vec3 lightAmbientColor;             // Lights ambient color
uniform vec3 lightDiffuseColor;             // Light diffuse color
uniform vec3 lightSpecularColor;            // Light specular color
uniform float shininess;                    // Shininess


uniform sampler2D texture;
uniform sampler2D ShadowMap;


uniform int Texturing;                     // True if we need to use the texture

varying vec4 ShadowMapTexCoord;             // Texture shadow coordinates 4d
varying vec2 texCoords;                     // Texture coordinates 2d


// Varying variables
varying vec3 worldPosition;                 // World position of the vertex
varying vec3 worldNormal;                   // World surface normalWorld





void main()
{



	
	//--------------------- Light mapping ------------------------------//


    // Compute the ambient term
    vec3 ambient = lightAmbientColor;

    // Get the texture color
    vec3 textureColor = vec3(1);
    if (Texturing > 0) textureColor = texture2D(texture, texCoords).rgb;

    // Compute the diffuse term
    vec3 L = normalize((lightWorldPosition - worldPosition));
    vec3 N = normalize(worldNormal);

    vec3 diffuse = lightDiffuseColor * max(dot(N, L), 0.0) * textureColor;

    // Compute the specular term
    vec3 V = normalize((cameraWorldPosition - worldPosition));
    vec3 H = normalize(V + L);

    vec3 R = reflect ( -V, N );

    vec3 specular = lightSpecularColor * pow(max(dot(L,R), 0), shininess);






    //--------------------- Shadow mapping  ------------------------------//
    
	vec2 poissonDisk[4];
	poissonDisk[0] =  vec2( -0.94201624 , -0.39906216 );
	poissonDisk[1] =  vec2(  0.94558609 , -0.76890725 );
	poissonDisk[2] =  vec2( -0.094184101, -0.92938870 );
	poissonDisk[3] =  vec2(  0.34495938 ,  0.29387760 );
    
    
    float intensive = 1.0;

	float bias = 0.005*tan(acos(dot(N, L)));
	bias = clamp(bias, 0.0 , 0.01);
	

	if(ShadowMapTexCoord.w > 0.0 )
	{
	
	vec3 ShadowMapTexCoordProj = ShadowMapTexCoord.xyz / ShadowMapTexCoord.w;

		if(ShadowMapTexCoordProj.x >= 0.0 && ShadowMapTexCoordProj.x < 1.0 &&
		   ShadowMapTexCoordProj.y >= 0.0 && ShadowMapTexCoordProj.y < 1.0 &&
		   ShadowMapTexCoordProj.z >= 0.0 && ShadowMapTexCoordProj.z < 1.0)
		{	
			for (int i=0;i<4;i++)
			{
				if ( texture2D(ShadowMap, ShadowMapTexCoordProj.xy + poissonDisk[i]/700.0 ).r  < ShadowMapTexCoordProj.z-bias )
                {
					intensive -= (0.25 - bias);
				}
			}
		}
	
	}


	//-------------------- finality color ----------------------------//

    // Compute the final color
    gl_FragColor = vec4(ambient + diffuse + specular, 1.0) * intensive;



}
