
uniform vec3  lightAmbientColor;             // Lights ambient color
uniform vec3  lightDiffuseColor;             // Light diffuse color
uniform vec3  lightSpecularColor;            // Light specular color
uniform float shininess;                     // Shininess


uniform sampler2D Texture;
uniform sampler2D ShadowMap;


uniform int Texturing;


varying vec4 ShadowMapTexCoord;         // Texture shadow coordinates 4d
varying vec2 texCoords;                 // Texture coordinates 2d


varying	vec3 L;
varying	vec3 V;
varying	vec3 N;
varying vec3 H;



void main()
{

	vec4  ambient      = vec4( lightAmbientColor  , 1.0 );
	vec4  diffColor    = vec4( lightDiffuseColor  , 1.0 );
	vec4  specColor    = vec4( lightSpecularColor , 1.0 );
	float specPower    = shininess;


	vec4 textureColor = vec4(1);
	if (Texturing == 1) textureColor = texture2D(Texture, texCoords);

	vec3 n2   = normalize(N);
	vec3 l2   = normalize(L);
	vec3 v2   = normalize(V);
	vec3 h2   = normalize(H);
	vec3 r    = reflect( -v2, n2 );


	//-----------------  Blinn light model -----------------------------------//
	//vec4 diff = diffColor * max ( dot ( n2, l2 ), 0.0 ) * textureColor;
	//vec4 spec = specColor * pow ( max ( dot ( l2, h2 ), 0.0 ), specPower );



	//--------------------Phong light model ------------------------------------//
	vec4 diff = diffColor * max( dot ( n2, l2 ), 0.0 ) * textureColor;
	vec4 spec = specColor * pow( max ( dot ( l2, r ), 0.0 ), specPower );



	//--------------------- shadow map ------------------------------//

	vec2 poissonDisk[4] = vec2[]
    (
    		vec2( -0.94201624, -0.39906216 ),
			vec2( 0.94558609, -0.76890725 ),
			vec2( -0.094184101, -0.92938870 ),
			vec2( 0.34495938, 0.29387760 )
    );


	float bias = 0.005*tan( acos(dot(n2, l2)) );
	bias = clamp(bias, 0,0.01);


	float visibility = 1.0;
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
					visibility -= 0.2;
				}
			}

		}
	}


	gl_FragColor = (ambient * visibility) +
			       (diff    * visibility) +
			       (spec    * visibility);


}
