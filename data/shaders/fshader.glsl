
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform vec3  lightAmbientColor;             // Lights ambient color
uniform vec3  lightDiffuseColor;             // Light diffuse color
uniform vec3  lightSpecularColor;            // Light specular color
uniform float shininess;                     // Shininess


varying	vec3 L;
varying	vec3 V;
varying	vec3 N;
varying vec3 H;

uniform int Texturing;



uniform sampler2D texture;

//uniform sampler2D ShadowMap;
//varying vec4 ShadowMapTexCoord;         // Texture shadow coordinates 4d


varying vec2 v_texcoord;


varying vec3 Position_worldspace;
varying vec3 Normal_cameraspace;
varying vec3 EyeDirection_cameraspace;
varying vec3 LightDirection_cameraspace;


void main()
{

    //--------------------  light color -------------------------//


      vec4  colorLight   = vec4( 1.0 , 1.0 , 1.0 , 1.0 );

      vec4  ambient      = vec4( lightAmbientColor  , 1.0 );
      vec4  diffColor    = vec4( lightDiffuseColor  , 1.0 );
      vec4  specColor    = vec4( lightSpecularColor , 1.0 );
      float specPower    = shininess;


      vec4 textureColor = vec4(1);
      if(Texturing == 1) textureColor = texture2D(texture, v_texcoord);


      //--------------------- init direction used vectors ------------------------//
      vec3 n   = normalize(N);
      vec3 l   = normalize(L);
      vec3 v   = normalize(V);
      vec3 h   = normalize(H);
      vec3 r    = reflect ( -v, n );



      //--------------------Phong light model ------------------------------------//
      vec4 ambi = ambient;
      vec4 diff = diffColor * max ( dot ( n , l ), 0.0 ) * textureColor;
      vec4 spec = specColor * pow ( max ( dot ( l, r ), 0.0 ), specPower );


      //-------------------------------------------------------------------//




//      //--------------------- shadow map ------------------------------//

//            vec2 poissonDisk[4];

//         poissonDisk[0] = vec2( -0.94201624, -0.39906216 );
//         poissonDisk[1] = vec2( 0.94558609, -0.76890725 );
//         poissonDisk[2] = vec2( -0.094184101, -0.92938870 );
//         poissonDisk[3] = vec2( 0.34495938, 0.29387760 );


//           float bias = 0.005*tan( acos(dot(n, l)) ); // cosTheta is dot( n,l ), clamped between 0 and 1
//                 bias = clamp(bias, 0,0.01);



//         float visibility = 1.0;
//         if(ShadowMapTexCoord.w > 0.0 )
//         {
//               vec3 ShadowMapTexCoordProj = ShadowMapTexCoord.xyz / ShadowMapTexCoord.w;

//               if(ShadowMapTexCoordProj.x >= 0.0 && ShadowMapTexCoordProj.x < 1.0 &&
//                  ShadowMapTexCoordProj.y >= 0.0 && ShadowMapTexCoordProj.y < 1.0 &&
//                  ShadowMapTexCoordProj.z >= 0.0 && ShadowMapTexCoordProj.z < 1.0)
//               {

//                   for (int i=0;i<4;i++)
//                   {
//                       if ( texture2D(ShadowMap, ShadowMapTexCoordProj.xy + poissonDisk[i]/700.0 ).r  <
//                            ShadowMapTexCoordProj.z-bias )
//                       {
//                           visibility -= 0.2;
//                       }
//                   }

//               }
//         }




      vec4 FragColor = ( ambi * colorLight ) +
                       ( diff * colorLight ) +
                       ( spec * colorLight );


      gl_FragColor = textureColor * 0.5 + FragColor * 0.5;

    // Set fragment color from texture
    // gl_FragColor = texture2D(texture, v_texcoord);
}



