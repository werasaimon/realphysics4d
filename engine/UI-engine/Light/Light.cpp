// Libraries
#include "Light.h"


using namespace utility_engine;

// Constructor
Light::Light(GLuint id)
      : mLightID(id), mDiffuseColor(Color::white()),
        mSpecularColor(Color::white()), mIsActive(false)
{

}

// Constructor
Light::Light(GLuint id, Color diffuseColor, Color specularColor)
      : mLightID(id), mDiffuseColor(diffuseColor),
        mSpecularColor(specularColor), mIsActive(false)
{

}

// Destructor
Light::~Light()
{

}

// Initialize the light
void Light::init()
{

    // Enable the light
    enable();

    // Set the diffuse and specular color
    //    GLfloat diffuseColor[] = {mDiffuseColor.r, mDiffuseColor.g, mDiffuseColor.b, mDiffuseColor.a};
    //    GLfloat specularColor[] = {mSpecularColor.r,mSpecularColor.g,mSpecularColor.b,mSpecularColor.a};
    //    glLightfv(mLightID, GL_DIFFUSE, diffuseColor);
    //    glLightfv(mLightID, GL_SPECULAR, specularColor);
}

// Create a shadow map associated with this light
bool Light::createShadowMap(uint width, uint height)
{

		// Destroy the current shadow map
		//    destroyShadowMap();

		//    // Create the Framebuffer object to render the shadow map
		//    bool isFBOCreated = mFBOShadowMap.create(width, height, false);
		//    if (!isFBOCreated)
		//    {
		//        std::cerr << "Error : Cannot create the Shadow Map !" << std::endl;
		//        destroyShadowMap();
		//        return false;
		//    }

		//    // Bind the Framebuffer object
		//    mFBOShadowMap.bind(GL_NONE);

		//    // Create the shadow map depth texture
		//    mShadowMap.create(width, height, GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE);

		//    // Attache the shadow map texture to the Framebuffer object
		//    mFBOShadowMap.attachTexture(GL_DEPTH_ATTACHMENT_EXT, mShadowMap.getID());

		// Unbind the Framebuffer object
		//  mFBOShadowMap.unbind();

		//    // TODO : Change the path of the shader here so that it does not depend on the build folder
		//    bool isShaderCreated = mDepthShader.create("shaders/depth.vert",
		//                                               "shaders/depth.vert");
		//    if (!isShaderCreated)
		//    {
		//        std::cerr << "Error : Cannot create the Shadow Map !" << std::endl;
		//        destroyShadowMap();
		//        return false;
		//    }

    return true;
}
