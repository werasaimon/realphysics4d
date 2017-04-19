#ifndef LIGHT_H
#define LIGHT_H

// Libraries
#include <cassert>

//#include "Engie/Math3Dvec/Maths.h"
#include "../maths/Color.h"
#include "../Geometry/Object/Object3D.h"
#include "../Texture/Texture2D.h"


namespace utility_engine
{


    typedef unsigned int uint;

    // Class Light
    class Light : public Object3D
    {

        private:

            // -------------------- Attributes -------------------- //

            // OpenGL light ID
            GLuint mLightID;

            // Diffuse color of the light
            Color mDiffuseColor;

            // Specular color of the light
            Color mSpecularColor;

            // True if the light is active
            bool mIsActive;

            // Shadow map associated with this light
            Texture2D mShadowMap;



        public:

            // -------------------- Methods -------------------- //

            Light(void){}

            // Constructor
            Light(GLuint id);

            // Constructor
            Light(GLuint id, Color diffuseColor, Color specularColor);

            // Destructor
            virtual ~Light();

            // Return the diffuse color
            Color getDiffuseColor() const;

            // Set the diffuse color
            void setDiffuseColor(const Color& color);

            // Return the specular color
            Color getSpecularColor() const;

            // Set the specular color
            void setSpecularColor(const Color& color);

            // Return true if the light is active
            bool getIsActive() const;

            // Initialize the light
            void init();

            // Enable the light
            void enable();

            // Disable the light
            void disable();

            // Create a shadow map associated with this light
            bool createShadowMap(uint width, uint height);

            // Call this method before rendering the scene for the shadow map computation
            void startRenderingShadowMap();

            // Call this method after having rendered the scene for the shadow map computation
            void stopRenderingShadowMap();

            // Destroy the shadow map associated with this light
            void destroyShadowMap();
    };

    // Return the diffuse color
    inline Color Light::getDiffuseColor() const
    {
        return mDiffuseColor;
    }

    // Set the diffuse color
    inline void Light::setDiffuseColor(const Color& color)
    {
        mDiffuseColor = color;
    }

    // Return the specular color
    inline Color Light::getSpecularColor() const
    {
        return mSpecularColor;
    }

    // Set the specular color
    inline void Light::setSpecularColor(const Color& color)
    {
        mSpecularColor = color;
    }

    // Return true if the light is active
    inline bool Light::getIsActive() const
    {
        return mIsActive;
    }

    // Enable the light
    inline void Light::enable()
    {

        mIsActive = true;

        // Enable the light
        //glEnable(GL_LIGHTING);
        //glEnable(GL_LIGHT0 + mLightID);
    }

    // Disable the light
    inline void Light::disable()
    {
        mIsActive = false;
        // Disable the light
        //glDisable(GL_LIGHT0 + mLightID);
    }

    // Destroy the shadow map associated with this light
    inline void Light::destroyShadowMap()
    {
        mShadowMap.destroy();
    }

    // Call this method before rendering the scene for the shadow map computation
    inline void Light::startRenderingShadowMap()
    {
        assert(mShadowMap.getID());
    }

    // Call this method after having rendered the scene for the shadow map computation
    inline void Light::stopRenderingShadowMap()
    {
        assert(mShadowMap.getID());
    }

}

#endif
