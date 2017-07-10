#ifndef RPTIMER_H
#define RPTIMER_H



// Libraries
#include <stdexcept>
#include <iostream>
#include <ctime>
#include <cassert>
#include "../LinearMaths/mathematics.h"

//#include "configuration.h"

#if defined(WINDOWS_OS)   // For Windows platform
   #define NOMINMAX       // This is used to avoid definition of max() and min() macros
   #include <windows.h>
#else                                   // For Mac OS or Linux platform
   #include <sys/time.h>
#endif


namespace real_physics
{


// Class Timer
/**
 * This class will take care of the time in the physics engine. It
 * uses functions that depend on the current platform to get the
 * current time.
 */
class rpTimer
{

    private :

        // -------------------- Attributes -------------------- //

        /// Timestep dt of the physics engine (timestep > 0.0)
        double mTimeStep;

        /// Last time the timer has been updated
        long double mLastUpdateTime;

        /// Time difference between the two last timer update() calls
        long double mDeltaTime;

        /// Used to fix the time step and avoid strange time effects
        double mAccumulator;

        /// True if the timer is running
        bool mIsRunning;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        rpTimer(const rpTimer& timer);

        /// Private assignment operator
        rpTimer& operator=(const rpTimer& timer);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        rpTimer(double timeStep);

        /// Destructor
        virtual ~rpTimer();

        /// Return the timestep of the physics engine
        double getTimeStep() const;

        /// Set the timestep of the physics engine
        void setTimeStep(double timeStep);

        /// Return the current time of the physics engine
        long double getPhysicsTime() const;

        /// Start the timer
        void start();

        /// Stop the timer
        void stop();

        /// Return true if the timer is running
        bool getIsRunning() const;

        /// True if it's possible to take a new step
        bool isPossibleToTakeStep() const;

        /// Compute the time since the last update() call and add it to the accumulator
        void update();

        /// Take a new step => update the timer by adding the timeStep value to the current time
        void nextStep();

        /// Compute the interpolation factor
        scalar computeInterpolationFactor();

        /// Return the current time of the system in seconds
        static long double getCurrentSystemTime();
};

// Return the timestep of the physics engine
SIMD_INLINE double rpTimer::getTimeStep() const
{
    return mTimeStep;
}

// Set the timestep of the physics engine
SIMD_INLINE void rpTimer::setTimeStep(double timeStep)
{
    assert(timeStep > 0.0f);
    mTimeStep = timeStep;
}

// Return the current time
SIMD_INLINE long double rpTimer::getPhysicsTime() const
{
    return mLastUpdateTime;
}

// Return if the timer is running
SIMD_INLINE bool rpTimer::getIsRunning() const
{
    return mIsRunning;
}

// Start the timer
SIMD_INLINE void rpTimer::start()
{
    if (!mIsRunning)
    {
        // Get the current system time
        mLastUpdateTime = getCurrentSystemTime();

        mAccumulator = 0.0;
        mIsRunning = true;
    }
}

// Stop the timer
SIMD_INLINE void rpTimer::stop()
{
    mIsRunning = false;
}

// True if it's possible to take a new step
SIMD_INLINE bool rpTimer::isPossibleToTakeStep() const
{
    return (mAccumulator >= mTimeStep);
}

// Take a new step => update the timer by adding the timeStep value to the current time
SIMD_INLINE void rpTimer::nextStep()
{
    assert(mIsRunning);
    // Update the accumulator value
    mAccumulator -= mTimeStep;
}

// Compute the interpolation factor
SIMD_INLINE scalar rpTimer::computeInterpolationFactor()
{
    return (scalar(mAccumulator / mTimeStep));
}

// Compute the time since the last update() call and add it to the accumulator
SIMD_INLINE void rpTimer::update()
{
    // Get the current system time
    long double currentTime = getCurrentSystemTime();

    // Compute the delta display time between two display frames
    mDeltaTime = currentTime - mLastUpdateTime;

    /**/ mDeltaTime = ( mDeltaTime > (mTimeStep * 10.0) )?  (mTimeStep * 10.0) : mDeltaTime; /**/

    // Update the current display time
    mLastUpdateTime = currentTime;

    // Update the accumulator value
    mAccumulator += mDeltaTime;
}


}

#endif // RPTIMER_H
