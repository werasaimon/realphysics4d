#include "rpTimer.h"


namespace real_physics
{

// Constructor
rpTimer::rpTimer(double timeStep)
    : mTimeStep(timeStep), mIsRunning(false)
{
    assert(timeStep > 0.0);
}

// Destructor
rpTimer::~rpTimer()
{

}

// Return the current time of the system in seconds
long double rpTimer::getCurrentSystemTime()
{

#if defined(WINDOWS_OS)
    LARGE_INTEGER ticksPerSecond;
    LARGE_INTEGER ticks;
    QueryPerformanceFrequency(&ticksPerSecond);
    QueryPerformanceCounter(&ticks);
    return  ( double(ticks.QuadPart) / double(ticksPerSecond.QuadPart));
#else
    // Initialize the lastUpdateTime with the current time in seconds
    timeval timeValue;
    gettimeofday(&timeValue, NULL);
    return (timeValue.tv_sec + (timeValue.tv_usec / 1000000.0));
#endif

}



}
