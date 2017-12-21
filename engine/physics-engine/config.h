/*
 * config.h
 *
 *  Created on: 12 янв. 2017 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_CONFIG_H_
#define SOURCE_ENGIE_CONFIG_H_

// Libraries
#include <limits>
#include <cfloat>
#include <utility>

#include "scalar.h"


// platform OS
#if defined(WIN32) ||defined(_WIN32) || defined(_WIN64) ||defined(__WIN32__) || defined(__WINDOWS__) // Windows platform
    #define WINDOWS_OS
#elif defined(__APPLE__)     // Apple platform
    #define APPLE_OS
#elif defined(__linux__) || defined(linux) || defined(__linux)    // Linux platform
    #define LINUX_OS
#endif

//#include "decimals/umHalf.h"


namespace real_physics
{


typedef int            iint;
//typedef float          scalar;
typedef unsigned int   uint;


typedef signed short   int16;
typedef signed int     int32;
typedef unsigned short uint16;


// ------------------- Enumerations ------------------- //

/// Position correction technique used in the constraint solver (for joints).
/// BAUMGARTE_JOINTS : Faster but can be innacurate in some situations.
/// NON_LINEAR_GAUSS_SEIDEL : Slower but more precise. This is the option used by default.
enum JointsPositionCorrectionTechnique {BAUMGARTE_JOINTS, NON_LINEAR_GAUSS_SEIDEL};

/// Position correction technique used in the contact solver (for contacts)
/// BAUMGARTE_CONTACTS : Faster but can be innacurate and can lead to unexpected bounciness
///                      in some situations (due to error correction factor being added to
///                      the bodies momentum).
/// SPLIT_IMPULSES : A bit slower but the error correction factor is not added to the
///                 bodies momentum. This is the option used by default.
enum ContactsPositionCorrectionTechnique {BAUMGARTE_CONTACTS, SPLIT_IMPULSES};



/// Pi constant
const scalar PI = scalar(3.14159265);

/// 2*Pi constant
const scalar PI_TIMES_2 = scalar(6.28318530);




/// Light Velocity c = 300000.kilometers / 1.second
const scalar LIGHT_MAX_VELOCITY_C = scalar(300.0);



/// Smallest scalar value (negative)
const scalar SCALAR_SMALLEST = - std::numeric_limits<scalar>::max();

/// Maximum scalar value
const scalar SCALAR_LARGEST = std::numeric_limits<scalar>::max();



/// Default friction coefficient for a rigid body
const scalar DEFAULT_FRICTION_COEFFICIENT = scalar(0.3);

/// Default bounciness factor for a rigid body
const scalar DEFAULT_BOUNCINESS = scalar(0.0);

/// Default rolling resistance
const scalar DEFAULT_ROLLING_RESISTANCE = scalar(0.0001);




/// Distance threshold for two contact points for a valid persistent contact (in meters)
const scalar PERSISTENT_CONTACT_DIST_THRESHOLD = scalar(0.03);

/// Velocity threshold for contact velocity restitution
const scalar RESTITUTION_VELOCITY_THRESHOLD = scalar(1.0);




/// Number of iterations when solving the velocity constraints of the Sequential Impulse technique
const uint DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS = 10;

/// Number of iterations when solving the position constraints of the Sequential Impulse technique
const uint DEFAULT_POSITION_SOLVER_NB_ITERATIONS = 10;




/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
/// inflated with a constant gap to allow the collision shape to move a little bit
/// without triggering a large modification of the tree which can be costly
const scalar DYNAMIC_TREE_AABB_GAP = scalar(0.1);

/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
/// also inflated in direction of the linear motion of the body by mutliplying the
/// followin constant with the linear velocity and the elapsed time between two frames.
const scalar DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER = scalar(1.7);





/// Maximum number of contact manifolds in an overlapping pair that involves two
/// convex collision shapes.
const int NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE = 1;

/// Maximum number of contact manifolds in an overlapping pair that involves at
/// least one concave collision shape.
const int NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE = 3;


/// Maximum Collison Shape Type
const int NB_COLLISION_SHAPE_TYPES = 9;





///// Time (in seconds) that a body must stay still to be considered sleeping
const float DEFAULT_TIME_BEFORE_SLEEP = 0.2;//1.0;

///// True if the spleeping technique is enabled
const bool  SLEEPING_ENABLED = true;





///// A body with a linear velocity smaller than the sleep linear velocity (in m/s)
///// might enter sleeping mode.
const scalar DEFAULT_SLEEP_LINEAR_VELOCITY = scalar(0.01);

///// A body with angular velocity smaller than the sleep angular velocity (in rad/s)
///// might enter sleeping mode
const scalar DEFAULT_SLEEP_ANGULAR_VELOCITY = scalar(3.0 * (PI / 180.0));

///// A body with minimum split
const scalar DEFAULT_SLEEP_SPLIT   = scalar(0.001);





/// Minimum for start damping
const scalar MINIMUM_FOR_DAPING = scalar(0.5);

///collision center point for the interpolation of (A+B) / 2.0
const bool   INTERPOLATION_CONTACT_POINTS = false;



}  // namespace

#endif /* SOURCE_ENGIE_CONFIG_H_ */
