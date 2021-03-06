#ifndef QuickHull_Ray_hpp
#define QuickHull_Ray_hpp

#include "../../../Geometry/QuickHull/Structs/Vector3.hpp"


namespace real_physics
{
	namespace quickhull
	{
	
		template <typename T>
		struct Ray
		{
			Vector3<T> m_S;
			Vector3<T> m_V;
			Ray() = default;
			Ray(const Vector3<T>& S,const Vector3<T>& V) : m_S(S), m_V(V)
			{
			}
		};

	}
}


#endif
