#ifndef RPPLANE_H
#define RPPLANE_H

#include "rpVector2D.h"
#include "rpVector3D.h"



namespace real_physics
{


template<class T> class  rpProjectPlane
{
public:

    rpProjectPlane(void);
   ~rpProjectPlane(void);
    rpProjectPlane(rpVector3D<T> Normal, rpVector3D<T> Origin);

    rpVector3D<T> projectionIsPlane2D(const rpVector3D<T> &point) const;
    rpVector3D<T> ProjectPoint3D(const rpVector3D<T> &point) const;


    rpVector3D<T> VecTo3d(const rpVector3D<T> &point) const;
    rpVector3D<T> VecTo3d(const rpVector2D<T> &point) const;


    SIMD_INLINE rpVector3D<T> getOrgin() const
    {
        return origin;
    }

    SIMD_INLINE rpVector3D<T> getNormal() const
    {
        return normal;
    }

    SIMD_INLINE rpVector3D<T> operator&(const rpVector3D<T>& _v) const
    {
        return ProjectPoint3D(_v);
    }

private:

    rpVector3D<T> UpVec;
    rpVector3D<T> RightVec;
    rpVector3D<T> normal;

    rpVector3D<T> origin;
};




template<class T>
rpProjectPlane<T>::rpProjectPlane()
{

}


template<class T>
rpProjectPlane<T>::rpProjectPlane(rpVector3D<T> Normal , rpVector3D<T> Origin)
{
    origin = Origin;
    normal = Normal;
    normal.normalize();

    rpVector3D<T>::btPlaneSpace1(  normal, RightVec, UpVec );


    /******
     UpVec = wsVector3( 0, 0, 0 );
     if( Normal.x || Normal.y )
     UpVec.z = 1;
     else
     UpVec.x = 1;
     RightVec = ( Normal ^ UpVec );
     RightVec.Normalize();
     UpVec = ( Normal ^ RightVec );
     UpVec.Normalize();
     /******/
}

template<class T>
rpProjectPlane<T>::~rpProjectPlane()
{

}



template<class T>
SIMD_INLINE rpVector3D<T> rpProjectPlane<T>::projectionIsPlane2D(const rpVector3D<T> &point) const
{
    rpVector3D<T> RES;
    RES.x = RightVec.dot(point - origin);
    RES.y = UpVec.dot(point - origin);
    return RES;
}


template<class T>
SIMD_INLINE rpVector3D<T> rpProjectPlane<T>::ProjectPoint3D(const rpVector3D<T> &point) const
{
    T lenghtPenetration = (point - origin).dot(normal);
    return  point - normal * lenghtPenetration;
}

template<class T>
SIMD_INLINE rpVector3D<T> rpProjectPlane<T>::VecTo3d(const rpVector3D<T> &point) const
{
    rpVector3D<T> RES;
    RES = origin + UpVec * point.y + RightVec * point.x;
    return RES;
}

template<class T>
SIMD_INLINE rpVector3D<T> rpProjectPlane<T>::VecTo3d(const rpVector2D<T> &point) const
{
    rpVector2D<T> RES;
    RES = origin + UpVec * point.y + RightVec * point.x;
    return RES;

}



}



#endif // RPPLANE_H
