/*
 * gjkEpa.h
 *
 *  Created on: 20 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_COLLISION_NARROWPHASE_GJKEPA_H_
#define SOURCE_ENGIE_COLLISION_NARROWPHASE_GJKEPA_H_


#include "../GJK_EPA/rpGjkCollisionDescription.h"

namespace real_physics
{

// Config

///* GJK	*/
//#define GJK_MAX_ITERATIONS	128
//#define GJK_ACCURARY		((scalar)0.0001)
//#define GJK_MIN_DISTANCE	((scalar)0.0001)
//#define GJK_DUPLICATED_EPS	((scalar)0.0001)
//#define GJK_SIMPLEX2_EPS	((scalar)0.0)
//#define GJK_SIMPLEX3_EPS	((scalar)0.0)
//#define GJK_SIMPLEX4_EPS	((scalar)0.0)

///* EPA	*/
//#define EPA_MAX_VERTICES	64
//#define EPA_MAX_FACES		(EPA_MAX_VERTICES*2)
//#define EPA_MAX_ITERATIONS	255
//#define EPA_ACCURACY		((scalar)0.0001)
//#define EPA_FALLBACK		(10*EPA_ACCURACY)
//#define EPA_PLANE_EPS		((scalar)0.0003)
//#define EPA_INSIDE_EPS		((scalar)0.01)



/* GJK	*/
#define GJK_MAX_ITERATIONS	128
#define GJK_ACCURARY		((scalar)0.0001)
#define GJK_MIN_DISTANCE	((scalar)0.0001)
#define GJK_DUPLICATED_EPS	((scalar)0.0001)
#define GJK_SIMPLEX2_EPS	((scalar)0.0)
#define GJK_SIMPLEX3_EPS	((scalar)0.0)
#define GJK_SIMPLEX4_EPS	((scalar)0.0)

/* EPA	*/
#define EPA_MAX_VERTICES	64
#define EPA_MAX_FACES		(EPA_MAX_VERTICES*2)
#define EPA_MAX_ITERATIONS	255
#define EPA_ACCURACY		((scalar)0.0001)
#define EPA_FALLBACK		(10*EPA_ACCURACY)
#define EPA_PLANE_EPS		((scalar)0.00001)
#define EPA_INSIDE_EPS		((scalar)0.001)




struct	rpGjkEpaSolver
{
    struct	sResults
    {
        enum eStatus
        {
            Separated,		/* Shapes doesnt penetrate												*/
            Penetrating,	/* Shapes are penetrating												*/
            GJK_Failed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
            EPA_Failed		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/

        } status;


        Vector3	witnesses[2];
        Vector3	normal;
        scalar	distance;
    };


    template <typename ConvexTemplate>
    static bool Distance(const ConvexTemplate& a,
                         const ConvexTemplate& b,
                         const Vector3& guess, rpGjkEpaSolver::sResults& results);



    template <typename ConvexTemplate>
    static bool  Penetration(const ConvexTemplate& a,
                             const ConvexTemplate& b,
                             const Vector3& guess, rpGjkEpaSolver::sResults& results);


    template <typename ConvexTemplate , typename DistanceInfoTemplate>
    static int	 ComputeGjkDistance(const ConvexTemplate& a,
                                    const ConvexTemplate& b,
                                    const rpGjkCollisionDescription& colDesc,
                                          DistanceInfoTemplate& distInfo);



    template <typename  ConvexTemplate>
    static bool SignedDistance(const ConvexTemplate& a,
                               const ConvexTemplate& b,
                               const Vector3& guess, rpGjkEpaSolver::sResults& results);

};





#if defined(DEBUG) || defined (_DEBUG)
#include <stdio.h> //for debug printf
#ifdef __SPU__
#include <spu_printf.h>
#define printf spu_printf
#endif //__SPU__
#endif



namespace //gjkepa2_impl
{




// Shorthands
typedef unsigned int	U;
typedef unsigned char	U1;





// MinkowskiDiff
template <typename ConvexTemplate>
struct	MinkowskiDiff
{
    const ConvexTemplate* m_convexAPtr;
    const ConvexTemplate* m_convexBPtr;

    Transform				m_world0;
    Transform				m_world1;

    bool					m_enableMargin;


    MinkowskiDiff(const ConvexTemplate& a,
                  const ConvexTemplate& b)
    :m_convexAPtr(&a),
     m_convexBPtr(&b)
    {

    }


    SIMD_INLINE Vector3		Support0( const Vector3& dir ) const
    {
        return m_world0 * m_convexAPtr->getLocalSupportPointWithMargin(m_world0.getOrientation().getInverse() * dir );
    }

    SIMD_INLINE Vector3		Support1( const Vector3& dir ) const
    {

       return  m_world1 * m_convexBPtr->getLocalSupportPointWithMargin(m_world1.getOrientation().getInverse() * dir );
    }



    SIMD_INLINE Vector3	 Support(const Vector3& d) const
    {
        return(Support0(d)-Support1(-d));
    }



    SIMD_INLINE Vector3	 Support(const Vector3& d,U index) const
    {
        if(index)
        {
            return(Support0(d));
        }
        else
        {
            return(Support1(d));
        }
    }

};


enum	eGjkStatus
{
    eGjkValid,
    eGjkInside,
    eGjkFailed
};






// GJK
template <typename ConvexTemplate>
struct	GJK
{
    /* Types		*/
    struct	sSV
    {
        Vector3	d,w;
    };



    struct	sSimplex
    {
        sSV*		c[4];
        scalar	    p[4];
        U			rank;
    };

    /* Fields		*/

    MinkowskiDiff<ConvexTemplate>			m_shape;
    Vector3		    m_ray;
    scalar		    m_distance;
    sSimplex		m_simplices[2];
    sSV			    m_store[4];
    sSV*			m_free[4];
    U				m_nfree;
    U				m_current;
    sSimplex*		m_simplex;

    eGjkStatus      m_status;
    /* Methods		*/

    GJK(const ConvexTemplate& a, const ConvexTemplate& b)
    :m_shape(a,b)
    {
        Initialize();
    }





    void				Initialize()
    {
        m_ray		=	Vector3(0,0,0);
        m_nfree		=	0;
        m_status	=	eGjkFailed;
        m_current	=	0;
        m_distance	=	0;
    }




    eGjkStatus			Evaluate(const MinkowskiDiff<ConvexTemplate>& shapearg,const Vector3& guess)
    {
        U			iterations=0;
        scalar	    sqdist=0;
        scalar	    alpha=0;
        Vector3	    lastw[4];
        U			clastw=0;


        /* Initialize solver		*/
        m_free[0]			=	&m_store[0];
        m_free[1]			=	&m_store[1];
        m_free[2]			=	&m_store[2];
        m_free[3]			=	&m_store[3];
        m_nfree				=	4;
        m_current			=	0;
        m_status			=	eGjkValid;
        m_shape				=	shapearg;
        m_distance			=	0;
        /* Initialize simplex		*/
        m_simplices[0].rank	=	0;
        m_ray				=	guess;
        const scalar	sqrl=	m_ray.length2();
        appendvertice(m_simplices[0],sqrl>0?-m_ray:Vector3(1,0,0));
        m_simplices[0].p[0]	=	1;
        m_ray				=	m_simplices[0].c[0]->w;
        sqdist				=	sqrl;
        lastw[0]			=
        lastw[1]			=
        lastw[2]			=
        lastw[3]			=	m_ray;
        /* Loop						*/
        do
        {
            const U		next=1-m_current;
            sSimplex&	cs=m_simplices[m_current];
            sSimplex&	ns=m_simplices[next];
            /* Check zero							*/
            const scalar	rl=m_ray.length();
            if(rl<GJK_MIN_DISTANCE)
            {/* Touching or inside				*/
                m_status=eGjkInside;
                break;
            }
            /* Append new vertice in -'v' direction	*/
            appendvertice(cs,-m_ray);
            const Vector3&	w=cs.c[cs.rank-1]->w;
            bool				found=false;
            for(U i=0;i<4;++i)
            {
                if((w-lastw[i]).length2()<GJK_DUPLICATED_EPS)
                { found=true;break; }
            }
            if(found)
            {/* Return old simplex				*/
                removevertice(m_simplices[m_current]);
                break;
            }
            else
            {/* Update lastw					*/
                lastw[clastw=(clastw+1)&3]=w;
            }
            /* Check for termination				*/
            const scalar	omega=(m_ray.dot(w))/rl;
            alpha=Max(omega,alpha);
            if(((rl-alpha)-(GJK_ACCURARY*rl))<=0)
            {/* Return old simplex				*/
                removevertice(m_simplices[m_current]);
                break;
            }
            /* Reduce simplex						*/
            scalar	weights[4];
            U			mask=0;
            switch(cs.rank)
            {
            case	2:	sqdist=projectorigin(	cs.c[0]->w,
                    cs.c[1]->w,
                    weights,mask);break;
            case	3:	sqdist=projectorigin(	cs.c[0]->w,
                    cs.c[1]->w,
                    cs.c[2]->w,
                    weights,mask);break;
            case	4:	sqdist=projectorigin(	cs.c[0]->w,
                    cs.c[1]->w,
                    cs.c[2]->w,
                    cs.c[3]->w,
                    weights,mask);break;
            }
            if(sqdist>=0)
            {/* Valid	*/
                ns.rank		=	0;
                m_ray		=	Vector3(0,0,0);
                m_current	=	next;
                for(U i=0,ni=cs.rank;i<ni;++i)
                {
                    if(mask&(1<<i))
                    {
                        ns.c[ns.rank]		=	cs.c[i];
                        ns.p[ns.rank++]		=	weights[i];
                        m_ray				+=	cs.c[i]->w*weights[i];
                    }
                    else
                    {
                        m_free[m_nfree++]	=	cs.c[i];
                    }
                }
                if(mask==15) m_status=eGjkInside;
            }
            else
            {/* Return old simplex				*/
                removevertice(m_simplices[m_current]);
                break;
            }
            m_status=((++iterations)<GJK_MAX_ITERATIONS)?m_status:eGjkFailed;
        } while(m_status==eGjkValid);
        m_simplex=&m_simplices[m_current];
        switch(m_status)
        {
        case	eGjkValid:		m_distance=m_ray.length();break;
        case	eGjkInside:	m_distance=0;break;
        default:
        {
        }
        }
        return(m_status);
    }
    bool					EncloseOrigin()
    {
        switch(m_simplex->rank)
        {
        case	1:
        {
            for(U i=0;i<3;++i)
            {
                Vector3		axis=Vector3(0,0,0);
                axis[i]=1;
                appendvertice(*m_simplex, axis);
                if(EncloseOrigin())	return(true);
                removevertice(*m_simplex);
                appendvertice(*m_simplex,-axis);
                if(EncloseOrigin())	return(true);
                removevertice(*m_simplex);
            }
        }
        break;
        case	2:
        {
            const Vector3	d=m_simplex->c[1]->w-m_simplex->c[0]->w;
            for(U i=0;i<3;++i)
            {
                Vector3		axis=Vector3(0,0,0);
                axis[i]=1;
                const Vector3	p=(d.cross(axis));
                if(p.length2()>0)
                {
                    appendvertice(*m_simplex, p);
                    if(EncloseOrigin())	return(true);
                    removevertice(*m_simplex);
                    appendvertice(*m_simplex,-p);
                    if(EncloseOrigin())	return(true);
                    removevertice(*m_simplex);
                }
            }
        }
        break;
        case	3:
        {
            const Vector3	n=(m_simplex->c[1]->w-m_simplex->c[0]->w).
                    cross(m_simplex->c[2]->w-m_simplex->c[0]->w);
            ;
            if(n.length2()>0)
            {
                appendvertice(*m_simplex,n);
                if(EncloseOrigin())	return(true);
                removevertice(*m_simplex);
                appendvertice(*m_simplex,-n);
                if(EncloseOrigin())	return(true);
                removevertice(*m_simplex);
            }
        }
        break;
        case	4:
        {
            if(Abs(det(	m_simplex->c[0]->w-m_simplex->c[3]->w,
                        m_simplex->c[1]->w-m_simplex->c[3]->w,
                        m_simplex->c[2]->w-m_simplex->c[3]->w))>0)
                return(true);
        }
        break;
        }
        return(false);
    }




    /* Internals	*/
    void				getsupport(const Vector3& d,sSV& sv) const
    {
        sv.d	=	d/d.length();
        sv.w	=	m_shape.Support(sv.d);
    }




    void				removevertice(sSimplex& simplex)
    {
        m_free[m_nfree++]=simplex.c[--simplex.rank];
    }



    void				appendvertice(sSimplex& simplex,const Vector3& v)
    {
        simplex.p[simplex.rank]=0;
        simplex.c[simplex.rank]=m_free[--m_nfree];
        getsupport(v,*simplex.c[simplex.rank++]);
    }



    static scalar		det(const Vector3& a,const Vector3& b,const Vector3& c)
    {
        return(	a.y*b.z*c.x+a.z*b.x*c.y-
                a.x*b.z*c.y-a.y*b.x*c.z+
                a.x*b.y*c.z-a.z*b.y*c.x);
    }




    static scalar		projectorigin(	const Vector3& a, const Vector3& b, scalar* w, U& m)
    {
        const Vector3	d=b-a;
        const scalar	l=d.length2();
        if(l>GJK_SIMPLEX2_EPS)
        {
            const scalar	t(l>0?-(a.dot(d))/l:0);
            if(t>=1)		{ w[0]=0;w[1]=1;m=2;return(b.length2()); }
            else if(t<=0)	{ w[0]=1;w[1]=0;m=1;return(a.length2()); }
            else			{ w[0]=1-(w[1]=t);m=3;return((a+d*t).length2()); }
        }
        return(-1);
    }




    static scalar		projectorigin(	const Vector3& a,
                                        const Vector3& b,
                                        const Vector3& c,
                                        scalar* w,U& m)
    {
        static const U		imd3[]={1,2,0};
        const Vector3*	      vt[]={&a,&b,&c};
        const Vector3		  dl[]={a-b,b-c,c-a};
        const Vector3		   n=(dl[0].cross(dl[1]));
        const scalar		   l=n.length2();
        if(l>GJK_SIMPLEX3_EPS)
        {
            scalar	mindist=-1;
            scalar	subw[2]={0.f,0.f};
            U			subm(0);
            for(U i=0;i<3;++i)
            {
                if( (*vt[i]).dot((dl[i].cross(n)))>0)
                {
                    const U			j=imd3[i];
                    const scalar	subd(projectorigin(*vt[i],*vt[j],subw,subm));
                    if((mindist<0)||(subd<mindist))
                    {
                        mindist		=	subd;
                        m			=	static_cast<U>(((subm&1)?1<<i:0)+((subm&2)?1<<j:0));
                        w[i]		=	subw[0];
                        w[j]		=	subw[1];
                        w[imd3[j]]	=	0;
                    }
                }
            }
            if(mindist<0)
            {
                const scalar	d=(a.dot(n));
                const scalar	s=SquareRoot(l);
                const Vector3	p=n*(d/l);
                mindist	=	p.length2();
                m		=	7;
                w[0]	=	((dl[1].cross(b-p))).length()/s;
                w[1]	=	((dl[2].cross(c-p))).length()/s;
                w[2]	=	1-(w[0]+w[1]);
            }
            return(mindist);
        }
        return(-1);
    }





    static scalar		projectorigin(	const Vector3& a, const Vector3& b,
                                        const Vector3& c, const Vector3& d,
                                        scalar* w,U& m)
    {
        static const U		imd3[]={1,2,0};
        const Vector3*	    vt[]={&a,&b,&c,&d};
        const Vector3		dl[]={a-d,b-d,c-d};
        const scalar		vl=det(dl[0],dl[1],dl[2]);
        const bool			ng=(vl* (a.dot( (b-c).cross(a-b) )))<=0;

        if(ng&&(Abs(vl)>GJK_SIMPLEX4_EPS))
        {
            scalar	mindist=-1;
            scalar	subw[3]={0.f,0.f,0.f};
            U			subm(0);
            for(U i=0;i<3;++i)
            {
                const U			j=imd3[i];
                const scalar	s=vl*(d.dot((dl[i].cross(dl[j]))));
                if(s>0)
                {
                    const scalar	subd=projectorigin(*vt[i],*vt[j],d,subw,subm);
                    if((mindist<0)||(subd<mindist))
                    {
                        mindist		=	subd;
                        m			=	static_cast<U>((subm&1?1<<i:0)+
                                (subm&2?1<<j:0)+
                                (subm&4?8:0));
                        w[i]		=	subw[0];
                        w[j]		=	subw[1];
                        w[imd3[j]]	=	0;
                        w[3]		=	subw[2];
                    }
                }
            }
            if(mindist<0)
            {
                mindist	=	0;
                m		=	15;
                w[0]	=	det(c,b,d)/vl;
                w[1]	=	det(a,c,d)/vl;
                w[2]	=	det(b,a,d)/vl;
                w[3]	=	1-(w[0]+w[1]+w[2]);
            }
            return(mindist);
        }
        return(-1);
    }
};


enum	eEpaStatus
{
    eEpaValid,
    eEpaTouching,
    eEpaDegenerated,
    eEpaNonConvex,
    eEpaInvalidHull,
    eEpaOutOfFaces,
    eEpaOutOfVertices,
    eEpaAccuraryReached,
    eEpaFallBack,
    eEpaFailed
};




// EPA
template <typename ConvexTemplate>
struct	EPA
{
    /* Types		*/

    struct	sFace
    {
        Vector3	n;
        scalar	d;
        typename GJK<ConvexTemplate>::sSV*		c[3];
        sFace*		f[3];
        sFace*		l[2];
        U1			e[3];
        U1			pass;
    };


    struct	sList
    {
        sFace*		root;
        U			count;
        sList() : root(0),count(0)	{}
    };


    struct	sHorizon
    {
        sFace*		cf;
        sFace*		ff;
        U			nf;
        sHorizon() : cf(0),ff(0),nf(0)	{}
    };




    /* Fields		*/
    eEpaStatus		                        m_status;
    typename GJK<ConvexTemplate>::sSimplex	m_result;
    Vector3		                            m_normal;
    scalar		                            m_depth;
    typename GJK<ConvexTemplate>::sSV	    m_sv_store[EPA_MAX_VERTICES];
    sFace			                        m_fc_store[EPA_MAX_FACES];
    U			         	                m_nextsv;
    sList			                        m_hull;
    sList			                        m_stock;
    /* Methods		*/
    EPA()
    {
        Initialize();
    }


    static SIMD_INLINE void		bind(sFace* fa,U ea,sFace* fb,U eb)
    {
        fa->e[ea]=(U1)eb;fa->f[ea]=fb;
        fb->e[eb]=(U1)ea;fb->f[eb]=fa;
    }
    static SIMD_INLINE void		append(sList& list,sFace* face)
    {
        face->l[0]	=	0;
        face->l[1]	=	list.root;
        if(list.root) list.root->l[0]=face;
        list.root	=	face;
        ++list.count;
    }
    static SIMD_INLINE void		remove(sList& list,sFace* face)
    {
        if(face->l[1]) face->l[1]->l[0]=face->l[0];
        if(face->l[0]) face->l[0]->l[1]=face->l[1];
        if(face==list.root) list.root=face->l[1];
        --list.count;
    }


    void				Initialize()
    {
        m_status	=	eEpaFailed;
        m_normal	=	Vector3(0,0,0);
        m_depth		=	0;
        m_nextsv	=	0;
        for(U i=0;i<EPA_MAX_FACES;++i)
        {
            append(m_stock,&m_fc_store[EPA_MAX_FACES-i-1]);
        }
    }



    eEpaStatus	Evaluate(GJK<ConvexTemplate>& gjk,const Vector3& guess)
    {
        typename GJK<ConvexTemplate>::sSimplex&	simplex=*gjk.m_simplex;
        if((simplex.rank>1)&&gjk.EncloseOrigin())
        {

            /* Clean up				*/
            while(m_hull.root)
            {
                sFace*	f = m_hull.root;
                remove(m_hull,f);
                append(m_stock,f);
            }
            m_status	=	eEpaValid;
            m_nextsv	=	0;
            /* Orient simplex		*/
            if(gjk.det(	simplex.c[0]->w-simplex.c[3]->w,
                        simplex.c[1]->w-simplex.c[3]->w,
                        simplex.c[2]->w-simplex.c[3]->w)<0)
            {
                Swap(simplex.c[0],simplex.c[1]);
                Swap(simplex.p[0],simplex.p[1]);
            }
            /* Build initial hull	*/
            sFace*	tetra[]={newface(simplex.c[0],simplex.c[1],simplex.c[2],true),
                    newface(simplex.c[1],simplex.c[0],simplex.c[3],true),
                    newface(simplex.c[2],simplex.c[1],simplex.c[3],true),
                    newface(simplex.c[0],simplex.c[2],simplex.c[3],true)};
            if(m_hull.count==4)
            {
                sFace*		best=findbest();
                sFace		outer=*best;
                U			pass=0;
                U			iterations=0;
                bind(tetra[0],0,tetra[1],0);
                bind(tetra[0],1,tetra[2],0);
                bind(tetra[0],2,tetra[3],0);
                bind(tetra[1],1,tetra[3],2);
                bind(tetra[1],2,tetra[2],1);
                bind(tetra[2],2,tetra[3],1);
                m_status=eEpaValid;
                for(;iterations<EPA_MAX_ITERATIONS;++iterations)
                {
                    if(m_nextsv<EPA_MAX_VERTICES)
                    {
                        sHorizon		horizon;
                        typename GJK<ConvexTemplate>::sSV*			w=&m_sv_store[m_nextsv++];
                        bool			valid=true;
                        best->pass	=	(U1)(++pass);
                        gjk.getsupport(best->n,*w);
                        const scalar	wdist=best->n.dot(w->w)-best->d;
                        if(wdist>EPA_ACCURACY)
                        {
                            for(U j=0;(j<3)&&valid;++j)
                            {
                                valid&=expand(	pass,w,
                                        best->f[j],best->e[j],
                                        horizon);
                            }
                            if(valid&&(horizon.nf>=3))
                            {
                                bind(horizon.cf,1,horizon.ff,2);
                                remove(m_hull,best);
                                append(m_stock,best);
                                best=findbest();
                                outer=*best;
                            } else { m_status=eEpaInvalidHull;break; }
                        } else { m_status=eEpaAccuraryReached;break; }
                    } else { m_status=eEpaOutOfVertices;break; }
                }
                const Vector3	projection=outer.n*outer.d;
                m_normal	=	outer.n;
                m_depth		=	outer.d;
                m_result.rank	=	3;
                m_result.c[0]	=	outer.c[0];
                m_result.c[1]	=	outer.c[1];
                m_result.c[2]	=	outer.c[2];
                m_result.p[0]	=	(outer.c[1]->w-projection.cross(outer.c[2]->w-projection)).length();
                m_result.p[1]	=	(outer.c[2]->w-projection.cross(outer.c[0]->w-projection)).length();
                m_result.p[2]	=	(outer.c[0]->w-projection.cross(outer.c[1]->w-projection)).length();
                const scalar	sum=m_result.p[0]+m_result.p[1]+m_result.p[2];
                m_result.p[0]	/=	sum;
                m_result.p[1]	/=	sum;
                m_result.p[2]	/=	sum;
                return(m_status);
            }
        }
        /* Fallback		*/
        m_status	=	eEpaFallBack;
        m_normal	=	-guess;
        const scalar	nl=m_normal.length();
        if(nl>0)
            m_normal	=	m_normal/nl;
        else
            m_normal	=	Vector3(1,0,0);
        m_depth	=	0;
        m_result.rank=1;
        m_result.c[0]=simplex.c[0];
        m_result.p[0]=1;
        return(m_status);
    }




    bool getedgedist(sFace* face, typename GJK<ConvexTemplate>::sSV* a, typename GJK<ConvexTemplate>::sSV* b, scalar& dist)
    {
        const Vector3 ba = b->w - a->w;
        const Vector3 n_ab = (ba).cross(face->n); // Outward facing edge normal direction, on triangle plane
        const scalar a_dot_nab = (a->w).dot(n_ab); // Only care about the sign to determine inside/outside, so not normalization required

        if(a_dot_nab < 0)
        {
            // Outside of edge a->b

            const scalar ba_l2 = ba.length2();
            const scalar a_dot_ba = (a->w).dot(ba);
            const scalar b_dot_ba = (b->w).dot(ba);

            if(a_dot_ba > 0)
            {
                // Pick distance vertex a
                dist = a->w.length();
            }
            else if(b_dot_ba < 0)
            {
                // Pick distance vertex b
                dist = b->w.length();
            }
            else
            {
                // Pick distance to edge a->b
                const scalar a_dot_b = (a->w).dot(b->w);
                dist = SquareRoot(Max((a->w.length2() * b->w.length2() - a_dot_b * a_dot_b) / ba_l2, (scalar)0));
            }

            return true;
        }

        return false;
    }
    sFace*	newface(typename GJK<ConvexTemplate>::sSV* a,typename GJK<ConvexTemplate>::sSV* b,typename GJK<ConvexTemplate>::sSV* c,bool forced)
    {
        if(m_stock.root)
        {
            sFace*	face=m_stock.root;
            remove(m_stock,face);
            append(m_hull,face);
            face->pass	=	0;
            face->c[0]	=	a;
            face->c[1]	=	b;
            face->c[2]	=	c;
            face->n		=	(b->w-a->w).cross(c->w-a->w);
            const scalar	l=face->n.length();
            const bool		v=l>EPA_ACCURACY;

            if(v)
            {
                if(!(getedgedist(face, a, b, face->d) ||
                     getedgedist(face, b, c, face->d) ||
                     getedgedist(face, c, a, face->d)))
                {
                    // Origin projects to the interior of the triangle
                    // Use distance to triangle plane
                    face->d = (a->w.dot(face->n)) / l;
                }

                face->n /= l;
                if(forced || (face->d >= -EPA_PLANE_EPS))
                {
                    return face;
                }
                else
                    m_status=eEpaNonConvex;
            }
            else
                m_status=eEpaDegenerated;

            remove(m_hull, face);
            append(m_stock, face);
            return 0;

        }
        m_status = m_stock.root ? eEpaOutOfVertices : eEpaOutOfFaces;
        return 0;
    }


    sFace*	findbest()
    {
        sFace*		minf=m_hull.root;
        scalar	mind=minf->d*minf->d;
        for(sFace* f=minf->l[1];f;f=f->l[1])
        {
            const scalar	sqd=f->d*f->d;
            if(sqd<mind)
            {
                minf=f;
                mind=sqd;
            }
        }
        return(minf);
    }


    bool expand(U pass,typename GJK<ConvexTemplate>::sSV* w,sFace* f,U e,sHorizon& horizon)
    {
        static const U	i1m3[]={1,2,0};
        static const U	i2m3[]={2,0,1};
        if(f->pass!=pass)
        {
            const U	e1=i1m3[e];
            if( ((f->n.dot(w->w))-f->d)<-EPA_PLANE_EPS)
            {
                sFace*	nf=newface(f->c[e1],f->c[e],w,false);
                if(nf)
                {
                    bind(nf,0,f,e);
                    if(horizon.cf) bind(horizon.cf,1,nf,2); else horizon.ff=nf;
                    horizon.cf=nf;
                    ++horizon.nf;
                    return(true);
                }
            }
            else
            {
                const U	e2=i2m3[e];
                f->pass		=	(U1)pass;
                if(	expand(pass,w,f->f[e1],f->e[e1],horizon)&&
                        expand(pass,w,f->f[e2],f->e[e2],horizon))
                {
                    remove(m_hull,f);
                    append(m_stock,f);
                    return(true);
                }
            }
        }
        return(false);
    }

};


template <typename ConvexTemplate>
static void	Initialize(	const ConvexTemplate& a,
                        const ConvexTemplate& b,
                        rpGjkEpaSolver::sResults& results,
                        MinkowskiDiff<ConvexTemplate>& shape)
{
    /* Results		*/
    results.witnesses[0]	=
    results.witnesses[1]	=	Vector3(0,0,0);
    results.status			=	rpGjkEpaSolver::sResults::Separated;
    /* Shape		*/

    shape.m_world1		=	b.getWorldTransform();//.getBasis().transposeTimes(a.getWorldTransform().getBasis());
    shape.m_world0		=	a.getWorldTransform();//.inverseTimes(b.getWorldTransform());

}


}

//----------------------- Api -----------------------------//
template<typename ConvexTemplate>
bool rpGjkEpaSolver::Distance(const ConvexTemplate &a,
                              const ConvexTemplate &b,
                              const Vector3 &guess, rpGjkEpaSolver::sResults &results)
{

    MinkowskiDiff<ConvexTemplate>     shape(a,b);
    Initialize(a,b,results,shape);
    GJK<ConvexTemplate>			      gjk(a,b);
    eGjkStatus	gjk_status=gjk.Evaluate(shape,guess);
    if(gjk_status==eGjkValid)
    {
        Vector3	w0=Vector3(0,0,0);
        Vector3	w1=Vector3(0,0,0);
        for(U i=0;i<gjk.m_simplex->rank;++i)
        {
            const scalar	p=gjk.m_simplex->p[i];
            w0+=shape.Support( gjk.m_simplex->c[i]->d,0)*p;
            w1+=shape.Support(-gjk.m_simplex->c[i]->d,1)*p;
        }
        results.witnesses[0]	=	a.getWorldTransform()*w0;
        results.witnesses[1]	=	a.getWorldTransform()*w1;
        results.normal			=	w0-w1;
        results.distance		=	results.normal.length();
        results.normal			/=	results.distance>GJK_MIN_DISTANCE?results.distance:1;
        return(true);
    }
    else
    {
        results.status	=	gjk_status==eGjkInside?
                            rpGjkEpaSolver::sResults::Penetrating	:
                            rpGjkEpaSolver::sResults::GJK_Failed	;
        return(false);
    }
}


template<typename ConvexTemplate>
bool rpGjkEpaSolver::Penetration(const ConvexTemplate &a,
                                 const ConvexTemplate &b,
                                 const Vector3 &guess, rpGjkEpaSolver::sResults &results)
{

    MinkowskiDiff<ConvexTemplate>	shape(a,b);
    Initialize(a,b,results,shape);
    GJK<ConvexTemplate>				gjk(a,b);
    eGjkStatus	gjk_status=gjk.Evaluate(shape,-guess);
    switch(gjk_status)
    {
        case	eGjkInside:
        {
            EPA<ConvexTemplate>				epa;
            eEpaStatus	epa_status=epa.Evaluate(gjk,-guess);
            if(epa_status!=eEpaFailed)
            {
                Vector3	w0=Vector3(0,0,0);
                for(U i=0;i<epa.m_result.rank;++i)
                {
                    w0+=shape.Support(epa.m_result.c[i]->d,0)*epa.m_result.p[i];
                }
                results.status			=	rpGjkEpaSolver::sResults::Penetrating;
                results.witnesses[0]	=	a.getWorldTransform()*w0;
                results.witnesses[1]   	=	a.getWorldTransform()*(w0-epa.m_normal*epa.m_depth);
                results.normal			=	-epa.m_normal;
                results.distance		=	-epa.m_depth;
                return(true);
            } else results.status=rpGjkEpaSolver::sResults::EPA_Failed;
        }
        break;
        case	eGjkFailed:
            results.status=rpGjkEpaSolver::sResults::GJK_Failed;
            break;
        default:
        {
        }
    }
    return(false);
}

template<typename ConvexTemplate, typename DistanceInfoTemplate>
int rpGjkEpaSolver::ComputeGjkDistance(const ConvexTemplate &a,
                                       const ConvexTemplate &b,
                                       const rpGjkCollisionDescription &colDesc,
                                             DistanceInfoTemplate &distInfo)
{

    rpGjkEpaSolver::sResults results;
    Vector3 guess = colDesc.m_firstDir;

    bool isSeparated = Distance(a,b,guess, results);
    if (isSeparated)
    {
        distInfo.m_penetrationDepth   = results.distance;
        distInfo.pALocal  = results.witnesses[0];
        distInfo.pBLocal   = results.witnesses[1];
        distInfo.m_normal = results.normal;


        return 0;
    }

    return -1;
}


template<typename ConvexTemplate>
bool rpGjkEpaSolver::SignedDistance(const ConvexTemplate &a,
                                    const ConvexTemplate &b,
                                    const Vector3 &guess, rpGjkEpaSolver::sResults &results)
{

    if(!Distance(a,b,guess,results))
    {
        return(Penetration(a,b,guess,results));
    }
    else
    {
        return(true);
    }
}





/* Symbols cleanup		*/

#undef GJK_MAX_ITERATIONS
#undef GJK_ACCURARY
#undef GJK_MIN_DISTANCE
#undef GJK_DUPLICATED_EPS
#undef GJK_SIMPLEX2_EPS
#undef GJK_SIMPLEX3_EPS
#undef GJK_SIMPLEX4_EPS

#undef EPA_MAX_VERTICES
#undef EPA_MAX_FACES
#undef EPA_MAX_ITERATIONS
#undef EPA_ACCURACY
#undef EPA_FALLBACK
#undef EPA_PLANE_EPS
#undef EPA_INSIDE_EPS



} /* namespace real_physics */

#endif /* SOURCE_ENGIE_COLLISION_NARROWPHASE_GJKEPA_H_ */
