/**
 * ----------------------------------------------------------------------------
 * This source file is part of the ODE4J library (ported to
 * Java from the GIMPACT Library).
 * 
 * For the latest info on ODE4J, see http://www.ode4j.org/
 * For the latest info on GIMPACT, see http://gimpact.sourceforge.net/
 * 
 * Copyright of GIMPACT (c) 2006 Francisco Leon. C.C. 80087371.
 * email: projectileman@yahoo.com
 * Copyright of ODE4J (c) 2009-2014 Tilmann Zäschke.
 * email: ode4j.gmx.de
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this library in the
 *       file GIMPACT-LICENSE-LGPL.TXT and LICENSE.TXT.
 *   (2) The BSD-style license that is included with this library in
 *       the file GIMPACT-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-BSD.TXT, LICENSE.TXT and 
 * ODE4J-LICENSE-BSD.TXT for more details.
 * 
 * ----------------------------------------------------------------------------
 */
package org.ode4j.ode.internal.gimpact;

import org.ode4j.ode.internal.cpp4j.java.RefBoolean;
import org.ode4j.ode.internal.cpp4j.java.RefFloat;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

import static org.ode4j.ode.internal.gimpact.GimGeometry.*;

/**
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
public class GimTriCollision {

	static final int MAX_TRI_CLIPPING = 8;

	/** Clips a polygon by a plane. */
	//#define PLANE_CLIP_POLYGON(plane,polygon_points,polygon_point_count,clipped,clipped_count,max_clipped) \
	static int PLANE_CLIP_POLYGON(final vec4f plane, final vec3f[] polygon_points, 
			final int polygon_point_count, final vec3f[] clipped, final int max_clipped) 
	{ 
	    int clipped_count = 0; 
	    int  _vi, _prevclassif=32000; 
	    boolean _classif;
		float _d; 
		for (int _i=0; _i<=polygon_point_count; _i++) 
		{ 
			_vi = _i%polygon_point_count; 
			_d = DISTANCE_PLANE_POINT(plane,polygon_points[_vi]); 
			_classif = _d>G_EPSILON; //TODO remove this variable TZ
			if (!_classif) 
			{ 
				if(_prevclassif==1) 
				{
					if(clipped_count<max_clipped) //TODO TZ change forloop to end at min(max_clipped, polygon_cnt) ?
					{
						PLANE_CLIP_SEGMENT(polygon_points[_i-1],polygon_points[_vi],plane,clipped[clipped_count]); 
						clipped_count++; 
					} 
				} 
				if(clipped_count<max_clipped&&_i<polygon_point_count) 
				{ 
					VEC_COPY(clipped[clipped_count],polygon_points[_vi]); 
					clipped_count++; //TZ TODO? weird: so this can be incremented twice in if(!_classif) ?
				} 
				_prevclassif = 0;
			} 
			else 
			{ 
				if(_prevclassif==0) 
				{ 
					if(clipped_count<max_clipped) 
					{ 
						PLANE_CLIP_SEGMENT(polygon_points[_i-1],polygon_points[_vi],plane,clipped[clipped_count]); 
						clipped_count++; 
					} 
				}
				_prevclassif = 1;
			} 
			//_prevclassif = _classif ? 1:0; 
		}
		return clipped_count;
	}


	static class GIM_TRIPLANES_CACHE
	{
	    /*!
	    Planes are:
	    0 : Face normal plane (0,3)
	    1 : Edge 1 plane (4,7)
	    2 : Edge 2 plane (8,11)
	    3 : Edge 3 plane (12,15)
	    */
	    final vec4f[] m_planes = {new vec4f(), new vec4f(), new vec4f(), new vec4f() };//[4];
	};
	//typedef struct _GIM_TRIPLANES_CACHE GIM_TRIPLANES_CACHE;


	static class GIM_TRIANGLE_DATA
	{
	    final vec3f[] m_vertices = { new vec3f(), new vec3f(), new vec3f() };
	    final GIM_TRIPLANES_CACHE m_planes = new GIM_TRIPLANES_CACHE();
	};
	//typedef struct _GIM_TRIANGLE_DATA GIM_TRIANGLE_DATA;

	/** tri_data is a GIM_TRIANGLE_DATA. */
	//#define GIM_CALC_TRIANGLE_DATA_PLANES(tri_data)\
	void GIM_CALC_TRIANGLE_DATA_PLANES(GIM_TRIANGLE_DATA tri_data)
	{
	        TRIANGLE_PLANE((tri_data).m_vertices[0],(tri_data).m_vertices[1],(tri_data).m_vertices[2],(tri_data).m_planes.m_planes[0]);
	        EDGE_PLANE((tri_data).m_vertices[0],(tri_data).m_vertices[1],((tri_data).m_planes.m_planes[0]),((tri_data).m_planes.m_planes[1]));
	        EDGE_PLANE((tri_data).m_vertices[1],(tri_data).m_vertices[2],((tri_data).m_planes.m_planes[0]),((tri_data).m_planes.m_planes[2]));
	        EDGE_PLANE((tri_data).m_vertices[2],(tri_data).m_vertices[0],((tri_data).m_planes.m_planes[0]), ((tri_data).m_planes.m_planes[3]));
	}

	
	/** Structure for collision. */
	static class GIM_TRIANGLE_CONTACT_DATA
	{
//	    GREAL m_penetration_depth;
//	    GUINT32 m_point_count;
//	    vec3f m_separating_normal;
//	    vec3f m_points[MAX_TRI_CLIPPING];
	    float m_penetration_depth;
	    int m_point_count;
	    final vec3f m_separating_normal = new vec3f();
	    final vec3f[] m_points = new vec3f[MAX_TRI_CLIPPING];
	    {
	    	for (int i = 0; i < m_points.length; i++) m_points[i] = new vec3f();
	    }
	}
	//typedef struct _GIM_TRIANGLE_CONTACT_DATA GIM_TRIANGLE_CONTACT_DATA;

	/** Structure for collision. */
	public static class GIM_TRIANGLE_RAY_CONTACT_DATA
	{
//	    GREAL u;
//	    GREAL v;
//	    GREAL tparam;
//	    GUINT32 m_face_id;
//	    vec3f m_point;
//	    vec3f m_normal;
	    float u;
	    float v;
	    float tparam;
	    int m_face_id;
	    final vec3f m_point = new vec3f();
	    final vec3f m_normal = new vec3f();
	    public vec3f getPoint() { return m_point; }
	    public vec3f getNormal() { return m_normal; }
	    public float getU() { return u; }
	    public float getV() { return v; }
	    public float getTParam() { return tparam; }
	    public int getFaceID() { return m_face_id; }

		public GIM_TRIANGLE_RAY_CONTACT_DATA() {}
	}
	//typedef struct _GIM_TRIANGLE_RAY_CONTACT_DATA GIM_TRIANGLE_RAY_CONTACT_DATA;



	//! Fast but inacurate conservative Triangle Triangle overlapping test
//	int gim_triangle_triangle_overlap_fast(
//								GIM_TRIANGLE_DATA *tri1,
//								GIM_TRIANGLE_DATA *tri2);
//	int gim_triangle_triangle_overlap_fast(  //TZ TODO?? apparently there is no implementation.
//			GIM_TRIANGLE_DATA tri1,
//			GIM_TRIANGLE_DATA tri2);


	/**
	 * Finds the contact points from a collision of two triangles.
	 * Returns the contact points, the penetration depth and the separating normal of the collision
	 * between two triangles. The normal is pointing toward triangle 1 from triangle 2.
	 */
	boolean gim_triangle_triangle_collision(
								GIM_TRIANGLE_DATA tri1,
								GIM_TRIANGLE_DATA tri2,
								GIM_TRIANGLE_CONTACT_DATA contact_data) {
		return GimTrimeshTrimeshCol.gim_triangle_triangle_collision(tri1, tri2, contact_data);
	}

	//Ray triangle


	/**
	 * Ray triagle.
	 * 
	 * Solve the System for u,v parameters:

		u*axe1[i1] + v*axe2[i1] = vecproj[i1]
		u*axe1[i2] + v*axe2[i2] = vecproj[i2]

		sustitute:
		v = (vecproj[i2] - u*axe1[i2])/axe2[i2]

		then the first equation in terms of 'u':

		--> u*axe1[i1] + ((vecproj[i2] - u*axe1[i2])/axe2[i2])*axe2[i1] = vecproj[i1]

		--> u*axe1[i1] + vecproj[i2]*axe2[i1]/axe2[i2] - u*axe1[i2]*axe2[i1]/axe2[i2] = vecproj[i1]

		--> u*(axe1[i1]  - axe1[i2]*axe2[i1]/axe2[i2]) = vecproj[i1] - vecproj[i2]*axe2[i1]/axe2[i2]

		--> u*((axe1[i1]*axe2[i2]  - axe1[i2]*axe2[i1])/axe2[i2]) = (vecproj[i1]*axe2[i2] - vecproj[i2]*axe2[i1])/axe2[i2]

		--> u*(axe1[i1]*axe2[i2]  - axe1[i2]*axe2[i1]) = vecproj[i1]*axe2[i2] - vecproj[i2]*axe2[i1]

		--> u = (vecproj[i1]*axe2[i2] - vecproj[i2]*axe2[i1]) /(axe1[i1]*axe2[i2]  - axe1[i2]*axe2[i1])

	if 0.0<= u+v <=1.0 then they are inside of triangle

		*/
	//#define TRIANGLE_GET_UVPARAMETERS(point,vec1,vec2,vec3,tri_plane,u,v,outside)\
	static void TRIANGLE_GET_UVPARAMETERS(final vec3f point, final vec3f vec1, 
			final vec3f vec2, final vec3f vec3, final vec4f tri_plane, 
			final RefFloat u, final RefFloat v, final RefBoolean outside)
	{
		vec3f _axe1 = new vec3f(), _axe2 = new vec3f(), _vecproj = new vec3f();
		VEC_DIFF(_axe1,vec2,vec1);
		VEC_DIFF(_axe2,vec3,vec1);
		VEC_DIFF(_vecproj,point,vec1);
		RefInt _i1 = new RefInt(), _i2 = new RefInt();
		PLANE_MINOR_AXES(tri_plane, _i1, _i2);
		if(Math.abs(_axe2.f[_i2.i])<G_EPSILON)
		{
			u.d = (_vecproj.f[_i2.i]*_axe2.f[_i1.i] - _vecproj.f[_i1.i]*_axe2.f[_i2.i]) /(_axe1.f[_i2.i]*_axe2.f[_i1.i]  - _axe1.f[_i1.i]*_axe2.f[_i2.i]);
			v.d = (_vecproj.f[_i1.i] - u.d*_axe1.f[_i1.i])/_axe2.f[_i1.i];
		}
		else
		{
			u.d = (_vecproj.f[_i1.i]*_axe2.f[_i2.i] - _vecproj.f[_i2.i]*_axe2.f[_i1.i]) /(_axe1.f[_i1.i]*_axe2.f[_i2.i]  - _axe1.f[_i2.i]*_axe2.f[_i1.i]);
			v.d = (_vecproj.f[_i2.i] - u.d*_axe1.f[_i2.i])/_axe2.f[_i2.i];
		}
		if(u.d<-G_EPSILON)
		{
			outside.b = true;
		}
		else if(v.d<-G_EPSILON)
		{
			outside.b = true;
		}
		else
		{
			float sumuv;
			sumuv = u.d+v.d;
			if(sumuv<-G_EPSILON)
			{
				outside.b = true;
			}
			else if(sumuv-1.0f>G_EPSILON)
			{
				outside.b = true;
			}
			else
			{
				outside.b = false;
			}
		}
	}

	/** Finds the collision of a ray and a triangle. */
	//#define RAY_TRIANGLE_INTERSECTION(vOrigin,vDir,vec1,vec2,vec3,tri_plane,pout,u,v,tparam,tmax,does_intersect)\
	static void RAY_TRIANGLE_INTERSECTION(vec3f vOrigin, vec3f vDir, vec3f vec1, vec3f vec2, vec3f vec3,
			vec4f tri_plane, vec3f pout, RefFloat u, RefFloat v, RefFloat tparam, final float tmax, RefBoolean does_intersect)
	{
		RAY_PLANE_COLLISION(tri_plane,vDir,vOrigin,pout,tparam,does_intersect);
		if(does_intersect.b)
		{
	        if(tparam.d<-G_EPSILON||tparam.d>tmax+G_EPSILON)
	        {
	            does_intersect.b = false;
	        }
	        else
	        {
	            TRIANGLE_GET_UVPARAMETERS(pout,vec1,vec2,vec3,tri_plane,u,v,does_intersect);
	            does_intersect.b = !does_intersect.b;
	        }
		}
	}



	//#define FABS(x) (float(fabs(x)))        /* implement as is fastest on your machine */
	private static float FABS(float x) { return Math.abs(x); }        /* implement as is fastest on your machine */

	/* some macros */

	//#define CLASSIFY_TRIPOINTS_BY_FACE(v1,v2,v3,faceplane,out_of_face)\
	private int CLASSIFY_TRIPOINTS_BY_FACE(final vec3f v1, final vec3f v2, final vec3f v3,
			final vec4f faceplane, final vec3f _distances)
	{   
		_distances.f[0] = DISTANCE_PLANE_POINT(faceplane,v1);
	    _distances.f[1] =  _distances.f[0] * DISTANCE_PLANE_POINT(faceplane,v2);
	    _distances.f[2] =  _distances.f[0] * DISTANCE_PLANE_POINT(faceplane,v3); 
		if(_distances.f[1]>0.0f && _distances.f[2]>0.0f)
		{
		    return 1;
		}
		else
		{
		    return 0;
		}
	}

	/* sort so that a<=b */
	//#define SORT(a,b)       \
//	private void SORT(RefFloat a, RefFloat b) {
//	             if(a.d>b.d)    
//	             {          
//	               float c; 
//	               c=a.d;     
//	               a.d=b.d;     
//	               b.d=c;     
//	             }
//	}
	private void SORT(float[] f) {
        if(f[0] > f[1])    
        {          
          float c; 
          c = f[0];     
          f[0] = f[1];     
          f[1] = c;     
        }
}

	/** this edge to edge test is based on Franlin Antonio's gem:
	   "Faster Line Segment Intersection", in Graphics Gems III,
	   pp. 199-202 */
	//#define EDGE_EDGE_TEST(V0,U0,U1)                      \
	private boolean EDGE_EDGE_TEST(final vec3f V0,final vec3f U0,final vec3f U1,
			final float Ax, final float Ay, final int i0, final int i1) {
	  float Bx=U0.f[i0]-U1.f[i0];                                   
	  float By=U0.f[i1]-U1.f[i1];                                   
	  float Cx=V0.f[i0]-U0.f[i0];                                   
	  float Cy=V0.f[i1]-U0.f[i1];                                   
	  float f=Ay*Bx-Ax*By;                                      
	  float d=By*Cx-Bx*Cy;                                      
	  if((f>0 && d>=0 && d<=f) || (f<0 && d<=0 && d>=f))  
	  {                                                   
	    float e=Ax*Cy-Ay*Cx;                                    
	    if(f>0)                                           
	    {                                                 
	      if(e>=0 && e<=f) return true;                     
	    }                                                 
	    else                                              
	    {                                                 
	      if(e<=0 && e>=f) return true;                     
	    }                                                 
	  }
	  return false;//TZ
	}

	//#define EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2) \
	private boolean EDGE_AGAINST_TRI_EDGES(final vec3f V0, final vec3f V1,
			final vec3f U0, final vec3f U1, final vec3f U2, final int i0, final int i1)
	{                                              
	  float Ax,Ay;//,Bx,By,Cx,Cy,e,d,f;              
	  Ax=V1.f[i0]-V0.f[i0];                            
	  Ay=V1.f[i1]-V0.f[i1];                            
	  /* test edge U0,U1 against V0,V1 */          
	  if (EDGE_EDGE_TEST(V0,U0,U1,Ax,Ay,i0,i1)) return true;                    
	  /* test edge U1,U2 against V0,V1 */          
	  if (EDGE_EDGE_TEST(V0,U1,U2,Ax,Ay,i0,i1)) return true;                    
	  /* test edge U2,U1 against V0,V1 */          
	  if (EDGE_EDGE_TEST(V0,U2,U0,Ax,Ay,i0,i1)) return true;
	  return false; //TZ
	}

	//#define POINT_IN_TRI(V0,U0,U1,U2)           \
//	private void POINT_IN_TRI(final vec3f V0, 
//			final vec3f U0, final vec3f U1, final vec3f U2) {
//	{                                           
//	  float a,b,c,d0,d1,d2;                    
//	  /* is T1 completly inside T2? */          
//	  /* check if V0 is inside tri(U0,U1,U2) */ 
//	  a=U1[i1]-U0[i1];                          
//	  b=-(U1[i0]-U0[i0]);                       
//	  c=-a*U0[i0]-b*U0[i1];                     
//	  d0=a*V0[i0]+b*V0[i1]+c;                   
//	                                            
//	  a=U2[i1]-U1[i1];                          
//	  b=-(U2[i0]-U1[i0]);                       
//	  c=-a*U1[i0]-b*U1[i1];                     
//	  d1=a*V0[i0]+b*V0[i1]+c;                   
//	                                            
//	  a=U0[i1]-U2[i1];                          
//	  b=-(U0[i0]-U2[i0]);                       
//	  c=-a*U2[i0]-b*U2[i1];                     
//	  d2=a*V0[i0]+b*V0[i1]+c;                   
//	  if(d0*d1>0.0)                             
//	  {                                         
//	    if(d0*d2>0.0) return 1;                
//	  }                                         
//	}

//	int coplanar_tri_tri(GIM_TRIANGLE_DATA *tri1,
//	                    GIM_TRIANGLE_DATA *tri2)
	int coplanar_tri_tri(GIM_TRIANGLE_DATA tri1,
            GIM_TRIANGLE_DATA tri2)
	{
	   final RefInt i0 = new RefInt(), i1 = new RefInt();
	   /* first project onto an axis-aligned plane, that maximizes the area */
	   /* of the triangles, compute indices: i0,i1. */
	   PLANE_MINOR_AXES(tri1.m_planes.m_planes[0], i0, i1);

	    /* test all edges of triangle 1 against the edges of triangle 2 */
	    if( EDGE_AGAINST_TRI_EDGES(tri1.m_vertices[0],tri1.m_vertices[1],
	    		tri2.m_vertices[0],tri2.m_vertices[1],tri2.m_vertices[2],i0.i,i1.i)) return 1;
	    if( EDGE_AGAINST_TRI_EDGES(tri1.m_vertices[1],tri1.m_vertices[2],
	    		tri2.m_vertices[0],tri2.m_vertices[1],tri2.m_vertices[2],i0.i,i1.i)) return 1;
	    if( EDGE_AGAINST_TRI_EDGES(tri1.m_vertices[2],tri1.m_vertices[0],
	    		tri2.m_vertices[0],tri2.m_vertices[1],tri2.m_vertices[2],i0.i,i1.i)) return 1;

	    /* finally, test if tri1 is totally contained in tri2 or vice versa */
	    i0.i = POINT_IN_HULL_TZ(tri1.m_vertices[0],tri2.m_planes.m_planes, 1,3);
	    if(i0.i==0) return 1;

	    i0.i = POINT_IN_HULL_TZ(tri2.m_vertices[0],tri1.m_planes.m_planes,1,3);
	    if(i0.i==0) return 1;

	    return 0;
	}



	//#define NEWCOMPUTE_INTERVALS(VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,A,B,C,X0,X1) \
	//TZ moved down to the caller.
//	private boolean NEWCOMPUTE_INTERVALS(final float VV0, final float VV1, final float VV2,
//			final float D0, final float D1, final float D2,
//			final float D0D1, final float D0D2,
//			final float A, final float B, final float C,
//			final float X0, final float X1,
//			GIM_TRIANGLE_DATA tri1, GIM_TRIANGLE_DATA tri2)
//	{ 
//		if(D0D1>0.0f) { 
//			/* here we know that D0D2<=0.0 */ 
//			/* that is D0, D1 are on the same side, D2 on the other or on the plane */ 
//			A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; 
//		} else if(D0D2>0.0f) { 
//			/* here we know that d0d1<=0.0 */ 
//			A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; 
//		} else if(D1*D2>0.0f || D0!=0.0f) { 
//			/* here we know that d0d1<=0.0 or that D0!=0.0 */ 
//			A=VV0; B=(VV1-VV0)*D0; C=(VV2-VV0)*D0; X0=D0-D1; X1=D0-D2; 
//		} else if(D1!=0.0f) { 
//			A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; 
//		} else if(D2!=0.0f) { 
//			A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; 
//		} else { 
//			/* triangles are coplanar */ 
//			return coplanar_tri_tri(tri1,tri2); 
//		}
//	}



	/** Fast Triangle Triangle overlapping test. */
//	int gim_triangle_triangle_overlap(
//								GIM_TRIANGLE_DATA *tri1,
//								GIM_TRIANGLE_DATA *tri2)
	int gim_triangle_triangle_overlap(
			GIM_TRIANGLE_DATA tri1,
			GIM_TRIANGLE_DATA tri2)
	{
	    vec3f _distances = new vec3f();
	    int out_of_face;
	    out_of_face = CLASSIFY_TRIPOINTS_BY_FACE(tri1.m_vertices[0],tri1.m_vertices[1],tri1.m_vertices[2],
	    		tri2.m_planes.m_planes[0], _distances);
	    if(out_of_face==1) return 0;

	    out_of_face = CLASSIFY_TRIPOINTS_BY_FACE(tri2.m_vertices[0],tri2.m_vertices[1],tri2.m_vertices[2],
	    		tri1.m_planes.m_planes[0], _distances);
	    if(out_of_face==1) return 0;


	    float du0=0,du1=0,du2=0,dv0=0,dv1=0,dv2=0;
	    vec3f D = new vec3f();//float D[3];
	    float[] isect1=new float[2], isect2 = new float[2];  //TODO optimize? TZ
	    float du0du1=0,du0du2=0,dv0dv1=0,dv0dv2=0;
	    short index;
	    float vp0,vp1,vp2;
	    float up0,up1,up2;
	    float bb,cc,max;

	    /* compute direction of intersection line */
	    VEC_CROSS(D,tri1.m_planes.m_planes[0],tri2.m_planes.m_planes[0]);

	    /* compute and index to the largest component of D */
	    max=FABS(D.f[0]);
	    index=0;
	    bb=FABS(D.f[1]);
	    cc=FABS(D.f[2]);
	    if(bb>max) { max=bb; index=1; }
	    if(cc>max) { max=cc; index=2; }

	     /* this is the simplified projection onto L*/
	     vp0= tri1.m_vertices[0].f[index];
	     vp1= tri1.m_vertices[1].f[index];
	     vp2= tri1.m_vertices[2].f[index];

	     up0= tri2.m_vertices[0].f[index];
	     up1= tri2.m_vertices[1].f[index];
	     up2= tri2.m_vertices[2].f[index];

	    /* compute interval for triangle 1 */
	    float a,b,c,x0,x1;
	    //NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1,tri1,tri2);
	    {
	    	final float VV0 = vp0, VV1 = vp1, VV2 = vp2;
	    	final float D0 = dv0, D1 = dv1, D2 = dv2;
			final float D0D1 = dv0dv1, D0D2 = dv0dv2;
			if(D0D1>0.0f) { 
				/* here we know that D0D2<=0.0 */ 
				/* that is D0, D1 are on the same side, D2 on the other or on the plane */ 
				a=VV2; b=(VV0-VV2)*D2; c=(VV1-VV2)*D2; x0=D2-D0; x1=D2-D1; 
			} else if(D0D2>0.0f) { 
				/* here we know that d0d1<=0.0 */ 
				a=VV1; b=(VV0-VV1)*D1; c=(VV2-VV1)*D1; x0=D1-D0; x1=D1-D2; 
			} else if(D1*D2>0.0f || D0!=0.0f) { 
				/* here we know that d0d1<=0.0 or that D0!=0.0 */ 
				a=VV0; b=(VV1-VV0)*D0; c=(VV2-VV0)*D0; x0=D0-D1; x1=D0-D2; 
			} else if(D1!=0.0f) { 
				a=VV1; b=(VV0-VV1)*D1; c=(VV2-VV1)*D1; x0=D1-D0; x1=D1-D2; 
			} else if(D2!=0.0f) { 
				a=VV2; b=(VV0-VV2)*D2; c=(VV1-VV2)*D2; x0=D2-D0; x1=D2-D1; 
			} else { 
				/* triangles are coplanar */ 
				return coplanar_tri_tri(tri1,tri2); 
			}
	    }
	    
	    

	    /* compute interval for triangle 2 */
	    float d,e,f,y0,y1;
	    //NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1,tri1,tri2);
	    {
	    	final float VV0 = up0, VV1 = up1, VV2 = up2;
	    	final float D0 = du0, D1 = du1, D2 = du2;
			final float D0D1 = du0du1, D0D2 = du0du2;
			if(D0D1>0.0f) { 
				/* here we know that D0D2<=0.0 */ 
				/* that is D0, D1 are on the same side, D2 on the other or on the plane */ 
				d=VV2; e=(VV0-VV2)*D2; f=(VV1-VV2)*D2; y0=D2-D0; y1=D2-D1; 
			} else if(D0D2>0.0f) { 
				/* here we know that d0d1<=0.0 */ 
				d=VV1; e=(VV0-VV1)*D1; f=(VV2-VV1)*D1; y0=D1-D0; y1=D1-D2; 
			} else if(D1*D2>0.0f || D0!=0.0f) { 
				/* here we know that d0d1<=0.0 or that D0!=0.0 */ 
				d=VV0; e=(VV1-VV0)*D0; f=(VV2-VV0)*D0; y0=D0-D1; y1=D0-D2; 
			} else if(D1!=0.0f) { 
				d=VV1; e=(VV0-VV1)*D1; f=(VV2-VV1)*D1; y0=D1-D0; y1=D1-D2; 
			} else if(D2!=0.0f) { 
				d=VV2; e=(VV0-VV2)*D2; f=(VV1-VV2)*D2; y0=D2-D0; y1=D2-D1; 
			} else { 
				/* triangles are coplanar */ 
				return coplanar_tri_tri(tri1,tri2); 
			}
	    }

	    float xx,yy,xxyy,tmp;
	    xx=x0*x1;
	    yy=y0*y1;
	    xxyy=xx*yy;

	    tmp=a*xxyy;
	    isect1[0]=tmp+b*x1*yy;
	    isect1[1]=tmp+c*x0*yy;

	    tmp=d*xxyy;
	    isect2[0]=tmp+e*xx*y1;
	    isect2[1]=tmp+f*xx*y0;

	    SORT(isect1);//SORT(isect1[0],isect1[1]);
	    SORT(isect2);//SORT(isect2[0],isect2[1]);

	    if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return 0;
	    return 1;
	}

	private GimTriCollision() {}
}
