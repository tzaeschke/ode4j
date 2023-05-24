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

import org.ode4j.math.DVector3;
import org.ode4j.ode.internal.cpp4j.java.FloatArray;
import org.ode4j.ode.internal.cpp4j.java.RefBoolean;
import org.ode4j.ode.internal.cpp4j.java.RefFloat;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

/**
 * From gim_geometry.h .
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
public class GimGeometry extends GimMath {

	private static final int M(int y, int x) { return y*4 + x; }
	

	/**
	 * Basic types and constants for geometry
	 */

	//! Integer vector 2D
	//	//typedef GINT32 vec2i[2];
	//	static class vec2i { int[] i = new int[2]; }
	//	//! Integer vector 3D
	//	//typedef GINT32 vec3i[3];
	//	static class vec3i { int[] i = new int[3]; }
	//	//! Integer vector 4D
	//	//typedef GINT32 vec4i[4];
	//	static class vec4i { int[] i = new int[4]; }

	//! Float vector 2D
	//typedef GREAL vec2f[2];
	static class vec2f { float[] f = new float[2]; }
	//! Float vector 3D
	//typedef GREAL vec3f[3];
	public static class vec3f {
		public float[] f = new float[3];
		public vec3f() {}
	}
	//! Float vector 4D
	//typedef GREAL vec4f[4];
	public static class vec4f {
		public float[] f = new float[4];
		public vec4f() {}
	}

	//! Matrix 2D, row ordered
	//typedef GREAL mat2f.f[M(2,2)];
	static class mat2f { float[] f = new float[4]; }
	//! Matrix 3D, row ordered
	//typedef GREAL mat3f.f[M(3,3)];
	static class mat3f { float[] f = new float[9]; }
	//! Matrix 4D, row ordered
	//typedef GREAL mat4f[4][4];
	public static class mat4f {
		public float[] f = new float[16];
		public mat4f() {}
	}

	//! Quaternion
	//typedef GREAL quatf[4];
	// static class quatf { float[] f = new float[4]; }

	//! Axis aligned box
	public static class aabb3f{
	    public float minX;
	    public float maxX;
	    public float minY;
	    public float maxY;
	    public float minZ;
	    public float maxZ;
		public aabb3f() {}

		public void set(double minX, double maxX, double minY, double maxY, double minZ, double maxZ) {
			this.minX = (float) minX;
			this.maxX = (float) maxX;
			this.minY = (float) minY;
			this.maxY = (float) maxY;
			this.minZ = (float) minZ;
			this.maxZ = (float) maxZ;
		}
	}
	//typedef struct _aabb3f aabb3f;
	//! @}


	/**
	 * Operations for vectors : vec2f,vec3f and vec4f
	 */

	//! Zero out a 2D vector
	static void VEC_ZERO_2(vec2f a)
	{
	   (a).f[0] = (a).f[1] = 0.0f;
	}


	//! Zero out a 3D vector
	static void VEC_ZERO(vec3f a)
	{
	   (a).f[0] = (a).f[1] = (a).f[2] = 0.0f;
	}


	/// Zero out a 4D vector
	static void VEC_ZERO_4(vec4f a)
	{
	   (a).f[0] = (a).f[1] = (a).f[2] = (a).f[3] = 0.0f;
	}


	/// Vector copy
	static void VEC_COPY_2(vec2f b, vec2f a)
	{						
	   (b).f[0] = (a).f[0];				
	   (b).f[1] = (a).f[1];				
	}


	/** Copy 3D vector from <tt>a</tt> to <tt>b</tt>. */
	static void VEC_COPY(vec3f b, vec3f a)
	{						
	   (b).f[0] = (a).f[0];				
	   (b).f[1] = (a).f[1];				
	   (b).f[2] = (a).f[2];				
	}
	static void VEC_COPY(vec3f b, vec4f a)
	{						
	   (b).f[0] = (a).f[0];				
	   (b).f[1] = (a).f[1];				
	   (b).f[2] = (a).f[2];				
	}
	static void VEC_COPY(vec4f b, vec3f a)
	{						
	   (b).f[0] = (a).f[0];				
	   (b).f[1] = (a).f[1];				
	   (b).f[2] = (a).f[2];				
	}
	static void VEC_COPY(DVector3 b, vec3f a) {
		b.set(a.f[0], a.f[1], a.f[2]);
	}


	/// Copy 4D vector
	static void VEC_COPY_4(vec4f b, vec4f a)
	{						
	   (b).f[0] = (a).f[0];				
	   (b).f[1] = (a).f[1];				
	   (b).f[2] = (a).f[2];				
	   (b).f[3] = (a).f[3];				
	}


	/// Vector difference
	static void VEC_DIFF_2(vec2f v21, vec2f v2, vec2f v1)
	{						
	   (v21).f[0] = (v2).f[0] - (v1).f[0];		
	   (v21).f[1] = (v2).f[1] - (v1).f[1];		
	}


	/// Vector difference
	static void VEC_DIFF(vec3f v21, vec3f v2, vec3f v1)
	{						
	   (v21).f[0] = (v2).f[0] - (v1).f[0];		
	   (v21).f[1] = (v2).f[1] - (v1).f[1];		
	   (v21).f[2] = (v2).f[2] - (v1).f[2];		
	}
	static void VEC_DIFF(vec4f v21, vec3f v2, vec3f v1)
	{						
	   (v21).f[0] = (v2).f[0] - (v1).f[0];		
	   (v21).f[1] = (v2).f[1] - (v1).f[1];		
	   (v21).f[2] = (v2).f[2] - (v1).f[2];		
	}


	/// Vector difference
	static final void VEC_DIFF_4(vec4f v21, vec4f v2, vec4f v1)			
	{						
	   (v21).f[0] = (v2).f[0] - (v1).f[0];		
	   (v21).f[1] = (v2).f[1] - (v1).f[1];		
	   (v21).f[2] = (v2).f[2] - (v1).f[2];		
	   (v21).f[3] = (v2).f[3] - (v1).f[3];		
	}


	/// Vector sum
	static final void VEC_SUM_2(vec2f v21, vec2f v2, vec2f v1)			
	{						
	   (v21).f[0] = (v2).f[0] + (v1).f[0];		
	   (v21).f[1] = (v2).f[1] + (v1).f[1];		
	}


	/// Vector sum
	static final void VEC_SUM(vec3f v21, vec3f v2, vec3f v1)			
	{						
	   (v21).f[0] = (v2).f[0] + (v1).f[0];		
	   (v21).f[1] = (v2).f[1] + (v1).f[1];		
	   (v21).f[2] = (v2).f[2] + (v1).f[2];		
	}


	/// Vector sum
	static final void VEC_SUM_4(vec4f v21, vec4f v2, vec4f v1)			
	{						
	   (v21).f[0] = (v2).f[0] + (v1).f[0];		
	   (v21).f[1] = (v2).f[1] + (v1).f[1];		
	   (v21).f[2] = (v2).f[2] + (v1).f[2];		
	   (v21).f[3] = (v2).f[3] + (v1).f[3];		
	}


	/// scalar times vector
	static final void VEC_SCALE_2(vec2f c, final float a, vec2f b)			
	{						
	   (c).f[0] = (a)*(b).f[0];				
	   (c).f[1] = (a)*(b).f[1];				
	}


	/// scalar times vector
	static final void VEC_SCALE(vec3f c, final float a, vec3f b)			
	{						
	   (c).f[0] = (a)*(b).f[0];				
	   (c).f[1] = (a)*(b).f[1];				
	   (c).f[2] = (a)*(b).f[2];				
	}
	static final void VEC_SCALE(vec3f c, final float a, vec4f b)			
	{						
	   (c).f[0] = (a)*(b).f[0];				
	   (c).f[1] = (a)*(b).f[1];				
	   (c).f[2] = (a)*(b).f[2];				
	}


	/// scalar times vector
	static final void VEC_SCALE_4(vec4f c, final float a, vec4f b)			
	{						
	   (c).f[0] = (a)*(b).f[0];				
	   (c).f[1] = (a)*(b).f[1];				
	   (c).f[2] = (a)*(b).f[2];				
	   (c).f[3] = (a)*(b).f[3];				
	}


	/// accumulate scaled vector
	static final void VEC_ACCUM_2(vec2f c, final float a, vec2f b)			
	{						
	   (c).f[0] += (a)*(b).f[0];			
	   (c).f[1] += (a)*(b).f[1];			
	}


	/// accumulate scaled vector
	static final void VEC_ACCUM(vec3f c, final float a, vec3f b)			
	{						
	   (c).f[0] += (a)*(b).f[0];			
	   (c).f[1] += (a)*(b).f[1];			
	   (c).f[2] += (a)*(b).f[2];			
	}


	/// accumulate scaled vector
	static final void VEC_ACCUM_4(vec4f c, final float a, vec4f b)			
	{						
	   (c).f[0] += (a)*(b).f[0];			
	   (c).f[1] += (a)*(b).f[1];			
	   (c).f[2] += (a)*(b).f[2];			
	   (c).f[3] += (a)*(b).f[3];			
	}


	/// Vector dot product
	static final float VEC_DOT_2(vec2f a, vec2f b) { return ((a).f[0]*(b).f[0] + (a).f[1]*(b).f[1]); }


	/// Vector dot product
	static final float VEC_DOT(vec3f a, vec3f b) { return a.f[0]*b.f[0] + a.f[1]*b.f[1] + a.f[2]*b.f[2]; }
	static final float VEC_DOT(vec4f a, vec3f b) { return a.f[0]*b.f[0] + a.f[1]*b.f[1] + a.f[2]*b.f[2]; }
	static final float VEC_DOT(vec3f a, vec4f b) { return a.f[0]*b.f[0] + a.f[1]*b.f[1] + a.f[2]*b.f[2]; }
	static final float VEC_DOT(vec4f a, vec4f b) { return a.f[0]*b.f[0] + a.f[1]*b.f[1] + a.f[2]*b.f[2]; }
	static final float VEC_DOT(vec4f a, float[] b) { return a.f[0]*b[0] + a.f[1]*b[1] + a.f[2]*b[2]; }

	/// Vector dot product
	static final float VEC_DOT_4(vec4f a, vec4f b) { return ((a).f[0]*(b).f[0] + (a).f[1]*(b).f[1] + (a).f[2]*(b).f[2] + (a).f[3]*(b).f[3]); }

	/// vector impact parameter (squared)
	static final void VEC_IMPACT_SQ(final RefFloat bsq, final vec3f direction, 
			final vec3f position) {
		   float _llel_ = VEC_DOT(direction, position);
		   bsq.d = VEC_DOT(position, position) - _llel_*_llel_;
	}
	static final float VEC_IMPACT_SQ(final vec3f direction, final vec3f position) {
		   float _llel_ = VEC_DOT(direction, position);
		   return VEC_DOT(position, position) - _llel_*_llel_;
		}


	/// vector impact parameter
	static final void VEC_IMPACT(final RefFloat bsq, final vec3f direction, 
			final vec3f position)	{
		   VEC_IMPACT_SQ(bsq,direction,position);		
		   bsq.d = GIM_SQRT(bsq.d);					
	}
	static final float VEC_IMPACT(final vec3f direction, final vec3f position)	{
		   float bsq = VEC_IMPACT_SQ(direction,position);		
		   bsq = GIM_SQRT(bsq);
		   return bsq;
		}

	/// Vector length
	static final void VEC_LENGTH_2(final vec2f  a, final RefFloat l)
	{
	    float _pp = VEC_DOT_2(a,a);
	    l.d = GIM_SQRT(_pp);
	}


	/// Vector length
	static final void VEC_LENGTH(final vec3f a, final RefFloat l)
	{
	    float _pp = VEC_DOT(a,a);
	    l.d = GIM_SQRT(_pp);
	}
	static final float VEC_LENGTH(final vec3f a)
	{
	    float _pp = VEC_DOT(a,a);
	    return GIM_SQRT(_pp);
	}


	/// Vector length
	static final void VEC_LENGTH_4(final vec4f a, final RefFloat l)
	{
	    float _pp = VEC_DOT_4(a,a);
	    l.d = GIM_SQRT(_pp);
	}

	/// Vector inv length
	static final void VEC_INV_LENGTH_2(final vec2f a, final RefFloat l)
	{
	    float _pp = VEC_DOT_2(a,a);
	    l.d = GIM_INV_SQRT(_pp);
	}


	/// Vector inv length
	static final float VEC_INV_LENGTH(final vec3f a)
	{
	    float _pp = VEC_DOT(a,a);
	    return GIM_INV_SQRT(_pp);
	}
	static final float VEC_INV_LENGTH(final vec4f a)
	{
	    float _pp = VEC_DOT(a,a);
	    return GIM_INV_SQRT(_pp);
	}


	/// Vector inv length
	static final void VEC_INV_LENGTH_4(final vec4f a, final RefFloat l)
	{
	    float _pp = VEC_DOT_4(a,a);
	    l.d = GIM_INV_SQRT(_pp);
	}



	/// distance between two points
//	private static final void VEC_DISTANCE(final RefFloat _len, 
//			final vec3f _va, final vec3f _vb) {
//	    vec3f _tmp_ = new vec3f();				
//	    VEC_DIFF(_tmp_, _vb, _va);			
//	    VEC_LENGTH(_tmp_,_len);			
//	}


	/// Vector length
	static final void VEC_CONJUGATE_LENGTH(final vec3f a, final RefFloat l)
	{
	    float _pp = 1.0f - a.f[0]*a.f[0] - a.f[1]*a.f[1] - a.f[2]*a.f[2];
	    l.d = GIM_SQRT(_pp);
	}


	/// Vector length
	static final void VEC_NORMALIZE(vec3f a) {	
	    float len = VEC_INV_LENGTH(a); 
	    if(len<G_REAL_INFINITY)
	    {
	        a.f[0] *= len;				
	        a.f[1] *= len;				
	        a.f[2] *= len;				
	    }						
	}
	static final void VEC_NORMALIZE(vec4f a) {	
	    float len = VEC_INV_LENGTH(a); 
	    if(len<G_REAL_INFINITY)
	    {
	        a.f[0] *= len;				
	        a.f[1] *= len;				
	        a.f[2] *= len;				
	    }						
	}

	/// Set Vector size
	static final void VEC_RENORMALIZE(vec3f a, final float newlen) {	
	    float len = VEC_INV_LENGTH(a); 
	    if(len<G_REAL_INFINITY)
	    {
	        len *= newlen;
	        a.f[0] *= len;				
	        a.f[1] *= len;				
	        a.f[2] *= len;			
	    }						
	}

	/// Vector cross
	static final void VEC_CROSS(vec3f c, vec3f a, vec3f b)		
	{						
	   c.f[0] = (a).f[1] * (b).f[2] - (a).f[2] * (b).f[1];	
	   c.f[1] = (a).f[2] * (b).f[0] - (a).f[0] * (b).f[2];	
	   c.f[2] = (a).f[0] * (b).f[1] - (a).f[1] * (b).f[0];	
	}
	static final void VEC_CROSS(vec4f c, vec3f a, vec3f b)		
	{						
	   c.f[0] = (a).f[1] * (b).f[2] - (a).f[2] * (b).f[1];	
	   c.f[1] = (a).f[2] * (b).f[0] - (a).f[0] * (b).f[2];	
	   c.f[2] = (a).f[0] * (b).f[1] - (a).f[1] * (b).f[0];	
	}
	static final void VEC_CROSS(vec4f c, vec3f a, vec4f b)		
	{						
	   c.f[0] = (a).f[1] * (b).f[2] - (a).f[2] * (b).f[1];	
	   c.f[1] = (a).f[2] * (b).f[0] - (a).f[0] * (b).f[2];	
	   c.f[2] = (a).f[0] * (b).f[1] - (a).f[1] * (b).f[0];	
	}
	static final void VEC_CROSS(vec3f c, vec4f a, vec4f b)		
	{						
	   c.f[0] = (a).f[1] * (b).f[2] - (a).f[2] * (b).f[1];	
	   c.f[1] = (a).f[2] * (b).f[0] - (a).f[0] * (b).f[2];	
	   c.f[2] = (a).f[0] * (b).f[1] - (a).f[1] * (b).f[0];	
	}


	/*! Vector perp -- assumes that n is of unit length
	 * accepts vector v, subtracts out any component parallel to n */
	static final void VEC_PERPENDICULAR(vec3f vp, vec3f v, vec3f n)			
	{						
	   float dot = VEC_DOT(v, n);			
	   vp.f[0] = (v).f[0] - dot*(n).f[0];		
	   vp.f[1] = (v).f[1] - dot*(n).f[1];		
	   vp.f[2] = (v).f[2] - dot*(n).f[2];		
	}


	/*! Vector parallel -- assumes that n is of unit length */
	static final void VEC_PARALLEL(vec3f vp, vec3f v, vec3f n)			
	{						
	   float dot = VEC_DOT(v, n);			
	   vp.f[0] = (dot) * (n).f[0];			
	   vp.f[1] = (dot) * (n).f[1];			
	   vp.f[2] = (dot) * (n).f[2];			
	}

	/*! Same as Vector parallel --  n can have any length
	 * accepts vector v, subtracts out any component perpendicular to n */
	static final void VEC_PROJECT(vec3f vp, vec3f v, vec3f n)			
	{ 
		float scalar = VEC_DOT(v, n);			
		scalar/= VEC_DOT(n, n); 
		vp.f[0] = (scalar) * (n).f[0];			
	    vp.f[1] = (scalar) * (n).f[1];			
	    vp.f[2] = (scalar) * (n).f[2];			
	}


	/*! accepts vector v*/
	static final void VEC_UNPROJECT(vec3f vp, vec3f v, vec3f n)			
	{ 
		float scalar = VEC_DOT(v, n);			
		scalar = VEC_DOT(n, n)/scalar; 
		vp.f[0] = (scalar) * (n).f[0];			
	    vp.f[1] = (scalar) * (n).f[1];			
	    vp.f[2] = (scalar) * (n).f[2];			
	}


	/*! Vector reflection -- assumes n is of unit length
	 Takes vector v, reflects it against reflector n, and returns vr */
	static final void VEC_REFLECT(vec3f vr, vec3f v, vec3f n)			
	{						
	   float dot = VEC_DOT(v, n);			
	   vr.f[0] = (v).f[0] - 2.0f * (dot) * (n).f[0];	
	   vr.f[1] = (v).f[1] - 2.0f * (dot) * (n).f[1];	
	   vr.f[2] = (v).f[2] - 2.0f * (dot) * (n).f[2];	
	}


	/*! Vector blending
	Takes two vectors a, b, blends them together with two scalars */
	static final void VEC_BLEND_AB(vec3f vr, final float sa, vec3f a, final float sb, vec3f b)			
	{						
	   vr.f[0] = (sa) * (a).f[0] + (sb) * (b).f[0];	
	   vr.f[1] = (sa) * (a).f[1] + (sb) * (b).f[1];	
	   vr.f[2] = (sa) * (a).f[2] + (sb) * (b).f[2];	
	}

	/*! Vector blending
	Takes two vectors a, b, blends them together with s <=1 */
//TZ	static final void VEC_BLEND(vec3f vr, final float a, vec3f b, final float s)	{ VEC_BLEND_AB(vr,1-s,a,sb,s); }

//TZ	static final void VEC_SET3(vec3f a, vec3f b,op, vec3f c) { a[0]=b[0] op c[0]; a[1]=b[1] op c[1]; a[2]=b[2] op c[2]; }
	//! @}


	/*! \defgroup MATRIX_OPERATIONS
	Operations for matrices : mat2f, mat3f and mat4f
	*/
	//! @{

	/// initialize matrix
	static final void IDENTIFY_MATRIX_3X3(mat3f m)			
	{						
	   m.f[M(0,0)] = 1.0f;				
	   m.f[M(0,1)] = 0.0f;				
	   m.f[M(0,2)] = 0.0f;				
							
	   m.f[M(1,0)] = 0.0f;				
	   m.f[M(1,1)] = 1.0f;				
	   m.f[M(1,2)] = 0.0f;				
							
	   m.f[M(2,0)] = 0.0f;				
	   m.f[M(2,1)] = 0.0f;				
	   m.f[M(2,2)] = 1.0f;				
	}

	/*! initialize matrix */
	public static final void IDENTIFY_MATRIX_4X4(mat4f m)			
	{						
	   m.f[M(0,0)] = 1.0f;				
	   m.f[M(0,1)] = 0.0f;				
	   m.f[M(0,2)] = 0.0f;				
	   m.f[M(0,3)] = 0.0f;				
							
	   m.f[M(1,0)] = 0.0f;				
	   m.f[M(1,1)] = 1.0f;				
	   m.f[M(1,2)] = 0.0f;				
	   m.f[M(1,3)] = 0.0f;				
							
	   m.f[M(2,0)] = 0.0f;				
	   m.f[M(2,1)] = 0.0f;				
	   m.f[M(2,2)] = 1.0f;				
	   m.f[M(2,3)] = 0.0f;				
							
	   m.f[M(3,0)] = 0.0f;				
	   m.f[M(3,1)] = 0.0f;				
	   m.f[M(3,2)] = 0.0f;				
	   m.f[M(3,3)] = 1.0f;				
	}

	/*! initialize matrix */
	static final void ZERO_MATRIX_4X4(mat4f m)			
	{						
	   m.f[M(0,0)] = 0.0f;				
	   m.f[M(0,1)] = 0.0f;				
	   m.f[M(0,2)] = 0.0f;				
	   m.f[M(0,3)] = 0.0f;				
							
	   m.f[M(1,0)] = 0.0f;				
	   m.f[M(1,1)] = 0.0f;				
	   m.f[M(1,2)] = 0.0f;				
	   m.f[M(1,3)] = 0.0f;				
							
	   m.f[M(2,0)] = 0.0f;				
	   m.f[M(2,1)] = 0.0f;				
	   m.f[M(2,2)] = 0.0f;				
	   m.f[M(2,3)] = 0.0f;				
							
	   m.f[M(3,0)] = 0.0f;				
	   m.f[M(3,1)] = 0.0f;				
	   m.f[M(3,2)] = 0.0f;				
	   m.f[M(3,3)] = 0.0f;				
	}

	/*! matrix rotation  X */
	static final void ROTX_CS(mat4f m, final float cosine, final float sine)		
	{					
	   /* rotation about the x-axis */	
						
	   m.f[M(0,0)] = 1.0f;			
	   m.f[M(0,1)] = 0.0f;			
	   m.f[M(0,2)] = 0.0f;			
	   m.f[M(0,3)] = 0.0f;			
						
	   m.f[M(1,0)] = 0.0f;			
	   m.f[M(1,1)] = (cosine);			
	   m.f[M(1,2)] = (sine);			
	   m.f[M(1,3)] = 0.0f;			
						
	   m.f[M(2,0)] = 0.0f;			
	   m.f[M(2,1)] = -(sine);			
	   m.f[M(2,2)] = (cosine);			
	   m.f[M(2,3)] = 0.0f;			
						
	   m.f[M(3,0)] = 0.0f;			
	   m.f[M(3,1)] = 0.0f;			
	   m.f[M(3,2)] = 0.0f;			
	   m.f[M(3,3)] = 1.0f;			
	}

	/*! matrix rotation  Y */
	static final void ROTY_CS(mat4f m, final float cosine, final float sine)		
	{					
	   /* rotation about the y-axis */	
						
	   m.f[M(0,0)] = (cosine);			
	   m.f[M(0,1)] = 0.0f;			
	   m.f[M(0,2)] = -(sine);			
	   m.f[M(0,3)] = 0.0f;			
						
	   m.f[M(1,0)] = 0.0f;			
	   m.f[M(1,1)] = 1.0f;			
	   m.f[M(1,2)] = 0.0f;			
	   m.f[M(1,3)] = 0.0f;			
						
	   m.f[M(2,0)] = (sine);			
	   m.f[M(2,1)] = 0.0f;			
	   m.f[M(2,2)] = (cosine);			
	   m.f[M(2,3)] = 0.0f;			
						
	   m.f[M(3,0)] = 0.0f;			
	   m.f[M(3,1)] = 0.0f;			
	   m.f[M(3,2)] = 0.0f;			
	   m.f[M(3,3)] = 1.0f;			
	}

	/*! matrix rotation  Z */
	static final void ROTZ_CS(mat4f m, final float cosine, final float sine)		
	{					
	   /* rotation about the z-axis */	
						
	   m.f[M(0,0)] = (cosine);			
	   m.f[M(0,1)] = (sine);			
	   m.f[M(0,2)] = 0.0f;			
	   m.f[M(0,3)] = 0.0f;			
						
	   m.f[M(1,0)] = -(sine);			
	   m.f[M(1,1)] = (cosine);			
	   m.f[M(1,2)] = 0.0f;			
	   m.f[M(1,3)] = 0.0f;			
						
	   m.f[M(2,0)] = 0.0f;			
	   m.f[M(2,1)] = 0.0f;			
	   m.f[M(2,2)] = 1.0f;			
	   m.f[M(2,3)] = 0.0f;			
						
	   m.f[M(3,0)] = 0.0f;			
	   m.f[M(3,1)] = 0.0f;			
	   m.f[M(3,2)] = 0.0f;			
	   m.f[M(3,3)] = 1.0f;			
	}

	/*! matrix copy */
	static final void COPY_MATRIX_2X2(mat2f b, mat2f a)	
	{				
	   b.f[M(0,0)] = a.f[M(0,0)];		
	   b.f[M(0,1)] = a.f[M(0,1)];		
					
	   b.f[M(1,0)] = a.f[M(1,0)];		
	   b.f[M(1,1)] = a.f[M(1,1)];		
					
	}


	/*! matrix copy */
/*	static final void COPY_MATRIX_2X3(b,a)	
	{				
	   b.f[M(0,0)] = a.f[M(0,0)];		
	   b.f[M(0,1)] = a.f[M(0,1)];		
	   b.f[M(0,2)] = a.f[M(0,2)];		
					
	   b.f[M(1,0)] = a.f[M(1,0)];		
	   b.f[M(1,1)] = a.f[M(1,1)];		
	   b.f[M(1,2)] = a.f[M(1,2)];		
	}
*/

	/*! matrix copy */
	static final void COPY_MATRIX_3X3(mat3f b, mat3f a)	
	{				
	   b.f[M(0,0)] = a.f[M(0,0)];		
	   b.f[M(0,1)] = a.f[M(0,1)];		
	   b.f[M(0,2)] = a.f[M(0,2)];		
					
	   b.f[M(1,0)] = a.f[M(1,0)];		
	   b.f[M(1,1)] = a.f[M(1,1)];		
	   b.f[M(1,2)] = a.f[M(1,2)];		
					
	   b.f[M(2,0)] = a.f[M(2,0)];		
	   b.f[M(2,1)] = a.f[M(2,1)];		
	   b.f[M(2,2)] = a.f[M(2,2)];		
	}


	/*! matrix copy */
	static final void COPY_MATRIX_4X4(mat4f b, mat4f a)	
	{				
	   b.f[M(0,0)] = a.f[M(0,0)];		
	   b.f[M(0,1)] = a.f[M(0,1)];		
	   b.f[M(0,2)] = a.f[M(0,2)];		
	   b.f[M(0,3)] = a.f[M(0,3)];		
					
	   b.f[M(1,0)] = a.f[M(1,0)];		
	   b.f[M(1,1)] = a.f[M(1,1)];		
	   b.f[M(1,2)] = a.f[M(1,2)];		
	   b.f[M(1,3)] = a.f[M(1,3)];		
					
	   b.f[M(2,0)] = a.f[M(2,0)];		
	   b.f[M(2,1)] = a.f[M(2,1)];		
	   b.f[M(2,2)] = a.f[M(2,2)];		
	   b.f[M(2,3)] = a.f[M(2,3)];		
					
	   b.f[M(3,0)] = a.f[M(3,0)];		
	   b.f[M(3,1)] = a.f[M(3,1)];		
	   b.f[M(3,2)] = a.f[M(3,2)];		
	   b.f[M(3,3)] = a.f[M(3,3)];		
	}


	/*! matrix transpose */
	static final void TRANSPOSE_MATRIX_2X2(mat2f b, mat2f a)	
	{				
	   b.f[M(0,0)] = a.f[M(0,0)];		
	   b.f[M(0,1)] = a.f[M(1,0)];		
					
	   b.f[M(1,0)] = a.f[M(0,1)];		
	   b.f[M(1,1)] = a.f[M(1,1)];		
	}


	/*! matrix transpose */
	static final void TRANSPOSE_MATRIX_3X3(mat3f b, mat3f a)	
	{				
	   b.f[M(0,0)] = a.f[M(0,0)];		
	   b.f[M(0,1)] = a.f[M(1,0)];		
	   b.f[M(0,2)] = a.f[M(2,0)];		
					
	   b.f[M(1,0)] = a.f[M(0,1)];		
	   b.f[M(1,1)] = a.f[M(1,1)];		
	   b.f[M(1,2)] = a.f[M(2,1)];		
					
	   b.f[M(2,0)] = a.f[M(0,2)];		
	   b.f[M(2,1)] = a.f[M(1,2)];		
	   b.f[M(2,2)] = a.f[M(2,2)];		
	}


	/*! matrix transpose */
	static final void TRANSPOSE_MATRIX_4X4(mat4f b, mat4f a)	
	{				
	   b.f[M(0,0)] = a.f[M(0,0)];		
	   b.f[M(0,1)] = a.f[M(1,0)];		
	   b.f[M(0,2)] = a.f[M(2,0)];		
	   b.f[M(0,3)] = a.f[M(3,0)];		
					
	   b.f[M(1,0)] = a.f[M(0,1)];		
	   b.f[M(1,1)] = a.f[M(1,1)];		
	   b.f[M(1,2)] = a.f[M(2,1)];		
	   b.f[M(1,3)] = a.f[M(3,1)];		
					
	   b.f[M(2,0)] = a.f[M(0,2)];		
	   b.f[M(2,1)] = a.f[M(1,2)];		
	   b.f[M(2,2)] = a.f[M(2,2)];		
	   b.f[M(2,3)] = a.f[M(3,2)];		
					
	   b.f[M(3,0)] = a.f[M(0,3)];		
	   b.f[M(3,1)] = a.f[M(1,3)];		
	   b.f[M(3,2)] = a.f[M(2,3)];		
	   b.f[M(3,3)] = a.f[M(3,3)];		
	}


	/*! multiply matrix by scalar */
	static final void SCALE_MATRIX_2X2(mat2f b, final float s, mat2f a)		
	{					
	   b.f[M(0,0)] = (s) * a.f[M(0,0)];		
	   b.f[M(0,1)] = (s) * a.f[M(0,1)];		
						
	   b.f[M(1,0)] = (s) * a.f[M(1,0)];		
	   b.f[M(1,1)] = (s) * a.f[M(1,1)];		
	}


	/*! multiply matrix by scalar */
	static final void SCALE_MATRIX_3X3(mat3f b, final float s, mat3f a)		
	{					
	   b.f[M(0,0)] = (s) * a.f[M(0,0)];		
	   b.f[M(0,1)] = (s) * a.f[M(0,1)];		
	   b.f[M(0,2)] = (s) * a.f[M(0,2)];		
						
	   b.f[M(1,0)] = (s) * a.f[M(1,0)];		
	   b.f[M(1,1)] = (s) * a.f[M(1,1)];		
	   b.f[M(1,2)] = (s) * a.f[M(1,2)];		
						
	   b.f[M(2,0)] = (s) * a.f[M(2,0)];		
	   b.f[M(2,1)] = (s) * a.f[M(2,1)];		
	   b.f[M(2,2)] = (s) * a.f[M(2,2)];		
	}


	/*! multiply matrix by scalar */
	static final void SCALE_MATRIX_4X4(mat4f b, final float s, mat4f a)		
	{					
	   b.f[M(0,0)] = (s) * a.f[M(0,0)];		
	   b.f[M(0,1)] = (s) * a.f[M(0,1)];		
	   b.f[M(0,2)] = (s) * a.f[M(0,2)];		
	   b.f[M(0,3)] = (s) * a.f[M(0,3)];		
						
	   b.f[M(1,0)] = (s) * a.f[M(1,0)];		
	   b.f[M(1,1)] = (s) * a.f[M(1,1)];		
	   b.f[M(1,2)] = (s) * a.f[M(1,2)];		
	   b.f[M(1,3)] = (s) * a.f[M(1,3)];		
						
	   b.f[M(2,0)] = (s) * a.f[M(2,0)];		
	   b.f[M(2,1)] = (s) * a.f[M(2,1)];		
	   b.f[M(2,2)] = (s) * a.f[M(2,2)];		
	   b.f[M(2,3)] = (s) * a.f[M(2,3)];		
						
	   b.f[M(3,0)] = s * a.f[M(3,0)];		
	   b.f[M(3,1)] = s * a.f[M(3,1)];		
	   b.f[M(3,2)] = s * a.f[M(3,2)];		
	   b.f[M(3,3)] = s * a.f[M(3,3)];		
	}


	/*! multiply matrix by scalar */
	static final void ACCUM_SCALE_MATRIX_2X2(mat2f b, final float s, mat2f a)		
	{					
	   b.f[M(0,0)] += (s) * a.f[M(0,0)];		
	   b.f[M(0,1)] += (s) * a.f[M(0,1)];		
						
	   b.f[M(1,0)] += (s) * a.f[M(1,0)];		
	   b.f[M(1,1)] += (s) * a.f[M(1,1)];		
	}


	/*! multiply matrix by scalar */
	static final void ACCUM_SCALE_MATRIX_3X3(mat3f b, final float s, mat3f a)		
	{					
	   b.f[M(0,0)] += (s) * a.f[M(0,0)];		
	   b.f[M(0,1)] += (s) * a.f[M(0,1)];		
	   b.f[M(0,2)] += (s) * a.f[M(0,2)];		
						
	   b.f[M(1,0)] += (s) * a.f[M(1,0)];		
	   b.f[M(1,1)] += (s) * a.f[M(1,1)];		
	   b.f[M(1,2)] += (s) * a.f[M(1,2)];		
						
	   b.f[M(2,0)] += (s) * a.f[M(2,0)];		
	   b.f[M(2,1)] += (s) * a.f[M(2,1)];		
	   b.f[M(2,2)] += (s) * a.f[M(2,2)];		
	}


	/*! multiply matrix by scalar */
	static final void ACCUM_SCALE_MATRIX_4X4(mat4f b, final float s, mat4f a)		
	{					
	   b.f[M(0,0)] += (s) * a.f[M(0,0)];		
	   b.f[M(0,1)] += (s) * a.f[M(0,1)];		
	   b.f[M(0,2)] += (s) * a.f[M(0,2)];		
	   b.f[M(0,3)] += (s) * a.f[M(0,3)];		
						
	   b.f[M(1,0)] += (s) * a.f[M(1,0)];		
	   b.f[M(1,1)] += (s) * a.f[M(1,1)];		
	   b.f[M(1,2)] += (s) * a.f[M(1,2)];		
	   b.f[M(1,3)] += (s) * a.f[M(1,3)];		
						
	   b.f[M(2,0)] += (s) * a.f[M(2,0)];		
	   b.f[M(2,1)] += (s) * a.f[M(2,1)];		
	   b.f[M(2,2)] += (s) * a.f[M(2,2)];		
	   b.f[M(2,3)] += (s) * a.f[M(2,3)];		
						
	   b.f[M(3,0)] += (s) * a.f[M(3,0)];		
	   b.f[M(3,1)] += (s) * a.f[M(3,1)];		
	   b.f[M(3,2)] += (s) * a.f[M(3,2)];		
	   b.f[M(3,3)] += (s) * a.f[M(3,3)];		
	}

	/*! matrix product */
	/*! c[x][y] = a[x][0]*b[0][y]+a[x][1]*b[1][y]+a[x][2]*b[2][y]+a[x][3]*b[3][y];*/
	static final void MATRIX_PRODUCT_2X2(mat2f c, mat2f a, mat2f b)		
	{						
	   c.f[M(0,0)] = a.f[M(0,0)]*b.f[M(0,0)]+a.f[M(0,1)]*b.f[M(1,0)];	
	   c.f[M(0,1)] = a.f[M(0,0)]*b.f[M(0,1)]+a.f[M(0,1)]*b.f[M(1,1)];	
							
	   c.f[M(1,0)] = a.f[M(1,0)]*b.f[M(0,0)]+a.f[M(1,1)]*b.f[M(1,0)];	
	   c.f[M(1,1)] = a.f[M(1,0)]*b.f[M(0,1)]+a.f[M(1,1)]*b.f[M(1,1)];	
							
	}

	/*! matrix product */
	/*! c[x][y] = a[x][0]*b[0][y]+a[x][1]*b[1][y]+a[x][2]*b[2][y]+a[x][3]*b[3][y];*/
	static final void MATRIX_PRODUCT_3X3(mat3f c, mat3f a, mat3f b)				
	{								
	   c.f[M(0,0)] = a.f[M(0,0)]*b.f[M(0,0)]+a.f[M(0,1)]*b.f[M(1,0)]+a.f[M(0,2)]*b.f[M(2,0)];	
	   c.f[M(0,1)] = a.f[M(0,0)]*b.f[M(0,1)]+a.f[M(0,1)]*b.f[M(1,1)]+a.f[M(0,2)]*b.f[M(2,1)];	
	   c.f[M(0,2)] = a.f[M(0,0)]*b.f[M(0,2)]+a.f[M(0,1)]*b.f[M(1,2)]+a.f[M(0,2)]*b.f[M(2,2)];	
									
	   c.f[M(1,0)] = a.f[M(1,0)]*b.f[M(0,0)]+a.f[M(1,1)]*b.f[M(1,0)]+a.f[M(1,2)]*b.f[M(2,0)];	
	   c.f[M(1,1)] = a.f[M(1,0)]*b.f[M(0,1)]+a.f[M(1,1)]*b.f[M(1,1)]+a.f[M(1,2)]*b.f[M(2,1)];	
	   c.f[M(1,2)] = a.f[M(1,0)]*b.f[M(0,2)]+a.f[M(1,1)]*b.f[M(1,2)]+a.f[M(1,2)]*b.f[M(2,2)];	
									
	   c.f[M(2,0)] = a.f[M(2,0)]*b.f[M(0,0)]+a.f[M(2,1)]*b.f[M(1,0)]+a.f[M(2,2)]*b.f[M(2,0)];	
	   c.f[M(2,1)] = a.f[M(2,0)]*b.f[M(0,1)]+a.f[M(2,1)]*b.f[M(1,1)]+a.f[M(2,2)]*b.f[M(2,1)];	
	   c.f[M(2,2)] = a.f[M(2,0)]*b.f[M(0,2)]+a.f[M(2,1)]*b.f[M(1,2)]+a.f[M(2,2)]*b.f[M(2,2)];	
	}


	/*! matrix product */
	/*! c[x][y] = a[x][0]*b[0][y]+a[x][1]*b[1][y]+a[x][2]*b[2][y]+a[x][3]*b[3][y];*/
	static final void MATRIX_PRODUCT_4X4(mat4f c, mat4f a, mat4f b)		
	{						
	   c.f[M(0,0)] = a.f[M(0,0)]*b.f[M(0,0)]+a.f[M(0,1)]*b.f[M(1,0)]+a.f[M(0,2)]*b.f[M(2,0)]+a.f[M(0,3)]*b.f[M(3,0)];
	   c.f[M(0,1)] = a.f[M(0,0)]*b.f[M(0,1)]+a.f[M(0,1)]*b.f[M(1,1)]+a.f[M(0,2)]*b.f[M(2,1)]+a.f[M(0,3)]*b.f[M(3,1)];
	   c.f[M(0,2)] = a.f[M(0,0)]*b.f[M(0,2)]+a.f[M(0,1)]*b.f[M(1,2)]+a.f[M(0,2)]*b.f[M(2,2)]+a.f[M(0,3)]*b.f[M(3,2)];
	   c.f[M(0,3)] = a.f[M(0,0)]*b.f[M(0,3)]+a.f[M(0,1)]*b.f[M(1,3)]+a.f[M(0,2)]*b.f[M(2,3)]+a.f[M(0,3)]*b.f[M(3,3)];
							
	   c.f[M(1,0)] = a.f[M(1,0)]*b.f[M(0,0)]+a.f[M(1,1)]*b.f[M(1,0)]+a.f[M(1,2)]*b.f[M(2,0)]+a.f[M(1,3)]*b.f[M(3,0)];
	   c.f[M(1,1)] = a.f[M(1,0)]*b.f[M(0,1)]+a.f[M(1,1)]*b.f[M(1,1)]+a.f[M(1,2)]*b.f[M(2,1)]+a.f[M(1,3)]*b.f[M(3,1)];
	   c.f[M(1,2)] = a.f[M(1,0)]*b.f[M(0,2)]+a.f[M(1,1)]*b.f[M(1,2)]+a.f[M(1,2)]*b.f[M(2,2)]+a.f[M(1,3)]*b.f[M(3,2)];
	   c.f[M(1,3)] = a.f[M(1,0)]*b.f[M(0,3)]+a.f[M(1,1)]*b.f[M(1,3)]+a.f[M(1,2)]*b.f[M(2,3)]+a.f[M(1,3)]*b.f[M(3,3)];
							
	   c.f[M(2,0)] = a.f[M(2,0)]*b.f[M(0,0)]+a.f[M(2,1)]*b.f[M(1,0)]+a.f[M(2,2)]*b.f[M(2,0)]+a.f[M(2,3)]*b.f[M(3,0)];
	   c.f[M(2,1)] = a.f[M(2,0)]*b.f[M(0,1)]+a.f[M(2,1)]*b.f[M(1,1)]+a.f[M(2,2)]*b.f[M(2,1)]+a.f[M(2,3)]*b.f[M(3,1)];
	   c.f[M(2,2)] = a.f[M(2,0)]*b.f[M(0,2)]+a.f[M(2,1)]*b.f[M(1,2)]+a.f[M(2,2)]*b.f[M(2,2)]+a.f[M(2,3)]*b.f[M(3,2)];
	   c.f[M(2,3)] = a.f[M(2,0)]*b.f[M(0,3)]+a.f[M(2,1)]*b.f[M(1,3)]+a.f[M(2,2)]*b.f[M(2,3)]+a.f[M(2,3)]*b.f[M(3,3)];
							
	   c.f[M(3,0)] = a.f[M(3,0)]*b.f[M(0,0)]+a.f[M(3,1)]*b.f[M(1,0)]+a.f[M(3,2)]*b.f[M(2,0)]+a.f[M(3,3)]*b.f[M(3,0)];
	   c.f[M(3,1)] = a.f[M(3,0)]*b.f[M(0,1)]+a.f[M(3,1)]*b.f[M(1,1)]+a.f[M(3,2)]*b.f[M(2,1)]+a.f[M(3,3)]*b.f[M(3,1)];
	   c.f[M(3,2)] = a.f[M(3,0)]*b.f[M(0,2)]+a.f[M(3,1)]*b.f[M(1,2)]+a.f[M(3,2)]*b.f[M(2,2)]+a.f[M(3,3)]*b.f[M(3,2)];
	   c.f[M(3,3)] = a.f[M(3,0)]*b.f[M(0,3)]+a.f[M(3,1)]*b.f[M(1,3)]+a.f[M(3,2)]*b.f[M(2,3)]+a.f[M(3,3)]*b.f[M(3,3)];
	}


	/*! matrix times vector */
	static final void MAT_DOT_VEC_2X2(vec2f p, mat2f m, vec2f v)					
	{								
	   p.f[0] = m.f[M(0,0)]*v.f[0] + m.f[M(0,1)]*v.f[1];				
	   p.f[1] = m.f[M(1,0)]*v.f[0] + m.f[M(1,1)]*v.f[1];				
	}


	/*! matrix times vector */
	static final void MAT_DOT_VEC_3X3(vec3f p,mat3f m, vec3f v)					
	{								
	   p.f[0] = m.f[M(0,0)]*v.f[0] + m.f[M(0,1)]*v.f[1] + m.f[M(0,2)]*v.f[2];		
	   p.f[1] = m.f[M(1,0)]*v.f[0] + m.f[M(1,1)]*v.f[1] + m.f[M(1,2)]*v.f[2];		
	   p.f[2] = m.f[M(2,0)]*v.f[0] + m.f[M(2,1)]*v.f[1] + m.f[M(2,2)]*v.f[2];		
	}


	/*! matrix times vector
	v is a vec4f
	*/
	static final void MAT_DOT_VEC_4X4(vec4f p, mat4f m, vec4f v)					
	{								
	   p.f[0] = m.f[M(0,0)]*v.f[0] + m.f[M(0,1)]*v.f[1] + m.f[M(0,2)]*v.f[2] + m.f[M(0,3)]*v.f[3];	
	   p.f[1] = m.f[M(1,0)]*v.f[0] + m.f[M(1,1)]*v.f[1] + m.f[M(1,2)]*v.f[2] + m.f[M(1,3)]*v.f[3];	
	   p.f[2] = m.f[M(2,0)]*v.f[0] + m.f[M(2,1)]*v.f[1] + m.f[M(2,2)]*v.f[2] + m.f[M(2,3)]*v.f[3];	
	   p.f[3] = m.f[M(3,0)]*v.f[0] + m.f[M(3,1)]*v.f[1] + m.f[M(3,2)]*v.f[2] + m.f[M(3,3)]*v.f[3];	
	}

	/*! matrix times vector
	v is a vec3f
	and m is a mat4f<br>
	Last column is added as the position
	*/
	static final void MAT_DOT_VEC_3X4(vec3f p, mat4f m, vec3f v)					
	{								
	   p.f[0] = m.f[M(0,0)]*v.f[0] + m.f[M(0,1)]*v.f[1] + m.f[M(0,2)]*v.f[2] + m.f[M(0,3)];	
	   p.f[1] = m.f[M(1,0)]*v.f[0] + m.f[M(1,1)]*v.f[1] + m.f[M(1,2)]*v.f[2] + m.f[M(1,3)];	
	   p.f[2] = m.f[M(2,0)]*v.f[0] + m.f[M(2,1)]*v.f[1] + m.f[M(2,2)]*v.f[2] + m.f[M(2,3)];	
	}
	static final void MAT_DOT_VEC_3X4(FloatArray p, mat4f m, FloatArray v)					
	{								
	   p.setAt(0, m.f[M(0,0)]*v.at(0) + m.f[M(0,1)]*v.at(1) + m.f[M(0,2)]*v.at(2) + m.f[M(0,3)] );	
	   p.setAt(1, m.f[M(1,0)]*v.at(0) + m.f[M(1,1)]*v.at(1) + m.f[M(1,2)]*v.at(2) + m.f[M(1,3)] );	
	   p.setAt(2, m.f[M(2,0)]*v.at(0) + m.f[M(2,1)]*v.at(1) + m.f[M(2,2)]*v.at(2) + m.f[M(2,3)] );	
	}

	/*! vector transpose times matrix */
	/*! p.f[j] = v.f[0]*m[0][j] + v.f[1]*m[1][j] + v.f[2]*m[2][j]; */
	static final void VEC_DOT_MAT_3X3(vec3f p, vec3f v, mat3f m)					
	{								
	   p.f[0] = v.f[0]*m.f[M(0,0)] + v.f[1]*m.f[M(1,0)] + v.f[2]*m.f[M(2,0)];		
	   p.f[1] = v.f[0]*m.f[M(0,1)] + v.f[1]*m.f[M(1,1)] + v.f[2]*m.f[M(2,1)];		
	   p.f[2] = v.f[0]*m.f[M(0,2)] + v.f[1]*m.f[M(1,2)] + v.f[2]*m.f[M(2,2)];		
	}


	/*! affine matrix times vector */
	/** The matrix is assumed to be an affine matrix, with last two
	 * entries representing a translation */
	static final void MAT_DOT_VEC_2X3(vec2f p, mat3f m, vec2f v)					
	{								
	   p.f[0] = m.f[M(0,0)]*v.f[0] + m.f[M(0,1)]*v.f[1] + m.f[M(0,2)];		
	   p.f[1] = m.f[M(1,0)]*v.f[0] + m.f[M(1,1)]*v.f[1] + m.f[M(1,2)];		
	}


	/** inverse transpose of matrix times vector
	 *
	 * This macro computes inverse transpose of matrix m,
	 * and multiplies vector v into it, to yeild vector p
	 *
	 * DANGER !!! Do Not use this on normal vectors!!!
	 * It will leave normals the wrong length !!!
	 * See macro below for use on normals.
	 */
	static final void INV_TRANSP_MAT_DOT_VEC_2X2(final vec2f p, final mat2f m, final vec2f v)			
	{								
	   float det;						
									
	   det = m.f[M(0,0)]*m.f[M(1,1)] - m.f[M(0,1)]*m.f[M(1,0)];			
	   p.f[0] = m.f[M(1,1)]*v.f[0] - m.f[M(1,0)]*v.f[1];				
	   p.f[1] = - m.f[M(0,1)]*v.f[0] + m.f[M(0,0)]*v.f[1];			
									
	   /* if matrix not singular, and not orthonormal, then renormalize */ 
	   if ((det!=1.0f) && (det != 0.0f)) {				
	      det = 1.0f / det;						
	      p.f[0] *= det;						
	      p.f[1] *= det;						
	   }								
	}


	/** transform normal vector by inverse transpose of matrix
	 * and then renormalize the vector
	 *
	 * This macro computes inverse transpose of matrix m,
	 * and multiplies vector v into it, to yeild vector p
	 * Vector p is then normalized.
	 */
	static final void NORM_XFORM_2X2(final vec2f p, final mat2f m, final vec2f v)					
	{								
	   double len;							
									
	   /* do nothing if off-diagonals are zero and diagonals are 	
	    * equal */							
	   if ((m.f[M(0,1)] != 0.0) || (m.f[M(1,0)] != 0.0) || (m.f[M(0,0)] != m.f[M(1,1)])) { 
	      p.f[0] = m.f[M(1,1)]*v.f[0] - m.f[M(1,0)]*v.f[1];			
	      p.f[1] = - m.f[M(0,1)]*v.f[0] + m.f[M(0,0)]*v.f[1];			
									
	      len = p.f[0]*p.f[0] + p.f[1]*p.f[1];				
	      len = GIM_INV_SQRT((float) len);					
	      p.f[0] *= len;						
	      p.f[1] *= len;						
	   } else {							
	      VEC_COPY_2 (p, v);					
	   }								
	}


	/** outer product of vector times vector transpose
	 *
	 * The outer product of vector v and vector transpose t yeilds
	 * dyadic matrix m.
	 */
	static final void OUTER_PRODUCT_2X2(final mat2f m, final vec2f v, final vec2f t)				
	{								
	   m.f[M(0,0)] = v.f[0] * t.f[0];					
	   m.f[M(0,1)] = v.f[0] * t.f[1];					
									
	   m.f[M(1,0)] = v.f[1] * t.f[0];					
	   m.f[M(1,1)] = v.f[1] * t.f[1];					
	}


	/** outer product of vector times vector transpose
	 *
	 * The outer product of vector v and vector transpose t yeilds
	 * dyadic matrix m.
	 */
	static final void OUTER_PRODUCT_3X3(mat3f m, vec3f v, vec3f t)				
	{								
	   m.f[M(0,0)] = v.f[0] * t.f[0];					
	   m.f[M(0,1)] = v.f[0] * t.f[1];					
	   m.f[M(0,2)] = v.f[0] * t.f[2];					
									
	   m.f[M(1,0)] = v.f[1] * t.f[0];					
	   m.f[M(1,1)] = v.f[1] * t.f[1];					
	   m.f[M(1,2)] = v.f[1] * t.f[2];					
									
	   m.f[M(2,0)] = v.f[2] * t.f[0];					
	   m.f[M(2,1)] = v.f[2] * t.f[1];					
	   m.f[M(2,2)] = v.f[2] * t.f[2];					
	}


	/** outer product of vector times vector transpose
	 *
	 * The outer product of vector v and vector transpose t yeilds
	 * dyadic matrix m.
	 */
	static final void OUTER_PRODUCT_4X4(mat4f m, vec4f v, vec4f t)				
	{								
	   m.f[M(0,0)] = v.f[0] * t.f[0];					
	   m.f[M(0,1)] = v.f[0] * t.f[1];					
	   m.f[M(0,2)] = v.f[0] * t.f[2];					
	   m.f[M(0,3)] = v.f[0] * t.f[3];					
									
	   m.f[M(1,0)] = v.f[1] * t.f[0];					
	   m.f[M(1,1)] = v.f[1] * t.f[1];					
	   m.f[M(1,2)] = v.f[1] * t.f[2];					
	   m.f[M(1,3)] = v.f[1] * t.f[3];					
									
	   m.f[M(2,0)] = v.f[2] * t.f[0];					
	   m.f[M(2,1)] = v.f[2] * t.f[1];					
	   m.f[M(2,2)] = v.f[2] * t.f[2];					
	   m.f[M(2,3)] = v.f[2] * t.f[3];					
									
	   m.f[M(3,0)] = v.f[3] * t.f[0];					
	   m.f[M(3,1)] = v.f[3] * t.f[1];					
	   m.f[M(3,2)] = v.f[3] * t.f[2];					
	   m.f[M(3,3)] = v.f[3] * t.f[3];					
	}


	/** outer product of vector times vector transpose
	 *
	 * The outer product of vector v and vector transpose t yeilds
	 * dyadic matrix m.
	 */
	static final void ACCUM_OUTER_PRODUCT_2X2(mat2f m, vec2f v, vec2f t)				
	{								
	   m.f[M(0,0)] += v.f[0] * t.f[0];					
	   m.f[M(0,1)] += v.f[0] * t.f[1];					
									
	   m.f[M(1,0)] += v.f[1] * t.f[0];					
	   m.f[M(1,1)] += v.f[1] * t.f[1];					
	}


	/** outer product of vector times vector transpose
	 *
	 * The outer product of vector v and vector transpose t yeilds
	 * dyadic matrix m.
	 */
	static final void ACCUM_OUTER_PRODUCT_3X3(mat3f m, vec3f v, vec3f t)				
	{								
	   m.f[M(0,0)] += v.f[0] * t.f[0];					
	   m.f[M(0,1)] += v.f[0] * t.f[1];					
	   m.f[M(0,2)] += v.f[0] * t.f[2];					
									
	   m.f[M(1,0)] += v.f[1] * t.f[0];					
	   m.f[M(1,1)] += v.f[1] * t.f[1];					
	   m.f[M(1,2)] += v.f[1] * t.f[2];					
									
	   m.f[M(2,0)] += v.f[2] * t.f[0];					
	   m.f[M(2,1)] += v.f[2] * t.f[1];					
	   m.f[M(2,2)] += v.f[2] * t.f[2];					
	}


	/** outer product of vector times vector transpose
	 *
	 * The outer product of vector v and vector transpose t yeilds
	 * dyadic matrix m.
	 */
	static final void ACCUM_OUTER_PRODUCT_4X4(mat4f m, vec4f v, vec4f t)				
	{								
	   m.f[M(0,0)] += v.f[0] * t.f[0];					
	   m.f[M(0,1)] += v.f[0] * t.f[1];					
	   m.f[M(0,2)] += v.f[0] * t.f[2];					
	   m.f[M(0,3)] += v.f[0] * t.f[3];					
									
	   m.f[M(1,0)] += v.f[1] * t.f[0];					
	   m.f[M(1,1)] += v.f[1] * t.f[1];					
	   m.f[M(1,2)] += v.f[1] * t.f[2];					
	   m.f[M(1,3)] += v.f[1] * t.f[3];					
									
	   m.f[M(2,0)] += v.f[2] * t.f[0];					
	   m.f[M(2,1)] += v.f[2] * t.f[1];					
	   m.f[M(2,2)] += v.f[2] * t.f[2];					
	   m.f[M(2,3)] += v.f[2] * t.f[3];					
									
	   m.f[M(3,0)] += v.f[3] * t.f[0];					
	   m.f[M(3,1)] += v.f[3] * t.f[1];					
	   m.f[M(3,2)] += v.f[3] * t.f[2];					
	   m.f[M(3,3)] += v.f[3] * t.f[3];					
	}


	/** determinant of matrix
	 *
	 * Computes determinant of matrix m, returning d
	 */
	static final void DETERMINANT_2X2(final RefFloat d, mat2f m)					
	{								
	   d.d = m.f[M(0,0)] * m.f[M(1,1)] - m.f[M(0,1)] * m.f[M(1,0)];			
	}


	/** determinant of matrix
	 *
	 * Computes determinant of matrix m, returning d
	 */
	static final void DETERMINANT_3X3(final RefFloat d, mat3f m)					
	{								
	   d.d = m.f[M(0,0)] * (m.f[M(1,1)]*m.f[M(2,2)] - m.f[M(1,2)] * m.f[M(2,1)]);		
	   d.d -= m.f[M(0,1)] * (m.f[M(1,0)]*m.f[M(2,2)] - m.f[M(1,2)] * m.f[M(2,0)]);	
	   d.d += m.f[M(0,2)] * (m.f[M(1,0)]*m.f[M(2,1)] - m.f[M(1,1)] * m.f[M(2,0)]);	
	}


	/** i,j,th cofactor of a 4x4 matrix
	 *
	 */
	static final float COFACTOR_4X4_IJ(final mat4f m, final int i, final int j) 				
	{			
		float fac = 0;
	   int __ii[]=new int[4], __jj[]=new int[4], __k;						
									
	   for (__k=0; __k<i; __k++) __ii[__k] = __k;				
	   for (__k=i; __k<3; __k++) __ii[__k] = __k+1;				
	   for (__k=0; __k<j; __k++) __jj[__k] = __k;				
	   for (__k=j; __k<3; __k++) __jj[__k] = __k+1;				
									
	   (fac) = m.f[M(__ii[0],__jj[0])] * (m.f[M(__ii[1],__jj[1])]*m.f[M(__ii[2],__jj[2])] 	
	                            - m.f[M(__ii[1],__jj[2])]*m.f[M(__ii[2],__jj[1])]); 
	   (fac) -= m.f[M(__ii[0],__jj[1])] * (m.f[M(__ii[1],__jj[0])]*m.f[M(__ii[2],__jj[2])]	
	                             - m.f[M(__ii[1],__jj[2])]*m.f[M(__ii[2],__jj[0])]);
	   (fac) += m.f[M(__ii[0],__jj[2])] * (m.f[M(__ii[1],__jj[0])]*m.f[M(__ii[2],__jj[1])]	
	                             - m.f[M(__ii[1],__jj[1])]*m.f[M(__ii[2],__jj[0])]);
									
	   __k = i+j;							
	   if ( __k != (__k/2)*2) {						
	      (fac) = -(fac);						
	   }		
	   return fac;
	}


	/** determinant of matrix
	 *
	 * Computes determinant of matrix m, returning d
	 */
	static final void DETERMINANT_4X4(final RefFloat d, final mat4f m)					
	{								
	   double cofac;						
	   cofac = COFACTOR_4X4_IJ (m, 0, 0);				
	   d.d = (float) (m.f[M(0,0)] * cofac);						
	   cofac = COFACTOR_4X4_IJ (m, 0, 1);				
	   d.d += m.f[M(0,1)] * cofac;					
	   cofac = COFACTOR_4X4_IJ (m, 0, 2);				
	   d.d += m.f[M(0,2)] * cofac;					
	   cofac = COFACTOR_4X4_IJ (m, 0, 3);				
	   d.d += m.f[M(0,3)] * cofac;					
	}


	/** cofactor of matrix
	 *
	 * Computes cofactor of matrix m, returning a
	 */
	static final void COFACTOR_2X2(mat2f a, mat2f m)					
	{								
	   a.f[M(0,0)] = (m).f[M(1,1)];						
	   a.f[M(0,1)] = - (m).f[M(1,0)];						
	   a.f[M(1,0)] = - (m).f[M(0,1)];						
	   a.f[M(1,1)] = (m).f[M(0,0)];						
	}


//	/** cofactor of matrix
//	 *
//	 * Computes cofactor of matrix m, returning a
//	 */
//	private static final void COFACTOR_3X3(mat3f a, mat3f m)					
//	{								
//	   a.f[M(0,0)] = m.f[M(1,1)]*m.f[M(2,2)] - m.f[M(1,2)]*m.f[M(2,1)];			
//	   a.f[M(0,1)] = - (m.f[M(1,0)]*m.f[M(2,2)] - m.f[M(2,0)]*m.f[M(1,2)]);		
//	   a.f[M(0,2)] = m.f[M(1,0)]*m.f[M(2,1)] - m.f[M(1,1)]*m.f[M(2,0)];			
//	   a.f[M(1,0)] = - (m.f[M(0,1)]*m.f[M(2,2)] - m.f[M(0,2)]*m.f[M(2,1)]);		
//	   a.f[M(1,1)] = m.f[M(0,0)]*m.f[M(2,2)] - m.f[M(0,2)]*m.f[M(2,0)];			
//	   a.f[M(1,2)] = - (m.f[M(0,0)]*m.f[M(2,1)] - m.f[M(0,1)]*m.f[M(2,0)]);		
//	   a.f[M(2,0)] = m.f[M(0,1)]*m.f[M(1,2)] - m.f[M(0,2)]*m.f[M(1,1)];			
//	   a.f[M(2,1)] = - (m.f[M(0,0)]*m.f[M(1,2)] - m.f[M(0,2)]*m.f[M(1,0)]);		
//	   a.f[M(2,2)] = m.f[M(0,0)]*m.f[M(1,1)] - m.f[M(0,1)]*m.f[M(1,0)];		
//	}


	/** cofactor of matrix
	 *
	 * Computes cofactor of matrix m, returning a
	 */
	static final void COFACTOR_4X4(final mat4f a, final mat4f m)					
	{								
	   int i,j;							
									
	   for (i=0; i<4; i++) {					
	      for (j=0; j<4; j++) {					
	    	  a.f[M(i,j)] = COFACTOR_4X4_IJ (m, i, j);			
	      }								
	   }								
	}


	/** adjoint of matrix
	 *
	 * Computes adjoint of matrix m, returning a
	 * (Note that adjoint is just the transpose of the cofactor matrix)
	 */
	static final void ADJOINT_2X2(mat2f a, mat2f m)					
	{								
	   a.f[M(0,0)] = (m).f[M(1,1)];						
	   a.f[M(1,0)] = - (m).f[M(1,0)];						
	   a.f[M(0,1)] = - (m).f[M(0,1)];						
	   a.f[M(1,1)] = (m).f[M(0,0)];						
	}


//	/** adjoint of matrix
//	 *
//	 * Computes adjoint of matrix m, returning a
//	 * (Note that adjoint is just the transpose of the cofactor matrix)
//	 */
//	private static final void ADJOINT_3X3(mat3f a, mat3f m)					
//	{								
//	   a.f[M(0,0)] = m.f[M(1,1)]*m.f[M(2,2)] - m.f[M(1,2)]*m.f[M(2,1)];			
//	   a.f[M(1,0)] = - (m.f[M(1,0)]*m.f[M(2,2)] - m.f[M(2,0)]*m.f[M(1,2)]);		
//	   a.f[M(2,0)] = m.f[M(1,0)]*m.f[M(2,1)] - m.f[M(1,1)]*m.f[M(2,0)];			
//	   a.f[M(0,1)] = - (m.f[M(0,1)]*m.f[M(2,2)] - m.f[M(0,2)]*m.f[M(2,1)]);		
//	   a.f[M(1,1)] = m.f[M(0,0)]*m.f[M(2,2)] - m.f[M(0,2)]*m.f[M(2,0)];			
//	   a.f[M(2,1)] = - (m.f[M(0,0)]*m.f[M(2,1)] - m.f[M(0,1)]*m.f[M(2,0)]);		
//	   a.f[M(0,2)] = m.f[M(0,1)]*m.f[M(1,2)] - m.f[M(0,2)]*m.f[M(1,1)];			
//	   a.f[M(1,2)] = - (m.f[M(0,0)]*m.f[M(1,2)] - m.f[M(0,2)]*m.f[M(1,0)]);		
//	   a.f[M(2,2)] = m.f[M(0,0)]*m.f[M(1,1)] - m.f[M(0,1)]*m.f[M(1,0)];		
//	}


	/** adjoint of matrix
	 *
	 * Computes adjoint of matrix m, returning a
	 * (Note that adjoint is just the transpose of the cofactor matrix)
	 */
	static final void ADJOINT_4X4(mat4f a, mat4f m)					
	{								
	   char _i_,_j_;							
									
	   for (_i_=0; _i_<4; _i_++) {					
	      for (_j_=0; _j_<4; _j_++) {					
	    	  a.f[M(_j_,_i_)] = COFACTOR_4X4_IJ (m, _i_, _j_);			
	      }								
	   }								
	}


	/** compute adjoint of matrix and scale
	 *
	 * Computes adjoint of matrix m, scales it by s, returning a
	 */
	static final void SCALE_ADJOINT_2X2(mat2f a, final float s, mat2f m)				
	{								
	   a.f[M(0,0)] = (s) * m.f[M(1,1)];					
	   a.f[M(1,0)] = - (s) * m.f[M(1,0)];					
	   a.f[M(0,1)] = - (s) * m.f[M(0,1)];					
	   a.f[M(1,1)] = (s) * m.f[M(0,0)];					
	}


	/** compute adjoint of matrix and scale
	 *
	 * Computes adjoint of matrix m, scales it by s, returning a
	 */
	static final void SCALE_ADJOINT_3X3(mat3f a, final float s, mat3f m)				
	{								
	   a.f[M(0,0)] = (s) * (m.f[M(1,1)] * m.f[M(2,2)] - m.f[M(1,2)] * m.f[M(2,1)]);	
	   a.f[M(1,0)] = (s) * (m.f[M(1,2)] * m.f[M(2,0)] - m.f[M(1,0)] * m.f[M(2,2)]);	
	   a.f[M(2,0)] = (s) * (m.f[M(1,0)] * m.f[M(2,1)] - m.f[M(1,1)] * m.f[M(2,0)]);	
									
	   a.f[M(0,1)] = (s) * (m.f[M(0,2)] * m.f[M(2,1)] - m.f[M(0,1)] * m.f[M(2,2)]);	
	   a.f[M(1,1)] = (s) * (m.f[M(0,0)] * m.f[M(2,2)] - m.f[M(0,2)] * m.f[M(2,0)]);	
	   a.f[M(2,1)] = (s) * (m.f[M(0,1)] * m.f[M(2,0)] - m.f[M(0,0)] * m.f[M(2,1)]);	
									
	   a.f[M(0,2)] = (s) * (m.f[M(0,1)] * m.f[M(1,2)] - m.f[M(0,2)] * m.f[M(1,1)]);	
	   a.f[M(1,2)] = (s) * (m.f[M(0,2)] * m.f[M(1,0)] - m.f[M(0,0)] * m.f[M(1,2)]);	
	   a.f[M(2,2)] = (s) * (m.f[M(0,0)] * m.f[M(1,1)] - m.f[M(0,1)] * m.f[M(1,0)]);	
	}


	/** compute adjoint of matrix and scale
	 *
	 * Computes adjoint of matrix m, scales it by s, returning a
	 */
	static final void SCALE_ADJOINT_4X4(mat4f a, final float s, mat4f m)				
	{								
	   char _i_,_j_; 
	   for (_i_=0; _i_<4; _i_++) {					
	      for (_j_=0; _j_<4; _j_++) {					
	    	  a.f[M(_j_,_i_)] = COFACTOR_4X4_IJ (m, _i_, _j_);			
	         a.f[M(_j_,_i_)] *= s;						
	      }								
	   }								
	}

	/** inverse of matrix
	 *
	 * Compute inverse of matrix a, returning determinant m and
	 * inverse b
	 */
	static final void INVERT_2X2(final mat2f b, final RefFloat det, final mat2f a)			
	{						
	   float _tmp_;					
	   DETERMINANT_2X2 (det, a);			
	   _tmp_ = 1.0f / (det.d);				
	   SCALE_ADJOINT_2X2 (b, _tmp_, a);		
	}


	/** inverse of matrix
	 *
	 * Compute inverse of matrix a, returning determinant m and
	 * inverse b
	 */
	static final void INVERT_3X3(final mat3f b, final RefFloat det, final mat3f a)			
	{						
	   float _tmp_;					
	   DETERMINANT_3X3 (det, a);			
	   _tmp_ = 1.0f / (det.d);				
	   SCALE_ADJOINT_3X3 (b, _tmp_, a);		
	}


	/** inverse of matrix
	 *
	 * Compute inverse of matrix a, returning determinant m and
	 * inverse b
	 */
	static final void INVERT_4X4(final mat4f b, final RefFloat det, final mat4f a)			
	{						
	   float _tmp_;					
	   DETERMINANT_4X4 (det, a);			
	   _tmp_ = 1.0f / (det.d);				
	   SCALE_ADJOINT_4X4 (b, _tmp_, a);		
	}

	//! @}

	/*! \defgroup BOUND_AABB_OPERATIONS
	*/
	//! @{

	//!Initializes an AABB
	static final void INVALIDATE_AABB(aabb3f aabb) {
	    (aabb).minX = G_REAL_INFINITY;
	    (aabb).maxX = G_REAL_INFINITY_N;
	    (aabb).minY = G_REAL_INFINITY;
	    (aabb).maxY = G_REAL_INFINITY_N;
	    (aabb).minZ = G_REAL_INFINITY;
	    (aabb).maxZ = G_REAL_INFINITY_N;
	}

	static final void AABB_GET_MIN(aabb3f aabb, vec3f vmin) {
	    vmin.f[0] = (aabb).minX;
	    vmin.f[1] = (aabb).minY;
	    vmin.f[2] = (aabb).minZ;
	}

	static final void AABB_GET_MAX(aabb3f aabb, vec3f vmax) {
	    vmax.f[0] = (aabb).maxX;
	    vmax.f[1] = (aabb).maxY;
	    vmax.f[2] = (aabb).maxZ;
	}

	//!Copy boxes
	static final void AABB_COPY(aabb3f dest_aabb, aabb3f src_aabb)
	{
	    (dest_aabb).minX = (src_aabb).minX;
	    (dest_aabb).maxX = (src_aabb).maxX;
	    (dest_aabb).minY = (src_aabb).minY;
	    (dest_aabb).maxY = (src_aabb).maxY;
	    (dest_aabb).minZ = (src_aabb).minZ;
	    (dest_aabb).maxZ = (src_aabb).maxZ;
	}

	//! Computes an Axis aligned box from  a triangle
	static final void COMPUTEAABB_FOR_TRIANGLE(aabb3f aabb, vec3f V1, vec3f V2, vec3f V3) {
	    (aabb).minX = MIN3(V1.f[0],V2.f[0],V3.f[0]);
	    (aabb).maxX = MAX3(V1.f[0],V2.f[0],V3.f[0]);
	    (aabb).minY = MIN3(V1.f[1],V2.f[1],V3.f[1]);
	    (aabb).maxY = MAX3(V1.f[1],V2.f[1],V3.f[1]);
	    (aabb).minZ = MIN3(V1.f[2],V2.f[2],V3.f[2]);
	    (aabb).maxZ = MAX3(V1.f[2],V2.f[2],V3.f[2]);
	}
	static final void COMPUTEAABB_FOR_TRIANGLE(aabb3f aabb, FloatArray V1, int o1, 
			FloatArray V2, int o2, FloatArray V3, int o3) {
	    (aabb).minX = MIN3(V1.at(o1+0),V2.at(o2+0),V3.at(o3+0));
	    (aabb).maxX = MAX3(V1.at(o1+0),V2.at(o2+0),V3.at(o3+0));
	    (aabb).minY = MIN3(V1.at(o1+1),V2.at(o2+1),V3.at(o3+1));
	    (aabb).maxY = MAX3(V1.at(o1+1),V2.at(o2+1),V3.at(o3+1));
	    (aabb).minZ = MIN3(V1.at(o1+2),V2.at(o2+2),V3.at(o3+2));
	    (aabb).maxZ = MAX3(V1.at(o1+2),V2.at(o2+2),V3.at(o3+2));
	}

	//! Merge two boxes to destaabb
	static final void MERGEBOXES(aabb3f destaabb, aabb3f aabb) {
	    (destaabb).minX = MIN((aabb).minX,(destaabb).minX);
	    (destaabb).minY = MIN((aabb).minY,(destaabb).minY);
	    (destaabb).minZ = MIN((aabb).minZ,(destaabb).minZ);
	    (destaabb).maxX = MAX((aabb).maxX,(destaabb).maxX);
	    (destaabb).maxY = MAX((aabb).maxY,(destaabb).maxY);
	    (destaabb).maxZ = MAX((aabb).maxZ,(destaabb).maxZ);
	}

	//! Extends the box
	static final void AABB_POINT_EXTEND(aabb3f destaabb, vec3f p) {
	    (destaabb).minX = MIN(p.f[0],(destaabb).minX);
	    (destaabb).maxX = MAX(p.f[0],(destaabb).maxX);
	    (destaabb).minY = MIN(p.f[1],(destaabb).minY);
	    (destaabb).maxY = MAX(p.f[1],(destaabb).maxY);
	    (destaabb).minZ = MIN(p.f[2],(destaabb).minZ);
	    (destaabb).maxZ = MAX(p.f[2],(destaabb).maxZ);
	}

	//! Finds the intersection box of two boxes
	static final void BOXINTERSECTION(aabb3f aabb1, aabb3f aabb2, aabb3f iaabb) {
	    (iaabb).minX = MAX((aabb1).minX,(aabb2).minX);
	    (iaabb).minY = MAX((aabb1).minY,(aabb2).minY);
	    (iaabb).minZ = MAX((aabb1).minZ,(aabb2).minZ);
	    (iaabb).maxX = MIN((aabb1).maxX,(aabb2).maxX);
	    (iaabb).maxY = MIN((aabb1).maxY,(aabb2).maxY);
	    (iaabb).maxZ = MIN((aabb1).maxZ,(aabb2).maxZ);
	}

	//! Determines if two aligned boxes do intersect
	static final boolean AABBCOLLISION(aabb3f aabb1, aabb3f aabb2) {
		//intersected.i = 1;
		if ((aabb1).minX > (aabb2).maxX ||
	        (aabb1).maxX < (aabb2).minX ||
	        (aabb1).minY > (aabb2).maxY ||
	        (aabb1).maxY < (aabb2).minY ||
	        (aabb1).minZ > (aabb2).maxZ ||
	        (aabb1).maxZ < (aabb2).minZ )
		{
			return false;
		}
		return true;
	}

	static final void AXIS_INTERSECT(final float min, final float max, 
			final float a, final float d, 
			final RefFloat tfirst, final RefFloat tlast, final RefBoolean is_intersected) {
		if(IS_ZERO(d))
		{
	        is_intersected.b = !(a < min || a > max);
		}
		else
		{
	        float a0, a1;
	        a0 = (min - a) / (d);
	        a1 = (max - a) / (d);
	        if(a0 > a1) { float ax=a0; a0=a1; a1=ax; }//  SWAP_NUMBERS(a0, a1);
	        tfirst.d = MAX(a0, tfirst.d);
	        tlast.d = MIN(a1, tlast.d);
	        if (tlast.d < tfirst.d)
	        {
	             is_intersected.b = false;
	        }
	        else
	        {
	            is_intersected.b = true;
	        }
		}
	}

	/**
	 * Finds the Ray intersection parameter.
	 * 
	 * @param aabb Aligned box
	 * @param vorigin A vec3f with the origin of the ray
	 * @param vdir A vec3f with the direction of the ray
	 * @param tparam Output parameter
	 * @param tmax Max lenght of the ray
	 * @param is_intersected 1 if the ray collides the box, else false
 	 */
	static final void BOX_INTERSECTS_RAY(final aabb3f aabb, final vec3f vorigin, 
			final vec3f vdir, final RefFloat tparam, final float tmax, 
			final RefBoolean is_intersected) { 
		//float _tfirst = 0.0f, _tlast = tmax;
		RefFloat _tfirst = new RefFloat(0.0f), _tlast = new RefFloat(tmax);
		AXIS_INTERSECT(aabb.minX,aabb.maxX,vorigin.f[0], vdir.f[0], _tfirst, _tlast,is_intersected);
		if(is_intersected.b)
		{
	        AXIS_INTERSECT(aabb.minY,aabb.maxY,vorigin.f[1], vdir.f[1], _tfirst, _tlast,is_intersected);
		}
		if(is_intersected.b)
		{
	        AXIS_INTERSECT(aabb.minZ,aabb.maxZ,vorigin.f[2], vdir.f[2], _tfirst, _tlast,is_intersected);
		}
		tparam.d = _tfirst.d;
	}

	static final void AABB_PROJECTION_INTERVAL(final aabb3f aabb, 
			final vec4f direction, final RefFloat vmin, final RefFloat vmax)
	{
	    float _center[] = { //TODO TZ optimize? use primitive variables? Store extent/centre in AABB?
	    		(aabb.minX + aabb.maxX)*0.5f, 
	    		(aabb.minY + aabb.maxY)*0.5f, 
	    		(aabb.minZ + aabb.maxZ)*0.5f};
	    
	    float _extend[] = {
	    		aabb.maxX-_center[0],
	    		aabb.maxY-_center[1],
	    		aabb.maxZ-_center[2]};
	    float _fOrigin =  VEC_DOT(direction,_center);
		float _fMaximumExtent = _extend[0]*Math.abs(direction.f[0]) + 
	                            _extend[1]*Math.abs(direction.f[1]) + 
	                            _extend[2]*Math.abs(direction.f[2]);
	
	    vmin.d = _fOrigin - _fMaximumExtent; 
	    vmax.d = _fOrigin + _fMaximumExtent; 
	}

//	/**
//	 * classify values:
//	 * <ol>
//	 * <li> 0 : In back of plane
//	 * <li> 1 : Spanning
//	 * <li> 2 : In front of
//	 * </ol>
//	 */
//	private static final void PLANE_CLASSIFY_BOX(final vec4f plane, 
//			final aabb3f aabb, final RefInt classify)
//	{
//		RefFloat _fmin = new RefFloat(),_fmax = new RefFloat(); 
//		AABB_PROJECTION_INTERVAL(aabb,plane, _fmin, _fmax); 
//		if(plane.f[3] >= _fmax.d) 
//		{ 
//			classify.i = 0;/*In back of*/ 
//		} 
//		else 
//		{ 
//			if(plane.f[3]+0.000001f>=_fmin.d) 
//			{ 
//				classify.i = 1;/*Spanning*/ 
//			} 
//			else 
//			{ 
//				classify.i = 2;/*In front of*/ 
//			} 
//		} 
//	}
	static final int PLANE_CLASSIFY_BOX(final vec4f plane, final aabb3f aabb)
	{
		RefFloat _fmin = new RefFloat(),_fmax = new RefFloat(); 
		AABB_PROJECTION_INTERVAL(aabb,plane, _fmin, _fmax); 
		if(plane.f[3] >= _fmax.d) 
		{ 
			return 0;/*In back of*/ 
		} 
		else 
		{ 
			if(plane.f[3]+0.000001f>=_fmin.d) 
			{ 
				return 1;/*Spanning*/ 
			} 
			else 
			{ 
				return 2;/*In front of*/ 
			} 
		} 
	}
	//TODO remove
	static final int PLANE_CLASSIFY_BOX_TZ(final vec4f plane, final aabb3f aabb)
	{
		RefFloat _fmin = new RefFloat(),_fmax = new RefFloat(); 
		AABB_PROJECTION_INTERVAL(aabb,plane, _fmin, _fmax); 
		if(plane.f[3] >= _fmax.d) 
		{ 
			return 0;/*In back of*/ 
		} 
		else 
		{ 
			if(plane.f[3]+0.000001f>=_fmin.d) 
			{ 
				return 1;/*Spanning*/ 
			} 
			else 
			{ 
				return 2;/*In front of*/ 
			} 
		} 
	}
	//! @}

	/*! \defgroup GEOMETRIC_OPERATIONS
	*/
	//! @{


	static final float PLANEDIREPSILON = 0.0000001f;
	static final float PARALELENORMALS = 0.000001f;

	static final void TRIANGLE_NORMAL(final vec3f v1,final  vec3f v2, 
			final vec3f v3, final vec4f n){
	    vec3f _dif1 = new vec3f(),_dif2 = new vec3f(); 
	    VEC_DIFF(_dif1,v2,v1); 
	    VEC_DIFF(_dif2,v3,v1); 
	    VEC_CROSS(n,_dif1,_dif2); 
	    VEC_NORMALIZE(n); 
	}

	/// plane is a vec4f
	static final void TRIANGLE_PLANE(final vec3f v1, final vec3f v2, final vec3f v3, final vec4f plane) {
	    TRIANGLE_NORMAL(v1,v2,v3,plane);
	    plane.f[3] = VEC_DOT(plane,v1);//,plane);
	}

	/// Calc a plane from an edge an a normal. plane is a vec4f
	static final void EDGE_PLANE(final vec3f e1, final vec3f e2, final vec4f n, final vec4f plane) {
	    vec3f _dif = new vec3f(); 
	    VEC_DIFF(_dif,e2,e1); 
	    VEC_CROSS(plane,_dif,n); 
	    VEC_NORMALIZE(plane); 
	    plane.f[3] = VEC_DOT(plane,e1);//,plane);
	}

	static final float DISTANCE_PLANE_POINT(final vec4f plane, final vec3f point) { 
		return (VEC_DOT(plane,point) - plane.f[3]); 
	}

	static final void PROJECT_POINT_PLANE(final vec3f point, final vec4f plane, final vec3f projected) {
		float _dis;
		_dis = DISTANCE_PLANE_POINT(plane,point);
		VEC_SCALE(projected,-_dis,plane);
		VEC_SUM(projected,projected,point);	
	}

	//static final void POINT_IN_HULL(vec3f point, vec4f[] planes, final int plane_count, RefInt outside)
	static final int POINT_IN_HULL(final vec3f point, final vec4f[] planes, final int plane_count)
	{
		float _dis;
		int outside = 0;
		int _i = 0;
		do
		{
		    _dis = DISTANCE_PLANE_POINT(planes[_i],point);
		    if(_dis>0.0f) outside = 1;
		    _i++;
		}while(_i<plane_count&&outside==0);
		return outside;
	}
	static final int POINT_IN_HULL_TZ(final vec3f point, final vec4f[] planes, 
			final int pos, final int plane_count)
	{
		float _dis;
		int outside = 0;
		int _i = 0;
		do
		{
		    _dis = DISTANCE_PLANE_POINT(planes[_i+pos],point);
		    if(_dis>0.0f) outside = 1;
		    _i++;
		}while(_i<plane_count&&outside==0);
		return outside;
	}


	static final void PLANE_CLIP_SEGMENT(final vec3f s1, final vec3f s2, 
			final vec4f plane, final vec3f clipped) {
		float _dis1,_dis2;
		_dis1 = DISTANCE_PLANE_POINT(plane,s1);
		VEC_DIFF(clipped,s2,s1);
		_dis2 = VEC_DOT(plane,clipped);//,plane);
		VEC_SCALE(clipped,-_dis1/_dis2,clipped);
		VEC_SUM(clipped,clipped,s1);	
	}

	//! Confirms if the plane intersect the edge or nor
	/*!
	intersection type must have the following values
	<ul>
	<li> 0 : Segment in front of plane, s1 closest
	<li> 1 : Segment in front of plane, s2 closest
	<li> 2 : Segment in back of plane, s1 closest
	<li> 3 : Segment in back of plane, s2 closest
	<li> 4 : Segment collides plane, s1 in back
	<li> 5 : Segment collides plane, s2 in back
	</ul>
	*/
	//static final void PLANE_CLIP_SEGMENT2(vec3f s1, vec3f s2, vec4f plane, vec3f clipped, RefInt intersection_type) 
	static final int PLANE_CLIP_SEGMENT2(final vec3f s1, final vec3f s2, 
			final vec4f plane, final vec3f clipped)
	{
		int intersection_type;
		float _dis1,_dis2;
		_dis1 = DISTANCE_PLANE_POINT(plane,s1);
		_dis2 = DISTANCE_PLANE_POINT(plane,s2);
		if(_dis1 >-G_EPSILON && _dis2 >-G_EPSILON)
		{
		    if(_dis1<_dis2) intersection_type = 0;
		    else  intersection_type = 1;
		}
		else if(_dis1 <G_EPSILON && _dis2 <G_EPSILON)
		{
		    if(_dis1>_dis2) intersection_type = 2;
		    else  intersection_type = 3;	    
		}
		else
		{
		    if(_dis1<_dis2) intersection_type = 4;
		    else  intersection_type = 5;
	        VEC_DIFF(clipped,s2,s1);
	        _dis2 = VEC_DOT(plane,clipped);//,plane);
	        VEC_SCALE(clipped,-_dis1/_dis2,clipped);
	        VEC_SUM(clipped,clipped,s1);	
		}
		return intersection_type;
	}

	//! Confirms if the plane intersect the edge or not
	/*!
	clipped1 and clipped2 are the vertices behind the plane.
	clipped1 is the closest

	intersection_type must have the following values
	<ul>
	<li> 0 : Segment in front of plane, s1 closest
	<li> 1 : Segment in front of plane, s2 closest
	<li> 2 : Segment in back of plane, s1 closest
	<li> 3 : Segment in back of plane, s2 closest
	<li> 4 : Segment collides plane, s1 in back
	<li> 5 : Segment collides plane, s2 in back
	</ul>
	*/
//	static final void PLANE_CLIP_SEGMENT_CLOSEST(vec3f s1, vec3f s2, vec4f plane, vec3f clipped1,
//			vec3f clipped2, RefInt intersection_type)
	static final int PLANE_CLIP_SEGMENT_CLOSEST(final vec3f s1, 
			final vec3f s2, final vec4f plane, final vec3f clipped1,
			final vec3f clipped2)
	{
		int intersection_type = PLANE_CLIP_SEGMENT2(s1,s2,plane,clipped1);
		if(intersection_type == 0)
		{
		    VEC_COPY(clipped1,s1);
		    VEC_COPY(clipped2,s2);
		}
		else if(intersection_type == 1)
		{
		    VEC_COPY(clipped1,s2);
		    VEC_COPY(clipped2,s1);
		}
		else if(intersection_type == 2)
		{
		    VEC_COPY(clipped1,s1);
		    VEC_COPY(clipped2,s2);
		}
		else if(intersection_type == 3)
		{
		    VEC_COPY(clipped1,s2);
		    VEC_COPY(clipped2,s1);
		}
		else if(intersection_type == 4)
		{	    
		    VEC_COPY(clipped2,s1);
		}
		else if(intersection_type == 5)
		{	    
		    VEC_COPY(clipped2,s2);
		}
		return intersection_type;
	}


	//! Finds the 2 smallest cartesian coordinates of a plane normal
	static final void PLANE_MINOR_AXES(final vec4f plane, 
			final RefInt i0, final RefInt i1)
	{
	    float A[] = {Math.abs(plane.f[0]),Math.abs(plane.f[1]),Math.abs(plane.f[2])};
	    if(A[0]>A[1])
		{
			if(A[0]>A[2])
			{
				i0.i=1;  /* A[0] is greatest */ 
				i1.i=2;
			}
			else 
			{
				i0.i=0;      /* A[2] is greatest */ 
				i1.i=1; 
			}
		}
		else   /* A[0]<=A[1] */ 
		{
			if(A[2]>A[1]) 
			{ 
				i0.i=0;      /* A[2] is greatest */ 
				i1.i=1; 
			}
			else 
			{
				i0.i=0;      /* A[1] is greatest */ 
				i1.i=2; 
			}
		} 
	}

	//! Ray plane collision
	static final void RAY_PLANE_COLLISION(final vec4f plane, final vec3f vDir, 
			final vec3f vPoint, final vec3f pout, 
			final RefFloat tparam, final RefBoolean does_intersect)
	{
		float _dis,_dotdir;	
		_dotdir = VEC_DOT(plane,vDir);
		if(_dotdir<PLANEDIREPSILON)
		{
		    does_intersect.b = false;
		}
		else
		{
	        _dis = DISTANCE_PLANE_POINT(plane,vPoint);	
	        tparam.d = -_dis/_dotdir;
	        VEC_SCALE(pout,tparam.d,vDir);
	        VEC_SUM(pout,vPoint,pout);	
	        does_intersect.b = true;
		}
	}

	//! Bidireccional ray
	static final void LINE_PLANE_COLLISION(final vec4f plane, final vec3f vDir, 
			final vec3f vPoint, final vec3f pout, final RefFloat tparam, 
			final float tmin, final float tmax)
	{
	    tparam.d = -DISTANCE_PLANE_POINT(plane,vPoint);
		tparam.d /= VEC_DOT(plane,vDir);
	    tparam.d = CLAMP(tparam.d,tmin,tmax);
	    VEC_SCALE(pout,tparam.d,vDir);
	    VEC_SUM(pout,vPoint,pout);	
	}

//	/*! \brief Returns the Ray on which 2 planes intersect if they do.
//	    Written by Rodrigo Hernandez on ODE convex collision
//
//	  \param p1 Plane 1
//	  \param p2 Plane 2
//	  \param p Contains the origin of the ray upon returning if planes intersect
//	  \param d Contains the direction of the ray upon returning if planes intersect
//	  \param dointersect 1 if the planes intersect, 0 if paralell.
//
//	*/
//	private static final void INTERSECT_PLANES(final vec3f p1, final vec3f p2, 
//			final vec3f p, final vec3f d, final RefBoolean dointersect) 
//	{ 
//	  VEC_CROSS(d,p1,p2); 
//	  float denom = VEC_DOT(d, d);
//	  if (IS_ZERO(denom)) 
//	  { 
//	      dointersect.b = false; 
//	  } 
//	  else 
//	  { 
//	      vec3f _n = new vec3f();
//	      _n.f[0]=p1.f[3]*p2.f[0] - p2.f[3]*p1.f[0]; 
//	      _n.f[1]=p1.f[3]*p2.f[1] - p2.f[3]*p1.f[1]; 
//	      _n.f[2]=p1.f[3]*p2.f[2] - p2.f[3]*p1.f[2]; 
//	      VEC_CROSS(p,_n,d); 
//	      p.f[0]/=denom; 
//	      p.f[1]/=denom; 
//	      p.f[2]/=denom; 
//	      dointersect.b = true; 
//	  }
//	}

	//***************** SEGMENT and LINE FUNCTIONS **********************************///

	/*! Finds the closest point(cp) to (v) on a segment (e1,e2)
	 */
	static final void CLOSEST_POINT_ON_SEGMENT(final vec3f cp, final vec3f v, 
			final vec3f e1, final vec3f e2)			
	{ 
	    vec3f _n = new vec3f();
	    VEC_DIFF(_n,e2,e1);
	    VEC_DIFF(cp,v,e1);
		float _scalar = VEC_DOT(cp, _n);			
		_scalar/= VEC_DOT(_n, _n); 
		if(_scalar <0.0f)
		{
		    VEC_COPY(cp,e1);
		}
		else if(_scalar >1.0f)
		{
		    VEC_COPY(cp,e2);
		}
		else 
		{
	        VEC_SCALE(cp,_scalar,_n);
	        VEC_SUM(cp,cp,e1);
		}	
	}


//	/*! \brief Finds the line params where these lines intersect.
//
//	\param dir1 Direction of line 1
//	\param point1 Point of line 1
//	\param dir2 Direction of line 2
//	\param point2 Point of line 2
//	\param t1 Result Parameter for line 1
//	\param t2 Result Parameter for line 2
//	\param dointersect  0  if the lines won't intersect, else 1
//
//	*/
//	private static final void LINE_INTERSECTION_PARAMS(final vec3f dir1, 
//			final vec3f point1, final vec3f dir2, final vec3f point2, 
//			final RefFloat t1, final RefFloat t2, final RefBoolean dointersect) {
//	    float det;
//		float e1e1 = VEC_DOT(dir1,dir1);
//		float e1e2 = VEC_DOT(dir1,dir2);
//		float e2e2 = VEC_DOT(dir2,dir2);
//		vec3f p1p2 = new vec3f();
//	    VEC_DIFF(p1p2,point1,point2);
//	    float p1p2e1 = VEC_DOT(p1p2,dir1);
//		float p1p2e2 = VEC_DOT(p1p2,dir2);
//		det = e1e2*e1e2 - e1e1*e2e2;
//		if(IS_ZERO(det))
//		{
//		     dointersect.b = false;
//		}
//		else
//		{
//	        t1.d = (e1e2*p1p2e2 - e2e2*p1p2e1)/det;
//	        t2.d = (e1e1*p1p2e2 - e1e2*p1p2e1)/det;
//	        dointersect.b = true;
//		}
//	}

	//! Find closest points on segments
	static final void SEGMENT_COLLISION(final vec3f vA1, final vec3f vA2, 
			final vec3f vB1, final vec3f vB2,
			final vec3f vPointA, final vec3f vPointB)
	{
	    vec3f _AD = new vec3f(), _BD = new vec3f(), _N = new vec3f();
	    vec4f _M = new vec4f();
	    VEC_DIFF(_AD,vA2,vA1);
	    VEC_DIFF(_BD,vB2,vB1);
	    VEC_CROSS(_N,_AD,_BD);
	    VEC_CROSS(_M,_N,_BD);
	    _M.f[3] = VEC_DOT(_M,vB1);
	    RefFloat _tp = new RefFloat(); //TZ TODO remove 
	    LINE_PLANE_COLLISION(_M,_AD,vA1,vPointA,_tp,0.0f, 1.0f);
	    /*Closest point on segment*/ 
	    VEC_DIFF(vPointB,vPointA,vB1);
		_tp.d = VEC_DOT(vPointB, _BD);			
		_tp.d/= VEC_DOT(_BD, _BD); 
		_tp.d = CLAMP(_tp.d,0.0f,1.0f);	
	    VEC_SCALE(vPointB,_tp.d,_BD);
	    VEC_SUM(vPointB,vPointB,vB1);
	}

	private GimGeometry() {
		super();
	}
}
