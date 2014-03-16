/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal;

import static org.ode4j.ode.OdeMath.dCalcVectorCross3;
import static org.ode4j.ode.OdeMath.dCalcVectorDot3;
import static org.ode4j.ode.OdeMath.dCalcVectorDot3_41;
import static org.ode4j.ode.OdeMath.dCalcVectorDot3_44;
import static org.ode4j.ode.OdeMath.dMultiply0_331;
import static org.ode4j.ode.OdeMath.dMultiply1_331;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Common.dFabs;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dRecip;
import static org.ode4j.ode.internal.cpp4j.Cmath.asin;
import static org.ode4j.ode.internal.cpp4j.Cmath.atan2;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector4;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.cpp4j.java.RefInt;


/**
 * some useful collision utility stuff. this includes some API utility
 * functions that are defined in the public header files.
 */
public class DxCollisionUtil {

	//given a pointer `p' to a dContactGeom, return the dContactGeom at
	//p + skip bytes.
	//#define CONTACT(p,skip) ((dContactGeom*) (((char*)p) + (skip)))
//	/**
//	 * given a pointer `p' to a dContactGeom, return the dContactGeom at
//	 * p + skip bytes.
//	 * @deprecated Instead, please use: <tt>dContagGeomBuffer.get(skip);</tt>
//	 */
//	DContactGeom CONTACT(DContactGeom[] p, int skip) {
//		throw new UnsupportedOperationException();
//	}

	//#if 1
	//#include "collision_kernel.h"
	//Fetches a contact
	//inline dContactGeom* SAFECONTACT(int Flags, dContactGeom* Contacts, int Index, int Stride){
	final DContactGeom[] SAFECONTACT(int Flags, DContactGeom[] Contacts, int Index, int Stride){
		dIASSERT(Index >= 0 && Index < (Flags & DxGeom.NUMC_MASK));
		//return ((dContactGeom*)(((char*)Contacts) + (Index * Stride)));
		throw new UnsupportedOperationException();
	}
	//#endif


	//20 Apr 2004
	//Start code by Nguyen Binh
	//int dClipEdgeToPlane(dVector3 &vEpnt0, dVector3 &vEpnt1, final dVector4& plPlane);
	//clip polygon with plane and generate new polygon points
	//void dClipPolyToPlane(final dVector3 avArrayIn[], final int ctIn, dVector3 avArrayOut[], int &ctOut, final dVector4 &plPlane );

	//void dClipPolyToCircle(final dVector3 avArrayIn[], final int ctIn, dVector3 avArrayOut[], int &ctOut, final dVector4 &plPlane ,double fRadius);

	//Some vector math
	//inline void dVector3Subtract(const dVector3& a,const dVector3& b,dVector3& c)
	static void dVector3Subtract(final DVector3C a,final DVector3C b,DVector3 c)
	{
		c.eqDiff(a, b);
		//		c.v[0] = a.v[0] - b.v[0];
		//		c.v[1] = a.v[1] - b.v[1];
		//		c.v[2] = a.v[2] - b.v[2];
	}

	//Some vector math
	//inline void dVector3Scale(dVector3& a,dReal nScale)
	void dVector3Scale(DVector3 a,double nScale)
	{
		a.scale(nScale);
		//		a.v[0] *= nScale ;
		//		a.v[1] *= nScale ;
		//		a.v[2] *= nScale ;
	}

	//inline void dVector3Add(final dVector3& a,final dVector3& b,dVector3& c)
	void dVector3Add(final DVector3 a,final DVector3 b,DVector3 c)
	{
		c.eqSum(a, b);
		//		c.v[0] = a.v[0] + b.v[0];
		//		c.v[1] = a.v[1] + b.v[1];
		//		c.v[2] = a.v[2] + b.v[2];
	}

	//inline void dVector3Copy(final dVector3& a,dVector3& c)
	static void dVector3Copy(final DVector3C a, DVector3 c)
	{
		//		c.v[0] = a.v[0];
		//		c.v[1] = a.v[1];
		//		c.v[2] = a.v[2];
		c.set(a);
	}

	//inline void dVector4Copy(final dVector4& a,dVector4& c)
	//	void dVector4Copy(final DVector4 a,DVector4 c)
	//	{
	//		c.set(a);
	////		c.v[0] = a.v[0];
	////		c.v[1] = a.v[1];
	////		c.v[2] = a.v[2];
	////		c.v[3] = a.v[3];
	//	}

	//inline void dVector3Cross(final dVector3& a,final dVector3& b,dVector3& c)
	static void dVector3Cross(final DVector3C a,final DVector3C b,DVector3 c)
	{
	    dCalcVectorCross3(c, a, b);
	}

	//inline dReal dVector3Length(final dVector3& a)
	double dVector3Length(final DVector3 a)
	{
		//return dSqrt(a.v[0]*a.v[0]+a.v[1]*a.v[1]+a.v[2]*a.v[2]);
		return a.length();
	}

	//inline dReal dVector3Dot(final dVector3& a,final dVector3& b)
	double dVector3Dot(final DVector3C a,final DVector3C b)
	{
		return a.dot(b);
	}

	//inline void dVector3Inv(dVector3& a)
	static void dVector3Inv(DVector3 a)
	{
		//		a.v[0] = -a.v[0];
		//		a.v[1] = -a.v[1];
		//		a.v[2] = -a.v[2];
		a.scale(-1);
	}

	//   inline dReal dVector3Length2(final dVector3& a)
	double dVector3LengthSquare(final DVector3 a)
	{
		//return (a.get0()*a.get0()+a.v[1]*a.v[1]+a.v[2]*a.v[2]);
		return a.lengthSquared();
	}

	//inline void dMat3GetCol(final dMatrix3& m,final int col, dVector3& v)
	static void dMat3GetCol(final DMatrix3 m,final int col, DVector3 v)
	{
//		v.set0( m.v[col + 0] );
//		v.set1( m.v[col + 4] );
//		v.set2( m.v[col + 8] );
		if (col ==0) {
			v.set( m.get00(), m.get10(), m.get20() );
		} else if (col == 1) {
			v.set( m.get01(), m.get11(), m.get21() );
		} else if (col == 2) {
			v.set( m.get02(), m.get12(), m.get22() );
		} else {
			throw new IllegalArgumentException("col="+col);
		}
	}

	//inline void dVector3CrossMat3Col(final dMatrix3& m,final int col,final dVector3& v,dVector3& r)
//	void dVector3CrossMat3Col(final DMatrix3 m,final int col,final DVector3 v,DVector3 r)
//	{
//		r.set0( v.get1() * m.v[2*4 + col] - v.get2() * m.v[1*4 + col] ); 
//		r.set1( v.get2() * m.v[0*4 + col] - v.get0() * m.v[2*4 + col] ); 
//		r.set2( v.get0() * m.v[1*4 + col] - v.get1() * m.v[0*4 + col] );
//	}
	void dVector3CrossMat3Col(DMatrix3C m, final int col, DVector3C v,DVector3 r)
	{
		if (col == 0) {
			r.set0( v.get1() * m.get20() - v.get2() * m.get10() ); 
			r.set1( v.get2() * m.get00() - v.get0() * m.get20() ); 
			r.set2( v.get0() * m.get10() - v.get1() * m.get00() );
		} else if (col ==1) {

			r.set0( v.get1() * m.get21() - v.get2() * m.get11() ); 
			r.set1( v.get2() * m.get01() - v.get0() * m.get21() ); 
			r.set2( v.get0() * m.get11() - v.get1() * m.get01() );
		} else if (col ==2 ) {

			r.set0( v.get1() * m.get22() - v.get2() * m.get12() ); 
			r.set1( v.get2() * m.get02() - v.get0() * m.get22() ); 
			r.set2( v.get0() * m.get12() - v.get1() * m.get02() );
		} else {
			throw new IllegalArgumentException("col="+col);
		}
	}

	//inline void dMat3ColCrossVector3(final dMatrix3& m,final int col,final dVector3& v,dVector3& r)
	void dMat3ColCrossVector3(final DMatrix3 m,final int col,final DVector3 v,DVector3 r)
	{
//		r.set0( v.get2() * m.v[1*4 + col] - v.get1() * m.v[2*4 + col] ); 
//		r.set1( v.get0() * m.v[2*4 + col] - v.get2() * m.v[0*4 + col] ); 
//		r.set2( v.get1() * m.v[0*4 + col] - v.get0() * m.v[1*4 + col] );
		if (col ==0) {
			r.set0( v.get2() * m.get10() - v.get1() * m.get20() ); 
			r.set1( v.get0() * m.get20() - v.get2() * m.get00() ); 
			r.set2( v.get1() * m.get00() - v.get0() * m.get10() );
		} else if (col == 1) {
			r.set0( v.get2() * m.get11() - v.get1() * m.get21() ); 
			r.set1( v.get0() * m.get21() - v.get2() * m.get01() ); 
			r.set2( v.get1() * m.get01() - v.get0() * m.get11() );
		} else if (col == 2) {
			r.set0( v.get2() * m.get12() - v.get1() * m.get22() ); 
			r.set1( v.get0() * m.get22() - v.get2() * m.get02() ); 
			r.set2( v.get1() * m.get02() - v.get0() * m.get12() );
		} else {
			throw new IllegalArgumentException("col="+col);
		}
	}

	//inline void dMultiplyMat3Vec3(final dMatrix3& m,final dVector3& v, dVector3& r)
	void dMultiplyMat3Vec3(final DMatrix3 m,final DVector3 v, DVector3 r)
	{
		dMultiply0_331(r,m,v);
	}

	//inline dReal dPointPlaneDistance(final dVector3& point,final dVector4& plane)
	static double dPointPlaneDistance(final DVector3 point,final DVector4 plane)
	{
		return (plane.get0()*point.get0() + plane.get1()*point.get1() + plane.get2()*point.get2() + plane.get3());
	}

	//inline void dConstructPlane(final dVector3& normal,final dReal& distance, dVector4& plane)
	static void dConstructPlane(final DVector3 normal,final double distance, DVector4 plane)
	{
		plane.set( normal.get0(),
			normal.get1(),
			normal.get2(),
			distance);
	}

	//inline void dMatrix3Copy(final double* source,dMatrix3& dest)
	/** @deprecated TZ Use dVector instead */
	static void dMatrix3Copy(final DMatrix3C source,DMatrix3 dest)
	{
		//		dest.v[0]	=	source[0];
		//		dest.v[1]	=	source[1];
		//		dest.v[2]	=	source[2];
		//
		//		dest.v[4]	=	source[4];
		//		dest.v[5]	=	source[5];
		//		dest.v[6]	=	source[6];
		//
		//		dest.v[8]	=	source[8];
		//		dest.v[9]	=	source[9];
		//		dest.v[10]=	source[10];
		dest.set(source);
	}

	// inline dReal dMatrix3Det( final dMatrix3& mat )
	double dMatrix3Det( final DMatrix3 mat )
	{
		double det;

//		det = mat.v[0] * ( mat.v[5]*mat.v[10] - mat.v[9]*mat.v[6] )
//		- mat.v[1] * ( mat.v[4]*mat.v[10] - mat.v[8]*mat.v[6] )
//		+ mat.v[2] * ( mat.v[4]*mat.v[9]  - mat.v[8]*mat.v[5] );
		det = mat.get00() * ( mat.get11()*mat.get22() - mat.get21()*mat.get12() )
		- mat.get01() * ( mat.get10()*mat.get22() - mat.get20()*mat.get12() )
		+ mat.get02() * ( mat.get10()*mat.get21()  - mat.get20()*mat.get11() );

		return( det );
	}


	//inline void dMatrix3Inv( final dMatrix3& ma, dMatrix3& dst )
	void dMatrix3Inv( final DMatrix3 ma, DMatrix3 dst )
	{
		double det = dMatrix3Det( ma );

		if ( dFabs( det ) < (0.0005) )
		{
			dst.setIdentity();
			return;
		}
		
	    double detRecip = 1.0 / det;

		dst.set00( (ma.get11()*ma.get22() - ma.get12()*ma.get21()) * detRecip );
		dst.set01( (ma.get21()*ma.get02() - ma.get01()*ma.get22()) * detRecip );
		dst.set02( (ma.get01()*ma.get12() - ma.get11()*ma.get02()) * detRecip );

		dst.set10( (ma.get12()*ma.get20() - ma.get10()*ma.get22()) * detRecip );
		dst.set11( (ma.get00()*ma.get22() - ma.get20()*ma.get02()) * detRecip );
		dst.set12( (ma.get10()*ma.get02() - ma.get00()*ma.get12()) * detRecip );

		dst.set20( (ma.get10()*ma.get21() - ma.get20()*ma.get11()) * detRecip );
		dst.set21( (ma.get20()*ma.get01() - ma.get00()*ma.get21()) * detRecip );
		dst.set22( (ma.get00()*ma.get11() - ma.get01()*ma.get10()) * detRecip );
	}

	//inline void dQuatTransform(final dQuaternion& quat,final dVector3& source,dVector3& dest)
	static void dQuatTransform(final DQuaternion quat,final DVector3 source,DVector3 dest)
	{

		// Nguyen Binh : this code seem to be the fastest.
		double x0 = 	source.get0() * quat.get0() + source.get2() * quat.get2() - source.get1() * quat.get3();
		double x1 = 	source.get1() * quat.get0() + source.get0() * quat.get3() - source.get2() * quat.get1();
		double x2 = 	source.get2() * quat.get0() + source.get1() * quat.get1() - source.get0() * quat.get2();
		double x3 = 	source.get0() * quat.get1() + source.get1() * quat.get2() + source.get2() * quat.get3();

		dest.set0( quat.get0() * x0 + quat.get1() * x3 + quat.get2() * x2 - quat.get3() * x1 );
		dest.set1( quat.get0() * x1 + quat.get2() * x3 + quat.get3() * x0 - quat.get1() * x2 );
		dest.set2( quat.get0() * x2 + quat.get3() * x3 + quat.get1() * x1 - quat.get2() * x0 );

		/*
	// nVidia SDK implementation
	dVector3 uv, uuv; 
	dVector3 qvec;
	qvec[0] = quat[1];
	qvec[1] = quat[2];
	qvec[2] = quat[3];

	dVector3Cross(qvec,source,uv);
	dVector3Cross(qvec,uv,uuv);

	dVector3Scale(uv,REAL(2.0)*quat[0]);
	dVector3Scale(uuv,REAL(2.0));

	dest[0] = source[0] + uv[0] + uuv[0];
	dest[1] = source[1] + uv[1] + uuv[1];
	dest[2] = source[2] + uv[2] + uuv[2];   
		 */
	}

	//inline void dQuatInvTransform(final dQuaternion& quat,final dVector3& source,dVector3& dest)
	void dQuatInvTransform(final DQuaternion quat,final DVector3 source,DVector3 dest)
	{

		double norm = quat.get0()*quat.get0() + quat.get1()*quat.get1() + quat.get2()*quat.get2() + quat.get3()*quat.get3();

		if (norm > (0.0))
		{
			double invNorm = 1.0/norm;
			DQuaternion invQuat = new DQuaternion();
			invQuat.set0(  quat.get0() * invNorm );
			invQuat.set1( -quat.get1() * invNorm );
			invQuat.set2( -quat.get2() * invNorm );
			invQuat.set3( -quat.get3() * invNorm );	

			dQuatTransform(invQuat,source,dest);

		}
		else
		{
			// Singular -> return identity
			dVector3Copy(source,dest);
		}
	}

	//inline void dGetEulerAngleFromRot(final dMatrix3& mRot,dReal& rX,dReal& rY,dReal& rZ)
	void dGetEulerAngleFromRot(final DMatrix3 mRot,RefDouble rX,RefDouble rY,RefDouble rZ)
	{
//		rY.set( asin(mRot.get(0 * 4 + 2)) );
//		if (rY.get() < M_PI /2)
//		{
//			if (rY.get() > -M_PI /2)
//			{
//				rX.set( atan2(-mRot.v[1*4 + 2], mRot.v[2*4 + 2]) );
//				rZ.set( atan2(-mRot.v[0*4 + 1], mRot.v[0*4 + 0]) );
//			}
//			else
//			{
//				// not unique
//				rX.set( -atan2(mRot.v[1*4 + 0], mRot.v[1*4 + 1]) );
//				rZ.set(0.0);
//			}
//		}
//		else
//		{
//			// not unique
//			rX.set( atan2(mRot.v[1*4 + 0], mRot.v[1*4 + 1]) );
//			rZ.set(0.0);
//		}
		rY.set( asin(mRot.get02()) );
		if (rY.get() < M_PI /2)
		{
			if (rY.get() > -M_PI /2)
			{
				rX.set( atan2(-mRot.get12(), mRot.get22()) );
				rZ.set( atan2(-mRot.get01(), mRot.get00()) );
			}
			else
			{
				// not unique
				rX.set( -atan2(mRot.get10(), mRot.get11()) );
				rZ.set(0.0);
			}
		}
		else
		{
			// not unique
			rX.set( atan2(mRot.get10(), mRot.get11()) );
			rZ.set(0.0);
		}
	}

	//inline void dQuatInv(final dQuaternion& source, dQuaternion& dest)
	static void dQuatInv(final DQuaternion source, DQuaternion dest)
	{
		double norm = source.get0()*source.get0() + source.get1()*source.get1() + source.get2()*source.get2() + source.get3()*source.get3();

		if (norm > 0.0f)
		{
			double invNorm = 1.0/norm;
			dest.set0( source.get0() * invNorm );
			dest.set1( -source.get1() * invNorm );
			dest.set2( -source.get2() * invNorm );
			dest.set3( -source.get3() * invNorm);	
		}
		else
		{
			// Singular -> return identity
//			dest.v[0] = (1.0);
//			dest.v[1] = (0.0);
//			dest.v[2] = (0.0);
//			dest.v[3] = (0.0);
			dest.set( 1, 0, 0, 0 );
		}
	}


	//****************************************************************************
	/**
	 * if the spheres (p1,r1) and (p2,r2) collide, set the contact `c' and
	 * return 1, else return 0.
	 */
	//int dCollideSpheres (dVector3 p1, dReal r1,
	//		     dVector3 p2, dReal r2, dContactGeom *c)
	static int dCollideSpheres (DVector3C p1, double r1,
			DVector3C p2, double r2, DContactGeomBuffer cb)
	{
		DContactGeom c = cb.get();
		// printf ("d=%.2f  (%.2f %.2f %.2f) (%.2f %.2f %.2f) r1=%.2f r2=%.2f\n",
		//	  d,p1[0],p1[1],p1[2],p2[0],p2[1],p2[2],r1,r2);

		double d = p1.distance(p2);
		if (d > (r1 + r2)) return 0;
		if (d <= 0) {
//			c.pos.v[0] = p1.get0();
//			c.pos.v[1] = p1.get1();
//			c.pos.v[2] = p1.get2();
			c.pos.set( p1 );
//			c.normal.v[0] = 1;
//			c.normal.v[1] = 0;
//			c.normal.v[2] = 0;
			c.normal.set( 1, 0, 0 );
			c.depth = r1 + r2;
		}
		else {
			double d1 = dRecip (d);
			c.normal.set0( (p1.get0()-p2.get0())*d1 );
			c.normal.set1( (p1.get1()-p2.get1())*d1 );
			c.normal.set2( (p1.get2()-p2.get2())*d1 );
			double k = (0.5) * (r2 - r1 - d);
//			c.pos.v[0] = p1.get0() + c.normal.get0()*k;
//			c.pos.v[1] = p1.get1() + c.normal.get1()*k;
//			c.pos.v[2] = p1.get2() + c.normal.get2()*k;
			c.pos.eqSum( p1, c.normal, k );
			c.depth = r1 + r2 - d;
		}
		return 1;
	}

	/**
	 * given two lines
	 * qa = pa + alpha* ua
	 * qb = pb + beta * ub
	 * where pa,pb are two points, ua,ub are two unit length vectors, and alpha,
	 * beta go from [-inf,inf], return alpha and beta such that qa and qb are
	 * as close as possible
	 */
	//void dLineClosestApproach (final dVector3 pa, final dVector3 ua,
	//			   final dVector3 pb, final dVector3 ub,
	//			   double *alpha, double *beta)
	static void dLineClosestApproach (final DVector3 pa, final DVector3 ua,
			final DVector3 pb, final DVector3 ub,
			RefDouble alpha, RefDouble beta)
	{
		DVector3 p = new DVector3();
//		p.v[0] = pb.get0() - pa.get0();
//		p.v[1] = pb.get1() - pa.get1();
//		p.v[2] = pb.get2() - pa.get2();
		p.eqDiff( pb, pa );
		double uaub = dCalcVectorDot3(ua,ub);
		double q1 =  dCalcVectorDot3(ua,p);
		double q2 = -dCalcVectorDot3(ub,p);
		double d = 1-uaub*uaub;
		if (d <= (0.0001)) {
			// @@@ this needs to be made more robust
			alpha.set(0);
			beta.set(0);
		}
		else {
			d = dRecip(d);
			alpha.set( (q1 + uaub*q2)*d);
			beta.set( (uaub*q1 + q2)*d);
		}
	}


	//private static void SET2(DVector3 a, DVector3 b) { a.v[0]=b.get0(); a.v[1]=b.get1(); a.v[2]=b.get2();}
	private static void SET2(DVector3 a, DVector3 b) { a.set( b );}
	private static void SET3_SUB(DVector3 a, DVector3 b, DVector3 c) {
		SET3_SUB(a, 1, b, 1, c);
	}
	private static void SET3_SUB(DVector3 a, double f1, DVector3 b, double f2, DVector3 c) {
		a.set0(b.get0()*f1 - c.get0()*f2); a.set1(b.get1()*f1 - c.get1()*f2); a.set2(b.get2()*f1 - c.get2()*f2);
	}
	private static void SET3_ADD(DVector3 a, double f1, DVector3 b, double f2, DVector3 c) {
		a.set0(b.get0()*f1 + c.get0()*f2); a.set1(b.get1()*f1 + c.get1()*f2); a.set2(b.get2()*f1 + c.get2()*f2);
	}
	// given two line segments A and B with endpoints a1-a2 and b1-b2, return the
	// points on A and B that are closest to each other (in cp1 and cp2).
	// in the case of parallel lines where there are multiple solutions, a
	// solution involving the endpoint of at least one line will be returned.
	// this will work correctly for zero length lines, e.g. if a1==a2 and/or
	// b1==b2.
	//
	// the algorithm works by applying the voronoi clipping rule to the features
	// of the line segments. the three features of each line segment are the two
	// endpoints and the line between them. the voronoi clipping rule states that,
	// for feature X on line A and feature Y on line B, the closest points PA and
	// PB between X and Y are globally the closest points if PA is in V(Y) and
	// PB is in V(X), where V(X) is the voronoi region of X.

	static void dClosestLineSegmentPoints (final DVector3 a1, final DVector3 a2,
			final DVector3 b1, final DVector3 b2,
			DVector3 cp1, DVector3 cp2)
	{
		DVector3 a1a2=new DVector3(),b1b2=new DVector3(),a1b1=new DVector3(),a1b2=new DVector3(),
		a2b1=new DVector3(),a2b2=new DVector3(),n=new DVector3();
		double la,lb,k,da1,da2,da3,da4,db1,db2,db3,db4,det;


		// check vertex-vertex features

		SET3_SUB (a1a2,a2,a1);
		SET3_SUB (b1b2,b2,b1);
		SET3_SUB (a1b1,b1,a1);
		da1 = dCalcVectorDot3(a1a2,a1b1);
		db1 = dCalcVectorDot3(b1b2,a1b1);
		if (da1 <= 0 && db1 >= 0) {
			SET2 (cp1,a1);
			SET2 (cp2,b1);
			return;
		}

		SET3_SUB (a1b2,b2,a1);
		da2 = dCalcVectorDot3(a1a2,a1b2);
		db2 = dCalcVectorDot3(b1b2,a1b2);
		if (da2 <= 0 && db2 <= 0) {
			SET2 (cp1,a1);
			SET2 (cp2,b2);
			return;
		}

		SET3_SUB (a2b1,b1,a2);
		da3 = dCalcVectorDot3(a1a2,a2b1);
		db3 = dCalcVectorDot3(b1b2,a2b1);
		if (da3 >= 0 && db3 >= 0) {
			SET2 (cp1,a2);
			SET2 (cp2,b1);
			return;
		}

		SET3_SUB (a2b2,b2,a2);
		da4 = dCalcVectorDot3(a1a2,a2b2);
		db4 = dCalcVectorDot3(b1b2,a2b2);
		if (da4 >= 0 && db4 <= 0) {
			SET2 (cp1,a2);
			SET2 (cp2,b2);
			return;
		}

		// check edge-vertex features.
		// if one or both of the lines has zero length, we will never get to here,
		// so we do not have to worry about the following divisions by zero.

		la = dCalcVectorDot3(a1a2,a1a2);
		if (da1 >= 0 && da3 <= 0) {
			k = da1 / la;
			SET3_SUB (n,1,a1b1,k,a1a2);
			if (dCalcVectorDot3(b1b2,n) >= 0) {
				SET3_ADD (cp1,1,a1,k,a1a2);
				SET2 (cp2,b1);
				return;
			}
		}

		if (da2 >= 0 && da4 <= 0) {
			k = da2 / la;
			SET3_SUB (n,1,a1b2,k,a1a2);
			if (dCalcVectorDot3(b1b2,n) <= 0) {
				SET3_ADD (cp1,1,a1,k,a1a2);
				SET2 (cp2,b2);
				return;
			}
		}

		lb = dCalcVectorDot3(b1b2,b1b2);
		if (db1 <= 0 && db2 >= 0) {
			k = -db1 / lb;
			SET3_SUB (n,-1,a1b1,k,b1b2);
			if (dCalcVectorDot3(a1a2,n) >= 0) {
				SET2 (cp1,a1);
				SET3_ADD (cp2,1,b1,k,b1b2);
				return;
			}
		}

		if (db3 <= 0 && db4 >= 0) {
			k = -db3 / lb;
			SET3_SUB (n,-1,a2b1,k,b1b2);
			if (dCalcVectorDot3(a1a2,n) <= 0) {
				SET2 (cp1,a2);
				SET3_ADD (cp2,1,b1,k,b1b2);
				return;
			}
		}

		// it must be edge-edge

		k = dCalcVectorDot3(a1a2,b1b2);
		det = la*lb - k*k;
		if (det <= 0) {
			// this should never happen, but just in case...
			SET2(cp1,a1);
			SET2(cp2,b1);
			return;
		}
		det = dRecip (det);
		double alpha = (lb*da1 -  k*db1) * det;
		double beta  = ( k*da1 - la*db1) * det;
		SET3_ADD (cp1,1,a1,alpha,a1a2);
		SET3_ADD (cp2,1,b1,beta,b1b2);

		//# undef SET2
		//# undef SET3
	}


	/**
	 * given a line segment p1-p2 and a box (center 'c', rotation 'R', side length
	 * vector 'side'), compute the points of closest approach between the box
	 * and the line. return these points in 'lret' (the point on the line) and
	 * 'bret' (the point on the box). if the line actually penetrates the box
	 * then the solution is not unique, but only one solution will be returned.
	 * in this case the solution points will coincide.
	 *
	 *
	 *  a simple root finding algorithm is used to find the value of 't' that
	 *  satisfies:
	 * 		d|D(t)|^2/dt = 0
	 *  where:
	 * 		|D(t)| = |p(t)-b(t)|
	 *  where p(t) is a point on the line parameterized by t:
	 * 		p(t) = p1 + t*(p2-p1)
	 *  and b(t) is that same point clipped to the boundary of the box. in box-
	 *  relative coordinates d|D(t)|^2/dt is the sum of three x,y,z components
	 *  each of which looks like this:
	 * 
	 * 	    t_lo     /
	 * 	      ______/    -->t
	 * 	     /     t_hi
	 * 	    /
	 * 
	 *  t_lo and t_hi are the t values where the line passes through the planes
	 *  corresponding to the sides of the box. the algorithm computes d|D(t)|^2/dt
	 *  in a piecewise fashion from t=0 to t=1, stopping at the point where
	 * d|D(t)|^2/dt crosses from negative to positive.
	 */
	static void dClosestLineBoxPoints (final DVector3C p1, final DVector3C p2,
			final DVector3C c, final DMatrix3C R,
			final DVector3C side,
			DVector3 lret, DVector3 bret)
	{
		int i;

		// compute the start and delta of the line p1-p2 relative to the box.
		// we will do all subsequent computations in this box-relative coordinate
		// system. we have to do a translation and rotation for each point.
		DVector3 tmp=new DVector3(),s=new DVector3(),v=new DVector3();
//		tmp.v[0] = p1.get0() - c.get0();
//		tmp.v[1] = p1.get1() - c.get1();
//		tmp.v[2] = p1.get2() - c.get2();
		tmp.eqDiff( p1, c );
		dMultiply1_331 (s,R,tmp);
//		tmp.v[0] = p2.get0() - p1.get0();
//		tmp.v[1] = p2.get1() - p1.get1();
//		tmp.v[2] = p2.get2() - p1.get2();
		tmp.eqDiff( p2, p1 );
		dMultiply1_331 (v,R,tmp);

		// mirror the line so that v has all components >= 0
		DVector3 sign=new DVector3();
		for (i=0; i<3; i++) {
			if (v.get(i) < 0) {
				s.set(i, -s.get(i) );
				v.set(i, -v.get(i) );
				sign.set(i, -1);
			}
			else sign.set(i, 1);
		}

		// compute v^2
		DVector3 v2=new DVector3();
		v2.set0( v.get0()*v.get0() );
		v2.set1( v.get1()*v.get1() );
		v2.set2( v.get2()*v.get2() );

		// compute the half-sides of the box
		//TZ double h[3];
		DVector3 h = new DVector3();
//		h.v[0] = (0.5) * side.get0();
//		h.v[1] = (0.5) * side.get1();
//		h.v[2] = (0.5) * side.get2();
		h.set(side).scale(0.5);

		// region is -1,0,+1 depending on which side of the box planes each
		// coordinate is on. tanchor is the next t value at which there is a
		// transition, or the last one if there are no more.
		int[] region = new int[3];
		double[] tanchor=new double[3];

		// Denormals are a problem, because we divide by v[i], and then 
		// multiply that by 0. Alas, infinity times 0 is infinity (!)
		// We also use v2[i], which is v[i] squared. Here's how the epsilons 
		// are chosen:
		// float epsilon = 1.175494e-038 (smallest non-denormal number)
		// double epsilon = 2.225074e-308 (smallest non-denormal number)
		// For single precision, choose an epsilon such that v[i] squared is 
		// not a denormal; this is for performance.
		// For double precision, choose an epsilon such that v[i] is not a 
		// denormal; this is for correctness. (Jon Watte on mailinglist)

		//#if defined( dSINGLE )
		//  final double tanchor_eps = (1e-19);
		//#else
		final double tanchor_eps = (1e-307);
		//#endif

		// find the region and tanchor values for p1
		for (i=0; i<3; i++) {
			if (v.get(i) > tanchor_eps) {
				if (s.get(i) < -h.get(i)) {
					region[i] = -1;
					tanchor[i] = (-h.get(i)-s.get(i))/v.get(i);
				}
				else {
					region[i] = (s.get(i) > h.get(i)) ? 1 : 0;
					tanchor[i] = (h.get(i)-s.get(i))/v.get(i);
				}
			}
			else {
				region[i] = 0;
				tanchor[i] = 2;		// this will never be a valid tanchor
			}
		}

		// compute d|d|^2/dt for t=0. if it's >= 0 then p1 is the closest point
		double t=0;
		double dd2dt = 0;
		for (i=0; i<3; i++) dd2dt -= (region[i]!=0 ? v2.get(i) : 0) * tanchor[i];
		if (dd2dt >= 0)	{
			answer(t, tmp, sign, p1, s, R, lret, h, c, v, bret);//goto got_answer;
			return;
		}

		do {
			// find the point on the line that is at the next clip plane boundary
			double next_t = 1;
			for (i=0; i<3; i++) {
				if (tanchor[i] > t && tanchor[i] < 1 && tanchor[i] < next_t)
					next_t = tanchor[i];
			}

			// compute d|d|^2/dt for the next t
			double next_dd2dt = 0;
			for (i=0; i<3; i++) {
				next_dd2dt += (region[i]!=0 ? v2.get(i) : 0) * (next_t - tanchor[i]);
			}

			// if the sign of d|d|^2/dt has changed, solution = the crossover point
			if (next_dd2dt >= 0) {
				double m = (next_dd2dt-dd2dt)/(next_t - t);
				t -= dd2dt/m;
				answer(t, tmp, sign, p1, s, R, lret, h, c, v, bret);//goto got_answer;
				return;
			}

			// advance to the next anchor point / region
			for (i=0; i<3; i++) {
				if (tanchor[i] == next_t) {
					tanchor[i] = (h.get(i)-s.get(i))/v.get(i);
					region[i]++;
				}
			}
			t = next_t;
			dd2dt = next_dd2dt;
		}
		while (t < 1);
		t = 1;

		answer(t, tmp, sign, p1, s, R, lret, h, c, v, bret);//goto got_answer;
	}

	//TZ
	private static void answer(final double t, DVector3 tmp, DVector3C sign, 
			DVector3C p1, DVector3 s, DMatrix3C R,
			DVector3 lret, DVector3C h, DVector3C c, DVector3C v, DVector3 bret) {
		//got_answer:

		// compute closest point on the line
		//for (int i=0; i<3; i++) lret.v[i] = p1.v[i] + t*tmp.v[i];	// note: tmp=p2-p1
		lret.eqSum(p1, tmp, t);

		// compute closest point on the box
		for (int i=0; i<3; i++) {
			tmp.set(i, sign.get(i) * (s.get(i) + t*v.get(i)) );
			if (tmp.get(i) < -h.get(i)) tmp.set(i, -h.get(i) );
			else if (tmp.get(i) > h.get(i)) tmp.set(i, h.get(i) );
		}
		dMultiply0_331 (s,R,tmp);
		//for (int i=0; i<3; i++) bret.v[i] = s.v[i] + c.v[i];
		bret.eqSum(s, c);
	}


	// given boxes (p1,R1,side1) and (p1,R1,side1), return 1 if they intersect
	// or 0 if not.

	public static boolean dBoxTouchesBox (final DVector3 p1, final DMatrix3 R1,
			final DVector3 side1, final DVector3 p2,
			final DMatrix3 R2, final DVector3 side2)
	{
		// two boxes are disjoint if (and only if) there is a separating axis
		// perpendicular to a face from one box or perpendicular to an edge from
		// either box. the following tests are derived from:
		//    "OBB Tree: A Hierarchical Structure for Rapid Interference Detection",
		//    S.Gottschalk, M.C.Lin, D.Manocha., Proc of ACM Siggraph 1996.

		// Rij is R1'*R2, i.e. the relative rotation between R1 and R2.
		// Qij is abs(Rij)
		DVector3 p=new DVector3(),pp=new DVector3();
		double A1,A2,A3,B1,B2,B3,R11,R12,R13,R21,R22,R23,R31,R32,R33,
		Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33;

		// get vector from centers of box 1 to box 2, relative to box 1
		//		p.v[0] = p2.get0() - p1.get0();
		//		p.v[1] = p2.get1() - p1.get1();
		//		p.v[2] = p2.get2() - p1.get2();
		p.eqDiff(p2, p1);
		dMultiply1_331 (pp,R1,p);		// get pp = p relative to body 1

		// get side lengths / 2
		A1 = side1.get0()*(0.5); A2 = side1.get1()*(0.5); A3 = side1.get2()*(0.5);
		B1 = side2.get0()*(0.5); B2 = side2.get1()*(0.5); B3 = side2.get2()*(0.5);

		// for the following tests, excluding computation of Rij, in the worst case,
		// 15 compares, 60 adds, 81 multiplies, and 24 absolutes.
		// notation: R1=[u1 u2 u3], R2=[v1 v2 v3]

		// separating axis = u1,u2,u3
		R11 = dCalcVectorDot3_44(R1,0,R2,0); R12 = dCalcVectorDot3_44(R1,0,R2,1); R13 = dCalcVectorDot3_44(R1,0,R2,2);
		Q11 = dFabs(R11); Q12 = dFabs(R12); Q13 = dFabs(R13);
		if (dFabs(pp.get0()) > (A1 + B1*Q11 + B2*Q12 + B3*Q13)) return false;
		R21 = dCalcVectorDot3_44(R1,1,R2,0); R22 = dCalcVectorDot3_44(R1,1,R2,1); R23 = dCalcVectorDot3_44(R1,1,R2,2);
		Q21 = dFabs(R21); Q22 = dFabs(R22); Q23 = dFabs(R23);
		if (dFabs(pp.get1()) > (A2 + B1*Q21 + B2*Q22 + B3*Q23)) return false;
		R31 = dCalcVectorDot3_44(R1,2,R2,0); R32 = dCalcVectorDot3_44(R1,2,R2,1); R33 = dCalcVectorDot3_44(R1,2,R2,2);
		Q31 = dFabs(R31); Q32 = dFabs(R32); Q33 = dFabs(R33);
		if (dFabs(pp.get2()) > (A3 + B1*Q31 + B2*Q32 + B3*Q33)) return false;

		// separating axis = v1,v2,v3
		if (dFabs(dCalcVectorDot3_41(R2,0,p)) > (A1*Q11 + A2*Q21 + A3*Q31 + B1)) return false;
		if (dFabs(dCalcVectorDot3_41(R2,1,p)) > (A1*Q12 + A2*Q22 + A3*Q32 + B2)) return false;
		if (dFabs(dCalcVectorDot3_41(R2,2,p)) > (A1*Q13 + A2*Q23 + A3*Q33 + B3)) return false;

		// separating axis = u1 x (v1,v2,v3)
		if (dFabs(pp.get2()*R21-pp.get1()*R31) > A2*Q31 + A3*Q21 + B2*Q13 + B3*Q12) return false;
		if (dFabs(pp.get2()*R22-pp.get1()*R32) > A2*Q32 + A3*Q22 + B1*Q13 + B3*Q11) return false;
		if (dFabs(pp.get2()*R23-pp.get1()*R33) > A2*Q33 + A3*Q23 + B1*Q12 + B2*Q11) return false;

		// separating axis = u2 x (v1,v2,v3)
		if (dFabs(pp.get0()*R31-pp.get2()*R11) > A1*Q31 + A3*Q11 + B2*Q23 + B3*Q22) return false;
		if (dFabs(pp.get0()*R32-pp.get2()*R12) > A1*Q32 + A3*Q12 + B1*Q23 + B3*Q21) return false;
		if (dFabs(pp.get0()*R33-pp.get2()*R13) > A1*Q33 + A3*Q13 + B1*Q22 + B2*Q21) return false;

		// separating axis = u3 x (v1,v2,v3)
		if (dFabs(pp.get1()*R11-pp.get0()*R21) > A1*Q21 + A2*Q11 + B2*Q33 + B3*Q32) return false;
		if (dFabs(pp.get1()*R12-pp.get0()*R22) > A1*Q22 + A2*Q12 + B1*Q33 + B3*Q31) return false;
		if (dFabs(pp.get1()*R13-pp.get0()*R23) > A1*Q23 + A2*Q13 + B1*Q32 + B2*Q31) return false;

		return true;
	}

	//****************************************************************************
	// other utility functions



	//****************************************************************************
	// Helpers for Croteam's collider - by Nguyen Binh

	//int dClipEdgeToPlane( dVector3 &vEpnt0, dVector3 &vEpnt1, final dVector4& plPlane)
	static boolean dClipEdgeToPlane( DVector3 vEpnt0, DVector3 vEpnt1, final DVector4 plPlane)
	{
		// calculate distance of edge points to plane
		double fDistance0 = dPointPlaneDistance(  vEpnt0 ,plPlane );
		double fDistance1 = dPointPlaneDistance(  vEpnt1 ,plPlane );

		// if both points are behind the plane
		if ( fDistance0 < 0 && fDistance1 < 0 ) 
		{
			// do nothing
			return false;
			// if both points in front of the plane
		} 
		else if ( fDistance0 > 0 && fDistance1 > 0 ) 
		{
			// accept them
			return true;
			// if we have edge/plane intersection
		} else if ((fDistance0 > 0 && fDistance1 < 0) || ( fDistance0 < 0 && fDistance1 > 0)) 
		{

			// find intersection point of edge and plane
//			DVector3 vIntersectionPoint=new DVector3();
//			vIntersectionPoint.set0( vEpnt0.get0()-(vEpnt0.get0()-vEpnt1.get0())*fDistance0/(fDistance0-fDistance1) );
//			vIntersectionPoint.set1( vEpnt0.get1()-(vEpnt0.get1()-vEpnt1.get1())*fDistance0/(fDistance0-fDistance1) );
//			vIntersectionPoint.set2( vEpnt0.get2()-(vEpnt0.get2()-vEpnt1.get2())*fDistance0/(fDistance0-fDistance1) );
			double a0, a1, a2;
			a0 = vEpnt0.get0()-(vEpnt0.get0()-vEpnt1.get0())*fDistance0/(fDistance0-fDistance1);
			a1 = vEpnt0.get1()-(vEpnt0.get1()-vEpnt1.get1())*fDistance0/(fDistance0-fDistance1);
			a2 = vEpnt0.get2()-(vEpnt0.get2()-vEpnt1.get2())*fDistance0/(fDistance0-fDistance1);

			// clamp correct edge to intersection point
			if ( fDistance0 < 0 ) 
			{
				vEpnt0.set(a0, a1, a2);//dVector3Copy(vIntersectionPoint,vEpnt0);
			} else 
			{
				vEpnt1.set(a0, a1, a2);//dVector3Copy(vIntersectionPoint,vEpnt1);
			}
			return true;
		}
		return true;
	}

	// clip polygon with plane and generate new polygon points
	//void		 dClipPolyToPlane( final dVector3 avArrayIn[], final int ctIn, 
	//							  dVector3 avArrayOut[], int &ctOut, 
	//							  final dVector4 &plPlane )
	static int dClipPolyToPlane( final DVector3 avArrayIn[], final int ctIn, 
			DVector3 avArrayOut[],
			final DVector4 plPlane )
	{
		// start with no output points
		int ctOut = 0;

		int i0 = ctIn-1;

		// for each edge in input polygon
		for (int i1=0; i1<ctIn; i0=i1, i1++) {


			// calculate distance of edge points to plane
			double fDistance0 = dPointPlaneDistance(  avArrayIn[i0],plPlane );
			double fDistance1 = dPointPlaneDistance(  avArrayIn[i1],plPlane );

			// if first point is in front of plane
			if( fDistance0 >= 0 ) {
				// emit point
				//				avArrayOut[ctOut.get()].v[0] = avArrayIn[i0].get0();
				//				avArrayOut[ctOut.get()].v[1] = avArrayIn[i0].get1();
				//				avArrayOut[ctOut.get()].v[2] = avArrayIn[i0].get2();
				avArrayOut[ctOut].set( avArrayIn[i0] );
				ctOut++;
			}

			// if points are on different sides
			if( (fDistance0 > 0 && fDistance1 < 0) || ( fDistance0 < 0 && fDistance1 > 0) ) {

				// find intersection point of edge and plane
				DVector3 vIntersectionPoint = new DVector3();
				double f001 = fDistance0/(fDistance0-fDistance1); 
				vIntersectionPoint.set0( avArrayIn[i0].get0() - 
						(avArrayIn[i0].get0()-avArrayIn[i1].get0()) * f001 );
				vIntersectionPoint.set1( avArrayIn[i0].get1() - 
						(avArrayIn[i0].get1()-avArrayIn[i1].get1()) * f001 );
				vIntersectionPoint.set2( avArrayIn[i0].get2() - 
						(avArrayIn[i0].get2()-avArrayIn[i1].get2()) * f001 );

				// emit intersection point
				//				avArrayOut[ctOut.get()].v[0] = vIntersectionPoint.get0();
				//				avArrayOut[ctOut.get()].v[1] = vIntersectionPoint.get1();
				//				avArrayOut[ctOut.get()].v[2] = vIntersectionPoint.get2();
				avArrayOut[ctOut].set( vIntersectionPoint );
				ctOut++;
			}
		}
		return ctOut;
	}

	//void		 dClipPolyToCircle(final dVector3 avArrayIn[], final int ctIn, 
	//							   dVector3 avArrayOut[], int &ctOut, 
	//							   final dVector4 &plPlane ,double fRadius)
	void		 dClipPolyToCircle(final DVector3 avArrayIn[], final int ctIn, 
			DVector3 avArrayOut[], RefInt ctOut, 
			final DVector4 plPlane ,double fRadius)
	{
		// start with no output points
		ctOut.set(0);

		int i0 = ctIn-1;

		// for each edge in input polygon
		for (int i1=0; i1<ctIn; i0=i1, i1++) 
		{
			// calculate distance of edge points to plane
			double fDistance0 = dPointPlaneDistance(  avArrayIn[i0],plPlane );
			double fDistance1 = dPointPlaneDistance(  avArrayIn[i1],plPlane );

			// if first point is in front of plane
			if( fDistance0 >= 0 ) 
			{
				// emit point
				if (dVector3LengthSquare(avArrayIn[i0]) <= fRadius*fRadius)
				{
					//					avArrayOut[ctOut.get()].v[0] = avArrayIn[i0].get0();
					//					avArrayOut[ctOut.get()].v[1] = avArrayIn[i0].get1();
					//					avArrayOut[ctOut.get()].v[2] = avArrayIn[i0].get2();
					avArrayOut[ctOut.get()].set( avArrayIn[i0] );
					ctOut.inc();
				}
			}

			// if points are on different sides
			if( (fDistance0 > 0 && fDistance1 < 0) || ( fDistance0 < 0 && fDistance1 > 0) ) 
			{

				// find intersection point of edge and plane
				DVector3 vIntersectionPoint=new DVector3();
				double f001 = fDistance0/(fDistance0-fDistance1);
				vIntersectionPoint.set0( avArrayIn[i0].get0() - 
						(avArrayIn[i0].get0()-avArrayIn[i1].get0())*f001 );
				vIntersectionPoint.set1( avArrayIn[i0].get1() - 
						(avArrayIn[i0].get1()-avArrayIn[i1].get1())*f001 );
				vIntersectionPoint.set2( avArrayIn[i0].get2() - 
						(avArrayIn[i0].get2()-avArrayIn[i1].get2())*f001 );

				// emit intersection point
				if (dVector3LengthSquare(avArrayIn[i0]) <= fRadius*fRadius)
				{
					//					avArrayOut[ctOut.get()].v[0] = vIntersectionPoint.get0();
					//					avArrayOut[ctOut.get()].v[1] = vIntersectionPoint.get1();
					//					avArrayOut[ctOut.get()].v[2] = vIntersectionPoint.get2();
					avArrayOut[ctOut.get()].set( vIntersectionPoint );
					ctOut.inc();
				}
			}
		}	
	}
}

