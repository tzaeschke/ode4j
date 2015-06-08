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


import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector4;
import org.ode4j.math.DVector4C;
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.OdeConfig;
import org.ode4j.ode.internal.cpp4j.java.RefBoolean;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.cpp4j.java.RefInt;
import org.ode4j.ode.internal.gimpact.GimDynArrayInt;
import org.ode4j.ode.internal.gimpact.GimTrimesh;
import org.ode4j.ode.internal.gimpact.GimGeometry.aabb3f;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec3f;

import static org.ode4j.ode.internal.Common.*;

/*************************************************************************
 *                                                                       *
 * Triangle-box collider by Alen Ladavac and Vedran Klanac.              *
 * Ported to ODE by Oskari Nyman.                                        *  
 * Ported to Java by Tilmann Zaeschke.                                   *
 *                                                                       *
 *************************************************************************/
public class CollideTrimeshBox implements DColliderFn {

	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideBTL((DxTriMesh)o1, (DxBox)o2, flags, contacts, 1);
	}

	//	#if dTRIMESH_ENABLED


	//	static void
	//	GenerateContact(int in_Flags, dContactGeom* in_Contacts, int in_Stride,
	//	                dxGeom* in_g1,  dxGeom* in_g2, int TriIndex,
	//	                const dVector3 in_ContactPos, const dVector3 in_Normal, dReal in_Depth,
	//	                int& OutTriCount);


	// largest number, double or float
	//	#if defined(dSINGLE)
	//	  #define MAXVALUE FLT_MAX
	//	#else
	//	  #define MAXVALUE DBL_MAX
	//	#endif
	private static final double MAXVALUE = Double.MAX_VALUE;


	// dVector3
	// r=a-b
	//	#define SUBTRACT(a,b,r) do{ \
	//	  (r)[0]=(a)[0] - (b)[0]; \
	//	  (r)[1]=(a)[1] - (b)[1]; \
	//	  (r)[2]=(a)[2] - (b)[2]; }while(0)
	private void SUBTRACT(DVector3C a, DVector3C b, DVector3 r) {
		r.eqDiff(a, b);
	}


	// dVector3
	// a=b
	//	#define SET(a,b) do{ \
	//	  (a)[0]=(b)[0]; \
	//	  (a)[1]=(b)[1]; \
	//	  (a)[2]=(b)[2]; }while(0)
	private void SET(DVector3 a, DVector3C b) {
		a.set(b);
	}


	// dMatrix3
	// a=b
	//	#define SETM(a,b) do{ \
	//	  (a)[0]=(b)[0]; \
	//	  (a)[1]=(b)[1]; \
	//	  (a)[2]=(b)[2]; \
	//	  (a)[3]=(b)[3]; \
	//	  (a)[4]=(b)[4]; \
	//	  (a)[5]=(b)[5]; \
	//	  (a)[6]=(b)[6]; \
	//	  (a)[7]=(b)[7]; \
	//	  (a)[8]=(b)[8]; \
	//	  (a)[9]=(b)[9]; \
	//	  (a)[10]=(b)[10]; \
	//	                     (a)[11]=(b)[11]; }while(0)
//	/** a.set(b); */
//	private void SETM(DMatrix3 a, DMatrix3C b) {
//		a.set(b);
//	}


	// dVector3
	// r=a+b
	//	#define ADD(a,b,r) do{ \
	//	  (r)[0]=(a)[0] + (b)[0]; \
	//	  (r)[1]=(a)[1] + (b)[1]; \
	//	  (r)[2]=(a)[2] + (b)[2]; }while(0)
	private void ADD(DVector3C a, DVector3C b, DVector3 r) {
		r.eqSum(a, b);
	}


	// dMatrix3, int, dVector3
	// v=column a from m
	//	#define GETCOL(m,a,v) do{ \
	//	  (v)[0]=(m)[(a)+0]; \
	//	  (v)[1]=(m)[(a)+4]; \
	//	  (v)[2]=(m)[(a)+8]; }while(0)
	private void GETCOL(DMatrix3C m, int a, DVector3 v) {
		if (a==0) { v.set(m.get00(), m.get10(), m.get20()); }
		else if (a==1) { v.set(m.get01(), m.get11(), m.get21()); }
		else if (a==2) { v.set(m.get02(), m.get12(), m.get22()); }
		else throw new IllegalArgumentException("col=" + a);
	}


	// dVector4, dVector3
	// distance between plane p and point v
	//	#define POINTDISTANCE(p,v) \
	//	  ( p[0]*v[0] + p[1]*v[1] + p[2]*v[2] + p[3] )
	private double POINTDISTANCE(DVector4C p, DVector3C v) {
		return p.get0()*v.get0() + p.get1()*v.get1() + p.get2()*v.get2() + p.get3();
	}


	// dVector4, dVector3, dReal
	// construct plane from normal and d
	//	#define CONSTRUCTPLANE(plane,normal,d) do{ \
	//	  plane[0]=normal[0];\
	//	  plane[1]=normal[1];\
	//	  plane[2]=normal[2];\
	//	  plane[3]=d; }while(0)
	private void CONSTRUCTPLANE(DVector4 plane, DVector3C n, double d) {
		plane.set( n.get0(), n.get1(), n.get2(), d);
	}


	// dVector3
	// length of vector a
	//	#define LENGTHOF(a) \
	//	  dSqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])
	private double LENGTHOF(DVector3C a) {
		return a.length();  //TODO use squared? TZ
	}


	private class sTrimeshBoxColliderData
	{
		//sTrimeshBoxColliderData(): m_iBestAxis(0), m_iExitAxis(0), m_ctContacts(0) {}
		sTrimeshBoxColliderData() { m_iBestAxis=(0); m_iExitAxis=(0); }

		//		void SetupInitialContext(dxTriMesh *TriMesh, dxGeom *BoxGeom,
		//			int Flags, dContactGeom* Contacts, int Stride);
		//		int TestCollisionForSingleTriangle(int ctContacts0, int Triint, 
		//			dVector3 dv[3], bool &bOutFinishSearching);
		//
		//		bool _cldTestNormal(dReal fp0, dReal fR, dVector3 vNormal, int iAxis);
		//		bool _cldTestFace(dReal fp0, dReal fp1, dReal fp2, dReal fR, dReal fD,
		//			dVector3 vNormal, int iAxis);
		//		bool _cldTestEdge(dReal fp0, dReal fp1, dReal fR, dReal fD,
		//			dVector3 vNormal, int iAxis);
		//		bool _cldTestSeparatingAxes(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2);
		//		void _cldClipping(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, int TriIndex);
		//		void _cldTestOneTriangle(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, int TriIndex);

		// box data
		final DMatrix3 m_mHullBoxRot = new DMatrix3();
		final DVector3 m_vHullBoxPos = new DVector3();
		final DVector3 m_vBoxHalfSize = new DVector3();

		// mesh data
		final DVector3   m_vHullDstPos = new DVector3();

		// global collider data
		final DVector3 m_vBestNormal = new DVector3();
		//dReal    m_fBestDepth;
		double    m_fBestDepth;
		int    m_iBestAxis;
		@SuppressWarnings("unused")
		int    m_iExitAxis;
		final DVector3 m_vE0 = new DVector3(), m_vE1 = new DVector3(), m_vE2 = new DVector3(), m_vN = new DVector3();

		// global info for contact creation
		int m_iFlags;
		//		dContactGeom *m_ContactGeoms;
		DContactGeomBuffer m_ContactGeoms;
		int m_iStride;
		//		dxGeom *m_Geom1;
		//		dxGeom *m_Geom2;
		DxGeom m_Geom1;
		DxGeom m_Geom2;
		List<DContactGeom> m_TempContactGeoms;
		//	};

		// Test normal of mesh face as separating axis for intersection
		//boolean sTrimeshBoxColliderData::_cldTestNormal(dReal fp0, dReal fR, dVector3 vNormal, int iAxis)
		boolean _cldTestNormal(double fp0, double fR, DVector3C vNormal, int iAxis)
		{
			// calculate overlapping interval of box and triangle
			double fDepth = fR+fp0;

			// if we do not overlap
			if ( fDepth<0 ) {
				// do nothing
				return false;
			}

			// calculate normal's length
			double fLength = LENGTHOF(vNormal);
			// if long enough
			if ( fLength > 0.0f ) {

				double fOneOverLength = 1.0f/fLength;
				// normalize depth
				fDepth = fDepth*fOneOverLength;

				// get minimum depth
				if (fDepth < m_fBestDepth) {
					//	      m_vBestNormal[0] = -vNormal[0]*fOneOverLength;
					//	      m_vBestNormal[1] = -vNormal[1]*fOneOverLength;
					//	      m_vBestNormal[2] = -vNormal[2]*fOneOverLength;
					m_vBestNormal.set(vNormal).scale(-fOneOverLength);
					m_iBestAxis = iAxis;
					//dAASSERT(fDepth>=0);
					m_fBestDepth = fDepth;
				}
			}

			return true;
		}




		// Test box axis as separating axis
		//	bool sTrimeshBoxColliderData::_cldTestFace(dReal fp0, dReal fp1, dReal fp2, dReal fR, dReal fD,
		//            dVector3 vNormal, int iAxis)
		boolean _cldTestFace(double fp0, double fp1, double fp2, double fR, double fD,
				DVector3 vNormal, int iAxis)
		{
			double fMin, fMax;

			// find min of triangle interval
			if ( fp0 < fp1 ) {
				if ( fp0 < fp2 ) {
					fMin = fp0;
				} else {
					fMin = fp2;
				}
			} else {
				if( fp1 < fp2 ) {
					fMin = fp1;
				} else {
					fMin = fp2;
				}
			}

			// find max of triangle interval
			if ( fp0 > fp1 ) {
				if ( fp0 > fp2 ) {
					fMax = fp0;
				} else {
					fMax = fp2;
				}
			} else {
				if( fp1 > fp2 ) {
					fMax = fp1;
				} else {
					fMax = fp2;
				}
			}

			// calculate minimum and maximum depth
			double fDepthMin = fR - fMin;
			double fDepthMax = fMax + fR;

			// if we dont't have overlapping interval
			if ( fDepthMin < 0 || fDepthMax < 0 ) {
				// do nothing
				return false;
			}

			double fDepth = 0;

			// if greater depth is on negative side
			if ( fDepthMin > fDepthMax ) {
				// use smaller depth (one from positive side)
				fDepth = fDepthMax;
				// flip normal direction
				//	    vNormal[0] = -vNormal[0];
				//	    vNormal[1] = -vNormal[1];
				//	    vNormal[2] = -vNormal[2];
				vNormal.scale(-1);
				fD = -fD;
				// if greater depth is on positive side
			} else {
				// use smaller depth (one from negative side)
				fDepth = fDepthMin;
			}

			// if lower depth than best found so far
			if (fDepth < m_fBestDepth) {
				// remember current axis as best axis
				//	    m_vBestNormal[0]  = vNormal[0];
				//	    m_vBestNormal[1]  = vNormal[1];
				//	    m_vBestNormal[2]  = vNormal[2];
				m_vBestNormal.set(vNormal);
				m_iBestAxis    = iAxis;
				//dAASSERT(fDepth>=0);
				m_fBestDepth   = fDepth;
			}

			return true;
		}

		// Test cross products of box axis and triangle edges as separating axis
		//	bool sTrimeshBoxColliderData::_cldTestEdge(dReal fp0, dReal fp1, dReal fR, dReal fD,
		//	                          dVector3 vNormal, int iAxis)
		boolean _cldTestEdge(double fp0, double fp1, double fR, double fD,
				DVector3 vNormal, int iAxis)
		{
			double fMin, fMax;

			// ===== Begin Patch by Francisco Leon, 2006/10/28 =====

			// Fixed Null Normal. This prevents boxes passing
			// through trimeshes at certain contact angles

			//	  fMin = vNormal[0] * vNormal[0] +
			//			 vNormal[1] * vNormal[1] +
			//			 vNormal[2] * vNormal[2];
			fMin = vNormal.lengthSquared();

			if ( fMin <= dEpsilon ) /// THIS NORMAL WOULD BE DANGEROUS
				return true;

			// ===== Ending Patch by Francisco Leon =====


			// calculate min and max interval values
			if ( fp0 < fp1 ) {
				fMin = fp0;
				fMax = fp1;
			} else {
				fMin = fp1;
				fMax = fp0;
			}

			// check if we overlapp
			double fDepthMin = fR - fMin;
			double fDepthMax = fMax + fR;

			// if we don't overlapp
			if ( fDepthMin < 0 || fDepthMax < 0 ) {
				// do nothing
				return false;
			}

			double fDepth;

			// if greater depth is on negative side
			if ( fDepthMin > fDepthMax ) {
				// use smaller depth (one from positive side)
				fDepth = fDepthMax;
				// flip normal direction
				//	    vNormal[0] = -vNormal[0];
				//	    vNormal[1] = -vNormal[1];
				//	    vNormal[2] = -vNormal[2];
				vNormal.scale(-1);
				fD = -fD;
				// if greater depth is on positive side
			} else {
				// use smaller depth (one from negative side)
				fDepth = fDepthMin;
			}

			// calculate normal's length
			double fLength = LENGTHOF(vNormal);

			// if long enough
			if ( fLength > 0.0f ) {

				// normalize depth
				double fOneOverLength = 1.0f/fLength;
				fDepth = fDepth*fOneOverLength;
				fD*=fOneOverLength;

				// if lower depth than best found so far (favor face over edges)
				if (fDepth*1.5f < m_fBestDepth) {
					// remember current axis as best axis
					//	      m_vBestNormal[0]  = vNormal[0]*fOneOverLength;
					//	      m_vBestNormal[1]  = vNormal[1]*fOneOverLength;
					//	      m_vBestNormal[2]  = vNormal[2]*fOneOverLength;
					m_vBestNormal.set(vNormal).scale(fOneOverLength);  // TODO TZ here positive, above negative, correct? 
					m_iBestAxis    = iAxis;
					//dAASSERT(fDepth>=0);
					m_fBestDepth   = fDepth;
				}
			}

			return true;
		}


		// clip polygon with plane and generate new polygon points
		//	static void _cldClipPolyToPlane( dVector3 avArrayIn[], int ctIn,
		//	                      dVector3 avArrayOut[], int &ctOut,
		//	                      const dVector4 &plPlane )
		void _cldClipPolyToPlane( DVector3 avArrayIn[], int ctIn,
				DVector3 avArrayOut[], RefInt ctOut,
				final DVector4C plPlane )
		{
			// start with no output points
			ctOut.i = 0;

			int i0 = ctIn-1;

			// for each edge in input polygon
			for (int i1=0; i1<ctIn; i0=i1, i1++) {


				// calculate distance of edge points to plane
				double fDistance0 = POINTDISTANCE( plPlane ,avArrayIn[i0] );
				double fDistance1 = POINTDISTANCE( plPlane ,avArrayIn[i1] );


				// if first point is in front of plane
				if( fDistance0 >= 0 ) {
					// emit point
					//	      avArrayOut[ctOut][0] = avArrayIn[i0][0];
					//	      avArrayOut[ctOut][1] = avArrayIn[i0][1];
					//	      avArrayOut[ctOut][2] = avArrayIn[i0][2];
					avArrayOut[ctOut.i].set( avArrayIn[i0] ); 
					ctOut.i++;
				}

				// if points are on different sides
				if( (fDistance0 > 0 && fDistance1 < 0) || ( fDistance0 < 0 && fDistance1 > 0) ) {

					// find intersection point of edge and plane
					DVector3 vIntersectionPoint = new DVector3();
					//	      vIntersectionPoint[0]= avArrayIn[i0][0] - (avArrayIn[i0][0]-avArrayIn[i1][0])*fDistance0/(fDistance0-fDistance1);
					//	      vIntersectionPoint[1]= avArrayIn[i0][1] - (avArrayIn[i0][1]-avArrayIn[i1][1])*fDistance0/(fDistance0-fDistance1);
					//	      vIntersectionPoint[2]= avArrayIn[i0][2] - (avArrayIn[i0][2]-avArrayIn[i1][2])*fDistance0/(fDistance0-fDistance1);
					vIntersectionPoint.eqDiff( avArrayIn[i0], avArrayIn[i1] );
					vIntersectionPoint.scale( - fDistance0/(fDistance0-fDistance1) ); //negate!
					vIntersectionPoint.add( avArrayIn[i0] );

					// emit intersection point
					//	      avArrayOut[ctOut][0] = vIntersectionPoint[0];
					//	      avArrayOut[ctOut][1] = vIntersectionPoint[1];
					//	      avArrayOut[ctOut][2] = vIntersectionPoint[2];
					avArrayOut[ctOut.i].set( vIntersectionPoint );
					ctOut.i++;
				}
			}

		}


		//	bool sTrimeshBoxColliderData::_cldTestSeparatingAxes(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2) {
		boolean _cldTestSeparatingAxes(final DVector3C v0, final DVector3C v1, final DVector3C v2) {
			// reset best axis
			m_iBestAxis = 0;
			m_iExitAxis = -1;
			m_fBestDepth = MAXVALUE;

			// calculate edges
			SUBTRACT(v1,v0,m_vE0);
			SUBTRACT(v2,v0,m_vE1);
			SUBTRACT(m_vE1,m_vE0,m_vE2);

			// calculate poly normal
			//dCROSS(m_vN,=,m_vE0,m_vE1);
			m_vN.eqCross(m_vE0, m_vE1);

			// calculate length of face normal
			double fNLen = LENGTHOF(m_vN);

			// Even though all triangles might be initially valid, 
			// a triangle may degenerate into a segment after applying 
			// space transformation.
			if (fNLen == 0) {
				return false;
			}

			// extract box axes as vectors
			DVector3 vA0 = new DVector3(), vA1 = new DVector3(), vA2 = new DVector3();
			GETCOL(m_mHullBoxRot,0,vA0);
			GETCOL(m_mHullBoxRot,1,vA1);
			GETCOL(m_mHullBoxRot,2,vA2);

			// box halfsizes
			double fa0 = m_vBoxHalfSize.get0();
			double fa1 = m_vBoxHalfSize.get1();
			double fa2 = m_vBoxHalfSize.get2();

			// calculate relative position between box and triangle
			DVector3 vD = new DVector3();
			SUBTRACT(v0,m_vHullBoxPos,vD);

			DVector3 vL = new DVector3();
			double fp0, fp1, fp2, fR, fD;

			// Test separating axes for intersection
			// ************************************************
			// Axis 1 - Triangle Normal
			SET(vL,m_vN);
			fp0  = vL.dot(vD);
			fp1  = fp0;
			fp2  = fp0;
			fR=fa0*dFabs( m_vN.dot(vA0) ) + fa1 * dFabs( m_vN.dot(vA1) ) + fa2 * dFabs( m_vN.dot(vA2) );

			if (!_cldTestNormal(fp0, fR, vL, 1)) {
				m_iExitAxis=1;
				return false;
			}

			// ************************************************

			// Test Faces
			// ************************************************
			// Axis 2 - Box X-Axis
			SET(vL,vA0);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0 + vA0.dot(m_vE0);
			fp2 = fp0 + vA0.dot(m_vE1);
			fR  = fa0;

			if (!_cldTestFace(fp0, fp1, fp2, fR, fD, vL, 2)) {
				m_iExitAxis=2;
				return false;
			}
			// ************************************************

			// ************************************************
			// Axis 3 - Box Y-Axis
			SET(vL,vA1);
			fD = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0 + vA1.dot(m_vE0);
			fp2 = fp0 + vA1.dot(m_vE1);
			fR  = fa1;

			if (!_cldTestFace(fp0, fp1, fp2, fR, fD, vL, 3)) {
				m_iExitAxis=3;
				return false;
			}

			// ************************************************

			// ************************************************
			// Axis 4 - Box Z-Axis
			SET(vL,vA2);
			fD = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0 + vA2.dot(m_vE0);
			fp2 = fp0 + vA2.dot(m_vE1);
			fR  = fa2;

			if (!_cldTestFace(fp0, fp1, fp2, fR, fD, vL, 4)) {
				m_iExitAxis=4;
				return false;
			}

			// ************************************************

			// Test Edges
			// ************************************************
			// Axis 5 - Box X-Axis cross Edge0
			//dCROSS(vL,=,vA0,m_vE0);
			vL.eqCross(vA0, m_vE0);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0;
			fp2 = fp0 + vA0.dot(m_vN);
			fR  = fa1 * dFabs(vA2.dot(m_vE0)) + fa2 * dFabs(vA1.dot(m_vE0));

			if (!_cldTestEdge(fp1, fp2, fR, fD, vL, 5)) {
				m_iExitAxis=5;
				return false;
			}
			// ************************************************

			// ************************************************
			// Axis 6 - Box X-Axis cross Edge1
			//dCROSS(vL,=,vA0,m_vE1);
			vL.eqCross(vA0, m_vE1);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0 - vA0.dot(m_vN);
			fp2 = fp0;
			fR  = fa1 * dFabs(vA2.dot(m_vE1)) + fa2 * dFabs(vA1.dot(m_vE1));

			if (!_cldTestEdge(fp0, fp1, fR, fD, vL, 6)) {
				m_iExitAxis=6;
				return false;
			}
			// ************************************************

			// ************************************************
			// Axis 7 - Box X-Axis cross Edge2
			//dCROSS(vL,=,vA0,m_vE2);
			vL.eqCross(vA0, m_vE2);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0 - vA0.dot(m_vN);
			fp2 = fp0 - vA0.dot(m_vN);
			fR  = fa1 * dFabs(vA2.dot(m_vE2)) + fa2 * dFabs(vA1.dot(m_vE2));

			if (!_cldTestEdge(fp0, fp1, fR, fD, vL, 7)) {
				m_iExitAxis=7;
				return false;
			}

			// ************************************************

			// ************************************************
			// Axis 8 - Box Y-Axis cross Edge0
			//dCROSS(vL,=,vA1,m_vE0);
			vL.eqCross(vA1, m_vE0);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0;
			fp2 = fp0 + vA1.dot(m_vN);
			fR  = fa0 * dFabs(vA2.dot(m_vE0)) + fa2 * dFabs(vA0.dot(m_vE0));

			if (!_cldTestEdge(fp0, fp2, fR, fD, vL, 8)) {
				m_iExitAxis=8;
				return false;
			}

			// ************************************************

			// ************************************************
			// Axis 9 - Box Y-Axis cross Edge1
			//dCROSS(vL,=,vA1,m_vE1);
			vL.eqCross(vA1, m_vE1);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0 - vA1.dot(m_vN);
			fp2 = fp0;
			fR  = fa0 * dFabs(vA2.dot(m_vE1)) + fa2 * dFabs(vA0.dot(m_vE1));

			if (!_cldTestEdge(fp0, fp1, fR, fD, vL, 9)) {
				m_iExitAxis=9;
				return false;
			}

			// ************************************************

			// ************************************************
			// Axis 10 - Box Y-Axis cross Edge2
			//dCROSS(vL,=,vA1,m_vE2);
			vL.eqCross(vA1, m_vE2);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0 - vA1.dot(m_vN);
			fp2 = fp0 - vA1.dot(m_vN);
			fR  = fa0 * dFabs(vA2.dot(m_vE2)) + fa2 * dFabs(vA0.dot(m_vE2));

			if (!_cldTestEdge(fp0, fp1, fR, fD, vL, 10)) {
				m_iExitAxis=10;
				return false;
			}

			// ************************************************

			// ************************************************
			// Axis 11 - Box Z-Axis cross Edge0
			//dCROSS(vL,=,vA2,m_vE0);
			vL.eqCross(vA2, m_vE0);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0;
			fp2 = fp0 + vA2.dot(m_vN);
			fR  = fa0 * dFabs(vA1.dot(m_vE0)) + fa1 * dFabs(vA0.dot(m_vE0));

			if (!_cldTestEdge(fp0, fp2, fR, fD, vL, 11)) {
				m_iExitAxis=11;
				return false;
			}
			// ************************************************

			// ************************************************
			// Axis 12 - Box Z-Axis cross Edge1
			//dCROSS(vL,=,vA2,m_vE1);
			vL.eqCross(vA2, m_vE1);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0 - vA2.dot(m_vN);
			fp2 = fp0;
			fR  = fa0 * dFabs(vA1.dot(m_vE1)) + fa1 * dFabs(vA0.dot(m_vE1));

			if (!_cldTestEdge(fp0, fp1, fR, fD, vL, 12)) {
				m_iExitAxis=12;
				return false;
			}
			// ************************************************

			// ************************************************
			// Axis 13 - Box Z-Axis cross Edge2
			//	  OdeMath.dCROSS(vL,OP.EQ,vA2,m_vE2);
			vL.eqCross(vA2, m_vE2);
			fD  = vL.dot(m_vN)/fNLen;
			fp0 = vL.dot(vD);
			fp1 = fp0 - vA2.dot(m_vN);
			fp2 = fp0 - vA2.dot(m_vN);
			fR  = fa0 * dFabs(vA1.dot(m_vE2)) + fa1 * dFabs(vA0.dot(m_vE2));

			if (!_cldTestEdge(fp0, fp1, fR, fD, vL, 13)) {
				m_iExitAxis=13;
				return false;
			}

			// ************************************************
			return true;
		}





		// find two closest points on two lines
		//	static bool _cldClosestPointOnTwoLines( dVector3 vPoint1, dVector3 vLenVec1,
		//	                                        dVector3 vPoint2, dVector3 vLenVec2,
		//	                                        dReal &fvalue1, dReal &fvalue2)
		boolean _cldClosestPointOnTwoLines( DVector3C vPoint1, DVector3C vLenVec1,
				DVector3C vPoint2, DVector3C vLenVec2,
				RefDouble fvalue1, RefDouble fvalue2)
		{
			// calculate denominator
			DVector3 vp = new DVector3();
			SUBTRACT(vPoint2,vPoint1,vp);
			double fuaub  = vLenVec1.dot(vLenVec2);
			double fq1    = vLenVec1.dot(vp);
			double fq2    = -vLenVec2.dot(vp);
			double fd     = 1.0f - fuaub * fuaub;

			// if denominator is positive
			if (fd > 0.0f) {
				// calculate points of closest approach
				fd = 1.0f/fd;
				fvalue1.d = (fq1 + fuaub*fq2)*fd;
				fvalue2.d = (fuaub*fq1 + fq2)*fd;
				return true;
				// otherwise
			} else {
				// lines are parallel
				fvalue1.d = 0.0f;
				fvalue2.d = 0.0f;
				return false;
			}
		}





		// clip and generate contacts
		//	void sTrimeshBoxColliderData::_cldClipping(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, int TriIndex) {
		void _cldClipping(final DVector3C v0, final DVector3C v1, final DVector3C v2, final int TriIndex) {

			// if we have edge/edge intersection
			if (m_iBestAxis > 4 ) {
				DVector3 vub = new DVector3(), vPb = new DVector3(), vPa = new DVector3();

				SET(vPa,m_vHullBoxPos);

				// calculate point on box edge
				for( int i=0; i<3; i++) {
					DVector3 vRotCol = new DVector3();
					GETCOL(m_mHullBoxRot,i,vRotCol);
					double fSign = m_vBestNormal.dot(vRotCol) > 0 ? 1.0f : -1.0f;

					//	      vPa[0] += fSign * m_vBoxHalfSize[i] * vRotCol[0];
					//	      vPa[1] += fSign * m_vBoxHalfSize[i] * vRotCol[1];
					//	      vPa[2] += fSign * m_vBoxHalfSize[i] * vRotCol[2];
					vPa.addScaled( vRotCol, fSign * m_vBoxHalfSize.get(i));  //TODO use colView!
				}

				int iEdge = (m_iBestAxis-5)%3;

				// decide which edge is on triangle
				if ( iEdge == 0 ) {
					SET(vPb,v0);
					SET(vub,m_vE0);
				} else if ( iEdge == 1) {
					SET(vPb,v2);
					SET(vub,m_vE1);
				} else {
					SET(vPb,v1);
					SET(vub,m_vE2);
				}


				// setup direction parameter for face edge
				//dNormalize3(vub);
				vub.normalize();

				RefDouble fParam1 = new RefDouble(), fParam2 = new RefDouble();

				// setup direction parameter for box edge
				DVector3 vua = new DVector3();
				int col=(m_iBestAxis-5)/3;
				GETCOL(m_mHullBoxRot,col,vua);

				// find two closest points on both edges
				_cldClosestPointOnTwoLines( vPa, vua, vPb, vub, fParam1, fParam2 );
				//	    vPa[0] += vua[0]*fParam1;
				//	    vPa[1] += vua[1]*fParam1;
				//	    vPa[2] += vua[2]*fParam1;
				vPa.addScaled( vua, fParam1.d );

				//	    vPb[0] += vub[0]*fParam2;
				//	    vPb[1] += vub[1]*fParam2;
				//	    vPb[2] += vub[2]*fParam2;
				vPb.addScaled( vub, fParam2.d );

				// calculate collision point
				DVector3 vPntTmp = new DVector3();
				ADD(vPa,vPb,vPntTmp);

				//	    vPntTmp[0]*=0.5f;
				//	    vPntTmp[1]*=0.5f;
				//	    vPntTmp[2]*=0.5f;
				vPntTmp.scale( 0.5 );

				// generate contact point between two closest points
				//	#if 0 //#ifdef ORIG -- if to use conditional define, GenerateContact must be moved into #else
				//		dContactGeom* Contact = SAFECONTACT(m_iFlags, m_ContactGeoms, m_ctContacts, m_iStride);
				//		Contact->depth = m_fBestDepth;
				//		SET(Contact->normal,m_vBestNormal);
				//		SET(Contact->pos,vPntTmp);
				//		Contact->g1 = Geom1;
				//		Contact->g2 = Geom2;
				//		Contact->side1 = TriIndex;
				//		Contact->side2 = -1;
				//		m_ctContacts++;
				//	#endif
				GenerateContact(m_iFlags, m_TempContactGeoms, m_iStride, m_Geom1, m_Geom2, TriIndex,
						vPntTmp, m_vBestNormal, m_fBestDepth);


				// if triangle is the referent face then clip box to triangle face
			} else if (m_iBestAxis == 1) {

				DVector3 vNormal2 = new DVector3();
				//	    vNormal2[0]=-m_vBestNormal[0];
				//	    vNormal2[1]=-m_vBestNormal[1];
				//	    vNormal2[2]=-m_vBestNormal[2];
				vNormal2.set( m_vBestNormal ).scale( -1 );


				// vNr is normal in box frame, pointing from triangle to box
				DMatrix3 mTransposed;// = new DMatrix3();
				//	    mTransposed[0*4+0]=m_mHullBoxRot[0*4+0];
				//	    mTransposed[0*4+1]=m_mHullBoxRot[1*4+0];
				//	    mTransposed[0*4+2]=m_mHullBoxRot[2*4+0];
				//
				//	    mTransposed[1*4+0]=m_mHullBoxRot[0*4+1];
				//	    mTransposed[1*4+1]=m_mHullBoxRot[1*4+1];
				//	    mTransposed[1*4+2]=m_mHullBoxRot[2*4+1];
				//
				//	    mTransposed[2*4+0]=m_mHullBoxRot[0*4+2];
				//	    mTransposed[2*4+1]=m_mHullBoxRot[1*4+2];
				//	    mTransposed[2*4+2]=m_mHullBoxRot[2*4+2];
				mTransposed = m_mHullBoxRot.reTranspose();

				DVector3 vNr = new DVector3();
				//	    vNr[0]=mTransposed[0*4+0]*vNormal2[0]+  mTransposed[0*4+1]*vNormal2[1]+  mTransposed[0*4+2]*vNormal2[2];
				//	    vNr[1]=mTransposed[1*4+0]*vNormal2[0]+  mTransposed[1*4+1]*vNormal2[1]+  mTransposed[1*4+2]*vNormal2[2];
				//	    vNr[2]=mTransposed[2*4+0]*vNormal2[0]+  mTransposed[2*4+1]*vNormal2[1]+  mTransposed[2*4+2]*vNormal2[2];
				vNr.eqProd(mTransposed, vNormal2);


				DVector3 vAbsNormal = new DVector3();
				//	    vAbsNormal[0] = dFabs( vNr[0] );
				//	    vAbsNormal[1] = dFabs( vNr[1] );
				//	    vAbsNormal[2] = dFabs( vNr[2] );
				vAbsNormal.set( vNr ).eqAbs();

				// get closest face from box
				int iB0, iB1, iB2;
				if (vAbsNormal.get1() > vAbsNormal.get0()) {
					if (vAbsNormal.get1() > vAbsNormal.get2()) {
						iB1 = 0;  iB0 = 1;  iB2 = 2;
					} else {
						iB1 = 0;  iB2 = 1;  iB0 = 2;
					}
				} else {

					if (vAbsNormal.get0() > vAbsNormal.get2()) {
						iB0 = 0;  iB1 = 1;  iB2 = 2;
					} else {
						iB1 = 0;  iB2 = 1;  iB0 = 2;
					}
				}

				// Here find center of box face we are going to project
				DVector3 vCenter = new DVector3();
				DVector3 vRotCol = new DVector3();
				GETCOL(m_mHullBoxRot,iB0,vRotCol);

				if (vNr.get(iB0) > 0) {
					//	        vCenter[0] = m_vHullBoxPos[0] - v0[0] - m_vBoxHalfSize[iB0] * vRotCol[0];
					//	      vCenter[1] = m_vHullBoxPos[1] - v0[1] - m_vBoxHalfSize[iB0] * vRotCol[1];
					//	      vCenter[2] = m_vHullBoxPos[2] - v0[2] - m_vBoxHalfSize[iB0] * vRotCol[2];
					vCenter.eqSum(v0, -1, vRotCol, -m_vBoxHalfSize.get(iB0));
					vCenter.add(m_vHullBoxPos);
				} else {
					//	      vCenter[0] = m_vHullBoxPos[0] - v0[0] + m_vBoxHalfSize[iB0] * vRotCol[0];
					//	      vCenter[1] = m_vHullBoxPos[1] - v0[1] + m_vBoxHalfSize[iB0] * vRotCol[1];
					//	      vCenter[2] = m_vHullBoxPos[2] - v0[2] + m_vBoxHalfSize[iB0] * vRotCol[2];
					vCenter.eqSum(v0, -1, vRotCol, m_vBoxHalfSize.get(iB0));
					vCenter.add(m_vHullBoxPos);
				}

				// Here find 4 corner points of box
				DVector3[] avPoints = { new DVector3(), new DVector3(), new DVector3(), new DVector3() };//[4];

				DVector3 vRotCol2 = new DVector3();
				GETCOL(m_mHullBoxRot,iB1,vRotCol);
				GETCOL(m_mHullBoxRot,iB2,vRotCol2);

				//	    for(int x=0;x<3;x++) {
				//	        avPoints[0][x] = vCenter[x] + (m_vBoxHalfSize[iB1] * vRotCol[x]) - (m_vBoxHalfSize[iB2] * vRotCol2[x]);
				//	        avPoints[1][x] = vCenter[x] - (m_vBoxHalfSize[iB1] * vRotCol[x]) - (m_vBoxHalfSize[iB2] * vRotCol2[x]);
				//	        avPoints[2][x] = vCenter[x] - (m_vBoxHalfSize[iB1] * vRotCol[x]) + (m_vBoxHalfSize[iB2] * vRotCol2[x]);
				//	        avPoints[3][x] = vCenter[x] + (m_vBoxHalfSize[iB1] * vRotCol[x]) + (m_vBoxHalfSize[iB2] * vRotCol2[x]);
				//	    }
				double tz1 = m_vBoxHalfSize.get(iB1);
				double tz2 = m_vBoxHalfSize.get(iB2);
				avPoints[0].eqSum( vRotCol,  tz1, vRotCol2, -tz2).add (vCenter);
				avPoints[1].eqSum( vRotCol, -tz1, vRotCol2, -tz2).add (vCenter);
				avPoints[2].eqSum( vRotCol, -tz1, vRotCol2,  tz2).add (vCenter);
				avPoints[3].eqSum( vRotCol,  tz1, vRotCol2,  tz2).add (vCenter);

				// clip Box face with 4 planes of triangle (1 face plane, 3 egde planes)
				DVector3[] avTempArray1 = DVector3.newArray(9);
				DVector3[] avTempArray2 = DVector3.newArray(9);
				DVector4 plPlane = new DVector4();

				RefInt iTempCnt1 = new RefInt(0);
				RefInt iTempCnt2 = new RefInt(0);

				// zeroify vectors - necessary?
//				for(int i=0; i<9; i++) {
//					//	      avTempArray1[i][0]=0;
//					//	      avTempArray1[i][1]=0;
//					//	      avTempArray1[i][2]=0;
//					avTempArray1[i] = new DVector3();
//
//					//	      avTempArray2[i][0]=0;
//					//	      avTempArray2[i][1]=0;
//					//	      avTempArray2[i][2]=0;
//					avTempArray2[i] = new DVector3();
//				}


				// Normal plane
				DVector3 vTemp = new DVector3();
				//	    vTemp[0]=-m_vN[0];
				//	    vTemp[1]=-m_vN[1];
				//	    vTemp[2]=-m_vN[2];
				vTemp.set( m_vN ).scale(-1);
				//dNormalize3(vTemp);
				vTemp.normalize();
				CONSTRUCTPLANE(plPlane,vTemp,0);

				_cldClipPolyToPlane( avPoints, 4, avTempArray1, iTempCnt1, plPlane  );


				// Plane p0
				DVector3 vTemp2 = new DVector3();
				SUBTRACT(v1,v0,vTemp2);
				//dCROSS(vTemp,=,m_vN,vTemp2);
				vTemp.eqCross(m_vN, vTemp2);
				//dNormalize3(vTemp);
				vTemp.normalize();
				CONSTRUCTPLANE(plPlane,vTemp,0);

				_cldClipPolyToPlane( avTempArray1, iTempCnt1.i, avTempArray2, iTempCnt2, plPlane  );

				// Plane p1
				SUBTRACT(v2,v1,vTemp2);
				//dCROSS(vTemp,=,m_vN,vTemp2);
				vTemp.eqCross(m_vN, vTemp2);
				//dNormalize3(vTemp);
				vTemp.normalize();
				SUBTRACT(v0,v2,vTemp2);
				CONSTRUCTPLANE(plPlane,vTemp,vTemp2.dot(vTemp));

				_cldClipPolyToPlane( avTempArray2, iTempCnt2.i, avTempArray1, iTempCnt1, plPlane  );

				// Plane p2
				SUBTRACT(v0,v2,vTemp2);
				//dCROSS(vTemp,=,m_vN,vTemp2);
				vTemp.eqCross(m_vN, vTemp2);
				//dNormalize3(vTemp);
				vTemp.normalize();
				CONSTRUCTPLANE(plPlane,vTemp,0);

				_cldClipPolyToPlane( avTempArray1, iTempCnt1.i, avTempArray2, iTempCnt2, plPlane  );

				// END of clipping polygons

				// for each generated contact point
				for ( int i=0; i<iTempCnt2.i; i++ ) {
					// calculate depth
					double fTempDepth = vNormal2.dot(avTempArray2[i]);

					// clamp depth to zero
					if (fTempDepth > 0) {
						fTempDepth = 0;
					}

					DVector3 vPntTmp = new DVector3();
					ADD(avTempArray2[i],v0,vPntTmp);

					//	#if 0 //#ifdef ORIG -- if to use conditional define, GenerateContact must be moved into #else
					//	      dContactGeom* Contact = SAFECONTACT(m_iFlags, m_ContactGeoms, m_ctContacts, m_iStride);
					//		  
					//	      Contact->depth = -fTempDepth;
					//	      SET(Contact->normal,m_vBestNormal);
					//	      SET(Contact->pos,vPntTmp);
					//	      Contact->g1 = Geom1;
					//	      Contact->g2 = Geom2;
					//		  Contact->side1 = TriIndex;
					//		  Contact->side2 = -1;
					//	      m_ctContacts++;
					//	#endif
					GenerateContact(m_iFlags, m_TempContactGeoms, m_iStride,  m_Geom1, m_Geom2, TriIndex,
							vPntTmp, m_vBestNormal, -fTempDepth);

				}

				//dAASSERT(m_ctContacts>0);

				// if box face is the referent face, then clip triangle on box face
			} else { // 2 <= if iBestAxis <= 4

				// get normal of box face
				DVector3 vNormal2 = new DVector3();
				SET(vNormal2,m_vBestNormal);

				// get indices of box axes in correct order
				int iA0,iA1,iA2;
				iA0 = m_iBestAxis-2;
				if ( iA0 == 0 ) {
					iA1 = 1; iA2 = 2;
				} else if ( iA0 == 1 ) {
					iA1 = 0; iA2 = 2;
				} else {
					iA1 = 0; iA2 = 1;
				}

				DVector3[] avPoints = { new DVector3(), new DVector3(), new DVector3() };//new DVector3[3];
				// calculate triangle vertices in box frame
				SUBTRACT(v0,m_vHullBoxPos,avPoints[0]);
				SUBTRACT(v1,m_vHullBoxPos,avPoints[1]);
				SUBTRACT(v2,m_vHullBoxPos,avPoints[2]);

				// CLIP Polygons
				// define temp data for clipping
				DVector3[] avTempArray1 = DVector3.newArray(9);
				DVector3[] avTempArray2 = DVector3.newArray(9);

				RefInt iTempCnt1 = new RefInt(), iTempCnt2 = new RefInt();

				// zeroify vectors - necessary?
//				for(int i=0; i<9; i++) {
//					//	      avTempArray1[i][0]=0;
//					//	      avTempArray1[i][1]=0;
//					//	      avTempArray1[i][2]=0;
//					avTempArray1[i] = new DVector3();
//
//					//	      avTempArray2[i][0]=0;
//					//	      avTempArray2[i][1]=0;
//					//	      avTempArray2[i][2]=0;
//					avTempArray2[i] = new DVector3();
//				}

				// clip triangle with 5 box planes (1 face plane, 4 edge planes)

				DVector4 plPlane = new DVector4();

				// Normal plane
				DVector3 vTemp = new DVector3();
				//	    vTemp[0]=-vNormal2[0];
				//	    vTemp[1]=-vNormal2[1];
				//	    vTemp[2]=-vNormal2[2];
				vTemp.set(vNormal2).scale(-1);
				CONSTRUCTPLANE(plPlane,vTemp,m_vBoxHalfSize.get(iA0));

				_cldClipPolyToPlane( avPoints, 3, avTempArray1, iTempCnt1, plPlane );


				// Plane p0
				GETCOL(m_mHullBoxRot,iA1,vTemp);
				CONSTRUCTPLANE(plPlane,vTemp,m_vBoxHalfSize.get(iA1));

				_cldClipPolyToPlane( avTempArray1, iTempCnt1.i, avTempArray2, iTempCnt2, plPlane );


				// Plane p1
				GETCOL(m_mHullBoxRot,iA1,vTemp);
				//	    vTemp[0]=-vTemp[0];
				//	    vTemp[1]=-vTemp[1];
				//	    vTemp[2]=-vTemp[2];
				vTemp.scale(-1);
				CONSTRUCTPLANE(plPlane,vTemp,m_vBoxHalfSize.get(iA1));

				_cldClipPolyToPlane( avTempArray2, iTempCnt2.i, avTempArray1, iTempCnt1, plPlane );

				// Plane p2
				GETCOL(m_mHullBoxRot,iA2,vTemp);
				CONSTRUCTPLANE(plPlane,vTemp,m_vBoxHalfSize.get(iA2));

				_cldClipPolyToPlane( avTempArray1, iTempCnt1.i, avTempArray2, iTempCnt2, plPlane );

				// Plane p3
				GETCOL(m_mHullBoxRot,iA2,vTemp);
				//	    vTemp[0]=-vTemp[0];
				//	    vTemp[1]=-vTemp[1];
				//	    vTemp[2]=-vTemp[2];
				vTemp.scale(-1);
				CONSTRUCTPLANE(plPlane,vTemp,m_vBoxHalfSize.get(iA2));

				_cldClipPolyToPlane( avTempArray2, iTempCnt2.i, avTempArray1, iTempCnt1, plPlane );


				// for each generated contact point
				for ( int i=0; i<iTempCnt1.i; i++ ) {
					// calculate depth
					double fTempDepth = vNormal2.dot(avTempArray1[i])-m_vBoxHalfSize.get(iA0);

					// clamp depth to zero
					if (fTempDepth > 0) {
						fTempDepth = 0;
					}

					// generate contact data
					DVector3 vPntTmp = new DVector3();
					ADD(avTempArray1[i],m_vHullBoxPos,vPntTmp);

					//	#if 0 //#ifdef ORIG -- if to use conditional define, GenerateContact must be moved into #else
					//	      dContactGeom* Contact = SAFECONTACT(m_iFlags, m_ContactGeoms, m_ctContacts, m_iStride);
					//
					//	      Contact->depth = -fTempDepth;
					//	      SET(Contact->normal,m_vBestNormal);
					//	      SET(Contact->pos,vPntTmp);
					//	      Contact->g1 = Geom1;
					//	      Contact->g2 = Geom2;
					//		  Contact->side1 = TriIndex;
					//		  Contact->side2 = -1;
					//	      m_ctContacts++;
					//	#endif
					GenerateContact(m_iFlags, m_TempContactGeoms, m_iStride,  m_Geom1, m_Geom2, TriIndex,
							vPntTmp, m_vBestNormal, -fTempDepth);

				}

				//dAASSERT(m_ctContacts>0);
			}
		}





		// test one mesh triangle on intersection with given box
		//	void sTrimeshBoxColliderData::_cldTestOneTriangle(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, int TriIndex)//, void *pvUser)
		private void _cldTestOneTriangle(final DVector3C v0, final DVector3C v1, 
				final DVector3C v2, int TriIndex)//, void *pvUser)
		{
			// do intersection test and find best separating axis
			if(!_cldTestSeparatingAxes(v0, v1, v2)) {
				// if not found do nothing
				return;
			}

			// if best separation axis is not found
			if (m_iBestAxis == 0) {
				// this should not happen (we should already exit in that case)
				//dMessage (0, "best separation axis not found");
				// do nothing
				return;
			}

			_cldClipping(v0, v1, v2, TriIndex);
		}


		//	void sTrimeshBoxColliderData::SetupInitialContext(dxTriMesh *TriMesh, dxGeom *BoxGeom,
		//		int Flags, dContactGeom* Contacts, int Stride)
		void SetupInitialContext(DxTriMesh TriMesh, DxBox BoxGeom,
				int Flags, DContactGeomBuffer Contacts, int Stride)
		{
			//	  // get source hull position, orientation and half size
			//	  const dMatrix3& mRotBox=*(const dMatrix3*)dGeomGetRotation(BoxGeom);
			//	  const dVector3& vPosBox=*(const dVector3*)dGeomGetPosition(BoxGeom);
			//
			//	  // to global
			//	  SETM(m_mHullBoxRot,mRotBox);
			//	  SET(m_vHullBoxPos,vPosBox);
			m_mHullBoxRot.set(BoxGeom.getRotation());
			m_vHullBoxPos.set(BoxGeom.getPosition());

			//dGeomBoxGetLengths(BoxGeom, m_vBoxHalfSize);
			m_vBoxHalfSize.set(BoxGeom.getLengths());
			//	  m_vBoxHalfSize[0] *= 0.5f;
			//	  m_vBoxHalfSize[1] *= 0.5f;
			//	  m_vBoxHalfSize[2] *= 0.5f;
			m_vBoxHalfSize.scale(0.5f);

			// get destination hull position and orientation
			//	  const dVector3& vPosMesh=*(const dVector3*)dGeomGetPosition(TriMesh);
			//
			//	  // to global
			//	  SET(m_vHullDstPos,vPosMesh);
			m_vHullDstPos.set(TriMesh.getPosition());

			// global info for contact creation
			m_TempContactGeoms = new ArrayList<DContactGeom>();
			if (Stride != 1) throw new IllegalArgumentException("stride = " + Stride);
			m_iStride=Stride;
			m_iFlags=Flags;
			m_ContactGeoms=Contacts;
			m_Geom1=TriMesh;
			m_Geom2=BoxGeom;

			// reset stuff
			m_fBestDepth = MAXVALUE;
			//	  m_vBestNormal[0]=0;
			//	  m_vBestNormal[1]=0;
			//	  m_vBestNormal[2]=0;
			m_vBestNormal.setZero();
		}

		//	int sTrimeshBoxColliderData::TestCollisionForSingleTriangle(int ctContacts0, int Triint, 
		//		dVector3 dv[3], bool &bOutFinishSearching)
		private int TestCollisionForSingleTriangle(int ctContacts0, int Triint, 
				DVector3 dv[], RefBoolean bOutFinishSearching)
		{
			// test this triangle
			_cldTestOneTriangle(dv[0],dv[1],dv[2],Triint);

			// fill-in tri index for generated contacts
			for (; ctContacts0 < m_TempContactGeoms.size(); ctContacts0++) {
				//DContactGeom pContact = SAFECONTACT(m_iFlags, m_ContactGeoms, ctContacts0, m_iStride);
				DContactGeom pContact = m_TempContactGeoms.get(ctContacts0);
				pContact.side1 = Triint;
				pContact.side2 = -1;
			}

			return ctContacts0;
		}


		//	// OPCODE version of box to mesh collider
		//	#if dTRIMESH_OPCODE
		//	static void dQueryBTLPotentialCollisionTriangles(OBBCollider &Collider, 
		//	  const sTrimeshBoxColliderData &cData, dxTriMesh *TriMesh, dxGeom *BoxGeom,
		//	  OBBCache &BoxCache)
		//	{
		//	  // get source hull position, orientation and half size
		//	  const dMatrix3& mRotBox=*(const dMatrix3*)dGeomGetRotation(BoxGeom);
		//	  const dVector3& vPosBox=*(const dVector3*)dGeomGetPosition(BoxGeom);
		//
		//	  // Make OBB
		//	  OBB Box;
		//	  Box.mCenter.x = vPosBox[0];
		//	  Box.mCenter.y = vPosBox[1];
		//	  Box.mCenter.z = vPosBox[2];
		//
		//	  // It is a potential issue to explicitly cast to float 
		//	  // if custom width floating point type is introduced in OPCODE.
		//	  // It is necessary to make a typedef and cast to it
		//	  // (e.g. typedef float opc_float;)
		//	  // However I'm not sure in what header it should be added.
		//
		//	  Box.mExtents.x = /*(float)*/cData.m_vBoxHalfSize[0];
		//	  Box.mExtents.y = /*(float)*/cData.m_vBoxHalfSize[1];
		//	  Box.mExtents.z = /*(float)*/cData.m_vBoxHalfSize[2];
		//
		//	  Box.mRot.m[0][0] = /*(float)*/mRotBox[0];
		//	  Box.mRot.m[1][0] = /*(float)*/mRotBox[1];
		//	  Box.mRot.m[2][0] = /*(float)*/mRotBox[2];
		//
		//	  Box.mRot.m[0][1] = /*(float)*/mRotBox[4];
		//	  Box.mRot.m[1][1] = /*(float)*/mRotBox[5];
		//	  Box.mRot.m[2][1] = /*(float)*/mRotBox[6];
		//
		//	  Box.mRot.m[0][2] = /*(float)*/mRotBox[8];
		//	  Box.mRot.m[1][2] = /*(float)*/mRotBox[9];
		//	  Box.mRot.m[2][2] = /*(float)*/mRotBox[10];
		//
		//	  Matrix4x4 amatrix;
		//	  Matrix4x4 BoxMatrix = MakeMatrix(vPosBox, mRotBox, amatrix);
		//
		//	  Matrix4x4 InvBoxMatrix;
		//	  InvertPRMatrix(InvBoxMatrix, BoxMatrix);
		//
		//	  // get destination hull position and orientation
		//	  const dMatrix3& mRotMesh=*(const dMatrix3*)dGeomGetRotation(TriMesh);
		//	  const dVector3& vPosMesh=*(const dVector3*)dGeomGetPosition(TriMesh);
		//
		//	  // TC results
		//	  if (TriMesh->doBoxTC) {
		//		dxTriMesh::BoxTC* BoxTC = 0;
		//		for (int i = 0; i < TriMesh->BoxTCCache.size(); i++){
		//			if (TriMesh->BoxTCCache[i].Geom == BoxGeom){
		//				BoxTC = &TriMesh->BoxTCCache[i];
		//				break;
		//			}
		//		}
		//		if (!BoxTC){
		//			TriMesh->BoxTCCache.push(dxTriMesh::BoxTC());
		//
		//			BoxTC = &TriMesh->BoxTCCache[TriMesh->BoxTCCache.size() - 1];
		//			BoxTC->Geom = BoxGeom;
		//		    BoxTC->FatCoeff = 1.1f; // Pierre recommends this, instead of 1.0
		//		}
		//
		//		// Intersect
		//		Collider.SetTemporalCoherence(true);
		//		Collider.Collide(*BoxTC, Box, TriMesh->Data->BVTree, null, &MakeMatrix(vPosMesh, mRotMesh, amatrix));
		//	  }
		//	  else {
		//			Collider.SetTemporalCoherence(false);
		//			Collider.Collide(BoxCache, Box, TriMesh->Data->BVTree, null,
		//							 &MakeMatrix(vPosMesh, mRotMesh, amatrix));
		//		}
		//	}
		//
		//	int dCollideBTL(dxGeom* g1, dxGeom* BoxGeom, int Flags, dContactGeom* Contacts, int Stride){
		//	  dIASSERT (Stride >= (int)sizeof(dContactGeom));
		//	  dIASSERT (g1->type == dTriMeshClass);
		//	  dIASSERT (BoxGeom->type == dBoxClass);
		//	  dIASSERT ((Flags & NUMC_MASK) >= 1);
		//
		//	  dxTriMesh* TriMesh = (dxTriMesh*)g1;
		//
		//	  sTrimeshBoxColliderData cData;
		//	  cData.SetupInitialContext(TriMesh, BoxGeom, Flags, Contacts, Stride);
		//
		//	  const unsigned uiTLSKind = TriMesh->getParentSpaceTLSKind();
		//	  dIASSERT(uiTLSKind == BoxGeom->getParentSpaceTLSKind()); // The colliding spaces must use matching cleanup method
		//	  TrimeshCollidersCache *pccColliderCache = GetTrimeshCollidersCache(uiTLSKind);
		//	  OBBCollider& Collider = pccColliderCache->_OBBCollider;
		//
		//	  dQueryBTLPotentialCollisionTriangles(Collider, cData, TriMesh, BoxGeom,
		//	    pccColliderCache->defaultBoxCache);
		//
		//	  if (!Collider.GetContactStatus()) {
		//	  	// no collision occurred
		//	  	return 0;
		//	  }
		//
		//	  // Retrieve data
		//	  int TriCount = Collider.GetNbTouchedPrimitives();
		//	  const int* Triangles = (const int*)Collider.GetTouchedPrimitives();
		//
		//	  if (TriCount != 0){
		//	      if (TriMesh->ArrayCallback != null){
		//	         TriMesh->ArrayCallback(TriMesh, BoxGeom, Triangles, TriCount);
		//	    }
		//
		//	    // get destination hull position and orientation
		//	    const dMatrix3& mRotMesh=*(const dMatrix3*)dGeomGetRotation(TriMesh);
		//	    const dVector3& vPosMesh=*(const dVector3*)dGeomGetPosition(TriMesh);
		//
		//	    int ctContacts0 = 0;
		//
		//	    // loop through all intersecting triangles
		//	    for (int i = 0; i < TriCount; i++){
		//	        const int Triint = Triangles[i];
		//	        if (!Callback(TriMesh, BoxGeom, Triint)) continue;
		//
		//	        dVector3 dv[3];
		//			FetchTriangle(TriMesh, Triint, vPosMesh, mRotMesh, dv);
		//
		//			bool bFinishSearching;
		//			ctContacts0 = cData.TestCollisionForSingleTriangle(ctContacts0, Triint, dv, bFinishSearching);
		//
		//			if (bFinishSearching) {
		//				break;
		//			}
		//		}
		//	  }
		//
		//	  return cData.m_ctContacts;
		//	}
		//	#endif

	}  //TZ end of data?

	// GIMPACT version of box to mesh collider
	//	#if dTRIMESH_GIMPACT
	//	int dCollideBTL(dxGeom* g1, dxGeom* BoxGeom, int Flags, dContactGeom* Contacts, int Stride)
	int dCollideBTL(DxTriMesh g1, DxBox BoxGeom, int Flags, DContactGeomBuffer Contacts, int Stride)
	{
		dIASSERT (Stride >= 1);//(int)sizeof(dContactGeom));
		//	  dIASSERT (g1.type == dTriMeshClass);
		//	  dIASSERT (BoxGeom.type == dBoxClass);
		dIASSERT ((Flags & DxGeom.NUMC_MASK) >= 1);


		DxGimpact TriMesh = (DxGimpact) g1;

		g1.recomputeAABB();
		BoxGeom.recomputeAABB();


		sTrimeshBoxColliderData cData = new sTrimeshBoxColliderData();
		cData.SetupInitialContext(TriMesh, BoxGeom, Flags, Contacts, Stride);

		//*****at first , collide box aabb******//

		//GIM_TRIMESH * ptrimesh = &TriMesh.m_collision_trimesh;
		GimTrimesh ptrimesh = TriMesh.m_collision_trimesh;
		aabb3f test_aabb = new aabb3f();

		DAABBC aabb = BoxGeom.getAABB();
		test_aabb.minX = (float) aabb.getMin0();
		test_aabb.maxX = (float) aabb.getMax0();
		test_aabb.minY = (float) aabb.getMin1();
		test_aabb.maxY = (float) aabb.getMax1();
		test_aabb.minZ = (float) aabb.getMin2();
		test_aabb.maxZ = (float) aabb.getMax2();

		GimDynArrayInt collision_result = GimDynArrayInt.GIM_CREATE_BOXQUERY_LIST();

		ptrimesh.getAabbSet().gim_aabbset_box_collision(test_aabb, collision_result);

		if(collision_result.size()==0)
		{
			collision_result.GIM_DYNARRAY_DESTROY();
			return 0;
		}
		//*****Set globals for box collision******//

		//collide triangles

		//GUINT32 * boxesresult = GIM_DYNARRAY_POINTER(GUINT32,collision_result);
		int[] boxesresult = collision_result.GIM_DYNARRAY_POINTER();
		ptrimesh.gim_trimesh_locks_work_data();

		int ctContacts0 = 0;

		DVector3[] dvTZ = { new DVector3(), new DVector3(), new DVector3() };
		vec3f[] dv = new vec3f[] { new vec3f(), new vec3f(), new vec3f() };//[3];
		for(int i=0;i<collision_result.size();i++)
		{
			//			DVector3[] dvTZ = new DVector3[3];
			//			vec3f[] dv = new vec3f[3];

			int Triint = boxesresult[i];
			ptrimesh.gim_trimesh_get_triangle_vertices(Triint, dv[0], dv[1], dv[2]);

			RefBoolean bFinishSearching = new RefBoolean(false);
			dvTZ[0].set(dv[0].f);
			dvTZ[1].set(dv[1].f);
			dvTZ[2].set(dv[2].f);
			ctContacts0 = cData.TestCollisionForSingleTriangle(ctContacts0, Triint, dvTZ, bFinishSearching);
		}
		int contactcount = cData.m_TempContactGeoms.size();
		int contactmax = (Flags & DxGeom.NUMC_MASK);
		if (contactcount > contactmax)
		{
			if (OdeConfig.ENABLE_CONTACT_SORTING) {
				Collections.sort(cData.m_TempContactGeoms, new Comparator<DContactGeom>() {
					@Override
					public int compare(DContactGeom o1, DContactGeom o2) {
						return Double.compare(o2.depth, o1.depth);
					}
				});
			}
			contactcount = contactmax;
		}

		ptrimesh.gim_trimesh_unlocks_work_data();
		collision_result.GIM_DYNARRAY_DESTROY();
		for (int i = 0; i < contactcount; i++) {
			cData.m_ContactGeoms.get(i).depth = cData.m_TempContactGeoms.get(i).depth;
			cData.m_ContactGeoms.get(i).g1 = cData.m_TempContactGeoms.get(i).g1;
			cData.m_ContactGeoms.get(i).g2 = cData.m_TempContactGeoms.get(i).g2;
			cData.m_ContactGeoms.get(i).normal.set(cData.m_TempContactGeoms.get(i).normal);
			cData.m_ContactGeoms.get(i).pos.set(cData.m_TempContactGeoms.get(i).pos);
			cData.m_ContactGeoms.get(i).side1 = cData.m_TempContactGeoms.get(i).side1;
			cData.m_ContactGeoms.get(i).side2 = cData.m_TempContactGeoms.get(i).side2;
		}
		return contactcount;
	}
	//	#endif  //GIMPACT


	static void
	GenerateContact(int in_Flags, List<DContactGeom> in_Contacts, int in_Stride,
			DxGeom in_g1,  DxGeom in_g2, final int TriIndex,
			DVector3C in_ContactPos, DVector3C in_Normal, final double in_Depth)
	{
		if (in_Stride != 1) throw new IllegalArgumentException("in_Stride = " + in_Stride);
		boolean duplicate = false;
		DVector3 diff = new DVector3();
		for (DContactGeom contact : in_Contacts)
		{
			diff.eqDiff(in_ContactPos, contact.pos);
			if (diff.dot(diff) < dEpsilon)
			{
				// same normal?
				if (1.0 - dFabs(in_Normal.dot(contact.normal)) < dEpsilon)
				{
					if (in_Depth > contact.depth)
						contact.depth = in_Depth;
							duplicate = true;
				}
			}
		}
		if (!duplicate) {
			DContactGeom Contact = new DContactGeom();
			Contact.pos.set( in_ContactPos );
			Contact.normal.set( in_Normal );
			Contact.depth = in_Depth;
			Contact.g1 = in_g1;
			Contact.g2 = in_g2;
			Contact.side1 = TriIndex;
			Contact.side2 = -1;
			in_Contacts.add(Contact);
		}
	}


	//	#endif // dTRIMESH_ENABLED


}
