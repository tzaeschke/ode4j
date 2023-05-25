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

import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.DxCollisionUtil.dClipEdgeToPlane;
import static org.ode4j.ode.internal.DxCollisionUtil.dClipPolyToPlane;
import static org.ode4j.ode.internal.DxCollisionUtil.dConstructPlane;
import static org.ode4j.ode.internal.DxCollisionUtil.dMat3GetCol;
import static org.ode4j.ode.internal.DxCollisionUtil.dPointPlaneDistance;
import static org.ode4j.ode.internal.DxCollisionUtil.dQuatInv;
import static org.ode4j.ode.internal.DxCollisionUtil.dQuatTransform;
import static org.ode4j.ode.internal.DxCollisionUtil.dVector3Copy;
import static org.ode4j.ode.internal.DxCollisionUtil.dVector3Cross;
import static org.ode4j.ode.internal.DxCollisionUtil.dVector3Inv;
import static org.ode4j.ode.internal.DxCollisionUtil.dVector3Subtract;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector4;
import org.ode4j.ode.*;
import org.ode4j.ode.internal.cpp4j.java.RefBoolean;
import org.ode4j.ode.internal.gimpact.GimDynArrayInt;
import org.ode4j.ode.internal.gimpact.GimGeometry.aabb3f;
import org.ode4j.ode.internal.gimpact.GimTrimesh;
import org.ode4j.ode.internal.trimesh.DxTriMesh;

/**
 * Cylinder-trimesh collider by Alen Ladavac
 * Ported to ODE by Nguyen Binh
 * Ported to Java by Tilmann Zaeschke
 */
class CollideCylinderTrimesh implements DColliderFn {

	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideCylinderTrimesh((DxCylinder)o1, (DxGimpact)o2, flags, contacts, 1);
	}



//
//	#include <ode/collision.h>
//	#include <ode/matrix.h>
//	#include <ode/rotation.h>
//	#include <ode/odemath.h>
//	#include "collision_util.h"
//	#include "collision_trimesh_internal.h"
//	#include "util.h"
//
//	#if dTRIMESH_ENABLED

	//#define MAX_REAL	dInfinity
	private static final double MAX_REAL	= Common.dInfinity;
	private static final int	nCYLINDER_AXIS				= 2;
	private static final int    nCYLINDER_CIRCLE_SEGMENTS	= 8;
	private static final int    nMAX_CYLINDER_TRIANGLE_CLIP_POINTS	= 12;

	//#define OPTIMIZE_CONTACTS 1

	// Local contacts data
	//typedef struct _sLocalContactData
	private static class sLocalContactData
	{
		final DVector3	vPos = new DVector3();
		final DVector3	vNormal = new DVector3();
		double		fDepth;
		int			triIndex;
		int			nFlags; // 0 = filtered out, 1 = OK
	}//sLocalContactData;

	private static class sCylinderTrimeshColliderData
	{
		//sCylinderTrimeshColliderData(int flags, int skip): 
		//m_iFlags(flags), m_iSkip(skip), m_nContacts(0), m_gLocalContacts(NULL) {}
		sCylinderTrimeshColliderData(int flags, int skip) {
			m_iFlags = flags;
//			m_iSkip = skip;
			m_nContacts = 0;
			m_gLocalContacts = null;
		}

////	#ifdef OPTIMIZE_CONTACTS
//		void _OptimizeLocalContacts();
////	#endif
//		void _InitCylinderTrimeshData(dxGeom *Cylinder, dxTriMesh *Trimesh);
//		int	_ProcessLocalContacts(dContactGeom *contact, dxGeom *Cylinder, dxTriMesh *Trimesh);
//
//		bool _cldTestAxis(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, 
//			dVector3& vAxis, int iAxis, bool bNoFlip = false);
//		bool _cldTestCircleToEdgeAxis(
//			const dVector3 &v0, const dVector3 &v1, const dVector3 &v2,
//			const dVector3 &vCenterPoint, const dVector3 &vCylinderAxis1,
//			const dVector3 &vVx0, const dVector3 &vVx1, int iAxis);
//		bool _cldTestSeparatingAxes(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2);
//		bool _cldClipCylinderEdgeToTriangle(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2);
//		void _cldClipCylinderToTriangle(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2);
//		void TestOneTriangleVsCylinder(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, 
//			const bool bDoubleSided);
//		int TestCollisionForSingleTriangle(int ctContacts0, int Triint, dVector3 dv[3], 
//			bool &bOutFinishSearching);

		// cylinder data
		private final DMatrix3 	m_mCylinderRot = new DMatrix3();
		private final DQuaternion 	m_qCylinderRot = new DQuaternion();
		private final DQuaternion 	m_qInvCylinderRot = new DQuaternion();
		private final DVector3 	m_vCylinderPos = new DVector3();
		private final DVector3 	m_vCylinderAxis = new DVector3();
		private double		m_fCylinderRadius;
		private double		m_fCylinderSize;
		private final DVector3[] 	m_avCylinderNormals = new DVector3[nCYLINDER_CIRCLE_SEGMENTS];

		// mesh data
		//TODO remove/report ?
		@SuppressWarnings("unused")
		private DQuaternionC 		m_qTrimeshRot;
		//TODO remove/report ?
		@SuppressWarnings("unused")
		private DQuaternion 		m_qInvTrimeshRot;
		private final DMatrix3 	m_mTrimeshRot = new DMatrix3();
		private final DVector3 	m_vTrimeshPos = new DVector3();

		// global collider data
		private final DVector3 	m_vBestPoint = new DVector3();
		private double		m_fBestDepth;
		//TODO remove/report ?
		@SuppressWarnings("unused")
		private double		m_fBestCenter;
		private double		m_fBestrt;
		private int				m_iBestAxis;
		private final DVector3 	m_vContactNormal = new DVector3();
		private final DVector3 	m_vNormal = new DVector3();
		private final DVector3 	m_vE0 = new DVector3();
		private final DVector3		m_vE1 = new DVector3();
		private final DVector3 	m_vE2 = new DVector3();

		// ODE stuff
		int					m_iFlags;
//TZ		int					m_iSkip;
		int					m_nContacts;// = 0;
		//sLocalContactData*	m_gLocalContacts;
		sLocalContactData[]	m_gLocalContacts;
	//};


//	#ifdef OPTIMIZE_CONTACTS

	// Use to classify contacts to be "near" in position
	private static final double fSameContactPositionEpsilon = (0.0001); // 1e-4
	// Use to classify contacts to be "near" in normal direction
	private static final double fSameContactNormalEpsilon = (0.0001); // 1e-4

	// If this two contact can be classified as "near"
	//inline int _IsNearContacts(sLocalContactData& c1,sLocalContactData& c2)
	private static boolean _IsNearContacts(sLocalContactData c1,sLocalContactData c2)
	{
		boolean bPosNear = false;
		boolean bSameDir = false;
		DVector3	vDiff = new DVector3();

		// First check if they are "near" in position
		vDiff.eqDiff(c1.vPos,c2.vPos);//dVector3Subtract(c1.vPos,c2.vPos,vDiff);
		if (  (Math.abs(vDiff.get0()) < fSameContactPositionEpsilon)
			&&(Math.abs(vDiff.get1()) < fSameContactPositionEpsilon)
			&&(Math.abs(vDiff.get2()) < fSameContactPositionEpsilon))
		{
			bPosNear = true;
		}

		// Second check if they are "near" in normal direction
		vDiff.eqDiff(c1.vNormal,c2.vNormal);//dVector3Subtract(c1.vNormal,c2.vNormal,vDiff);
		if (  (Math.abs(vDiff.get0()) < fSameContactNormalEpsilon)
			&&(Math.abs(vDiff.get1()) < fSameContactNormalEpsilon)
			&&(Math.abs(vDiff.get2()) < fSameContactNormalEpsilon) )
		{
			bSameDir = true;
		}

		// Will be "near" if position and normal direction are "near"
		return (bPosNear && bSameDir);
	}

	private static boolean _IsBetter(sLocalContactData c1,sLocalContactData c2)
	{
		// The not better will be throw away
		// You can change the selection criteria here
		return (c1.fDepth > c2.fDepth);
	}

	// iterate through gLocalContacts and filtered out "near contact"
	void _OptimizeLocalContacts()
	{
		int nContacts = m_nContacts;

		for (int i = 0; i < nContacts-1; i++)
		{
			for (int j = i+1; j < nContacts; j++)
			{
				if (_IsNearContacts(m_gLocalContacts[i],m_gLocalContacts[j]))
				{
					// If they are seem to be the same then filtered 
					// out the least penetrate one
					if (_IsBetter(m_gLocalContacts[j],m_gLocalContacts[i]))
					{
						m_gLocalContacts[i].nFlags = 0; // filtered 1st contact
					}
					else
					{
						m_gLocalContacts[j].nFlags = 0; // filtered 2nd contact
					}

					// NOTE
					// There is other way is to add two depth together but
					// it not work so well. Why???
				}
			}
		}
	}
//	#endif // OPTIMIZE_CONTACTS

//	int	sCylinderTrimeshColliderData::_ProcessLocalContacts(dContactGeom *contact, 
//		dxGeom *Cylinder, dxTriMesh *Trimesh)
	int	_ProcessLocalContacts(DContactGeomBuffer contacts, 
			DxCylinder Cylinder, DxTriMesh Trimesh)
	{
//	#ifdef OPTIMIZE_CONTACTS
		if (m_nContacts > 1 && !((m_iFlags & OdeConstants.CONTACTS_UNIMPORTANT) != 0))
		{
			// Can be optimized...
			_OptimizeLocalContacts();
		}
//	#endif		

		int iContact = 0;
		DContactGeom Contact = null;

		int nFinalContact = 0;

		for (iContact = 0; iContact < m_nContacts; iContact ++)
		{
			if (1 == m_gLocalContacts[iContact].nFlags)
			{
				// TODO 76
				// ode4j fix: see issue #76
				// side1: see TestCollisionForSingleTriangle()
				if (!Trimesh.invokeCallback(Cylinder, m_gLocalContacts[iContact].triIndex)) {
					continue;
				}
				//getSAFECONTACT(m_iFlags, contact, nFinalContact, m_iSkip);
				Contact = contacts.getSafe(m_iFlags, nFinalContact);
				Contact.depth = m_gLocalContacts[iContact].fDepth;
				//dVector3Copy(m_gLocalContacts[iContact].vNormal,Contact.normal);
				Contact.normal.set(m_gLocalContacts[iContact].vNormal);
				//dVector3Copy(m_gLocalContacts[iContact].vPos,Contact.pos);
				Contact.pos.set(m_gLocalContacts[iContact].vPos);
				Contact.g1 = Cylinder;
				Contact.g2 = Trimesh;
				Contact.side1 = -1;
				Contact.side2 = m_gLocalContacts[iContact].triIndex;
				//dVector3Inv(Contact.normal);
				Contact.normal.scale(-1);

				nFinalContact++;
			}
		}
		// debug
		//if (nFinalContact != m_nContacts)
		//{
		//	printf("[Info] %d contacts generated,%d  filtered.\n",m_nContacts,m_nContacts-nFinalContact);
		//}

		return nFinalContact;
	}


//	bool sCylinderTrimeshColliderData::_cldTestAxis(
//					  const dVector3 &v0,
//					  const dVector3 &v1,
//					  const dVector3 &v2, 
//	                  dVector3& vAxis, 
//					  int iAxis,
//					  bool bNoFlip/* = false*/)
	boolean _cldTestAxis(
			  final DVector3C v0,
			  final DVector3C v1,
			  final DVector3C v2, 
			  final DVector3 vAxis, 
			  final int iAxis,
			  final boolean bNoFlip/* = false*/)
	{
	  
		// calculate length of separating axis vector
		double fL = vAxis.length();//dVector3Length(vAxis);
		// if not long enough
		if ( fL < (1e-5) )
		{
			// do nothing
			return true;
		}

		// otherwise normalize it
//		vAxis[0] /= fL;
//		vAxis[1] /= fL;
//		vAxis[2] /= fL;
		vAxis.scale( 1./fL );

		double fdot1 = m_vCylinderAxis.dot(vAxis);//dVector3Dot(m_vCylinderAxis,vAxis);
		// project capsule on vAxis
		double frc;

		if (Math.abs(fdot1) > (1.0) ) 
		{
//			fdot1 = REAL(1.0);
			frc = Math.abs(m_fCylinderSize* (0.5));
		}
		else
		{
			frc = Math.abs((m_fCylinderSize* (0.5)) * fdot1)
				+ m_fCylinderRadius * Math.sqrt((1.0)-(fdot1*fdot1));
		}
	  
		DVector3 vV0 = new DVector3();
		vV0.eqDiff( v0, m_vCylinderPos ); //dVector3Subtract(v0,m_vCylinderPos,vV0);
		DVector3 vV1 = new DVector3();
		vV1.eqDiff( v1, m_vCylinderPos ); //dVector3Subtract(v1,m_vCylinderPos,vV1);
		DVector3 vV2 = new DVector3();
		vV2.eqDiff( v2, m_vCylinderPos ); //dVector3Subtract(v2,m_vCylinderPos,vV2);

		// project triangle on vAxis
		double[] afv=new double[3];
		afv[0] = vV0.dot( vAxis );//dVector3Dot( vV0 , vAxis );
		afv[1] = vV1.dot( vAxis );//dVector3Dot( vV1 , vAxis );
		afv[2] = vV2.dot( vAxis );//dVector3Dot( vV2 , vAxis );

		double fMin = MAX_REAL;
		double fMax = -MAX_REAL;

		// for each vertex 
		for(int i = 0; i < 3; i++) 
		{
			// find minimum
			if (afv[i]<fMin) 
			{
				fMin = afv[i];
			}
			// find maximum
			if (afv[i]>fMax) 
			{
				fMax = afv[i];
			}
		}

		// find capsule's center of interval on axis
		double fCenter = (fMin+fMax)* (0.5);
		// calculate triangles halfinterval 
		double fTriangleRadius = (fMax-fMin)*(0.5);

		// if they do not overlap, 
		if( Math.abs(fCenter) > (frc+fTriangleRadius) ) 
		{ 
			// exit, we have no intersection
			return false; 
		}

		// calculate depth 
		double fDepth = -(Math.abs(fCenter) - (frc + fTriangleRadius ) );

		// if greater then best found so far
		if ( fDepth < m_fBestDepth ) 
		{
			// remember depth
			m_fBestDepth			= fDepth;
			m_fBestCenter		    = fCenter;
			m_fBestrt				= frc;
			dVector3Copy(vAxis,m_vContactNormal);
			m_iBestAxis				= iAxis;
		  
			// flip normal if interval is wrong faced
			if ( fCenter< (0.0) && !bNoFlip) 
			{ 
				dVector3Inv(m_vContactNormal);
				m_fBestCenter = -fCenter;
			}
		}
	  
		return true;
	}

	// intersection test between edge and circle
//	bool sCylinderTrimeshColliderData::_cldTestCircleToEdgeAxis(
//		const dVector3 &v0, const dVector3 &v1, const dVector3 &v2,
//		const dVector3 &vCenterPoint, const dVector3 &vCylinderAxis1,
//		const dVector3 &vVx0, const dVector3 &vVx1, int iAxis) 
	boolean _cldTestCircleToEdgeAxis(
			final DVector3C v0, final DVector3C v1, final DVector3C v2,
			final DVector3C vCenterPoint, final DVector3C vCylinderAxis1,
			final DVector3C vVx0, final DVector3C vVx1, final int iAxis) 
	{
		// calculate direction of edge
		DVector3 vkl = new DVector3();
		dVector3Subtract( vVx1 , vVx0 , vkl);
		vkl.normalize();//dNormalize3(vkl);
		// starting point of edge 
		DVector3 vol = new DVector3();
		dVector3Copy(vVx0,vol);

		// calculate angle cosine between cylinder axis and edge
		double fdot2 = vkl.dot(vCylinderAxis1);

		// if edge is perpendicular to cylinder axis
		if(Math.abs(fdot2)<(1e-5))
		{
			// this can't be separating axis, because edge is parallel to circle plane
			return true;
		}
	    
		// find point of intersection between edge line and circle plane
		DVector3 vTemp = new DVector3();
		dVector3Subtract(vCenterPoint,vol,vTemp);
		double fdot1 = vTemp.dot(vCylinderAxis1);
		DVector3 vpnt = new DVector3();// = vol + vkl * (fdot1/fdot2);
//		vpnt[0] = vol[0] + vkl[0] * fdot1/fdot2;
//		vpnt[1] = vol[1] + vkl[1] * fdot1/fdot2;
//		vpnt[2] = vol[2] + vkl[2] * fdot1/fdot2;
		vpnt.eqSum( vol, vkl, fdot1/fdot2);

		// find tangent vector on circle with same center (vCenterPoint) that touches point of intersection (vpnt)
		DVector3 vTangent = new DVector3();
		dVector3Subtract(vCenterPoint,vpnt,vTemp);
		dVector3Cross(vTemp,vCylinderAxis1,vTangent);
	  
		// find vector orthogonal both to tangent and edge direction
		DVector3 vAxis = new DVector3();
		dVector3Cross(vTangent,vkl,vAxis);

		// use that vector as separating axis
		return _cldTestAxis( v0, v1, v2, vAxis, iAxis, false );
	}

	// helper for less key strokes
	// r = ( (v1 - v2) cross v3 ) cross v3
//	inline void _CalculateAxis(const dVector3& v1,
//							   const dVector3& v2,
//							   const dVector3& v3,
//							   dVector3& r)
	private static void _CalculateAxis(final DVector3C v1,
			   final DVector3C v2,
			   final DVector3C v3,
			   final DVector3 r)
	{
		DVector3 t1 = new DVector3();
		DVector3 t2 = new DVector3();

		t1.eqDiff(v1, v2);//dVector3Subtract(v1,v2,t1);
		OdeMath.dCalcVectorCross3(t2,t1,v3);//dVector3Cross(t1,v3,t2);
		OdeMath.dCalcVectorCross3(r,t2,v3);//dVector3Cross(t2,v3,r);
	}

//	boolean sCylinderTrimeshColliderData::_cldTestSeparatingAxes(
//								const dVector3 &v0,
//								const dVector3 &v1,
//								const dVector3 &v2) 
	boolean _cldTestSeparatingAxes(
			final DVector3C v0,
			final DVector3C v1,
			final DVector3C v2) 
	{

		// calculate edge vectors
		dVector3Subtract(v1 ,v0 , m_vE0);
		// m_vE1 has been calculated before -> so save some cycles here
		dVector3Subtract(v0 ,v2 , m_vE2);

		// calculate caps centers in absolute space
		DVector3 vCp0 = new DVector3();
//		vCp0[0] = m_vCylinderPos[0] + m_vCylinderAxis[0]*(m_fCylinderSize* REAL(0.5));
//		vCp0[1] = m_vCylinderPos[1] + m_vCylinderAxis[1]*(m_fCylinderSize* REAL(0.5));
//		vCp0[2] = m_vCylinderPos[2] + m_vCylinderAxis[2]*(m_fCylinderSize* REAL(0.5));
		vCp0.eqSum( m_vCylinderPos, m_vCylinderAxis, m_fCylinderSize* 0.5);

//	#if 0
//		DVector3 vCp1 = new DVector3();
////		vCp1[0] = m_vCylinderPos[0] - m_vCylinderAxis[0]*(m_fCylinderSize* REAL(0.5));
////		vCp1[1] = m_vCylinderPos[1] - m_vCylinderAxis[1]*(m_fCylinderSize* REAL(0.5));
////		vCp1[2] = m_vCylinderPos[2] - m_vCylinderAxis[2]*(m_fCylinderSize* REAL(0.5));
//		vCp1.eqSum( m_vCylinderPos, m_vCylinderAxis, -m_fCylinderSize* 0.5);
//	#endif
		
		// reset best axis
		m_iBestAxis = 0;
		DVector3 vAxis = new DVector3();

		// axis m_vNormal
		//vAxis = -m_vNormal;
//		vAxis[0] = -m_vNormal[0];
//		vAxis[1] = -m_vNormal[1];
//		vAxis[2] = -m_vNormal[2];
		vAxis.set( m_vNormal ).scale(-1);
		if (!_cldTestAxis(v0, v1, v2, vAxis, 1, true)) 
		{ 
			return false; 
		}

		// axis CxE0
		// vAxis = ( m_vCylinderAxis cross m_vE0 );
		dVector3Cross(m_vCylinderAxis, m_vE0,vAxis);
		if (!_cldTestAxis(v0, v1, v2, vAxis, 2, false)) 
		{ 
			return false; 
		}

		// axis CxE1
		// vAxis = ( m_vCylinderAxis cross m_vE1 );
		dVector3Cross(m_vCylinderAxis, m_vE1,vAxis);
		if (!_cldTestAxis(v0, v1, v2, vAxis, 3, false)) 
		{ 
			return false; 
		}

		// axis CxE2
		// vAxis = ( m_vCylinderAxis cross m_vE2 );
		dVector3Cross(m_vCylinderAxis, m_vE2,vAxis);
		if (!_cldTestAxis(v0, v1, v2, vAxis, 4, false)) 
		{ 
			return false; 
		}

		// first vertex on triangle
		// axis ((V0-Cp0) x C) x C
		//vAxis = ( ( v0-vCp0 ) cross m_vCylinderAxis ) cross m_vCylinderAxis;
		_CalculateAxis(v0 , vCp0 , m_vCylinderAxis , vAxis);
		if (!_cldTestAxis(v0, v1, v2, vAxis, 11, false)) 
		{ 
			return false; 
		}

		// second vertex on triangle
		// axis ((V1-Cp0) x C) x C
		// vAxis = ( ( v1-vCp0 ) cross m_vCylinderAxis ) cross m_vCylinderAxis;
		_CalculateAxis(v1 , vCp0 , m_vCylinderAxis , vAxis);
		if (!_cldTestAxis(v0, v1, v2, vAxis, 12, false)) 
		{ 
			return false; 
		}

		// third vertex on triangle
		// axis ((V2-Cp0) x C) x C
		//vAxis = ( ( v2-vCp0 ) cross m_vCylinderAxis ) cross m_vCylinderAxis;
		_CalculateAxis(v2 , vCp0 , m_vCylinderAxis , vAxis);
		if (!_cldTestAxis(v0, v1, v2, vAxis, 13, false))
		{ 
			return false; 
		}

		// test cylinder axis
		// vAxis = m_vCylinderAxis;
		dVector3Copy(m_vCylinderAxis , vAxis);
		if (!_cldTestAxis(v0, v1, v2, vAxis, 14, false)) 
		{ 
			return false; 
		}

		// Test top and bottom circle ring of cylinder for separation
		DVector3 vccATop = new DVector3();
//		vccATop[0] = m_vCylinderPos[0] + m_vCylinderAxis[0]*(m_fCylinderSize * (0.5));
//		vccATop[1] = m_vCylinderPos[1] + m_vCylinderAxis[1]*(m_fCylinderSize * (0.5));
//		vccATop[2] = m_vCylinderPos[2] + m_vCylinderAxis[2]*(m_fCylinderSize * (0.5));
		vccATop.eqSum( m_vCylinderPos, m_vCylinderAxis, m_fCylinderSize * 0.5);

		DVector3 vccABottom = new DVector3();
//		vccABottom[0] = m_vCylinderPos[0] - m_vCylinderAxis[0]*(m_fCylinderSize * (0.5));
//		vccABottom[1] = m_vCylinderPos[1] - m_vCylinderAxis[1]*(m_fCylinderSize * (0.5));
//		vccABottom[2] = m_vCylinderPos[2] - m_vCylinderAxis[2]*(m_fCylinderSize * (0.5));
		vccABottom.eqSum( m_vCylinderPos, m_vCylinderAxis, -m_fCylinderSize * (0.5));


	  if (!_cldTestCircleToEdgeAxis(v0, v1, v2, vccATop, m_vCylinderAxis, v0, v1, 15)) 
	  {
	    return false;
	  }

	  if (!_cldTestCircleToEdgeAxis(v0, v1, v2, vccATop, m_vCylinderAxis, v1, v2, 16)) 
	  {
	    return false;
	  }

	  if (!_cldTestCircleToEdgeAxis(v0, v1, v2, vccATop, m_vCylinderAxis, v0, v2, 17)) 
	  {
	    return false;
	  }

	  if (!_cldTestCircleToEdgeAxis(v0, v1, v2, vccABottom, m_vCylinderAxis, v0, v1, 18)) 
	  {
	    return false;
	  }

	  if (!_cldTestCircleToEdgeAxis(v0, v1, v2, vccABottom, m_vCylinderAxis, v1, v2, 19)) 
	  {
	    return false;
	  }

	  if (!_cldTestCircleToEdgeAxis(v0, v1, v2, vccABottom, m_vCylinderAxis, v0, v2, 20)) 
	  {
	    return false;
	  }

	  return true;
	}

//	bool sCylinderTrimeshColliderData::_cldClipCylinderEdgeToTriangle(
//		const dVector3 &v0, const dVector3 &v1, const dVector3 &v2)
	boolean _cldClipCylinderEdgeToTriangle(
			final DVector3C v0, final DVector3C v1, final DVector3C v2)
	{
		// translate cylinder
		double fTemp = m_vCylinderAxis.dot(m_vContactNormal);//dVector3Dot(m_vCylinderAxis , m_vContactNormal);
		DVector3 vN2 = new DVector3();
//		vN2[0] = m_vContactNormal[0] - m_vCylinderAxis[0]*fTemp;
//		vN2[1] = m_vContactNormal[1] - m_vCylinderAxis[1]*fTemp;
//		vN2[2] = m_vContactNormal[2] - m_vCylinderAxis[2]*fTemp;
		vN2.eqSum( m_vContactNormal, m_vCylinderAxis, -fTemp );


		fTemp = vN2.length();//dVector3Length(vN2);
		if (fTemp < (1e-5))
		{
			return false;
		}

		// Normalize it
//		vN2[0] /= fTemp;
//		vN2[1] /= fTemp;
//		vN2[2] /= fTemp;
		vN2.scale( 1./fTemp );

		// calculate caps centers in absolute space
		DVector3 vCposTrans = new DVector3();
//		vCposTrans[0] = m_vCylinderPos[0] + vN2[0]*m_fCylinderRadius;
//		vCposTrans[1] = m_vCylinderPos[1] + vN2[1]*m_fCylinderRadius;
//		vCposTrans[2] = m_vCylinderPos[2] + vN2[2]*m_fCylinderRadius;
		vCposTrans.eqSum( m_vCylinderPos, vN2,m_fCylinderRadius);
		  
		DVector3 vCEdgePoint0 = new DVector3();
//		vCEdgePoint0[0]  = vCposTrans[0] + m_vCylinderAxis[0] * (m_fCylinderSize* (0.5));
//		vCEdgePoint0[1]  = vCposTrans[1] + m_vCylinderAxis[1] * (m_fCylinderSize* (0.5));
//		vCEdgePoint0[2]  = vCposTrans[2] + m_vCylinderAxis[2] * (m_fCylinderSize* (0.5));
		vCEdgePoint0.eqSum( vCposTrans, m_vCylinderAxis, m_fCylinderSize* (0.5));

		DVector3 vCEdgePoint1 = new DVector3();
//		vCEdgePoint1[0]  = vCposTrans[0] - m_vCylinderAxis[0] * (m_fCylinderSize* REAL(0.5));
//		vCEdgePoint1[1]  = vCposTrans[1] - m_vCylinderAxis[1] * (m_fCylinderSize* REAL(0.5));
//		vCEdgePoint1[2]  = vCposTrans[2] - m_vCylinderAxis[2] * (m_fCylinderSize* REAL(0.5));
		vCEdgePoint1.eqSum( vCposTrans, m_vCylinderAxis, -m_fCylinderSize* 0.5);

		// transform cylinder edge points into triangle space
//		vCEdgePoint0[0] -= v0[0];
//		vCEdgePoint0[1] -= v0[1];
//		vCEdgePoint0[2] -= v0[2];
		vCEdgePoint0.sub( v0 );

//		vCEdgePoint1[0] -= v0[0];
//		vCEdgePoint1[1] -= v0[1];
//		vCEdgePoint1[2] -= v0[2];
		vCEdgePoint1.sub( v0 );

		DVector4 plPlane = new DVector4();
		DVector3 vPlaneNormal = new DVector3();

		// triangle plane
		//plPlane = Plane4f( -m_vNormal, 0);
//		vPlaneNormal[0] = -m_vNormal[0];
//		vPlaneNormal[1] = -m_vNormal[1];
//		vPlaneNormal[2] = -m_vNormal[2];
		vPlaneNormal.set( m_vNormal ).scale(-1);
		dConstructPlane(vPlaneNormal,(0.0),plPlane);
		if(!dClipEdgeToPlane( vCEdgePoint0, vCEdgePoint1, plPlane )) 
		{ 
			return false; 
		}

		// plane with edge 0
		//plPlane = Plane4f( ( m_vNormal cross m_vE0 ), REAL(1e-5));
		dVector3Cross(m_vNormal,m_vE0,vPlaneNormal);
		dConstructPlane(vPlaneNormal,(1e-5),plPlane);
		if(!dClipEdgeToPlane( vCEdgePoint0, vCEdgePoint1, plPlane )) 
		{ 
			return false; 
		}
	  
		// plane with edge 1
		//dVector3 vTemp = ( m_vNormal cross m_vE1 );
		dVector3Cross(m_vNormal,m_vE1,vPlaneNormal);
		fTemp = m_vE0.dot(vPlaneNormal) - (1e-5);
		//plPlane = Plane4f( vTemp, -(( m_vE0 dot vTemp )-REAL(1e-5)));
		dConstructPlane(vPlaneNormal,-fTemp,plPlane);
		if(!dClipEdgeToPlane( vCEdgePoint0, vCEdgePoint1, plPlane )) 
		{
			return false;
		}

		// plane with edge 2
		// plPlane = Plane4f( ( m_vNormal cross m_vE2 ), REAL(1e-5));
		dVector3Cross(m_vNormal,m_vE2,vPlaneNormal);
		dConstructPlane(vPlaneNormal,(1e-5),plPlane);
		if(!dClipEdgeToPlane( vCEdgePoint0, vCEdgePoint1, plPlane )) 
		{ 
			return false; 
		}

		// return capsule edge points into absolute space
//		vCEdgePoint0[0] += v0[0];
//		vCEdgePoint0[1] += v0[1];
//		vCEdgePoint0[2] += v0[2];
		vCEdgePoint0.add( v0 );

//		vCEdgePoint1[0] += v0[0];
//		vCEdgePoint1[1] += v0[1];
//		vCEdgePoint1[2] += v0[2];
		vCEdgePoint1.add( v0 );

		// calculate depths for both contact points
		DVector3 vTemp = new DVector3();
		dVector3Subtract(vCEdgePoint0,m_vCylinderPos, vTemp);
		double fRestDepth0 = -vTemp.dot(m_vContactNormal) + m_fBestrt;
		dVector3Subtract(vCEdgePoint1,m_vCylinderPos, vTemp);
		double fRestDepth1 = -vTemp.dot(m_vContactNormal) + m_fBestrt;
		
		double fDepth0 = m_fBestDepth - (fRestDepth0);
		double fDepth1 = m_fBestDepth - (fRestDepth1);
			  
		// clamp depths to zero
		if(fDepth0 < (0.0) ) 
		{
			fDepth0 = (0.0);
		}

		if(fDepth1<(0.0)) 
		{
			fDepth1 = (0.0);
		}

		// Generate contact 0
		{
			m_gLocalContacts[m_nContacts].fDepth = fDepth0;
			dVector3Copy(m_vContactNormal,m_gLocalContacts[m_nContacts].vNormal);
			dVector3Copy(vCEdgePoint0,m_gLocalContacts[m_nContacts].vPos);
			m_gLocalContacts[m_nContacts].nFlags = 1;
			m_nContacts++;
			if(m_nContacts >= (m_iFlags & DxGeom.NUMC_MASK)) 
				return true;
		}

		// Generate contact 1
		{
			// generate contacts
			m_gLocalContacts[m_nContacts].fDepth = fDepth1;
			dVector3Copy(m_vContactNormal,m_gLocalContacts[m_nContacts].vNormal);
			dVector3Copy(vCEdgePoint1,m_gLocalContacts[m_nContacts].vPos);
			m_gLocalContacts[m_nContacts].nFlags = 1;
			m_nContacts++;		
		}

		return true;
	}

//	void sCylinderTrimeshColliderData::_cldClipCylinderToTriangle(
//		const dVector3 &v0, const dVector3 &v1, const dVector3 &v2)
	void _cldClipCylinderToTriangle(
			final DVector3C v0, final DVector3C v1, final DVector3C v2)
	{
		int i = 0;
		DVector3[] avPoints = { new DVector3(), new DVector3(), new DVector3() };//[3];
		DVector3[] avTempArray1 = DVector3.newArray(nMAX_CYLINDER_TRIANGLE_CLIP_POINTS);
		DVector3[] avTempArray2 = DVector3.newArray(nMAX_CYLINDER_TRIANGLE_CLIP_POINTS);

//		dSetZero(avTempArray1[0][0],nMAX_CYLINDER_TRIANGLE_CLIP_POINTS * 4);
//		dSetZero(avTempArray2[0][0],nMAX_CYLINDER_TRIANGLE_CLIP_POINTS * 4);

		// setup array of triangle vertices
		dVector3Copy(v0,avPoints[0]);
		dVector3Copy(v1,avPoints[1]);
		dVector3Copy(v2,avPoints[2]);

		DVector3 vCylinderCirclePos = new DVector3(), vCylinderCircleNormal_Rel = new DVector3();
		vCylinderCircleNormal_Rel.setZero();
		// check which circle from cylinder we take for clipping
		if ( m_vCylinderAxis.dot(m_vContactNormal) > (0.0)) 
		{
			// get top circle
//			vCylinderCirclePos[0] = m_vCylinderPos[0] + m_vCylinderAxis[0]*(m_fCylinderSize*(0.5));
//			vCylinderCirclePos[1] = m_vCylinderPos[1] + m_vCylinderAxis[1]*(m_fCylinderSize*(0.5));
//			vCylinderCirclePos[2] = m_vCylinderPos[2] + m_vCylinderAxis[2]*(m_fCylinderSize*(0.5));
			vCylinderCirclePos.eqSum( m_vCylinderPos, m_vCylinderAxis, m_fCylinderSize*0.5);

			vCylinderCircleNormal_Rel.set( nCYLINDER_AXIS, -1.0);
		} 
		else 
		{
			// get bottom circle
//			vCylinderCirclePos[0] = m_vCylinderPos[0] - m_vCylinderAxis[0]*(m_fCylinderSize*(0.5));
//			vCylinderCirclePos[1] = m_vCylinderPos[1] - m_vCylinderAxis[1]*(m_fCylinderSize*(0.5));
//			vCylinderCirclePos[2] = m_vCylinderPos[2] - m_vCylinderAxis[2]*(m_fCylinderSize*(0.5));
			vCylinderCirclePos.eqSum( m_vCylinderPos, m_vCylinderAxis, -m_fCylinderSize*0.5);

			vCylinderCircleNormal_Rel.set( nCYLINDER_AXIS, 1.0);
		}

		DVector3 vTemp = new DVector3();
		dQuatInv(m_qCylinderRot , m_qInvCylinderRot);
		// transform triangle points to space of cylinder circle
		for(i=0; i<3; i++) 
		{
			dVector3Subtract(avPoints[i] , vCylinderCirclePos , vTemp);
			dQuatTransform(m_qInvCylinderRot,vTemp,avPoints[i]);
		}

		int iTmpCounter1 = 0;
		int iTmpCounter2 = 0;
		DVector4 plPlane = new DVector4();

		// plane of cylinder that contains circle for intersection
		//plPlane = Plane4f( vCylinderCircleNormal_Rel, 0.0f );
		dConstructPlane(vCylinderCircleNormal_Rel,(0.0),plPlane);
		iTmpCounter1 = dClipPolyToPlane(avPoints, 3, avTempArray1, plPlane);

		// Body of base circle of Cylinder
		int nCircleSegment = 0;
		for (nCircleSegment = 0; nCircleSegment < nCYLINDER_CIRCLE_SEGMENTS; nCircleSegment++)
		{
			dConstructPlane(m_avCylinderNormals[nCircleSegment],m_fCylinderRadius,plPlane);

			if (0 == (nCircleSegment % 2))
			{
				iTmpCounter2 = dClipPolyToPlane( avTempArray1 , iTmpCounter1 , avTempArray2, plPlane);
			}
			else
			{
				iTmpCounter1 = dClipPolyToPlane( avTempArray2, iTmpCounter2, avTempArray1, plPlane );
			}

			Common.dIASSERT( iTmpCounter1 >= 0 && iTmpCounter1 <= nMAX_CYLINDER_TRIANGLE_CLIP_POINTS );
			Common.dIASSERT( iTmpCounter2 >= 0 && iTmpCounter2 <= nMAX_CYLINDER_TRIANGLE_CLIP_POINTS );
		}

		// back transform clipped points to absolute space
		double ftmpdot;	
		double fTempDepth;
		DVector3 vPoint = new DVector3();

		if (nCircleSegment %2 != 0)
		{
			for( i=0; i<iTmpCounter2; i++)
			{
				dQuatTransform(m_qCylinderRot,avTempArray2[i], vPoint);
//				vPoint[0] += vCylinderCirclePos[0];
//				vPoint[1] += vCylinderCirclePos[1];
//				vPoint[2] += vCylinderCirclePos[2];
				vPoint.add( vCylinderCirclePos );

				dVector3Subtract(vPoint,m_vCylinderPos,vTemp);
				ftmpdot	 = Math.abs(vTemp.dot(m_vContactNormal));
				fTempDepth = m_fBestrt - ftmpdot;
				// Depth must be positive
				if (fTempDepth > (0.0))
				{
					m_gLocalContacts[m_nContacts].fDepth = fTempDepth;
					dVector3Copy(m_vContactNormal,m_gLocalContacts[m_nContacts].vNormal);
					dVector3Copy(vPoint,m_gLocalContacts[m_nContacts].vPos);
					m_gLocalContacts[m_nContacts].nFlags = 1;
					m_nContacts++;
					if(m_nContacts >= (m_iFlags & DxGeom.NUMC_MASK)) 
						return;
				}
			}
		}
		else
		{
			for( i=0; i<iTmpCounter1; i++)
			{
				dQuatTransform(m_qCylinderRot,avTempArray1[i], vPoint);
//				vPoint[0] += vCylinderCirclePos[0];
//				vPoint[1] += vCylinderCirclePos[1];
//				vPoint[2] += vCylinderCirclePos[2];
				vPoint.add( vCylinderCirclePos );

				dVector3Subtract(vPoint,m_vCylinderPos,vTemp);
				ftmpdot	 = Math.abs(vTemp.dot(m_vContactNormal));
				fTempDepth = m_fBestrt - ftmpdot;
				// Depth must be positive
				if (fTempDepth > (0.0))
				{
					m_gLocalContacts[m_nContacts].fDepth = fTempDepth;
					dVector3Copy(m_vContactNormal,m_gLocalContacts[m_nContacts].vNormal);
					dVector3Copy(vPoint,m_gLocalContacts[m_nContacts].vPos);
					m_gLocalContacts[m_nContacts].nFlags = 1;
					m_nContacts++;
					if(m_nContacts >= (m_iFlags & DxGeom.NUMC_MASK)) 
						return;
				}
			}
		}
	}

//	void sCylinderTrimeshColliderData::TestOneTriangleVsCylinder(
//									  const dVector3 &v0, 
//	                                  const dVector3 &v1, 
//	                                  const dVector3 &v2, 
//	                                  const bool bDoubleSided)
	void TestOneTriangleVsCylinder(
			  final DVector3C v0, 
			  final DVector3C v1, 
			  final DVector3C v2, 
            final boolean bDoubleSided)
	{
		// calculate triangle normal
		dVector3Subtract( v2 , v1 , m_vE1);
		DVector3 vTemp = new DVector3();
		dVector3Subtract( v0 , v1 ,vTemp);
		dVector3Cross(m_vE1 , vTemp , m_vNormal );

		// Even though all triangles might be initially valid, 
		// a triangle may degenerate into a segment after applying 
		// space transformation.
		if (!OdeMath.dSafeNormalize3( m_vNormal))
		{
			return;
		}

		// create plane from triangle
		//Plane4f plTrianglePlane = Plane4f( vPolyNormal, v0 ); 
		double plDistance = -v0.dot( m_vNormal);
		DVector4 plTrianglePlane = new DVector4();
		dConstructPlane( m_vNormal,plDistance,plTrianglePlane);

		 // calculate sphere distance to plane
		double fDistanceCylinderCenterToPlane = dPointPlaneDistance(m_vCylinderPos , plTrianglePlane);

		// Sphere must be over positive side of triangle
		if(fDistanceCylinderCenterToPlane < 0 && !bDoubleSided) 
		{
			// if not don't generate contacts
			return;
		 }

		DVector3 vPnt0 = new DVector3();
		DVector3 vPnt1 = new DVector3();
		DVector3 vPnt2 = new DVector3();

		if (fDistanceCylinderCenterToPlane < (0.0) )
		{
			// flip it
			dVector3Copy(v0 , vPnt0);
			dVector3Copy(v1 , vPnt2);
			dVector3Copy(v2 , vPnt1);
		}
		else
		{
			dVector3Copy(v0 , vPnt0);
			dVector3Copy(v1 , vPnt1);
			dVector3Copy(v2 , vPnt2);
		}

		m_fBestDepth = MAX_REAL;

		// do intersection test and find best separating axis
		if(!_cldTestSeparatingAxes(vPnt0, vPnt1, vPnt2) ) 
		{
			// if not found do nothing
			return;
		}

		// if best separation axis is not found
		if ( m_iBestAxis == 0 ) 
		{
			// this should not happen (the function should have already returned in this case)
			Common.dIASSERT(false);
			// do nothing
			return;
		}

		double fdot = m_vContactNormal.dot( m_vCylinderAxis );

		// choose which clipping method are we going to apply
		if (Math.abs(fdot) < (0.9) ) 
		{
			if (!_cldClipCylinderEdgeToTriangle(vPnt0, vPnt1, vPnt2)) 
			{
				return;
			}
		}
		else 
		{
			_cldClipCylinderToTriangle(vPnt0, vPnt1, vPnt2);
		}
	}

//	void sCylinderTrimeshColliderData::_InitCylinderTrimeshData(dxGeom *Cylinder, dxTriMesh *Trimesh)
	void _InitCylinderTrimeshData(DxCylinder Cylinder, DxTriMesh Trimesh)
	{
		// get cylinder information
		// Rotation
		DMatrix3C pRotCyc = Cylinder.getRotation(); 
		m_mCylinderRot.set(pRotCyc);
		m_qCylinderRot.set( Cylinder.getQuaternion() );
		
		// Position
		DVector3C pPosCyc = Cylinder.getPosition();
		dVector3Copy(pPosCyc,m_vCylinderPos);
		// Cylinder axis
		dMat3GetCol(m_mCylinderRot,nCYLINDER_AXIS,m_vCylinderAxis);
		// get cylinder radius and size
		//dGeomCylinderGetParams(Cylinder,m_fCylinderRadius,m_fCylinderSize);
		m_fCylinderRadius = Cylinder.getRadius();
		m_fCylinderSize = Cylinder.getLength();
		
		// get trimesh position and orientation
		DMatrix3C pRotTris = Trimesh.getRotation(); 
		m_mTrimeshRot.set(pRotTris);
		m_qTrimeshRot = Trimesh.getQuaternion();  //TODO TZ copy instead?
		
		// Position
		DVector3C pPosTris = Trimesh.getPosition();
		dVector3Copy(pPosTris,m_vTrimeshPos);


		// calculate basic angle for 8-gon
		double fAngle = M_PI / nCYLINDER_CIRCLE_SEGMENTS;
		// calculate angle increment
		double fAngleIncrement = fAngle*(2.0); 

		// calculate plane normals
		// axis dependant code
		for(int i=0; i<nCYLINDER_CIRCLE_SEGMENTS; i++) 
		{
			m_avCylinderNormals[i] = new DVector3();
			m_avCylinderNormals[i].set0( -Math.cos(fAngle) );
			m_avCylinderNormals[i].set1( -Math.sin(fAngle) );
			m_avCylinderNormals[i].set2( 0.0 );

			fAngle += fAngleIncrement;
		}

		m_vBestPoint.setZero();
		// reset best depth
		m_fBestCenter = (0.0);	
	}

//	int sCylinderTrimeshColliderData::TestCollisionForSingleTriangle(int ctContacts0, 
//		int Triint, dVector3 dv[3], bool &bOutFinishSearching)
	int TestCollisionForSingleTriangle(int ctContacts0, 
			final int Triint, DVector3 dv0, DVector3 dv1, DVector3 dv2, final RefBoolean bOutFinishSearching)
	{
		// test this triangle
		TestOneTriangleVsCylinder(dv0,dv1,dv2, false);

		// fill-in tri index for generated contacts
		for (; ctContacts0<m_nContacts; ctContacts0++)
			m_gLocalContacts[ctContacts0].triIndex = Triint;

		// Putting "break" at the end of loop prevents unnecessary checks on first pass and "continue"
		bOutFinishSearching.b = (m_nContacts >= (m_iFlags & DxGeom.NUMC_MASK));

		return ctContacts0;
	}

}
	
//	// OPCODE version of cylinder to mesh collider
//	#if dTRIMESH_OPCODE
//	static void dQueryCTLPotentialCollisionTriangles(OBBCollider &Collider, 
//		sCylinderTrimeshColliderData &cData, dxGeom *Cylinder, dxTriMesh *Trimesh,
//		OBBCache &BoxCache)
//	{
//Matrix4x4 MeshMatrix;
//    const dVector3 vZeroVector3 = { REAL(0.0), };
//	MakeMatrix(vZeroVector3, cData.m_mTrimeshRot, MeshMatrix);
//
//    const dVector3 &vCylinderPos = cData.m_vCylinderPos;
//    const dMatrix3 &mCylinderRot = cData.m_mCylinderRot;
//
//	dVector3 vCylinderOffsetPos;
//	dSubtractVectors3(vCylinderOffsetPos, vCylinderPos, cData.m_vTrimeshPos);
//
//    const dReal fCylinderRadius = cData.m_fCylinderRadius, fCylinderHalfAxis = cData.m_fCylinderSize * REAL(0.5);
//
//	OBB obbCylinder;
//    obbCylinder.mCenter.Set(vCylinderOffsetPos[0], vCylinderOffsetPos[1], vCylinderOffsetPos[2]);
//    obbCylinder.mExtents.Set(
//			0 == nCYLINDER_AXIS ? fCylinderHalfAxis : fCylinderRadius,
//			1 == nCYLINDER_AXIS ? fCylinderHalfAxis : fCylinderRadius,
//			2 == nCYLINDER_AXIS ? fCylinderHalfAxis : fCylinderRadius);
//    obbCylinder.mRot.Set(
//	mCylinderRot[0], mCylinderRot[4], mCylinderRot[8],
//	mCylinderRot[1], mCylinderRot[5], mCylinderRot[9],
//	mCylinderRot[2], mCylinderRot[6], mCylinderRot[10]);
//
//	// TC results
//    if (Trimesh->getDoTC(dxTriMesh::TTC_BOX))
//	{
//		dxTriMesh::BoxTC* BoxTC = 0;
//        const int iBoxCacheSize = Trimesh->m_BoxTCCache.size();
//		for (int i = 0; i != iBoxCacheSize; i++)
//		{
//			if (Trimesh->m_BoxTCCache[i].Geom == Cylinder)
//			{
//				BoxTC = &Trimesh->m_BoxTCCache[i];
//				break;
//			}
//		}
//		if (!BoxTC)
//		{
//			Trimesh->m_BoxTCCache.push(dxTriMesh::BoxTC());
//
//			BoxTC = &Trimesh->m_BoxTCCache[Trimesh->m_BoxTCCache.size() - 1];
//			BoxTC->Geom = Cylinder;
//			BoxTC->FatCoeff = REAL(1.0);
//		}
//
//		// Intersect
//		Collider.SetTemporalCoherence(true);
//		Collider.Collide(*BoxTC, obbCylinder, Trimesh->retrieveMeshBVTreeRef(), null, &MeshMatrix);
//	}
//    else
//	{
//		Collider.SetTemporalCoherence(false);
//		Collider.Collide(BoxCache, obbCylinder, Trimesh->retrieveMeshBVTreeRef(), null, &MeshMatrix);
//	}
//}
//
//	int dCollideCylinderTrimesh(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
//	{
//		dIASSERT( skip >= (int)sizeof( dContactGeom ) );
//		dIASSERT( o1->type == dCylinderClass );
//		dIASSERT( o2->type == dTriMeshClass );
//		dIASSERT ((flags & NUMC_MASK) >= 1);
//
//		int nContactCount = 0;
//
//		dxGeom *Cylinder = o1;
//		dxTriMesh *Trimesh = (dxTriMesh *)o2;
//
//		// Main data holder
//		sCylinderTrimeshColliderData cData(flags, skip);
//		cData._InitCylinderTrimeshData(Cylinder, Trimesh);
//
//    const unsigned uiTLSKind = Trimesh->getParentSpaceTLSKind();
//		dIASSERT(uiTLSKind == Cylinder->getParentSpaceTLSKind()); // The colliding spaces must use matching cleanup method
//		TrimeshCollidersCache *pccColliderCache = GetTrimeshCollidersCache(uiTLSKind);
//		OBBCollider& Collider = pccColliderCache->m_OBBCollider;
//
//		dQueryCTLPotentialCollisionTriangles(Collider, cData, Cylinder, Trimesh, pccColliderCache->m_DefaultBoxCache);
//
//		// Retrieve data
//		int TriCount = Collider.GetNbTouchedPrimitives();
//
//		if (TriCount != 0)
//		{
//        const int* Triangles = (const int*)Collider.GetTouchedPrimitives();
//
//			if (Trimesh->m_ArrayCallback != NULL)
//			{
//				Trimesh->m_ArrayCallback(Trimesh, Cylinder, Triangles, TriCount);
//			}
//
//			// allocate buffer for local contacts on stack
//			cData.m_gLocalContacts = (sLocalContactData*)dALLOCA16(sizeof(sLocalContactData)*(cData.m_iFlags & NUMC_MASK));
//
//			int ctContacts0 = 0;
//
//			// loop through all intersecting triangles
//			for (int i = 0; i < TriCount; i++)
//			{
//            const int Triint = Triangles[i];
//				if (!Trimesh->invokeCallback(Cylinder, Triint)) continue;
//
//
//				dVector3 dv[3];
//				Trimesh->fetchMeshTriangle(dv, Triint, cData.m_vTrimeshPos, cData.m_mTrimeshRot);
//
//				bool bFinishSearching;
//				ctContacts0 = cData.TestCollisionForSingleTriangle(ctContacts0, Triint, dv, bFinishSearching);
//
//				if (bFinishSearching)
//				{
//					break;
//				}
//			}
//
//			if (cData.m_nContacts != 0)
//			{
//				nContactCount = cData._ProcessLocalContacts(contact, Cylinder, Trimesh);
//			}
//		}
//
//		return nContactCount;
//	}
//	#endif  // OPCODE

	// GIMPACT version of cylinder to mesh collider
//	#if dTRIMESH_GIMPACT
//	int dCollideCylinderTrimesh(DxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
	int dCollideCylinderTrimesh(DxCylinder o1, DxGimpact o2, final int flags, DContactGeomBuffer contacts, int skip)
	{
		Common.dIASSERT( skip == 1);//(int)sizeof( dContactGeom ) );
		//dIASSERT( o1->type == dCylinderClass );
		//dIASSERT( o2->type == dTriMeshClass );
		Common.dIASSERT ((flags & DxGeom.NUMC_MASK) >= 1);
		
		int nContactCount = 0;

		DxCylinder Cylinder = o1;
		DxGimpact Trimesh = o2;

		// Main data holder
		sCylinderTrimeshColliderData cData = new sCylinderTrimeshColliderData(flags, skip);
		cData._InitCylinderTrimeshData(Cylinder, Trimesh);

		//*****at first , collide box aabb******//

		aabb3f test_aabb = new aabb3f();
		DAABBC aabb = o1._aabb; // TODO TZ: remompute AABB with getAABB()?
		test_aabb.set(aabb.getMin0(), aabb.getMax0(), aabb.getMin1(), aabb.getMax1(), aabb.getMin2(), aabb.getMax2());


		GimDynArrayInt collision_result = GimDynArrayInt.GIM_CREATE_BOXQUERY_LIST();

		Trimesh.m_collision_trimesh().getAabbSet().gim_aabbset_box_collision(test_aabb, collision_result);

		if (collision_result.size() != 0)
		{
		//*****Set globals for box collision******//

			int ctContacts0 = 0;
			// cData.m_gLocalContacts = (sLocalContactData*)dALLOCA16(sizeof(sLocalContactData)*(cData.m_iFlags & NUMC_MASK));
			cData.m_gLocalContacts = new sLocalContactData[cData.m_iFlags & DxGeom.NUMC_MASK];
			for (int i = 0; i < cData.m_gLocalContacts.length; i++) {
				cData.m_gLocalContacts[i] = new sLocalContactData();
			}

			int[] boxesresult = collision_result.GIM_DYNARRAY_POINTER();
			GimTrimesh ptrimesh = Trimesh.m_collision_trimesh();

			ptrimesh.gim_trimesh_locks_work_data();

			for(int i=0;i<collision_result.size();i++)
			{
				final int Triint = boxesresult[i];
				
				//vec3f[] dvf = { new vec3f(), new vec3f(), new vec3f() };
				DVector3 dv0 = new DVector3(), dv1 = new DVector3(), dv2 = new DVector3();
				ptrimesh.gim_trimesh_get_triangle_vertices(Triint, dv0, dv1, dv2);
				
				//dv[0].set(dvf[0].f);
				//dv[1].set(dvf[1].f);
				//dv[2].set(dvf[2].f);
				RefBoolean bFinishSearching = new RefBoolean(false);
				ctContacts0 = cData.TestCollisionForSingleTriangle(ctContacts0, Triint, dv0, dv1, dv2, bFinishSearching);

				if (bFinishSearching.b) 
				{
					break;
				}
			}

			ptrimesh.gim_trimesh_unlocks_work_data();

			if (cData.m_nContacts != 0)
			{
				nContactCount = cData._ProcessLocalContacts(contacts, Cylinder, Trimesh);
			}
		}

		collision_result.GIM_DYNARRAY_DESTROY();

		return nContactCount;
	}
//	#endif
//
//	#endif // dTRIMESH_ENABLED
}
