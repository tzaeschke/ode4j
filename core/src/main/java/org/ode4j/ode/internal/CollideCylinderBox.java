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

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector4;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;

import static org.ode4j.ode.OdeMath.*;

/**
 *	Cylinder-box collider by Alen Ladavac
 *  Ported to ODE by Nguyen Binh
 */
class CollideCylinderBox extends DxCollisionUtil implements DColliderFn {

	private static final int MAX_CYLBOX_CLIP_POINTS  = 16;
	private static final int nCYLINDER_AXIS			 = 2;
	// Number of segment of cylinder base circle.
	// Must be divisible by 4.
	private static final int nCYLINDER_SEGMENT		 = 8;

	//check with actual definition
	//#define MAX_FLOAT	dInfinity

	// Data that passed through the collider's functions
	private class sCylinderBoxData
	{
		private sCylinderBoxData(DxCylinder cylinder, DxBox box, int flags, DContactGeomBuffer contacts, int skip)//:
			//m_gBox(Box), m_gCylinder(Cylinder), m_gContact(contact), m_iFlags(flags), m_iSkip(skip), m_nContacts(0)
		{
			m_gBox = box;
			m_gCylinder = cylinder;
			m_gContact = contacts;
			m_iFlags = flags;
			//TZ not used m_iSkip = skip;
			if (skip != 1) throw new IllegalArgumentException(
					"'skip' should be 1, but is: " + skip);
			m_nContacts = 0;
		}

		//	void _cldInitCylinderBox();
		//	int _cldTestAxis( dVector3& vInputNormal, int iAxis );
		//	int _cldTestEdgeCircleAxis( final dVector3 &vCenterPoint, 
		//		final dVector3 &vVx0, final dVector3 &vVx1, int iAxis );
		//	int _cldTestSeparatingAxes();
		//	int _cldClipCylinderToBox();
		//	void _cldClipBoxToCylinder();
		//	int PerformCollisionChecking();

		// cylinder parameters
		private DMatrix3			m_mCylinderRot = new DMatrix3();
		private DVector3			m_vCylinderPos = new DVector3();
		private DVector3			m_vCylinderAxis = new DVector3();
		private double				m_fCylinderRadius;
		private double				m_fCylinderSize;
		private DVector3[]			m_avCylinderNormals = new DVector3[nCYLINDER_SEGMENT];

		// box parameters

		private DMatrix3			m_mBoxRot = new DMatrix3();
		private DVector3			m_vBoxPos = new DVector3();
		private DVector3			m_vBoxHalfSize = new DVector3();
		// box vertices array : 8 vertices
		private DVector3[]			m_avBoxVertices = new DVector3[8];

		// global collider data
		private DVector3			m_vDiff = new DVector3();			
		private DVector3			m_vNormal = new DVector3();
		private double				m_fBestDepth;
		private double				m_fBestrb;
		private double				m_fBestrc;
		private int					m_iBestAxis;

		// contact data
		private DVector3			m_vEp0 = new DVector3(), m_vEp1 = new DVector3();
		private double				m_fDepth0, m_fDepth1;

		// ODE stuff
		private DxBox				m_gBox;
		private DxCylinder			m_gCylinder;
		//dContactGeom*		m_gContact;
		private DContactGeomBuffer		m_gContact;
		private int					m_iFlags;
		//TZ not used private int					m_iSkip;
		private int					m_nContacts;

		//	};


		// initialize collision data
		//void sCylinderBoxData::
		private void _cldInitCylinderBox() 
		{
			// get cylinder position, orientation
//			final DMatrix3C pRotCyc = m_gCylinder.dGeomGetRotation();//m_gCylinder); 
//			dMatrix3Copy(pRotCyc,m_mCylinderRot);
			m_mCylinderRot.set(m_gCylinder.dGeomGetRotation());

			final DVector3C pPosCyc = m_gCylinder.dGeomGetPosition();//m_gCylinder);
			dVector3Copy(pPosCyc,m_vCylinderPos);

			dMat3GetCol(m_mCylinderRot,nCYLINDER_AXIS,m_vCylinderAxis);

			// get cylinder radius and size
			//dGeomCylinderGetParams(m_gCylinder,&m_fCylinderRadius,&m_fCylinderSize);
			m_fCylinderRadius = m_gCylinder.getRadius();
			m_fCylinderSize = m_gCylinder.getLength();

			// get box position, orientation, size
//			final DMatrix3C pRotBox = m_gBox.dGeomGetRotation();//m_gBox);
//			dMatrix3Copy(pRotBox,m_mBoxRot);
			m_mBoxRot.set(m_gBox.dGeomGetRotation());
//			final DVector3C pPosBox = m_gBox.dGeomGetPosition();//m_gBox);
//			dVector3Copy(pPosBox,m_vBoxPos);
			m_vBoxPos.set(m_gBox.dGeomGetPosition());

			m_gBox.dGeomBoxGetLengths(m_vBoxHalfSize);
//			m_vBoxHalfSize[0] *= (0.5);
//			m_vBoxHalfSize[1] *= (0.5);
//			m_vBoxHalfSize[2] *= (0.5);
			m_vBoxHalfSize.scale(0.5);

			// vertex 0
//			m_avBoxVertices[0][0] = -m_vBoxHalfSize[0];
//			m_avBoxVertices[0][1] =  m_vBoxHalfSize[1];
//			m_avBoxVertices[0][2] = -m_vBoxHalfSize[2];
			m_avBoxVertices[0] = new DVector3(m_vBoxHalfSize).scale(-1, 1, -1);

			// vertex 1
//			m_avBoxVertices[1][0] =  m_vBoxHalfSize[0];
//			m_avBoxVertices[1][1] =  m_vBoxHalfSize[1];
//			m_avBoxVertices[1][2] = -m_vBoxHalfSize[2];
			m_avBoxVertices[1] = new DVector3(m_vBoxHalfSize).scale(1, 1, -1);

			// vertex 2
//			m_avBoxVertices[2][0] = -m_vBoxHalfSize[0];
//			m_avBoxVertices[2][1] = -m_vBoxHalfSize[1];
//			m_avBoxVertices[2][2] = -m_vBoxHalfSize[2];
			m_avBoxVertices[2] = new DVector3(m_vBoxHalfSize).scale(-1, -1, -1);

			// vertex 3
//			m_avBoxVertices[3][0] =  m_vBoxHalfSize[0];
//			m_avBoxVertices[3][1] = -m_vBoxHalfSize[1];
//			m_avBoxVertices[3][2] = -m_vBoxHalfSize[2];
			m_avBoxVertices[3] = new DVector3(m_vBoxHalfSize).scale(1, -1, -1);

			// vertex 4
//			m_avBoxVertices[4][0] =  m_vBoxHalfSize[0];
//			m_avBoxVertices[4][1] =  m_vBoxHalfSize[1];
//			m_avBoxVertices[4][2] =  m_vBoxHalfSize[2];
			m_avBoxVertices[4] = new DVector3(m_vBoxHalfSize).scale(1, 1, 1);

			// vertex 5
//			m_avBoxVertices[5][0] =  m_vBoxHalfSize[0];
//			m_avBoxVertices[5][1] = -m_vBoxHalfSize[1];
//			m_avBoxVertices[5][2] =  m_vBoxHalfSize[2];
			m_avBoxVertices[5] = new DVector3(m_vBoxHalfSize).scale(1, -1, 1);

			// vertex 6
//			m_avBoxVertices[6][0] = -m_vBoxHalfSize[0];
//			m_avBoxVertices[6][1] = -m_vBoxHalfSize[1];
//			m_avBoxVertices[6][2] =  m_vBoxHalfSize[2];
			m_avBoxVertices[6] = new DVector3(m_vBoxHalfSize).scale(-1, -1, 1);

			// vertex 7
//			m_avBoxVertices[7][0] = -m_vBoxHalfSize[0];
//			m_avBoxVertices[7][1] =  m_vBoxHalfSize[1];
//			m_avBoxVertices[7][2] =  m_vBoxHalfSize[2];
			m_avBoxVertices[7] = new DVector3(m_vBoxHalfSize).scale(-1, 1, 1);

			// temp index
			int i = 0;
			DVector3[]	vTempBoxVertices = DVector3.newArray(8);
			// transform vertices in absolute space
			for(i=0; i < 8; i++) 
			{
				dMultiplyMat3Vec3(m_mBoxRot,m_avBoxVertices[i], vTempBoxVertices[i]);
				dVector3Add(vTempBoxVertices[i], m_vBoxPos, m_avBoxVertices[i]);
			}

			// find relative position
			dVector3Subtract(m_vCylinderPos,m_vBoxPos,m_vDiff);
			m_fBestDepth = Common.MAX_FLOAT;
//			m_vNormal[0] = (0.0);
//			m_vNormal[1] = (0.0);
//			m_vNormal[2] = (0.0);
			m_vNormal.setZero();

			// calculate basic angle for nCYLINDER_SEGMENT-gon
			double fAngle = M_PI/nCYLINDER_SEGMENT;

			// calculate angle increment
			double fAngleIncrement = fAngle * (2.0); 

			// calculate nCYLINDER_SEGMENT-gon points
			for(i = 0; i < nCYLINDER_SEGMENT; i++) 
			{
//				m_avCylinderNormals[i][0] = -dCos(fAngle);
//				m_avCylinderNormals[i][1] = -dSin(fAngle);
//				m_avCylinderNormals[i][2] = 0;
				m_avCylinderNormals[i] = new DVector3( -dCos(fAngle), -dSin(fAngle), 0 );

				fAngle += fAngleIncrement;
			}

			m_fBestrb		= 0;
			m_fBestrc		= 0;
			m_iBestAxis		= 0;
			m_nContacts		= 0;

		}

		// test for given separating axis
		//int sCylinderBoxData::_cldTestAxis( dVector3& vInputNormal, int iAxis ) 
		private boolean _cldTestAxis( DVector3 vInputNormal, int iAxis )
		{
			// check length of input normal
			double fL = dVector3Length(vInputNormal);
			// if not long enough
			if ( fL < (1e-5) ) 
			{
				// do nothing
				return true;
			}

			// otherwise make it unit for sure
			vInputNormal.normalize();//dNormalize3(vInputNormal);

			// project box and Cylinder on mAxis
			double fdot1 = dVector3Dot(m_vCylinderAxis, vInputNormal);

			double frc;

			if (fdot1 > (1.0)) 
			{
				// assume fdot1 = 1
				frc = m_fCylinderSize*(0.5);
			}
			else if (fdot1 < (-1.0))
			{
				// assume fdot1 = -1
				frc = m_fCylinderSize*(0.5);
			}
			else
			{
				// project box and capsule on iAxis
				frc = dFabs( fdot1 * (m_fCylinderSize*(0.5))) + m_fCylinderRadius * dSqrt((1.0)-(fdot1*fdot1));
			}

			DVector3	vTemp1 = new DVector3();

			dMat3GetCol(m_mBoxRot,0,vTemp1);
			double frb = dFabs(dVector3Dot(vTemp1,vInputNormal))*m_vBoxHalfSize.get0();

			dMat3GetCol(m_mBoxRot,1,vTemp1);
			frb += dFabs(dVector3Dot(vTemp1,vInputNormal))*m_vBoxHalfSize.get1();

			dMat3GetCol(m_mBoxRot,2,vTemp1);
			frb += dFabs(dVector3Dot(vTemp1,vInputNormal))*m_vBoxHalfSize.get2();

			// project their distance on separating axis
			double fd  = dVector3Dot(m_vDiff,vInputNormal);

			// get depth 

			double fDepth = frc + frb;  // Calculate partial depth

			// if they do not overlap exit, we have no intersection
			if ( dFabs(fd) > fDepth )
			{ 
				return false; 
			} 

			// Finalyze the depth calculation
			fDepth -= dFabs(fd);

			// get maximum depth
			if ( fDepth < m_fBestDepth ) 
			{
				m_fBestDepth = fDepth;
				dVector3Copy(vInputNormal,m_vNormal);
				m_iBestAxis  = iAxis;
				m_fBestrb    = frb;
				m_fBestrc    = frc;

				// flip normal if interval is wrong faced
				if (fd > 0) 
				{ 
					dVector3Inv(m_vNormal);
				}
			}

			return true;
		}


		// check for separation between box edge and cylinder circle edge
		//int sCylinderBoxData::_cldTestEdgeCircleAxis( 
		//			final dVector3 &vCenterPoint, 
		//			final dVector3 &vVx0, final dVector3 &vVx1, 
		//			int iAxis ) 
		private boolean _cldTestEdgeCircleAxis( 
				final DVector3 vCenterPoint, 
				final DVector3 vVx0, final DVector3 vVx1, 
				int iAxis ) 
		{
			// calculate direction of edge
			DVector3 vDirEdge = new DVector3();
			dVector3Subtract(vVx1,vVx0,vDirEdge);
			vDirEdge.normalize();//dNormalize3(vDirEdge);
			// starting point of edge 
			DVector3 vEStart = new DVector3();
			dVector3Copy(vVx0,vEStart);;

			// calculate angle cosine between cylinder axis and edge
			double fdot2 = dVector3Dot (vDirEdge,m_vCylinderAxis);

			// if edge is perpendicular to cylinder axis
			if(dFabs(fdot2) < (1e-5)) 
			{
				// this can't be separating axis, because edge is parallel to circle plane
				return true;
			}

			// find point of intersection between edge line and circle plane
			DVector3 vTemp1 = new DVector3();
			dVector3Subtract(vCenterPoint,vEStart,vTemp1);
			double fdot1 = dVector3Dot(vTemp1,m_vCylinderAxis);
			DVector3 vpnt = new DVector3();
//			vpnt[0]= vEStart[0] + vDirEdge[0] * (fdot1/fdot2);
//			vpnt[1]= vEStart[1] + vDirEdge[1] * (fdot1/fdot2);
//			vpnt[2]= vEStart[2] + vDirEdge[2] * (fdot1/fdot2);
			vpnt.eqSum(vEStart, vDirEdge, fdot1/fdot2);

			// find tangent vector on circle with same center (vCenterPoint) that
			// touches point of intersection (vpnt)
			DVector3 vTangent = new DVector3();
			dVector3Subtract(vCenterPoint,vpnt,vTemp1);
			dVector3Cross(vTemp1,m_vCylinderAxis,vTangent);

			// find vector orthogonal both to tangent and edge direction
			DVector3 vAxis = new DVector3();
			dVector3Cross(vTangent,vDirEdge,vAxis);

			// use that vector as separating axis
			return _cldTestAxis( vAxis, iAxis );
		}

		// Test separating axis for collision
		//int sCylinderBoxData::
		private int _cldTestSeparatingAxes() 
		{
			// reset best axis
			m_fBestDepth = Common.MAX_FLOAT;
			m_iBestAxis = 0;
			m_fBestrb = 0;
			m_fBestrc = 0;
			m_nContacts = 0;

			DVector3  vAxis = new DVector3();//{(0.0),(0.0),(0.0),(0.0)};

			// Epsilon value for checking axis vector length 
			final double fEpsilon = (1e-6);

			// axis A0
			dMat3GetCol(m_mBoxRot, 0 , vAxis);
			if (!_cldTestAxis( vAxis, 1 )) 
			{
				return 0;
			}

			// axis A1
			dMat3GetCol(m_mBoxRot, 1 , vAxis);
			if (!_cldTestAxis( vAxis, 2 )) 
			{
				return 0;
			}

			// axis A2
			dMat3GetCol(m_mBoxRot, 2 , vAxis);
			if (!_cldTestAxis( vAxis, 3 )) 
			{
				return 0;
			}

			// axis C - Cylinder Axis
			//vAxis = vCylinderAxis;
			dVector3Copy(m_vCylinderAxis , vAxis);
			if (!_cldTestAxis( vAxis, 4 )) 
			{
				return 0;
			}

			// axis CxA0
			//vAxis = ( vCylinderAxis cross mthGetColM33f( mBoxRot, 0 ));
			dVector3CrossMat3Col(m_mBoxRot, 0 ,m_vCylinderAxis, vAxis);
			if(dVector3LengthSquare( vAxis ) > fEpsilon ) 
			{
				if (!_cldTestAxis( vAxis, 5 ))
				{
					return 0;
				}
			}

			// axis CxA1
			//vAxis = ( vCylinderAxis cross mthGetColM33f( mBoxRot, 1 ));
			dVector3CrossMat3Col(m_mBoxRot, 1 ,m_vCylinderAxis, vAxis);
			if(dVector3LengthSquare( vAxis ) > fEpsilon ) 
			{
				if (!_cldTestAxis( vAxis, 6 )) 
				{
					return 0;
				}
			}

			// axis CxA2
			//vAxis = ( vCylinderAxis cross mthGetColM33f( mBoxRot, 2 ));
			dVector3CrossMat3Col(m_mBoxRot, 2 ,m_vCylinderAxis, vAxis);
			if(dVector3LengthSquare( vAxis ) > fEpsilon ) 
			{
				if (!_cldTestAxis( vAxis, 7 ))
				{
					return 0;
				}
			}

			int i = 0;
			DVector3	vTemp1 = new DVector3();
			DVector3	vTemp2 = new DVector3();
			// here we check box's vertices axis
			for(i=0; i< 8; i++) 
			{
				//vAxis = ( vCylinderAxis cross (m_avBoxVertices[i] - vCylinderPos));
				dVector3Subtract(m_avBoxVertices[i],m_vCylinderPos,vTemp1);
				dVector3Cross(m_vCylinderAxis,vTemp1,vTemp2);
				//vAxis = ( vCylinderAxis cross vAxis );
				dVector3Cross(m_vCylinderAxis,vTemp2,vAxis);
				if(dVector3LengthSquare( vAxis ) > fEpsilon ) 
				{
					if (!_cldTestAxis( vAxis, 8 + i ))
					{
						return 0;
					}
				}
			}

			// ************************************
			// this is defined for first 12 axes
			// normal of plane that contains top circle of cylinder
			// center of top circle of cylinder
			DVector3 vcc = new DVector3();
//			vcc[0] = (m_vCylinderPos)[0] + m_vCylinderAxis[0]*(m_fCylinderSize*(0.5));
//			vcc[1] = (m_vCylinderPos)[1] + m_vCylinderAxis[1]*(m_fCylinderSize*(0.5));
//			vcc[2] = (m_vCylinderPos)[2] + m_vCylinderAxis[2]*(m_fCylinderSize*(0.5));
			vcc.eqSum(m_vCylinderPos, m_vCylinderAxis, m_fCylinderSize*0.5);
			// ************************************

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[1], m_avBoxVertices[0], 16)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[1], m_avBoxVertices[3], 17)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[2], m_avBoxVertices[3], 18))
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[2], m_avBoxVertices[0], 19)) 
			{
				return 0;
			}


			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[4], m_avBoxVertices[1], 20))
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[4], m_avBoxVertices[7], 21))
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[0], m_avBoxVertices[7], 22)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[5], m_avBoxVertices[3], 23)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[5], m_avBoxVertices[6], 24)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[2], m_avBoxVertices[6], 25)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[4], m_avBoxVertices[5], 26)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[6], m_avBoxVertices[7], 27)) 
			{
				return 0;
			}

			// ************************************
			// this is defined for second 12 axes
			// normal of plane that contains bottom circle of cylinder
			// center of bottom circle of cylinder
			//	vcc = vCylinderPos - vCylinderAxis*(fCylinderSize*(0.5));
//			vcc[0] = (m_vCylinderPos)[0] - m_vCylinderAxis[0]*(m_fCylinderSize*(0.5));
//			vcc[1] = (m_vCylinderPos)[1] - m_vCylinderAxis[1]*(m_fCylinderSize*(0.5));
//			vcc[2] = (m_vCylinderPos)[2] - m_vCylinderAxis[2]*(m_fCylinderSize*(0.5));
			vcc.eqSum(m_vCylinderPos, m_vCylinderAxis, -m_fCylinderSize*(0.5));
			// ************************************

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[1], m_avBoxVertices[0], 28)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[1], m_avBoxVertices[3], 29)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[2], m_avBoxVertices[3], 30)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[2], m_avBoxVertices[0], 31)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[4], m_avBoxVertices[1], 32)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[4], m_avBoxVertices[7], 33)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[0], m_avBoxVertices[7], 34)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[5], m_avBoxVertices[3], 35)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[5], m_avBoxVertices[6], 36)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[2], m_avBoxVertices[6], 37)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[4], m_avBoxVertices[5], 38)) 
			{
				return 0;
			}

			if (!_cldTestEdgeCircleAxis( vcc, m_avBoxVertices[6], m_avBoxVertices[7], 39)) 
			{
				return 0;
			}

			return 1;
		}

		//int sCylinderBoxData::
		private boolean _cldClipCylinderToBox()
		{
			dIASSERT(m_nContacts != (m_iFlags & DxGeom.NUMC_MASK));

			// calculate that vector perpendicular to cylinder axis which closes lowest angle with collision normal
			DVector3 vN = new DVector3();
			double fTemp1 = dVector3Dot(m_vCylinderAxis,m_vNormal);
//			vN[0]	=	m_vNormal[0] - m_vCylinderAxis[0]*fTemp1;
//			vN[1]	=	m_vNormal[1] - m_vCylinderAxis[1]*fTemp1;
//			vN[2]	=	m_vNormal[2] - m_vCylinderAxis[2]*fTemp1;
			vN.eqSum( m_vNormal, m_vCylinderAxis, -fTemp1);

			// normalize that vector
			dNormalize3(vN);

			// translate cylinder end points by the vector
			DVector3 vCposTrans = new DVector3();
//			vCposTrans[0] = m_vCylinderPos[0] + vN[0] * m_fCylinderRadius;
//			vCposTrans[1] = m_vCylinderPos[1] + vN[1] * m_fCylinderRadius;
//			vCposTrans[2] = m_vCylinderPos[2] + vN[2] * m_fCylinderRadius;
			vCposTrans.eqSum( m_vCylinderPos, vN, m_fCylinderRadius);

//			m_vEp0[0]  = vCposTrans[0] + m_vCylinderAxis[0]*(m_fCylinderSize*(0.5));
//			m_vEp0[1]  = vCposTrans[1] + m_vCylinderAxis[1]*(m_fCylinderSize*(0.5));
//			m_vEp0[2]  = vCposTrans[2] + m_vCylinderAxis[2]*(m_fCylinderSize*(0.5));
			m_vEp0.eqSum( vCposTrans, m_vCylinderAxis, m_fCylinderSize*0.5);

//			m_vEp1[0]  = vCposTrans[0] - m_vCylinderAxis[0]*(m_fCylinderSize*(0.5));
//			m_vEp1[1]  = vCposTrans[1] - m_vCylinderAxis[1]*(m_fCylinderSize*(0.5));
//			m_vEp1[2]  = vCposTrans[2] - m_vCylinderAxis[2]*(m_fCylinderSize*(0.5));
			m_vEp1.eqSum( vCposTrans, m_vCylinderAxis, -m_fCylinderSize*0.5);

			// transform edge points in box space
//			m_vEp0[0] -= m_vBoxPos[0];
//			m_vEp0[1] -= m_vBoxPos[1];
//			m_vEp0[2] -= m_vBoxPos[2];
			m_vEp0.sub(m_vBoxPos);

//			m_vEp1[0] -= m_vBoxPos[0];
//			m_vEp1[1] -= m_vBoxPos[1];
//			m_vEp1[2] -= m_vBoxPos[2];
			m_vEp1.sub( m_vBoxPos);

			DVector3 vTemp1 = new DVector3();
			// clip the edge to box 
			DVector4 plPlane = new DVector4();
			// plane 0 +x
			dMat3GetCol(m_mBoxRot,0,vTemp1);
			dConstructPlane(vTemp1,m_vBoxHalfSize.get0(),plPlane);
			if(!dClipEdgeToPlane( m_vEp0, m_vEp1, plPlane )) 
			{ 
				return false; 
			}

			// plane 1 +y
			dMat3GetCol(m_mBoxRot,1,vTemp1);
			dConstructPlane(vTemp1,m_vBoxHalfSize.get1(),plPlane);
			if(!dClipEdgeToPlane( m_vEp0, m_vEp1, plPlane )) 
			{ 
				return false; 
			}

			// plane 2 +z
			dMat3GetCol(m_mBoxRot,2,vTemp1);
			dConstructPlane(vTemp1,m_vBoxHalfSize.get2(),plPlane);
			if(!dClipEdgeToPlane( m_vEp0, m_vEp1, plPlane )) 
			{ 
				return false; 
			}

			// plane 3 -x
			dMat3GetCol(m_mBoxRot,0,vTemp1);
			dVector3Inv(vTemp1);
			dConstructPlane(vTemp1,m_vBoxHalfSize.get0(),plPlane);
			if(!dClipEdgeToPlane( m_vEp0, m_vEp1, plPlane )) 
			{ 
				return false; 
			}

			// plane 4 -y
			dMat3GetCol(m_mBoxRot,1,vTemp1);
			dVector3Inv(vTemp1);
			dConstructPlane(vTemp1,m_vBoxHalfSize.get1(),plPlane);
			if(!dClipEdgeToPlane( m_vEp0, m_vEp1, plPlane )) 
			{ 
				return false; 
			}

			// plane 5 -z
			dMat3GetCol(m_mBoxRot,2,vTemp1);
			dVector3Inv(vTemp1);
			dConstructPlane(vTemp1,m_vBoxHalfSize.get2(),plPlane);
			if(!dClipEdgeToPlane( m_vEp0, m_vEp1, plPlane )) 
			{ 
				return false; 
			}

			// calculate depths for both contact points
			m_fDepth0 = m_fBestrb + dVector3Dot(m_vEp0, m_vNormal);
			m_fDepth1 = m_fBestrb + dVector3Dot(m_vEp1, m_vNormal);

			// clamp depths to 0
			if(m_fDepth0<0) 
			{
				m_fDepth0 = (0.0);
			}

			if(m_fDepth1<0) 
			{
				m_fDepth1 = (0.0);
			}

			// back transform edge points from box to absolute space
//			m_vEp0[0] += m_vBoxPos[0];
//			m_vEp0[1] += m_vBoxPos[1];
//			m_vEp0[2] += m_vBoxPos[2];
			m_vEp0.add( m_vBoxPos );

//			m_vEp1[0] += m_vBoxPos[0];
//			m_vEp1[1] += m_vBoxPos[1];
//			m_vEp1[2] += m_vBoxPos[2];
			m_vEp1.add( m_vBoxPos );

			//dContactGeom* Contact0 = SAFECONTACT(m_iFlags, m_gContact, m_nContacts, m_iSkip);
			DContactGeom Contact0 = m_gContact.getSafe(m_iFlags, m_nContacts);
			Contact0.depth = m_fDepth0;
			dVector3Copy(m_vNormal,Contact0.normal);
			dVector3Copy(m_vEp0,Contact0.pos);
			Contact0.g1 = m_gCylinder;
			Contact0.g2 = m_gBox;
			Contact0.side1 = -1;
			Contact0.side2 = -1;
			dVector3Inv(Contact0.normal);
			m_nContacts++;

			if (m_nContacts != (m_iFlags & DxGeom.NUMC_MASK))
			{
				//dContactGeom* Contact1 = SAFECONTACT(m_iFlags, m_gContact, m_nContacts, m_iSkip);
				DContactGeom Contact1 = m_gContact.getSafe(m_iFlags, m_nContacts);
				Contact1.depth = m_fDepth1;
				dVector3Copy(m_vNormal,Contact1.normal);
				dVector3Copy(m_vEp1,Contact1.pos);
				Contact1.g1 = m_gCylinder;
				Contact1.g2 = m_gBox;
				Contact0.side1 = -1;
				Contact0.side2 = -1;
				dVector3Inv(Contact1.normal);
				m_nContacts++;
			}

			return true;
		}


		//void sCylinderBoxData::
		private void _cldClipBoxToCylinder() 
		{
			dIASSERT(m_nContacts != (m_iFlags & DxGeom.NUMC_MASK));

			DVector3 vCylinderCirclePos = new DVector3(), vCylinderCircleNormal_Rel = new DVector3();
			// check which circle from cylinder we take for clipping
			if ( dVector3Dot(m_vCylinderAxis, m_vNormal) > (0.0) ) 
			{
				// get top circle
//				vCylinderCirclePos[0] = m_vCylinderPos[0] + m_vCylinderAxis[0]*(m_fCylinderSize*(0.5));
//				vCylinderCirclePos[1] = m_vCylinderPos[1] + m_vCylinderAxis[1]*(m_fCylinderSize*(0.5));
//				vCylinderCirclePos[2] = m_vCylinderPos[2] + m_vCylinderAxis[2]*(m_fCylinderSize*(0.5));
				vCylinderCirclePos.eqSum( m_vCylinderPos, m_vCylinderAxis, m_fCylinderSize*0.5);

//				vCylinderCircleNormal_Rel[0] = (0.0);
//				vCylinderCircleNormal_Rel[1] = (0.0);
//				vCylinderCircleNormal_Rel[2] = (0.0);
//				vCylinderCircleNormal_Rel[nCYLINDER_AXIS] = (-1.0);
				//vCylinderCircleNormal_Rel.setValues(0);
				vCylinderCircleNormal_Rel.set( nCYLINDER_AXIS, -1.0 );
			}
			else 
			{
				// get bottom circle
//				vCylinderCirclePos[0] = m_vCylinderPos[0] - m_vCylinderAxis[0]*(m_fCylinderSize*(0.5));
//				vCylinderCirclePos[1] = m_vCylinderPos[1] - m_vCylinderAxis[1]*(m_fCylinderSize*(0.5));
//				vCylinderCirclePos[2] = m_vCylinderPos[2] - m_vCylinderAxis[2]*(m_fCylinderSize*(0.5));
				vCylinderCirclePos.eqSum( m_vCylinderPos, m_vCylinderAxis, -m_fCylinderSize*0.5);

//				vCylinderCircleNormal_Rel[0] = (0.0);
//				vCylinderCircleNormal_Rel[1] = (0.0);
//				vCylinderCircleNormal_Rel[2] = (0.0);
//				vCylinderCircleNormal_Rel[nCYLINDER_AXIS] = (1.0);
				vCylinderCircleNormal_Rel.set(nCYLINDER_AXIS,  1.0);
			}

			// vNr is normal in Box frame, pointing from Cylinder to Box
			DVector3 vNr = new DVector3();
			DMatrix3 mBoxInv = new DMatrix3();

			// Find a way to use quaternion
			dMatrix3Inv(m_mBoxRot,mBoxInv);
			dMultiplyMat3Vec3(mBoxInv,m_vNormal,vNr);

			DVector3 vAbsNormal;
//			vAbsNormal[0] = dFabs( vNr[0] );
//			vAbsNormal[1] = dFabs( vNr[1] );
//			vAbsNormal[2] = dFabs( vNr[2] );
			vAbsNormal = new DVector3(vNr);
			vAbsNormal.eqAbs();

			// find which face in box is closest to cylinder
			int iB0, iB1, iB2;

			// Different from Croteam's code
			if (vAbsNormal.get1() > vAbsNormal.get0()) 
			{
				// 1 > 0
				if (vAbsNormal.get0()> vAbsNormal.get2()) 
				{
					// 0 > 2 -> 1 > 0 >2
					iB0 = 1; iB1 = 0; iB2 = 2;
				} 
				else 
				{
					// 2 > 0-> Must compare 1 and 2
					if (vAbsNormal.get1() > vAbsNormal.get2())
					{
						// 1 > 2 -> 1 > 2 > 0
						iB0 = 1; iB1 = 2; iB2 = 0;
					}
					else
					{
						// 2 > 1 -> 2 > 1 > 0;
						iB0 = 2; iB1 = 1; iB2 = 0;
					}			
				}
			} 
			else 
			{
				// 0 > 1
				if (vAbsNormal.get1() > vAbsNormal.get2()) 
				{
					// 1 > 2 -> 0 > 1 > 2
					iB0 = 0; iB1 = 1; iB2 = 2;
				}
				else 
				{
					// 2 > 1 -> Must compare 0 and 2
					if (vAbsNormal.get0() > vAbsNormal.get2())
					{
						// 0 > 2 -> 0 > 2 > 1;
						iB0 = 0; iB1 = 2; iB2 = 1;
					}
					else
					{
						// 2 > 0 -> 2 > 0 > 1;
						iB0 = 2; iB1 = 0; iB2 = 1;
					}		
				}
			}

			DVector3 vCenter = new DVector3();
			// find center of box polygon
			DVector3 vTemp = new DVector3();
			if (vNr.get(iB0) > 0) 
			{
				dMat3GetCol(m_mBoxRot,iB0,vTemp);
//				vCenter[0] = m_vBoxPos[0] - m_vBoxHalfSize[iB0]*vTemp[0];
//				vCenter[1] = m_vBoxPos[1] - m_vBoxHalfSize[iB0]*vTemp[1];
//				vCenter[2] = m_vBoxPos[2] - m_vBoxHalfSize[iB0]*vTemp[2];
				vCenter.eqSum( m_vBoxPos, vTemp, -m_vBoxHalfSize.get(iB0) );
			}
			else 
			{
				dMat3GetCol(m_mBoxRot,iB0,vTemp);
//				vCenter[0] = m_vBoxPos[0] + m_vBoxHalfSize[iB0]*vTemp[0];
//				vCenter[1] = m_vBoxPos[1] + m_vBoxHalfSize[iB0]*vTemp[1];
//				vCenter[2] = m_vBoxPos[2] + m_vBoxHalfSize[iB0]*vTemp[2];
				vCenter.eqSum( m_vBoxPos, vTemp, m_vBoxHalfSize.get(iB0) );
			}

			// find the vertices of box polygon
			DVector3[] avPoints = DVector3.newArray(4);
			DVector3[] avTempArray1 = DVector3.newArray(MAX_CYLBOX_CLIP_POINTS);
			DVector3[] avTempArray2 = DVector3.newArray(MAX_CYLBOX_CLIP_POINTS);

//			int i=0;
//			for(i=0; i<MAX_CYLBOX_CLIP_POINTS; i++) 
//			{
////				avTempArray1[i][0] = (0.0);
////				avTempArray1[i][1] = (0.0);
////				avTempArray1[i][2] = (0.0);
//				avTempArray1[i] = new DVector3();
//
////				avTempArray2[i][0] = (0.0);
////				avTempArray2[i][1] = (0.0);
////				avTempArray2[i][2] = (0.0);
//				avTempArray2[i] = new DVector3();
//			}

			DVector3 vAxis1 = new DVector3(), vAxis2 = new DVector3();

			dMat3GetCol(m_mBoxRot,iB1,vAxis1);
			dMat3GetCol(m_mBoxRot,iB2,vAxis2);

//			avPoints[0][0] = vCenter[0] + m_vBoxHalfSize[iB1] * vAxis1[0] - m_vBoxHalfSize[iB2] * vAxis2[0];
//			avPoints[0][1] = vCenter[1] + m_vBoxHalfSize[iB1] * vAxis1[1] - m_vBoxHalfSize[iB2] * vAxis2[1];
//			avPoints[0][2] = vCenter[2] + m_vBoxHalfSize[iB1] * vAxis1[2] - m_vBoxHalfSize[iB2] * vAxis2[2];
			avPoints[0].eqSum( vAxis1, +m_vBoxHalfSize.get(iB1), vAxis2, -m_vBoxHalfSize.get(iB2) );
			avPoints[0].add( vCenter );

//			avPoints[1][0] = vCenter[0] - m_vBoxHalfSize[iB1] * vAxis1[0] - m_vBoxHalfSize[iB2] * vAxis2[0];
//			avPoints[1][1] = vCenter[1] - m_vBoxHalfSize[iB1] * vAxis1[1] - m_vBoxHalfSize[iB2] * vAxis2[1];
//			avPoints[1][2] = vCenter[2] - m_vBoxHalfSize[iB1] * vAxis1[2] - m_vBoxHalfSize[iB2] * vAxis2[2];
			avPoints[1].eqSum( vAxis1, -m_vBoxHalfSize.get(iB1), vAxis2, -m_vBoxHalfSize.get(iB2) );
			avPoints[1].add( vCenter );

//			avPoints[2][0] = vCenter[0] - m_vBoxHalfSize[iB1] * vAxis1[0] + m_vBoxHalfSize[iB2] * vAxis2[0];
//			avPoints[2][1] = vCenter[1] - m_vBoxHalfSize[iB1] * vAxis1[1] + m_vBoxHalfSize[iB2] * vAxis2[1];
//			avPoints[2][2] = vCenter[2] - m_vBoxHalfSize[iB1] * vAxis1[2] + m_vBoxHalfSize[iB2] * vAxis2[2];
			avPoints[2].eqSum( vAxis1, -m_vBoxHalfSize.get(iB1), vAxis2, +m_vBoxHalfSize.get(iB2) );
			avPoints[2].add( vCenter );

//			avPoints[3][0] = vCenter[0] + m_vBoxHalfSize[iB1] * vAxis1[0] + m_vBoxHalfSize[iB2] * vAxis2[0];
//			avPoints[3][1] = vCenter[1] + m_vBoxHalfSize[iB1] * vAxis1[1] + m_vBoxHalfSize[iB2] * vAxis2[1];
//			avPoints[3][2] = vCenter[2] + m_vBoxHalfSize[iB1] * vAxis1[2] + m_vBoxHalfSize[iB2] * vAxis2[2];
			avPoints[3].eqSum( vAxis1, +m_vBoxHalfSize.get(iB1), vAxis2, +m_vBoxHalfSize.get(iB2) );
			avPoints[3].add( vCenter );

			// transform box points to space of cylinder circle
			DMatrix3 mCylinderInv = new DMatrix3();
			dMatrix3Inv(m_mCylinderRot,mCylinderInv);

			for(int i=0; i<4; i++) 
			{
				dVector3Subtract(avPoints[i],vCylinderCirclePos,vTemp);
				dMultiplyMat3Vec3(mCylinderInv,vTemp,avPoints[i]);
			}

			int iTmpCounter1 = 0;
			int iTmpCounter2 = 0;
			DVector4 plPlane = new DVector4();

			// plane of cylinder that contains circle for intersection
			dConstructPlane(vCylinderCircleNormal_Rel,(0.0),plPlane);
			iTmpCounter1 = dClipPolyToPlane(avPoints, 4, avTempArray1, plPlane);


			// Body of base circle of Cylinder
			int nCircleSegment = 0;
			for (nCircleSegment = 0; nCircleSegment < nCYLINDER_SEGMENT; nCircleSegment++)
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

				dIASSERT( iTmpCounter1 >= 0 && iTmpCounter1 <= MAX_CYLBOX_CLIP_POINTS );
				dIASSERT( iTmpCounter2 >= 0 && iTmpCounter2 <= MAX_CYLBOX_CLIP_POINTS );
			}

			// back transform clipped points to absolute space
			double ftmpdot;	
			double fTempDepth;
			DVector3 vPoint = new DVector3();

			if (nCircleSegment % 2 != 0)
			{
				for(int i=0; i<iTmpCounter2; i++)
				{
					dMultiply0_331(vPoint,m_mCylinderRot,avTempArray2[i]);
//					vPoint[0] += vCylinderCirclePos[0];
//					vPoint[1] += vCylinderCirclePos[1];
//					vPoint[2] += vCylinderCirclePos[2];
					vPoint.add( vCylinderCirclePos );

					dVector3Subtract(vPoint,m_vCylinderPos,vTemp);
					ftmpdot	 = dVector3Dot(vTemp, m_vNormal);
					fTempDepth = m_fBestrc - ftmpdot;
					// Depth must be positive
					if (fTempDepth > (0.0))
					{
						// generate contacts
						//dContactGeom* Contact0 = SAFECONTACT(m_iFlags, m_gContact, m_nContacts, m_iSkip);
						DContactGeom Contact0 = m_gContact.getSafe(m_iFlags, m_nContacts);
						Contact0.depth = fTempDepth;
						dVector3Copy(m_vNormal,Contact0.normal);
						dVector3Copy(vPoint,Contact0.pos);
						Contact0.g1 = m_gCylinder;
						Contact0.g2 = m_gBox;
						Contact0.side1 = -1;
						Contact0.side2 = -1;
						dVector3Inv(Contact0.normal);
						m_nContacts++;

						if (m_nContacts == (m_iFlags & DxGeom.NUMC_MASK))
						{
							break;
						}
					}
				}
			}
			else
			{
				for(int i=0; i<iTmpCounter1; i++)
				{
					dMultiply0_331(vPoint,m_mCylinderRot,avTempArray1[i]);
//					vPoint[0] += vCylinderCirclePos[0];
//					vPoint[1] += vCylinderCirclePos[1];
//					vPoint[2] += vCylinderCirclePos[2];
					vPoint.add( vCylinderCirclePos );

					dVector3Subtract(vPoint,m_vCylinderPos,vTemp);
					ftmpdot	 = dVector3Dot(vTemp, m_vNormal);
					fTempDepth = m_fBestrc - ftmpdot;
					// Depth must be positive
					if (fTempDepth > (0.0))
					{
						// generate contacts
						//dContactGeom* Contact0 = SAFECONTACT(m_iFlags, m_gContact, m_nContacts, m_iSkip);
						DContactGeom Contact0 = m_gContact.getSafe(m_iFlags, m_nContacts);
						Contact0.depth = fTempDepth;
						dVector3Copy(m_vNormal,Contact0.normal);
						dVector3Copy(vPoint,Contact0.pos);
						Contact0.g1 = m_gCylinder;
						Contact0.g2 = m_gBox;
						Contact0.side1 = -1;
						Contact0.side2 = -1;
						dVector3Inv(Contact0.normal);
						m_nContacts++;

						if (m_nContacts == (m_iFlags & DxGeom.NUMC_MASK))
						{
							break;
						}
					}
				}
			}
		}

		//int sCylinderBoxData::
		private int PerformCollisionChecking()
		{
			// initialize collider
			_cldInitCylinderBox();

			// do intersection test and find best separating axis
			//if ( !_cldTestSeparatingAxes() ) 
			if ( 0==_cldTestSeparatingAxes() )
			{
				// if not found do nothing
				return 0;
			}

			// if best separation axis is not found
			if ( m_iBestAxis == 0 ) 
			{
				// this should not happen (we should already exit in that case)
				dIASSERT(false);
				// do nothing
				return 0;
			}

			double fdot = dVector3Dot(m_vNormal,m_vCylinderAxis);
			// choose which clipping method are we going to apply
			if (dFabs(fdot) < (0.9) ) 
			{
				// clip cylinder over box
				if(!_cldClipCylinderToBox()) 
				{
					return 0;
				}
			} 
			else 
			{
				_cldClipBoxToCylinder();  
			}

			return m_nContacts;
		}
	}
	
	// Cylinder - Box by CroTeam
	// Ported by Nguyen Binh
	private int dCollideCylinderBox(DxCylinder o1, DxBox o2, int flags, DContactGeomBuffer contacts, int skip)
	{
		dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
		//	dIASSERT (o1.type == dCylinderClass);
		//	dIASSERT (o2.type == dBoxClass);
		dIASSERT ((flags & DxGeom.NUMC_MASK) >= 1);

		sCylinderBoxData cData = new sCylinderBoxData(o1, o2, flags, contacts, skip);

		return cData.PerformCollisionChecking();
	}

	
	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideCylinderBox((DxCylinder)o1, (DxBox)o2, flags, contacts, 1);
	}
}