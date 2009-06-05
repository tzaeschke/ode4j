/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal.joints;

import org.cpp4j.java.RefDouble;

import static org.ode4j.ode.OdeMath.*;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DPUJoint;
import org.ode4j.ode.internal.DxWorld;


/**
 * ****************************************************************************
 * Prismatic and Universal.
 * 
 * Component of a Prismatic -- Universal joint.
 * The axisP must be perpendicular to axis1.
 * The second axis of the universal joint is perpendicular to axis1.
 *
 * Since the PU joint is derived from the Universal joint. Some variable
 * are reused.
 *
 * anchor1: Vector from body1 to the anchor point
 *          This vector is calculated when the body are attached or
 *          when the anchor point is set. It is like the offset of the Slider
 *          joint. Since their is a prismatic between the anchor and the body1
 *          the distance might change as the simulation goes on.
 * anchor2: Vector from body2 to the anchor point.
 * <PRE>
 *                                                 Body 2
 *                                                 +-------------+
 *                                                 |      x      |
 *                                                 +------------\+
 *          Prismatic articulation                   ..     ..
 *                                |                ..     ..
 *          Body 1                v             ..      ..
 *          +--------------+    --|        __..      ..  anchor2
 * <--------|      x       | .....|.......(__)     ..
 * axisP    +--------------+    --|         ^     <
 *                 |----------------------->|
 *                     anchor1              |--- Universal articulation
 *                                               axis1 going out of the plane
 *                                               axis2 is perpendicular to axis1
 *                                               (i.e. 2 rotoides)
 * </PRE>
 */
public class DxJointPU extends DxJointUniversal implements DPUJoint
{
	DVector3 axisP1 = new DVector3();    ///< @brief Axis for the prismatic articulation w.r.t first body.
	///< @note This is considered as axis2 from the parameter
	///< view
	public DxJointLimitMotor limotP = new DxJointLimitMotor(); 
	///< limit and motor information for the prismatic articulation.


	DxJointPU( DxWorld w ) 
	//dxJointUniversal( w )
	{
		super(w);
		// Default Position
		//               Y                ^ Axis2
		//              ^                 |
		//             /                  |     ^ Axis1
		// Z^         /                   |    /
		//  |        / Body 2             |   /         Body 1
		//  |       /  +---------+        |  /          +-----------+
		//  |      /  /         /|        | /          /           /|
		//  |     /  /         / +        _/     -    /           / +
		//  |    /  /         /-/--------(_)----|--- /-----------/-------> AxisP
		//  |   /  +---------+ /                 -  +-----------+ /
		//  |  /   |         |/                     |           |/
		//  | /    +---------+                      +-----------+
		//  |/
		//  .-----------------------------------------> X
		//             |----------------->
		//             Anchor2           <--------------|
		//                               Anchor1
		//

		// Setting member variables which are w.r.t body2
		_axis1.dSetZero();//dSetZero( _axis1, 4 );
		_axis1.v[1] = 1;

		// Setting member variables which are w.r.t body2
		_anchor2.dSetZero();//dSetZero( _anchor2, 4 );
		_axis2.dSetZero();//dSetZero( _axis2, 4 );
		_axis2.v[2] = 1;

		axisP1.dSetZero();//dSetZero( axisP1, 4 );
		axisP1.v[0] = 1;

		qrel1.dSetZero();//dSetZero( qrel1, 4 );
		qrel2.dSetZero();//dSetZero( qrel2, 4 );


		limotP.init( world );
		limot1.init( world );
		limot2.init( world );
	}


	double dJointGetPUPosition( )
	{
		//		dxJointPU joint = ( dxJointPU ) j;
		//		dUASSERT( joint, "bad joint argument" );
		//		checktype( joint, dxJointPU.class );

		DVector3 q = new DVector3();
		// get the offset in global coordinates
		dMULTIPLY0_331( q, node[0].body._posr.R, _anchor1 );

		if ( node[1].body!=null )
		{
			DVector3 anchor2 = new DVector3();

			// get the anchor2 in global coordinates
			dMULTIPLY0_331( anchor2, node[1].body._posr.R, _anchor2 );

			q.v[0] = (( node[0].body._posr.pos.v[0] + q.v[0] ) -
					( node[1].body._posr.pos.v[0] + anchor2.v[0] ) );
			q.v[1] = (( node[0].body._posr.pos.v[1] + q.v[1] ) -
					( node[1].body._posr.pos.v[1] + anchor2.v[1] ) );
			q.v[2] = (( node[0].body._posr.pos.v[2] + q.v[2] ) -
					( node[1].body._posr.pos.v[2] + anchor2.v[2] ) );
		}
		else
		{
			//N.B. When there is no body 2 the joint->anchor2 is already in
			//     global coordinates

			q.v[0] = (( node[0].body._posr.pos.v[0] + q.v[0] ) -
					( _anchor2.v[0] ) );
			q.v[1] = (( node[0].body._posr.pos.v[1] + q.v[1] ) -
					( _anchor2.v[1] ) );
			q.v[2] = (( node[0].body._posr.pos.v[2] + q.v[2] ) -
					( _anchor2.v[2] ) );

		}

		DVector3 axP = new DVector3();
		// get prismatic axis in global coordinates
		dMULTIPLY0_331( axP, node[0].body._posr.R, axisP1 );

		return dDOT( axP, q );
	}


	public double dJointGetPUPositionRate()
	{
		//    dxJointPU joint = ( dxJointPU ) j;
		//    dUASSERT( joint, "bad joint argument" );
		//    checktype( joint, dxJointPU.class );

		if ( node[0].body!=null )
		{
			// We want to find the rate of change of the prismatic part of the joint
			// We can find it by looking at the speed difference between body1 and the
			// anchor point.

			// r will be used to find the distance between body1 and the anchor point
			DVector3 r = new DVector3();
			DVector3 anchor2 = new DVector3(0,0,0);
			if ( node[1].body!=null )
			{
				// Find joint->anchor2 in global coordinates
				dMULTIPLY0_331( anchor2, node[1].body._posr.R, _anchor2 );

				//				r.v[0] = ( node[0].body._posr.pos.v[0] -
				//						( anchor2.v[0] + node[1].body._posr.pos.v[0] ) );
				//				r.v[1] = ( node[0].body._posr.pos.v[1] -
				//						( anchor2.v[1] + node[1].body._posr.pos.v[1] ) );
				//				r.v[2] = ( node[0].body._posr.pos.v[2] -
				//						( anchor2.v[2] + node[1].body._posr.pos.v[2] ) );
				r.eqDiff(node[0].body._posr.pos, node[1].body._posr.pos).sub(anchor2);
			}
			else
			{
				//N.B. When there is no body 2 the joint->anchor2 is already in
				//     global coordinates
				// r = joint->node[0].body->posr.pos -  joint->anchor2;
				//dOP( r.v, OP.SUB, node[0].body._posr.pos.v, _anchor2.v );
				r.eqDiff(node[0].body._posr.pos, _anchor2);
			}

			// The body1 can have velocity coming from the rotation of
			// the rotoide axis. We need to remove this.

			// N.B. We do vel = r X w instead of vel = w x r to have vel negative
			//      since we want to remove it from the linear velocity of the body
			DVector3 lvel1 = new DVector3();
			dCROSS( lvel1, OP.EQ , r, node[0].body.avel );

			// lvel1 += joint->node[0].body->lvel;
			dOPE( lvel1.v, 0, OP.ADD_EQ , node[0].body.lvel.v );

			if ( node[1].body!=null )
			{
				// Find the contribution of the angular rotation to the linear speed
				// N.B. We do vel = r X w instead of vel = w x r to have vel negative
				//      since we want to remove it from the linear velocity of the body
				DVector3 lvel2 = new DVector3();
				dCROSS( lvel2, OP.EQ , anchor2, node[1].body.avel );

				// lvel1 -=  lvel2 + joint->node[1].body->lvel;
				dOPE2( lvel1.v, OP.SUB_EQ , lvel2.v, OP.ADD , node[1].body.lvel.v );
			}


			// Since we want rate of change along the prismatic axis
			// get axisP1 in global coordinates and get the component
			// along this axis only
			DVector3 axP1 = new DVector3();
			dMULTIPLY0_331( axP1, node[0].body._posr.R, axisP1 );
			return dDOT( axP1, lvel1 );
		}

		return 0.0;
	}



	public void
	getInfo1( DxJoint.Info1 info )
	{
		info.setM(3);
		info.setNub(3);

		// powered needs an extra constraint row

		// see if we're at a joint limit.
		limotP.limit = 0;
		if (( limotP.lostop > -dInfinity || limotP.histop < dInfinity ) &&
				limotP.lostop <= limotP.histop )
		{
			// measure joint position
			double pos = dJointGetPUPosition();
			limotP.testRotationalLimit( pos );  // N.B. The function is ill named
		}

		if ( limotP.limit!=0 || limotP.fmax > 0 ) info.incM();


		boolean limiting1 = ( limot1.lostop >= -M_PI || limot1.histop <= M_PI ) &&
		limot1.lostop <= limot1.histop;
		boolean limiting2 = ( limot2.lostop >= -M_PI || limot2.histop <= M_PI ) &&
		limot2.lostop <= limot2.histop;

		// We need to call testRotationLimit() even if we're motored, since it
		// records the result.
		limot1.limit = 0;
		limot2.limit = 0;
		if ( limiting1 || limiting2 )
		{
			RefDouble angle1 = new RefDouble(0), angle2 = new RefDouble(0);
			getAngles( angle1, angle2 );
			if ( limiting1 )
				limot1.testRotationalLimit( angle1.get() );
			if ( limiting2 )
				limot2.testRotationalLimit( angle2.get() );
		}

		if ( limot1.limit!=0 || limot1.fmax > 0 ) info.incM();
		if ( limot2.limit!=0 || limot2.fmax > 0 ) info.incM();
	}



	public void
	getInfo2( DxJoint.Info2 info )
	{
		final int s0 = 0;
		final int s1 = info.rowskip();
		final int s2 = 2 * s1;

		final double k = info.fps * info.erp;

		// pull out pos and R for both bodies. also get the `connection'
		// vector pos2-pos1.

		//double *pos1, *pos2 = 0, *R1, *R2 = 0;
		DVector3 pos1 = new DVector3(), pos2 = null;
		DMatrix3 R1 = new DMatrix3(), R2 = null;
		pos1 = node[0].body._posr.pos;
		R1 = node[0].body._posr.R;
		if ( node[1].body!=null )
		{
			pos2 = node[1].body._posr.pos;
			R2 = node[1].body._posr.R;
		}

		DVector3 axP = new DVector3(); // Axis of the prismatic joint in global frame
		dMULTIPLY0_331( axP, R1, axisP1 );

		// distance between the body1 and the anchor2 in global frame
		// Calculated in the same way as the offset
		DVector3 dist = new DVector3();
		DVector3 wanchor2 = new DVector3(0,0,0);
		if ( node[1].body!=null )
		{
			dMULTIPLY0_331( wanchor2, R2, _anchor2 );
			dist.v[0] = wanchor2.v[0] + pos2.v[0] - pos1.v[0];
			dist.v[1] = wanchor2.v[1] + pos2.v[1] - pos1.v[1];
			dist.v[2] = wanchor2.v[2] + pos2.v[2] - pos1.v[2];
		}
		else
		{
			// dist[i] = joint->anchor2[i] - pos1[i];
			//TZ dOPE2( dist, OP.EQ , anchor2, OP.SUB, pos1 );
			dOP( dist.v, OP.SUB, _anchor2.v, pos1.v );
		}

		DVector3 q = new DVector3(); // Temporary axis vector
		// Will be used at 2 places with 2 different meaning

		// ======================================================================
		// Work on the angular part (i.e. row 0)
		//

		// The axis perpendicular to both axis1 and axis2 should be the only unconstrained
		// rotational axis, the angular velocity of the two bodies perpendicular to
		// the rotoide axes should be equal. Thus the constraint equations are
		//    p*w1 - p*w2 = 0
		// where p is a unit vector perpendicular to both axis1 and axis2
		// and w1 and w2 are the angular velocity vectors of the two bodies.
		DVector3 ax1 = new DVector3(), ax2 = new DVector3();
		getAxes( ax1, ax2 );
		double val = dDOT( ax1, ax2 );
		q.v[0] = ax2.v[0] - val * ax1.v[0];
		q.v[1] = ax2.v[1] - val * ax1.v[1];
		q.v[2] = ax2.v[2] - val * ax1.v[2];

		DVector3 p = new DVector3();
		dCROSS( p, OP.EQ , ax1, q );
		dNormalize3( p );

		//   info->J1a[s0+i] = p[i];
		dOPE(info._J, ( info.J1ap ) + s0, OP.EQ , p.v );

		if ( node[1].body!=null )
		{
			//   info->J2a[s0+i] = -p[i];
			dOPE(info._J, ( info.J2ap ) + s0, OP.EQ_SUB, p.v );
		}

		// compute the right hand side of the constraint equation. Set relative
		// body velocities along p to bring the axes back to perpendicular.
		// If ax1, ax2 are unit length joint axes as computed from body1 and
		// body2, we need to rotate both bodies along the axis p.  If theta
		// is the angle between ax1 and ax2, we need an angular velocity
		// along p to cover the angle erp * (theta - Pi/2) in one step:
		//
		//   |angular_velocity| = angle/time = erp*(theta - Pi/2) / stepsize
		//                      = (erp*fps) * (theta - Pi/2)
		//
		// if theta is close to Pi/2,
		// theta - Pi/2 ~= cos(theta), so
		//    |angular_velocity|  ~= (erp*fps) * (ax1 dot ax2)

		info.setC(0, k * - val );



		// ==========================================================================
		// Work on the linear part (i.e rows 1 and 2)
		//
		// We want: vel2 = vel1 + w1 x c ... but this would
		// result in three equations, so we project along the planespace vectors
		// so that sliding along the axisP is disregarded.
		//
		// p1 + R1 dist' = p2 + R2 anchor2'
		// v1 + w1 x R1 dist' + v_p = v2 + w2 x R2 anchor2'
		// v_p is speed of prismatic joint (i.e. elongation rate)
		// Since the constraints are perpendicular to v_p we have:
		// e1 dot v_p = 0 and e2 dot v_p = 0
		// e1 dot ( v1 + w1 x dist = v2 + w2 x anchor2 )
		// e2 dot ( v1 + w1 x dist = v2 + w2 x anchor2 )
		// ==
		// e1 . v1 + e1 . w1 x dist = e1 . v2 + e1 . w2 x anchor2
		// since a . (b x c) = - b . (a x c) = - (a x c) . b
		// and a x b = - b x a
		// e1 . v1 - e1 x dist . w1 - e1 . v2 - (- e1 x anchor2 . w2) = 0
		// e1 . v1 + dist x e1 . w1 - e1 . v2 - anchor2 x e1 . w2 = 0
		// Coeff for 1er line of: J1l => e1, J2l => -e1
		// Coeff for 2er line of: J1l => e2, J2l => -ax2
		// Coeff for 1er line of: J1a => dist x e1, J2a => - anchor2 x e1
		// Coeff for 2er line of: J1a => dist x e2, J2a => - anchor2 x e2
		// e1 and e2 are perpendicular to axP
		// so e1 = ax1 and e2 = ax1 x axP
		// N.B. ax2 is not always perpendicular to axP since it is attached to body 2
		dCROSS( q , OP.EQ , ax1, axP );

		dMULTIPLY0_331( axP, R1, axisP1 );

		dCROSS( info._J, ( info.J1ap ) + s1, OP.EQ , dist, ax1 );
		dCROSS(info._J, ( info.J1ap ) + s2, OP.EQ , dist, q );

		// info->J1l[s1+i] = ax[i];
		dOPE(info._J, ( info.J1lp ) + s1, OP.EQ , ax1.v );

		// info->J1l[s2+i] = q[i];
		dOPE(info._J, ( info.J1lp ) + s2, OP.EQ , q.v);

		if ( node[1].body!=null )
		{
			// Calculate anchor2 in world coordinate

			// q x anchor2 instead of anchor2 x q since we want the negative value
			dCROSS(info._J, ( info.J2ap ) + s1, OP.EQ , ax1, wanchor2 );
			// The cross product is in reverse order since we want the negative value
			dCROSS(info._J, ( info.J2ap ) + s2, OP.EQ , q, wanchor2 );


			// info->J2l[s1+i] = -ax1[i];
			dOPE(info._J, ( info.J2lp ) + s1, OP.EQ_SUB, ax1.v );
			// info->J2l[s2+i] = -ax1[i];
			dOPE(info._J, ( info.J2lp ) + s2, OP.EQ_SUB, q.v );

		}


		// We want to make correction for motion not in the line of the axisP
		// We calculate the displacement w.r.t. the anchor pt.
		//
		// compute the elements 1 and 2 of right hand side.
		// We want to align the offset point (in body 2's frame) with the center of body 1.
		// The position should be the same when we are not along the prismatic axis
		DVector3 err = new DVector3();
		dMULTIPLY0_331( err, R1, _anchor1 );
		// err[i] = dist[i] - err[i];
		//TZ dOPE2( err, OP.EQ , dist, OP.SUB, err );
		dOP( err.v, OP.SUB , dist.v, err.v );
		info.setC(1, k * dDOT( ax1, err ) );
		info.setC(2, k * dDOT( q, err ) );

		int row = 3 + limot1.addLimot( this, info, 3, ax1, true );
		row += limot2.addLimot( this, info, row, ax2, true );
		limotP.addLimot( this, info, row, axP, false );
	}

	public void dJointSetPUAnchor( double x, double y, double z )
	{
		dJointSetPUAnchor( new DVector3(x, y, z) );
	}
	public void dJointSetPUAnchor( DVector3C xyz )
	{
		//    dxJointPU joint = ( dxJointPU ) j;
		//    dUASSERT( joint, "bad joint argument" );
		//    checktype( joint, dxJointPU.class );
		setAnchors( xyz, _anchor1, _anchor2 );
		computeInitialRelativeRotations();
	}

	/**
	 * This function initialize the anchor and the relative position of each body
	 * as if body2 was at its current position + [dx,dy,dy].
	 * Ex:
	 * <PRE>
	 * dReal offset = 1;
	 * dVector3 dir;
	 * dJointGetPUAxis3(jId, dir);
	 * dJointSetPUAnchor(jId, 0, 0, 0);
	 * // If you request the position you will have: dJointGetPUPosition(jId) == 0
	 * dJointSetPUAnchorDelta(jId, 0, 0, 0, dir[X]*offset, dir[Y]*offset, dir[Z]*offset);
	 * // If you request the position you will have: dJointGetPUPosition(jId) == -offset
	 * </PRE>

	 * @param j The PU joint for which the anchor point will be set
	 * @param x The X position of the anchor point in world frame
	 * @param y The Y position of the anchor point in world frame
	 * @param z The Z position of the anchor point in world frame
	 * @param dx A delta to be added to the X position as if the anchor was set
	 *           when body1 was at current_position[X] + dx
	 * @param dx A delta to be added to the Y position as if the anchor was set
	 *           when body1 was at current_position[Y] + dy
	 * @param dx A delta to be added to the Z position as if the anchor was set
	 *           when body1 was at current_position[Z] + dz
	 * @note Should have the same meaning as dJointSetSliderAxisDelta
	 */
	void dJointSetPUAnchorDelta( double x, double y, double z,
			double dx, double dy, double dz )
	{
		//		dxJointPU joint = ( dxJointPU ) j;
		//		dUASSERT( joint, "bad joint argument" );
		//		checktype( joint, dxJointPU.class );

		if ( node[0].body!=null )
		{
			node[0].body._posr.pos.v[0] += dx;
			node[0].body._posr.pos.v[1] += dy;
			node[0].body._posr.pos.v[2] += dz;
		}

		setAnchors( new DVector3(x, y, z), _anchor1, _anchor2 );

		if ( node[0].body!=null )
		{
			node[0].body._posr.pos.v[0] -= dx;
			node[0].body._posr.pos.v[1] -= dy;
			node[0].body._posr.pos.v[2] -= dz;
		}

		computeInitialRelativeRotations();
	}



	public void dJointSetPUAxis1( double x, double y, double z )
	{
		//    dxJointPU joint = ( dxJointPU ) j;
		//    dUASSERT( joint, "bad joint argument" );
		//    checktype( joint, dxJointPU.class );
		if (( flags & dJOINT_REVERSE )!=0)
			setAxes( x, y, z, null, _axis2 );
		else
			setAxes( x, y, z, _axis1, null );
		computeInitialRelativeRotations();
	}

	public void dJointSetPUAxis2( double x, double y, double z )
	{
		//    dxJointPU joint = ( dxJointPU ) j;
		//    dUASSERT( joint, "bad joint argument" );
		//    checktype( joint, dxJointPU.class );
		if (( flags & dJOINT_REVERSE )!=0)
			setAxes( x, y, z, _axis1, null );
		else
			setAxes( x, y, z, null, _axis2 );
		computeInitialRelativeRotations();
	}


	public void dJointSetPUAxisP( double x, double y, double z )
	{
		dJointSetPUAxis3( x, y, z );
	}



	public void dJointSetPUAxis3( double x, double y, double z )
	{
		//    dxJointPU joint = ( dxJointPU ) j;
		//    dUASSERT( joint, "bad joint argument" );
		//    checktype( joint, dxJointPU.class );

		setAxes( x, y, z, axisP1, null );

		computeInitialRelativeRotations();
	}




	//void dJointGetPUAngles( dJoint j, double *angle1, double *angle2 )
	void dJointGetPUAngles( RefDouble angle1, RefDouble angle2 )
	{
		//		dxJointUniversal joint = ( dxJointUniversal ) j;
		//		dUASSERT( joint, "bad joint argument" );
		//		checktype( joint, dxJointPU.class );
		if (( flags & dJOINT_REVERSE )!=0)
			getAngles( angle2, angle1 );
		else
			getAngles( angle1, angle2 );
	}


	double dJointGetPUAngle1()
	{
		//		dxJointUniversal joint = ( dxJointUniversal ) j;
		//		dUASSERT( joint, "bad joint argument" );
		//		checktype( joint, dxJointPU.class );
		if (( flags & dJOINT_REVERSE )!=0)
			return getAngle2Internal();
		else
			return getAngle1Internal();
	}


	double dJointGetPUAngle2()
	{
		//		dxJointUniversal joint = ( dxJointUniversal ) j;
		//		dUASSERT( joint, "bad joint argument" );
		//		checktype( joint, dxJointPU.class );
		if (( flags & dJOINT_REVERSE )!=0)
			return getAngle1Internal();
		else
			return getAngle2Internal();
	}


	double dJointGetPUAngle1Rate()
	{
		//		dxJointPU joint = ( dxJointPU ) j;
		//		dUASSERT( joint, "bad joint argument" );
		//		checktype( joint, dxJointPU.class );

		if ( node[0].body!=null )
		{
			DVector3 axis = new DVector3();

			if (( flags & dJOINT_REVERSE )!=0)
				getAxis2( axis, _axis2 );
			else
				getAxis( axis, _axis1 );

			double rate = dDOT( axis, node[0].body.avel );
			if ( node[1].body!=null ) rate -= dDOT( axis, node[1].body.avel );
			return rate;
		}
		return 0;
	}


	double dJointGetPUAngle2Rate()
	{
		//		dxJointPU joint = ( dxJointPU ) j;
		//		dUASSERT( joint, "bad joint argument" );
		//		checktype( joint, dxJointPU.class );

		if ( node[0].body!=null )
		{
			DVector3 axis = new DVector3();

			if (( flags & dJOINT_REVERSE )!=0)
				getAxis( axis, _axis1 );
			else
				getAxis2( axis, _axis2 );

			double rate = dDOT( axis, node[0].body.avel );
			if ( node[1].body!=null ) rate -= dDOT( axis, node[1].body.avel );
			return rate;
		}
		return 0;
	}


	public void dJointSetPUParam( D_PARAM_NAMES_N parameter, double value )
	{
		//    dxJointPU joint = ( dxJointPU ) j;
		//    dUASSERT( joint, "bad joint argument" );
		//    checktype( joint, dxJointPU.class );

		switch ( parameter.toGROUP()) //.and( 0xff00 ))
		{
		case dParamGroup1:
			limot1.set( parameter.toSUB(), value );
			break;
		case dParamGroup2:
			limot2.set( parameter.toSUB(), value );//.and( 0xff), value );
			break;
		case dParamGroup3:
			limotP.set( parameter.toSUB(), value );//.and( 0xff ), value );
			break;
		default: //TODO TZ remove
			throw new IllegalArgumentException(parameter.name());
		}
	}

	//	void dJointGetPUAnchor( dJoint j, dVector3 result )
	public void dJointGetPUAnchor( DVector3 result )
	{
		//		dxJointPU joint = ( dxJointPU ) j;
		//		dUASSERT( joint, "bad joint argument" );
		//		dUASSERT( result, "bad result argument" );
		//		checktype( joint, dxJointPU.class );

		if ( node[1].body!=null )
			getAnchor2( result, _anchor2 );
		else
		{
			// result[i] = joint->anchor2[i];
			dOPE( result.v, 0, OP.EQ , _anchor2.v );
		}
	}

	void dJointGetPUAxis1( DVector3 result )
	{
		//		dxJointPU joint = ( dxJointPU ) j;
		//		dUASSERT( joint, "bad joint argument" );
		dUASSERT( result, "bad result argument" );
		//		checktype( joint, dxJointPU.class );
		getAxis( result, _axis1 );
	}

	void dJointGetPUAxis2( DVector3 result )
	{
		//		dxJointPU joint = ( dxJointPU ) j;
		//		dUASSERT( joint, "bad joint argument" );
		dUASSERT( result, "bad result argument" );
		//		checktype( joint, dxJointPU.class );
		getAxis( result, _axis2 );
	}

	/**
	 * @brief Get the prismatic axis
	 * @ingroup joints
	 *
	 * @note This function was added for convenience it is the same as
	 *       dJointGetPUAxis3
	 */
	void dJointGetPUAxisP( DVector3 result )
	{
		dJointGetPUAxis3( result );
	}


	void dJointGetPUAxis3( DVector3 result )
	{
		//		dxJointPU joint = ( dxJointPU ) j;
		//		dUASSERT( joint, "bad joint argument" );
		//		dUASSERT( result, "bad result argument" );
		//		checktype( joint, dxJointPU.class );
		getAxis( result, axisP1 );
	}

	public double dJointGetPUParam( D_PARAM_NAMES_N parameter )
	{
		//    dxJointPU joint = ( dxJointPU ) j;
		//    dUASSERT( joint, "bad joint argument" );
		//    checktype( joint, dxJointPU.class );

		switch ( parameter.toGROUP() )//and( 0xff00 ))
		{
		case dParamGroup1:
			return limot1.get( parameter.toSUB());// );
			//        break;
		case dParamGroup2:
			return limot2.get( parameter.toSUB());//and( 0xff ));
			//        break;
		case dParamGroup3:
			return limotP.get( parameter.toSUB());//and( 0xff ));
			//        break;
		default: 
			//TODO keep?
			throw new IllegalArgumentException(parameter.name());
		}

		//return 0;
	}


	// **********************************
	// API PUJoint
	// **********************************

	public final void setAnchor (double x, double y, double z)
	{ dJointSetPUAnchor (x, y, z); }
	public final void setAnchor (DVector3C a)
	{ dJointSetPUAnchor (a); }
	public final void setAxis1 (double x, double y, double z)
	{ dJointSetPUAxis1 (x, y, z); }
	public final void setAxis1 (DVector3C a)
	//TODO use dVector3
	{ setAxis1(a.get0(), a.get1(), a.get2()); }
	public final void setAxis2 (double x, double y, double z)
	{ dJointSetPUAxis2 (x, y, z); }
	public final void setAxis3 (double x, double y, double z)
	{ dJointSetPUAxis3 (x, y, z); }
	public final void setAxis3 (DVector3C a)
	//TODO use dVector3
	{ setAxis3(a.get0(), a.get1(), a.get2()); }
	public final void setAxisP (double x, double y, double z)
	{ dJointSetPUAxis3 (x, y, z); }
	public final void setAxisP (DVector3C a)
	//TODO use dVector3
	{ setAxisP(a.get0(), a.get1(), a.get2()); }

	//TZ not final: 'virtual'
	public void getAnchor (DVector3 result)
	{ dJointGetPUAnchor (result); }
	public final void getAxis1 (DVector3 result)
	{ dJointGetPUAxis1 (result); }
	public final void getAxis2 (DVector3 result)
	{ dJointGetPUAxis2 (result); }
	public final void getAxis3 (DVector3 result)
	{ dJointGetPUAxis3 (result); }
	public final void getAxisP (DVector3 result)
	{ dJointGetPUAxis3 (result); }

	/** TZ Take care to call getAngle1Internal() from dx-classes.*/
	public final double getAngle1()
	{ return dJointGetPUAngle1 (); }
	public final double getAngle1Rate()
	{ return dJointGetPUAngle1Rate (); }
	/** TZ Take care to call getAngle2Internal() from dx-classes.*/
	public final double getAngle2()
	{ return dJointGetPUAngle2 (); }
	public final double getAngle2Rate()
	{ return dJointGetPUAngle2Rate (); }

	public final double getPosition()
	{ return dJointGetPUPosition (); }
	public final double getPositionRate()
	{ return dJointGetPUPositionRate (); }

	public final void setParam (D_PARAM_NAMES_N parameter, double value)
	{ dJointSetPUParam (parameter, value); }
	public final double getParam (D_PARAM_NAMES_N parameter)
	{ return dJointGetPUParam (parameter); }
}

