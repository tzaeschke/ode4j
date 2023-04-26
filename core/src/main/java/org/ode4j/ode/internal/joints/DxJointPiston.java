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
package org.ode4j.ode.internal.joints;

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.dDEBUGMSG;
import static org.ode4j.ode.internal.Rotation.dQMultiply1;

import org.ode4j.math.*;
import org.ode4j.ode.DMatrix;
import org.ode4j.ode.DPistonJoint;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;


/**
 * ****************************************************************************
 * Piston
 *
 * ****************************************************************************
 * Component of a Piston joint
 * <PRE>
 *                              |- Anchor point
 *      Body_1                  |                       Body_2
 *      +---------------+       V                      +------------------+
 *     /               /|                             /                  /|
 *    /               / +       |--      ______      /                  / +
 *   /      x        /./........x.......(_____()..../         x        /.......&gt; axis
 *  +---------------+ /         |--                +------------------+ /
 *  |               |/                             |                  |/
 *  +---------------+                              +------------------+
 *          |                                                 |
 *          |                                                 |
 *          |------------------&gt; &lt;----------------------------|
 *              anchor1                  anchor2
 *
 *
 * </PRE>
 *
 * When the prismatic joint as been elongated (i.e. dJointGetPistonPosition)
 * return a value &gt; 0
 * <PRE>
 *                                   |- Anchor point
 *      Body_1                       |                       Body_2
 *      +---------------+            V                      +------------------+
 *     /               /|                                  /                  /|
 *    /               / +            |--      ______      /                  / +
 *   /      x        /./........_____x.......(_____()..../         x        /.......&gt; axis
 *  +---------------+ /              |--                +------------------+ /
 *  |               |/                                  |                  |/
 *  +---------------+                                   +------------------+
 *          |                                                      |
 *          |                                                      |
 *          |----------------.&gt;      &lt;----------------------------|
 *              anchor1         |----|         anchor2
 *                                ^
 *                                |-- This is what dJointGetPistonPosition will
 *                                    return
 * </PRE>
 * ****************************************************************************
 */
public class DxJointPiston extends DxJoint implements DPistonJoint
{
	DVector3 axis1 = new DVector3();          ///< Axis of the prismatic and rotoide w.r.t first body
	DVector3 axis2 = new DVector3();          ///< Axis of the prismatic and rotoide w.r.t second body


	DQuaternion qrel = new DQuaternion();        ///< Initial relative rotation body1 -> body2

	/** Anchor w.r.t first body.
	 * This is the same as the offset for the Slider joint
	 * <p>NOTE: To find the position of the anchor when the body 1 has moved
	 *        you must add the position of the prismatic joint
	 *        i.e anchor = R1 * anchor1 + dJointGetPistonPosition() * (R1 * axis1)
	 */        
	DVector3 anchor1 = new DVector3();
	DVector3 anchor2 = new DVector3();        //< anchor w.r.t second body

	/// limit and motor information for the prismatic
	/// part of the joint
	public DxJointLimitMotor limotP = new DxJointLimitMotor();

	/// limit and motor information for the rotoide
	/// part of the joint
	public DxJointLimitMotor limotR = new DxJointLimitMotor();


	DxJointPiston ( DxWorld w ) 
	//dxJoint ( w )
	{
		super (w);
//		dSetZero ( axis1, 4 );
//		dSetZero ( axis2, 4 );

		axis1.set0( 1 );
		axis2.set0( 1 );

//		dSetZero ( qrel, 4 );
//
//		dSetZero ( anchor1, 4 );
//		dSetZero ( anchor2, 4 );

		limotP.init ( world );

		limotR.init ( world );
	}


	double dJointGetPistonPosition (  )
	{
		if ( node[0].body!=null )
		{
			DVector3 q = new DVector3();
			// get the anchor (or offset) in global coordinates
			dMultiply0_331 ( q, node[0].body.posr().R(), anchor1 );

			if ( node[1].body!=null )
			{
				DVector3 anchor2 = new DVector3();
				// get the anchor2 in global coordinates
				dMultiply0_331 ( anchor2, node[1].body.posr().R(), this.anchor2 );

//				q.v[0] = ( ( node[0].body._posr.pos.v[0] + q.v[0] ) -
//						( node[1].body._posr.pos.v[0] + anchor2.v[0] ) );
//				q.v[1] = ( ( node[0].body._posr.pos.v[1] + q.v[1] ) -
//						( node[1].body._posr.pos.v[1] + anchor2.v[1] ) );
//				q.v[2] = ( ( node[0].body._posr.pos.v[2] + q.v[2] ) -
//						( node[1].body._posr.pos.v[2] + anchor2.v[2] ) );
				q.eqSum( node[0].body.posr().pos(), q );
				q.sub( node[1].body.posr().pos());
				q.sub( anchor2 ) ;
			}
			else
			{
				// N.B. When there is no body 2 the joint->anchor2 is already in
				//      global coordinates
//				q.v[0] = ( ( node[0].body._posr.pos.v[0] + q.v[0] ) -
//						( anchor2.v[0] ) );
//				q.v[1] = ( ( node[0].body._posr.pos.v[1] + q.v[1] ) -
//						( anchor2.v[1] ) );
//				q.v[2] = ( ( node[0].body._posr.pos.v[2] + q.v[2] ) -
//				( anchor2.v[2] ) );
				q.eqSum( node[0].body.posr().pos(), q ).sub( anchor2 ) ;

				if (isFlagsReverse())
				{
//					q.v[0] = -q.v[0];
//					q.v[1] = -q.v[1];
//					q.v[2] = -q.v[2];
					q.scale(-1);
				}
			}

			// get axis in global coordinates
			DVector3 ax = new DVector3();
			dMultiply0_331 ( ax, node[0].body.posr().R(), axis1 );
			
			return dCalcVectorDot3 ( ax, q );
		}

		dDEBUGMSG ( "The function always return 0 since no body are attached" );
		return 0;
	}


	public double dJointGetPistonPositionRate ( )
	{
		// get axis in global coordinates
		DVector3 ax = new DVector3();
		dMultiply0_331 ( ax, node[0].body.posr().R(), axis1 );

		// The linear velocity created by the rotation can be discarded since
		// the rotation is along the prismatic axis and this rotation don't create
		// linear velocity in the direction of the prismatic axis.
		if ( node[1].body!=null )
		{
			return ( dCalcVectorDot3 ( ax, node[0].body.lvel ) -
					dCalcVectorDot3 ( ax, node[1].body.lvel ) );
		}
		else
		{
			double rate = dCalcVectorDot3 ( ax, node[0].body.lvel );
			return isFlagsReverse() ? -rate : rate;
		}
	}


	double dJointGetPistonAngle ( )
	{
		if ( node[0].body!=null )
		{
			double ang = getHingeAngle ( node[0].body, node[1].body, axis1,
					qrel );
			if ( isFlagsReverse() )
				return -ang;
			else
				return ang;
		}
		else return 0;
	}


	double dJointGetPistonAngleRate ( )
	{
		if ( node[0].body!=null )
		{
			DVector3 axis = new DVector3();
			dMultiply0_331 ( axis, node[0].body.posr().R(), axis1 );
			double rate = dCalcVectorDot3 ( axis, node[0].body.avel );
			if ( node[1].body!=null ) rate -= dCalcVectorDot3 ( axis, node[1].body.avel );
			if ( isFlagsReverse() ) rate = - rate;
			return rate;
		}
		else return 0;
	}


	@Override
	void getSureMaxInfo( SureMaxInfo info )
	{
	  info.max_m = 6;
	}


	@Override
	public void
	getInfo1 ( DxJoint.Info1 info )
	{
		info.setNub(4); // Number of unbound variables
		// The only bound variable is one linear displacement

		info.setM(4); // Default number of constraint row

		// see if we're at a joint limit.
		limotP.limit = 0;
		if ( ( limotP.lostop > -dInfinity || limotP.histop < dInfinity ) &&
				limotP.lostop <= limotP.histop )
		{
			// measure joint position
			double pos = dJointGetPistonPosition ();
			limotP.testRotationalLimit ( pos );     // N.B. The fucntion is ill named
		}

		// powered Piston or at limits needs an extra constraint row
		if ( limotP.limit!=0 || limotP.fmax > 0 ) info.incM();


		// see if we're at a joint limit.
		limotR.limit = 0;
		if ( ( limotR.lostop > -dInfinity || limotR.histop < dInfinity ) &&
				limotR.lostop <= limotR.histop )
		{
			// measure joint position
			double angle = getHingeAngle ( node[0].body, node[1].body, axis1,
					qrel );
			limotR.testRotationalLimit ( angle );
		}

		// powered Piston or at limits needs an extra constraint row
		if ( limotR.limit!=0 || limotR.fmax > 0 ) info.incM();

	}


	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		final double k = worldFPS * worldERP;


		// Pull out pos and R for both bodies. also get the `connection'
		// vector pos2-pos1.

		DVector3 dist = new DVector3(); // Current position of body_1  w.r.t "anchor"
		// 2 bodies anchor is center of body 2
		// 1 bodies anchor is origin
		DVector3 lanchor2 = new DVector3(0, 0, 0);

		DVector3C pos1 = node[0].body.posr().pos();
		DMatrix3C R1 = node[0].body.posr().R();
		DMatrix3C R2 = null;

		DxBody body1 = node[1].body;

		if (body1 != null) {
			DVector3C pos2 = body1.posr().pos();
			R2 = body1.posr().R();

			dMultiply0_331(lanchor2, R2, anchor2);
			//			dist.v[0] = lanchor2.v[0] + pos2[0] - pos1[0];
			//			dist.v[1] = lanchor2.v[1] + pos2[1] - pos1[1];
			//			dist.v[2] = lanchor2.v[2] + pos2[2] - pos1[2];
			dist.eqSum(lanchor2, pos2).sub(pos1);
		} else {
			// pos2 = 0; // N.B. We can do that to be safe but it is no necessary
			// R2 = 0;   // N.B. We can do that to be safe but it is no necessary
			if (isFlagsReverse()) {
				dSubtractVectors3(dist, pos1, anchor2); // Invert the value
			} else {
				dSubtractVectors3(dist, anchor2, pos1);
			}
		}

		// ======================================================================
		// Work on the angular part (i.e. row 0, 1)
		// Set the two orientation rows. The rotoide axis should be the only
		// unconstrained rotational axis, the angular velocity of the two bodies
		// perpendicular to the rotoide axis should be equal.
		// Thus the constraint equations are:
		//    p*w1 - p*w2 = 0
		//    q*w1 - q*w2 = 0
		// where p and q are unit vectors normal to the rotoide axis, and w1 and w2
		// are the angular velocity vectors of the two bodies.
		// Since the rotoide axis is the same as the prismatic axis.
		//
		//
		// Also, compute the right hand side (RHS) of the rotation constraint equation set.
		// The first 2 element will result in the relative angular velocity of the two
		// bodies along axis p and q. This is set to bring the rotoide back into alignment.
		// if `theta' is the angle between ax1 and ax2, we need an angular velocity
		// along u to cover angle erp*theta in one step :
		//   |angular_velocity| = angle/time = erp*theta / stepsize
		//                      = (erp*fps) * theta
		//    angular_velocity  = |angular_velocity| * u
		//                      = (erp*fps) * theta * u
		// where rotation along unit length axis u by theta brings body 2's frame
		//
		// if theta is smallish, sin(theta) ~= theta and cos(theta) ~= 1
		// where the quaternion of the relative rotation between the two bodies is
		//    quat = [cos(theta/2) sin(theta/2)*u]
		//    quat = [1 theta/2*u]
		//         => q[0] ~= 1
		//            2 * q[1+i] = theta * u[i]
		//
		// Since there is no constraint along the rotoide axis
		// only along p and q that we want the same angular velocity and need to reduce
		// the error
		DVector3 b = new DVector3(), ax1 = new DVector3(), p = new DVector3(), q = new DVector3();
		dMultiply0_331(ax1, node[0].body.posr().R(), axis1);

		// Find the 2 axis perpendicular to the rotoide axis.
		dPlaneSpace(ax1, p, q);

		// LHS
		dCopyVector3(J1A, J1Ofs + GI2__JA_MIN, p);

		if (body1 != null) {
			dCopyNegatedVector3(J2A, J2Ofs + GI2__JA_MIN, p);
		}

		dCopyVector3(J1A, J1Ofs + rowskip + GI2__JA_MIN, q);

		if (body1 != null) {
			dCopyNegatedVector3(J2A, J2Ofs + rowskip + GI2__JA_MIN, q);

			// Some math for the RHS
			DVector3 ax2 = new DVector3();
			dMultiply0_331(ax2, R2, axis2);
			dCalcVectorCross3(b, ax1, ax2);
		} else {
			// Some math for the RHS
			dCalcVectorCross3(b, ax1, axis2);
		}

		// RHS
		pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = k * dCalcVectorDot3(p, b);
		pairRhsCfmA[pairRhsCfmOfs + pairskip + GI2_RHS] = k * dCalcVectorDot3(q, b);

		// ======================================================================
		// Work on the linear part (i.e row 2,3)
		// p2 + R2 anchor2' = p1 + R1 dist'
		// v2 + w2 R2 anchor2' + R2 d(anchor2')/dt  = v1 + w1 R1 dist' + R1 d(dist')/dt
		// v2 + w2 x anchor2 = v1 + w1 x dist + v_p
		// v_p is speed of prismatic joint (i.e. elongation rate)
		// Since the constraints are perpendicular to v_p we have:
		// p . v_p = 0 and q . v_p = 0
		// Along p and q we have (since sliding along the prismatic axis is disregarded):
		// u . ( v2 + w2 x anchor2 = v1 + w1 x dist + v_p) ( where u is p or q )
		// Simplify
		// u . v2 + u. w2 x anchor2 = u . v1 + u . w1 x dist
		// or
		// u . v1 - u . v2 + u . w1 x dist - u2 . w2 x anchor2 = 0
		// using the fact that (a x b = - b x a)
		// u . v1 - u . v2 - u . dist x w1  + u . anchor2 x w2 = 0
		// With the help of the triple product:
		//   i.e.  a . b x c = b . c x a = c . a x b  or  a . b x c = a x b . c
		//   Ref: http://mathworld.wolfram.com/ScalarTripleProduct.html
		// u . v1 - u . v2 - u x dist . w1 + u x anchor2 . w2 = 0
		// u . v1 - u . v2 + dist x u . w1 - u x anchor2 . w2 = 0
		//
		// Coeff for 1er line of: J1l => p, J2l => -p
		// Coeff for 2er line of: J1l => q, J2l => -q
		// Coeff for 1er line of: J1a => dist x p, J2a => p x anchor2
		// Coeff for 2er line of: J1a => dist x q, J2a => q x anchor2

		int currRowSkip = 2 * rowskip;
		{
			dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JL_MIN, p);
			dCalcVectorCross3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, dist, p);

			if (body1 != null) {
				// info->J2l[s2+i] = -p[i];
				dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JL_MIN, p);
				// q x anchor2 instead of anchor2 x q since we want the negative value
				dCalcVectorCross3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, p, lanchor2);
			}
		}

		currRowSkip += rowskip;
		{
			dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JL_MIN, q);
			dCalcVectorCross3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, dist, q);

			if (body1 != null) {
				// info->J2l[s3+i] = -q[i];
				dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JL_MIN, q);
				// The cross product is in reverse order since we want the negative value
				dCalcVectorCross3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, q, lanchor2);
			}
		}

		// We want to make correction for motion not in the line of the axis
		// We calculate the displacement w.r.t. the "anchor" pt.
		// i.e. Find the difference between the current position and the initial
		//      position along the constrained axies (i.e. axis p and q).
		// The bodies can move w.r.t each other only along the prismatic axis
		//
		// Compute the RHS of rows 2 and 3
		DVector3 err = new DVector3();
		dMultiply0_331(err, R1, anchor1);
		dSubtractVectors3(err, dist, err);

		int currPairSkip = 2 * pairskip;
		{
			pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * dCalcVectorDot3(p, err);
		}

		currPairSkip += pairskip;
		{
			pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * dCalcVectorDot3(q, err);
		}

		currRowSkip += rowskip;
		currPairSkip += pairskip;

		if (body1 != null || (flags & dJOINT_REVERSE) == 0) {
			if (limotP.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax1, false)) {
				currRowSkip += rowskip;
				currPairSkip += pairskip;
			}
		} else {
			DVector3 rAx1 = new DVector3();
			dCopyNegatedVector3(rAx1, ax1);

			if (limotP.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, rAx1, false)) {
				currRowSkip += rowskip;
				currPairSkip += pairskip;
			}
		}

		limotR.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
				pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax1, true);
	}


	public void dJointSetPistonAnchor ( DVector3C xyz )
	{
		setAnchors ( xyz, anchor1, anchor2 );
		computeInitialRelativeRotation();
	}

	void dJointSetPistonAnchorOffset (DVector3C xyz,
			double dx, double dy, double dz)
	{
		if (isFlagsReverse())
		{
			dx = -dx;
			dy = -dy;
			dz = -dz;
		}

		if (node[0].body!=null)
		{
//			node[0].body._posr.pos.v[0] -= dx;
//			node[0].body._posr.pos.v[1] -= dy;
//			node[0].body._posr.pos.v[2] -= dz;
			node[0].body._posr.pos.sub(dx, dy, dz);
		}

		setAnchors ( xyz, anchor1, anchor2 );

		if (node[0].body!=null)
		{
//			node[0].body._posr.pos.v[0] += dx;
//			node[0].body._posr.pos.v[1] += dy;
//			node[0].body._posr.pos.v[2] += dz;
			node[0].body._posr.pos.add(dx, dy, dz);
		}

		computeInitialRelativeRotation();
	}



	//void dJointGetPistonAnchor ( dJoint j, dVector3 result )
	public void dJointGetPistonAnchor ( DVector3 result )
	{
		if ( isFlagsReverse() )
			getAnchor2 ( result, anchor2 );
		else
			getAnchor ( result, anchor1 );
	}


	//void dJointGetPistonAnchor2 ( dJoint j, dVector3 result )
	void dJointGetPistonAnchor2 ( DVector3 result )
	{
		if ( isFlagsReverse() )
			getAnchor ( result, anchor1 );
		else
			getAnchor2 ( result, anchor2 );
	}



	void dJointSetPistonAxis ( double x, double y, double z )
	{
		setAxes ( x, y, z, axis1, axis2 );

		computeInitialRelativeRotation();
	}


	void dJointSetPistonAxisDelta ( double x, double y, double z,
			double dx, double dy, double dz )
	{
		setAxes ( x, y, z, axis1, axis2 );

		computeInitialRelativeRotation();

		DVector3 c = new DVector3(0,0,0);//= {0,0,0};
		if ( node[1].body!=null )
		{
//			c.v[0] = ( node[0].body._posr.pos.v[0] -
//					node[1].body._posr.pos.v[0] - dx );
//			c.v[1] = ( node[0].body._posr.pos.v[1] -
//					node[1].body._posr.pos.v[1] - dy );
//			c.v[2] = ( node[0].body._posr.pos.v[2] -
//					node[1].body._posr.pos.v[2] - dz );
			c.eqDiff(node[0].body.posr().pos(), node[1].body.posr().pos()).sub(dx, dy, dz);
		}
		else /*if ( node[0].body )*/ // -- body[0] should always be present -- there is a matrix multiplication below
		{
//			c.v[0] = node[0].body._posr.pos.v[0] - dx;
//			c.v[1] = node[0].body._posr.pos.v[1] - dy;
//			c.v[2] = node[0].body._posr.pos.v[2] - dz;
			c.set(node[0].body.posr().pos()).sub(dx, dy, dz);
		}

		// Convert into frame of body 1
		dMultiply1_331 ( anchor1, node[0].body.posr().R(), c );
	}



	void dJointGetPistonAxis ( DVector3 result )
	{
		getAxis ( result, axis1 );
	}

	public void dJointSetPistonParam ( PARAM_N parameter, double value )
	{
		if (  parameter.isGroup2())//and(0xff00).eq(0x100) )
		{
			limotR.set ( parameter.toSUB(), value );
		}
		else
		{
			limotP.set ( parameter.toSUB(), value );
		}
	}


	public double dJointGetPistonParam ( PARAM_N parameter )
	{
		if ( parameter.isGroup2())//and(0xff00).eq(0x100) )
		{
			return limotR.get ( parameter.toSUB() );
		}
		else
		{
			return limotP.get ( parameter.toSUB() );
		}
	}


	public void dJointAddPistonForce ( double force )
	{
		if ( isFlagsReverse() )
			force -= force;

		DVector3 axis = new DVector3();
		getAxis ( axis, axis1 );
		// axis[i] *= force
		//dOPEC ( axis.v, OP.MUL_EQ , force );
		axis.scale(force);


		if ( node[0].body != null )
			node[0].body.dBodyAddForce ( axis.get0(), axis.get1(), axis.get2() );
		if ( node[1].body != null )
			node[1].body.dBodyAddForce ( -axis.get0(), -axis.get1(), -axis.get2() );

		if ( node[0].body != null && node[1].body != null )
		{
			// Case where we don't need ltd since center of mass of both bodies
			// pass by the anchor point '*' when travelling along the prismatic axis.
			//                                     Body_2
			//   Body_1                             -----
			//    ---                |--           |     |
			//   |   |---------------*-------------|     |     ---> prismatic axis
			//    ---                |--           |     |
			//                                      -----
			//                                      Body_2
			// Case where we need ltd
			//   Body_1
			//    ---
			//   |   |---------
			//    ---          |
			//                 |     |--
			//                  -----*-----                    ---> prismatic axis
			//                       |--   |
			//                             |
			//                             |
			//                             |        -----
			//                             |       |     |
			//                              -------|     |
			//                                     |     |
			//                                      -----
			//                                      Body_2
			//
			// In real life force apply at the '*' point
			// But in ODE the force are applied on the center of mass of Body_1 and Body_2
			// So we have to add torques on both bodies to compensate for that when there
			// is an offset between the anchor point and the center of mass of both bodies.
			//
			// We need to add to each body T = r x F
			// Where r is the distance between the cm and '*'

			DVector3 ltd = new DVector3(); // Linear Torque Decoupling vector (a torque)
			DVector3 c = new DVector3();   // Distance of the body w.r.t the anchor
			// N.B. The distance along the prismatic axis might not
			//      not be included in this variable since it won't add
			//      anything to the ltd.

			// Calculate the distance of the body w.r.t the anchor

			// The anchor1 of body1 can be used since:
			// Real anchor = Position of body 1 + anchor + d* axis1 = anchor in world frame
			// d is the position of the prismatic joint (i.e. elongation)
			// Since axis1 x axis1 == 0
			// We can do the following.
			dMultiply0_331 ( c, node[0].body.posr().R(), anchor1 );
			dCalcVectorCross3( ltd, c, axis );
			node[0].body.dBodyAddTorque ( ltd );


			dMultiply0_331 ( c, node[1].body.posr().R(), anchor2 );
			dCalcVectorCross3 ( ltd, c, axis );
			node[1].body.dBodyAddTorque ( ltd );
		}
	}


	@Override
	void setRelativeValues()
	{
	    DVector3 vec = new DVector3();
	    dJointGetPistonAnchor(vec);
	    setAnchors( vec, anchor1, anchor2 );

	    dJointGetPistonAxis(vec);
	    setAxes( vec, axis1, axis2 );

	    computeInitialRelativeRotation();
	}




	private void computeInitialRelativeRotation()
	{
	    if ( node[0].body != null )
	    {
	        if ( node[1].body != null )
	        {
	            dQMultiply1 ( qrel, node[0].body._q, node[1].body._q );
	        }
	        else
	        {
	            // set joint->qrel to the transpose of the first body q
	            qrel.set0( node[0].body._q.get0() );
	            for ( int i = 1; i < 4; i++ )
	                qrel.set(i, -node[0].body._q.get(i) );
	            // WARNING do we need the - in -joint->node[0].body->q[i]; or not
	        }
	    }
	}

	
	// *********************************
	// API dPistinJoint
	// *********************************

	@Override
	public void setAnchor (double x, double y, double z)
	{ dJointSetPistonAnchor (new DVector3(x, y, z)); }
	@Override
	public void setAnchor (DVector3C a)
	{ dJointSetPistonAnchor (a); }
	@Override
	public void getAnchor (DVector3 result)
	{ dJointGetPistonAnchor (result); }
	@Override
	public void getAnchor2 (DVector3 result)
	{ dJointGetPistonAnchor2 (result); }

	@Override
	public void setAxis (double x, double y, double z)
	{ dJointSetPistonAxis (x, y, z); }
	@Override
	public void setAxis (DVector3C a)
	//TODO use dVector3
	{ dJointSetPistonAxis(a.get0(), a.get1(), a.get2()); }
	@Override
	public void getAxis (DVector3 result)
	{ dJointGetPistonAxis (result); }

	@Override
	public double getPosition()
	{ return dJointGetPistonPosition (); }
	@Override
	public double getPositionRate()
	{ return dJointGetPistonPositionRate (); }

	@Override
	public void setParam (PARAM_N parameter, double value)
		  { dJointSetPistonParam (parameter, value); }
	@Override
	public double getParam (PARAM_N parameter)
		    { return dJointGetPistonParam (parameter); }

	
	@Override
	public void addForce (double force)
	{ dJointAddPistonForce (force); }


	@Override
	public double getParamHiStop2() {
		return dJointGetPistonParam(PARAM_N.dParamHiStop2);
	}


	@Override
	public double getParamLoStop2() {
		return dJointGetPistonParam(PARAM_N.dParamLoStop2);
	}


	@Override
	public void setParamHiStop2(double d) {
		dJointSetPistonParam(PARAM_N.dParamHiStop2, d);
	}


	@Override
	public void setParamLoStop2(double d) {
		dJointSetPistonParam(PARAM_N.dParamLoStop2, d);
	}


	@Override
	public double getAngle() {
		return dJointGetPistonAngle();
	}


	@Override
	public double getAngleRate() {
		return dJointGetPistonAngleRate();
	}


	@Override
	public void setAnchorOffset(DVector3C xyz, double dx,
			double dy, double dz) {
		dJointSetPistonAnchorOffset(xyz, dx, dy, dz);
	}

}
