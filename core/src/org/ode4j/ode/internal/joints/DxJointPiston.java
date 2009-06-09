/** ***********************************************************************
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
 ************************************************************************ */
package org.ode4j.ode.internal.joints;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DPistonJoint;
import org.ode4j.ode.internal.DxWorld;
import static org.ode4j.ode.OdeMath.*;


/**
 * ****************************************************************************
 * Piston
 *
 * ****************************************************************************
 * Component of a Piston joint
 * <PRE>
 *                              |- Anchor point
 *      Body_1                  |                       Body_2
 *      +---------------+       V                       +------------------+
 *     /               /|                             /                  /|
 *    /               / +       |--      ______      /                  / +
 *   /      x        /./........x.......(_____()..../         x        /.......> axis
 *  +---------------+ /         |--                +------------------+ /
 *  |               |/                             |                  |/
 *  +---------------+                              +------------------+
 *          |                                                 |
 *          |                                                 |
 *          |------------------> <----------------------------|
 *              anchor1                  anchor2
 *
 *
 * </PRE>
 *
 * When the prismatic joint as been elongated (i.e. dJointGetPistonPosition)
 * return a value >  0
 * <PRE>
 *                                   |- Anchor point
 *      Body_1                       |                       Body_2
 *      +---------------+            V                       +------------------+
 *     /               /|                                  /                  /|
 *    /               / +            |--      ______      /                  / +
 *   /      x        /./........_____x.......(_____()..../         x        /.......> axis
 *  +---------------+ /              |--                +------------------+ /
 *  |               |/                                  |                  |/
 *  +---------------+                                   +------------------+
 *          |                                                      |
 *          |                                                      |
 *          |----------------.>      <----------------------------|
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

	/// Anchor w.r.t first body.
	/// This is the same as the offset for the Slider joint
	/// @note To find the position of the anchor when the body 1 has moved
	///       you must add the position of the prismatic joint
	///       i.e anchor = R1 * anchor1 + dJointGetPistonPosition() * (R1 * axis1)
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
			dMULTIPLY0_331 ( q, node[0].body._posr.R, anchor1 );

			if ( node[1].body!=null )
			{
				DVector3 anchor2 = new DVector3();
				// get the anchor2 in global coordinates
				dMULTIPLY0_331 ( anchor2, node[1].body._posr.R, anchor2 );

//				q.v[0] = ( ( node[0].body._posr.pos.v[0] + q.v[0] ) -
//						( node[1].body._posr.pos.v[0] + anchor2.v[0] ) );
//				q.v[1] = ( ( node[0].body._posr.pos.v[1] + q.v[1] ) -
//						( node[1].body._posr.pos.v[1] + anchor2.v[1] ) );
//				q.v[2] = ( ( node[0].body._posr.pos.v[2] + q.v[2] ) -
//						( node[1].body._posr.pos.v[2] + anchor2.v[2] ) );
				q.eqSum( node[0].body._posr.pos, q );
				q.sub( node[1].body._posr.pos);
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
				q.eqSum( node[0].body._posr.pos, q ).sub( anchor2 ) ;

				if (( flags & dJOINT_REVERSE )!=0)
				{
//					q.v[0] = -q.v[0];
//					q.v[1] = -q.v[1];
//					q.v[2] = -q.v[2];
					q.scale(-1);
				}
			}

			// get axis in global coordinates
			DVector3 ax = new DVector3();
			dMULTIPLY0_331 ( ax, node[0].body._posr.R, axis1 );

			return dDOT ( ax, q );
		}

		dDEBUGMSG ( "The function always return 0 since no body are attached" );
		return 0;
	}


	public double dJointGetPistonPositionRate ( )
	{
		// get axis in global coordinates
		DVector3 ax = new DVector3();
		dMULTIPLY0_331 ( ax, node[0].body._posr.R, axis1 );

		// The linear velocity created by the rotation can be discarded since
		// the rotation is along the prismatic axis and this rotation don't create
		// linear velocity in the direction of the prismatic axis.
		if ( node[1].body!=null )
		{
			return ( dDOT ( ax, node[0].body.lvel ) -
					dDOT ( ax, node[1].body.lvel ) );
		}
		else
		{
			double rate = dDOT ( ax, node[0].body.lvel );
			return ( ((flags & dJOINT_REVERSE)!=0) ? -rate : rate);
		}
	}


	double dJointGetPistonAngle ( )
	{
		if ( node[0].body!=null )
		{
			double ang = getHingeAngle ( node[0].body, node[1].body, axis1,
					qrel );
			if (( flags & dJOINT_REVERSE )!=0)
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
			dMULTIPLY0_331 ( axis, node[0].body._posr.R, axis1 );
			double rate = dDOT ( axis, node[0].body.avel );
			if ( node[1].body!=null ) rate -= dDOT ( axis, node[1].body.avel );
			if (( flags & dJOINT_REVERSE )!=0) rate = - rate;
			return rate;
		}
		else return 0;
	}


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


	public void
	getInfo2 ( DxJoint.Info2 info )
	{
		final int s0 = 0;
		final int s1 = info.rowskip();
		final int s2 = 2 * s1, s3 = 3 * s1 /*, s4=4*s1*/;

		final double k = info.fps * info.erp;


		// Pull out pos and R for both bodies. also get the `connection'
		// vector pos2-pos1.

		DMatrix3 R1 = null;//TZ
		DMatrix3 R2 = null;//TZ
		DVector3 dist = new DVector3(); // Current position of body_1  w.r.t "anchor"
		// 2 bodies anchor is center of body 2
		// 1 bodies anchor is origin
		DVector3 lanchor2 = new DVector3(0, 0, 0);

		DVector3C pos1 = node[0].body._posr.pos;
		R1   = node[0].body._posr.R;

		if ( node[1].body!=null )
		{
			DVector3C pos2 = node[1].body._posr.pos;
			R2   = node[1].body._posr.R;

			dMULTIPLY0_331 ( lanchor2, R2, anchor2 );
//			dist.v[0] = lanchor2.v[0] + pos2[0] - pos1[0];
//			dist.v[1] = lanchor2.v[1] + pos2[1] - pos1[1];
//			dist.v[2] = lanchor2.v[2] + pos2[2] - pos1[2];
			dist.eqSum(lanchor2, pos2).sub(pos1);
		}
		else
		{
			// pos2 = 0; // N.B. We can do that to be safe but it is no necessary
			// R2 = 0;   // N.B. We can do that to be safe but it is no necessary
			// dist[i] = joint.anchor2[i] - pos1[ui];
			//TZ dOPE2 ( dist, OP.EQ , anchor2, OP.SUB, pos1 );
			//dOP ( dist.v, OP.SUB , anchor2.v, pos1 );
			dist.eqDiff(anchor2, pos1);
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
		DVector3 ax1 = new DVector3(), p = new DVector3(), q = new DVector3();
		dMULTIPLY0_331 ( ax1, node[0].body._posr.R, axis1 );

		// Find the 2 axis perpendicular to the rotoide axis.
		dPlaneSpace ( ax1, p, q );

		// LHS
		dOPE ( info._J, ( info.J1ap ) + s0, OP.EQ , p );
		dOPE ( info._J, ( info.J1ap ) + s1, OP.EQ , q );

		DVector3 b = new DVector3();
		if ( node[1].body !=null)
		{
			// LHS
			//  info.J2a[s0+i] = -p[i]
			dOPE ( info._J, ( info.J2ap ) + s0, OP.EQ_SUB, p );
			dOPE ( info._J, ( info.J2ap ) + s1, OP.EQ_SUB, q );


			// Some math for the RHS
			DVector3 ax2 = new DVector3();
			dMULTIPLY0_331 ( ax2, R2, axis2 );
			dCROSS ( b, OP.EQ , ax1, ax2 );
		}
		else
		{
			// Some math for the RHS
			dCROSS ( b, OP.EQ , ax1, axis2 );
		}

		// RHS
		info.setC(0, k * dDOT ( p, b ) );
		info.setC(1, k * dDOT ( q, b ) );

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

		dCROSS ( info._J, info.J1ap + s2, OP.EQ , dist, p );

		dCROSS ( info._J, ( info.J1ap ) + s3, OP.EQ , dist, q );

		dOPE ( info._J, ( info.J1lp ) + s2, OP.EQ , p );
		dOPE ( info._J, ( info.J1lp ) + s3, OP.EQ , q );

		if ( node[1].body!=null )
		{
			// q x anchor2 instead of anchor2 x q since we want the negative value
			dCROSS ( info._J, ( info.J2ap ) + s2, OP.EQ , p, lanchor2 );

			// The cross product is in reverse order since we want the negative value
			dCROSS ( info._J, ( info.J2ap ) + s3, OP.EQ , q, lanchor2 );

			// info.J2l[s2+i] = -p[i];
			dOPE ( info._J, ( info.J2lp ) + s2, OP.EQ_SUB, p );
			dOPE ( info._J, ( info.J2lp ) + s3, OP.EQ_SUB, q );
		}


		// We want to make correction for motion not in the line of the axis
		// We calculate the displacement w.r.t. the "anchor" pt.
		// i.e. Find the difference between the current position and the initial
		//      position along the constrained axies (i.e. axis p and q).
		// The bodies can move w.r.t each other only along the prismatic axis
		//
		// Compute the RHS of rows 2 and 3
		DVector3 err = new DVector3();
		dMULTIPLY0_331 ( err, R1, anchor1 );
		//TZ dOPE2 ( err, OP.EQ , dist, -,  err );
		//dOP ( err.v, OP.SUB, dist.v, err.v );
		err.eqDiff(dist, err);

		info.setC(2, k * dDOT ( p, err ) );
		info.setC(3, k * dDOT ( q, err ) );


		int row = 4 + limotP.addLimot ( this, info, 4, ax1, false );
		limotR.addLimot ( this, info, row, ax1, true );
	}

	void
	computeInitialRelativeRotation()
	{
		if ( node[0].body!=null )
		{
			if ( node[1].body!=null )
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

	public void dJointSetPistonAnchor ( DVector3C xyz )
	{
		setAnchors ( xyz, anchor1, anchor2 );
		computeInitialRelativeRotation();
	}

	void dJointSetPistonAnchorOffset (double x, double y, double z,
			double dx, double dy, double dz)
	{
		if ((flags & dJOINT_REVERSE)!=0)
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

		setAnchors ( new DVector3(x ,y, z), anchor1, anchor2);

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
		if (( flags & dJOINT_REVERSE )!=0)
			getAnchor2 ( result, anchor2 );
		else
			getAnchor ( result, anchor1 );
	}


	//void dJointGetPistonAnchor2 ( dJoint j, dVector3 result )
	void dJointGetPistonAnchor2 ( DVector3 result )
	{
		if (( flags & dJOINT_REVERSE )!=0)
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
			c.eqDiff(node[0].body._posr.pos, node[1].body._posr.pos).sub(dx, dy, dz);
		}
		else if ( node[0].body!=null )
		{
//			c.v[0] = node[0].body._posr.pos.v[0] - dx;
//			c.v[1] = node[0].body._posr.pos.v[1] - dy;
//			c.v[2] = node[0].body._posr.pos.v[2] - dz;
			c.set(node[0].body._posr.pos).sub(dx, dy, dz);
		}

		// Convert into frame of body 1
		dMULTIPLY1_331 ( anchor1, node[0].body._posr.R, c );
	}



	void dJointGetPistonAxis ( DVector3 result )
	{
		getAxis ( result, axis1 );
	}

	public void dJointSetPistonParam ( D_PARAM_NAMES_N parameter, double value )
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


	public double dJointGetPistonParam ( D_PARAM_NAMES_N parameter )
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
		if (( flags & dJOINT_REVERSE )!=0)
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
			dMULTIPLY0_331 ( c, node[0].body._posr.R, anchor1 );
			dCROSS ( ltd, OP.EQ , c, axis );
			node[0].body.dBodyAddTorque ( ltd );


			dMULTIPLY0_331 ( c, node[1].body._posr.R, anchor2 );
			dCROSS ( ltd, OP.EQ , c, axis );
			node[1].body.dBodyAddTorque ( ltd );
		}
	}


	// *********************************
	// API dPistinJoint
	// *********************************

	public void setAnchor (double x, double y, double z)
	{ dJointSetPistonAnchor (new DVector3(x, y, z)); }
	public void setAnchor (DVector3C a)
	{ dJointSetPistonAnchor (a); }
	public void getAnchor (DVector3 result)
	{ dJointGetPistonAnchor (result); }
	public void getAnchor2 (DVector3 result)
	{ dJointGetPistonAnchor2 (result); }

	public void setAxis (double x, double y, double z)
	{ dJointSetPistonAxis (x, y, z); }
	public void setAxis (DVector3C a)
	//TODO use dVector3
	{ dJointSetPistonAxis(a.get0(), a.get1(), a.get2()); }
	public void getAxis (DVector3 result)
	{ dJointGetPistonAxis (result); }

	public double getPosition()
	{ return dJointGetPistonPosition (); }
	public double getPositionRate()
	{ return dJointGetPistonPositionRate (); }

	public void setParam (D_PARAM_NAMES_N parameter, double value)
		  { dJointSetPistonParam (parameter, value); }
	public double getParam (D_PARAM_NAMES_N parameter)
		    { return dJointGetPistonParam (parameter); }

	
	public void addForce (double force)
	{ dJointAddPistonForce (force); }


	@Override
	public double getParamHiStop2() {
		return dJointGetPistonParam(D_PARAM_NAMES_N.dParamHiStop2);
	}


	@Override
	public double getParamLoStop2() {
		return dJointGetPistonParam(D_PARAM_NAMES_N.dParamLoStop2);
	}


	@Override
	public void setParamHiStop2(double d) {
		dJointSetPistonParam(D_PARAM_NAMES_N.dParamHiStop2, d);
	}


	@Override
	public void setParamLoStop2(double d) {
		dJointSetPistonParam(D_PARAM_NAMES_N.dParamLoStop2, d);
	}

}
