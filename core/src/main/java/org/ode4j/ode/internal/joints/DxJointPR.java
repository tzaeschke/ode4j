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
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Rotation.dQMultiply1;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DPRJoint;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;


/**
 *  Prismatic and Rotoide.
 * 
 * The axisP must be perpendicular to axis2
 * <PRE>
 *                                        +-------------+
 *                                        |      x      |
 *                                        +------------\+
 * Prismatic articulation                   ..     ..
 *                       |                ..     ..
 *                      \/              ..      ..
 * +--------------+    --|        __..      ..  anchor2
 * |      x       | .....|.......(__)     ..
 * +--------------+    --|         ^     &lt;
 *        |-----------------------&gt;|
 *            Offset               |--- Rotoide articulation
 * </PRE>
 */
public class DxJointPR extends DxJoint implements DPRJoint
{

	/**
	 * Position of the rotoide articulation w.r.t second body.
	 * <p>NOTE: Position of body 2 in world frame + anchor2 in world frame give 
	 * the position of the rotoide articulation.
	 */
	DVector3 _anchor2;  
	
	/** 
	 * Axis of the rotoide articulation w.r.t first body.
	 * <p>NOTE: This is considered as axis1 from the parameter view.
	 */
	DVector3 axisR1;
	
	/** Axis of the rotoide articulation w.r.t second body.
	 * <p>NOTE: This is considered also as axis1 from the parameter view.
	 */
	DVector3 axisR2;
	
	/** Axis for the prismatic articulation w.r.t first body.
	 * <p>NOTE: This is considered as axis2 in from the parameter view.
	 */
	DVector3 axisP1;
	
	/** Initial relative rotation body1 -> body2. */ 
	DQuaternion qrel;
	
	/** 
	 * Vector between the body1 and the rotoide articulation.
	 *  
	 * Going from the first to the second in the frame of body1.
	 * That should be aligned with body1 center along axisP.
	 * This is calculated when the axis are set.
	 */
	DVector3 offset;
	
	/** limit and motor information for the rotoide articulation. */
	public DxJointLimitMotor limotR;
	
	/** limit and motor information for the prismatic articulation. */
	public DxJointLimitMotor limotP;


	DxJointPR( DxWorld w ) 
	//dxJoint( w )
	{
		super (w);
		// Default Position
		// Z^
		//  | Body 1       P      R          Body2
		//  |+---------+   _      _         +-----------+
		//  ||         |----|----(_)--------+           |
		//  |+---------+   -                +-----------+
		//  |
		// X.-----------------------------------------> Y
		// N.B. X is comming out of the page
		_anchor2 = new DVector3();

		axisR1 = new DVector3(1, 0, 0);
		axisR2 = new DVector3(1, 0, 0);

		axisP1 = new DVector3(0, 1, 0);
		qrel = new DQuaternion();
		offset = new DVector3();

		limotR = new DxJointLimitMotor();
		limotR.init( world );
		limotP = new DxJointLimitMotor();
		limotP.init( world );
	}


//	double dJointGetPRPosition( dJoint j )
	double dJointGetPRPosition()
	{
//		dxJointPR joint = ( dxJointPR ) j;
//		dUASSERT( joint, "bad joint argument" );
//		checktype( joint, dxJointPR.class );

		DVector3 q = new DVector3();
		// get the offset in global coordinates
		dMultiply0_331( q, node[0].body.posr().R(), offset );

		if ( node[1].body!=null )
		{
			DVector3 anchor2 = new DVector3();

			// get the anchor2 in global coordinates
			dMultiply0_331( anchor2, node[1].body.posr().R(), _anchor2 );

//			q.v[0] = (( node[0].body._posr.pos.v[0] + q.v[0] ) -
//					( node[1].body._posr.pos.v[0] + anchor2.v[0] ) );
//			q.v[1] = (( node[0].body._posr.pos.v[1] + q.v[1] ) -
//					( node[1].body._posr.pos.v[1] + anchor2.v[1] ) );
//			q.v[2] = (( node[0].body._posr.pos.v[2] + q.v[2] ) -
//					( node[1].body._posr.pos.v[2] + anchor2.v[2] ) );
			q.eqSum(node[0].body.posr().pos(), q).sub(node[1].body.posr().pos()).sub(anchor2);
		}
		else
		{
			//N.B. When there is no body 2 the joint->anchor2 is already in
			//     global coordinates

//			q.v[0] = (( node[0].body._posr.pos.v[0] + q.v[0] ) -
//					( _anchor2.v[0] ) );
//			q.v[1] = (( node[0].body._posr.pos.v[1] + q.v[1] ) -
//					( _anchor2.v[1] ) );
//			q.v[2] = (( node[0].body._posr.pos.v[2] + q.v[2] ) -
//					( _anchor2.v[2] ) );
			q.eqSum(node[0].body.posr().pos(), q).sub(_anchor2);

			if ( isFlagsReverse() ) 
			{
				q.scale( -1 );
	        }
		}

		DVector3 axP = new DVector3();
		// get prismatic axis in global coordinates
		dMultiply0_331( axP, node[0].body.posr().R(), axisP1 );

		return dCalcVectorDot3( axP, q );
	}

	public double dJointGetPRPositionRate( )
	{
		//    dxJointPR joint = ( dxJointPR ) j;
		//    dUASSERT( joint, "bad joint argument" );
		//    checktype( joint, dxJointPR.class );
		// get axis1 in global coordinates
		DVector3 ax1 = new DVector3();
		dMultiply0_331( ax1, node[0].body.posr().R(), axisP1 );

		if ( node[1].body!=null )
		{
			DVector3 lv2 = new DVector3();
			node[1].body.dBodyGetRelPointVel( _anchor2, lv2 );
			return dCalcVectorDot3( ax1, node[0].body.lvel ) - dCalcVectorDot3( ax1, lv2 );
		}
		else
	    {
	        double rate = ax1.dot( node[0].body.lvel );
	        return ( isFlagsReverse() ? -rate : rate);
	    }
	}


	double dJointGetPRAngle()
	{
//	    dxJointPR* joint = ( dxJointPR* )j;
//	    dAASSERT( joint );
//	    checktype( joint, PR );
	    if ( node[0].body != null )
	    {
	        double ang = getHingeAngle( node[0].body,
	                                   node[1].body,
	                                   axisR1,
	                                   qrel );
	        if ( isFlagsReverse() )
	            return -ang;
	        else
	            return ang;
	    }
	    else return 0;
	}



	double dJointGetPRAngleRate()
	{
//	    dxJointPR* joint = ( dxJointPR* )j;
//	    dAASSERT( joint );
//	    checktype( joint, PR );
	    if ( node[0].body != null )
	    {
	        DVector3 axis = new DVector3();
	        dMultiply0_331( axis, node[0].body.posr().R(), axisR1 );
	        double rate = dCalcVectorDot3( axis, node[0].body.avel );
	        if ( node[1].body != null ) rate -= dCalcVectorDot3( axis, node[1].body.avel );
	        if ( isFlagsReverse() ) rate = -rate;
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
	getInfo1( DxJoint.Info1 info )
	{
		info.setNub(4);
		info.setM(4);


		// see if we're at a joint limit.
		limotP.limit = 0;
		if (( limotP.lostop > -dInfinity || limotP.histop < dInfinity ) &&
				limotP.lostop <= limotP.histop )
		{
			// measure joint position
			double pos = dJointGetPRPosition();
			limotP.testRotationalLimit( pos );  // N.B. The function is ill named
		}

		// powered needs an extra constraint row
		if ( limotP.limit!=0 || limotP.fmax > 0 ) info.incM();


	    // see if we're at a joint limit.
	    limotR.limit = 0;
	    if (( limotR.lostop >= -M_PI || limotR.histop <= M_PI ) &&
	            limotR.lostop <= limotR.histop )
	    {
	        double angle = getHingeAngle( node[0].body,
	                                     node[1].body,
	                                     axisR1, qrel );
	        limotR.testRotationalLimit( angle );
	    }

	    // powered morit or at limits needs an extra constraint row
	    if ( limotR.limit != 0 || limotR.fmax > 0 ) info.incM();
	}


	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		double k = worldFPS * worldERP;


		DVector3 q = new DVector3();  // plane space of axP and after that axR

		// pull out pos and R for both bodies. also get the `connection'
		// vector pos2-pos1.

		DVector3C pos2 = null;
		DMatrix3C R2 = null;

		DVector3C pos1 = node[0].body.posr().pos();
		DMatrix3C R1 = node[0].body.posr().R();

		DxBody body1 = node[1].body;

		if (body1 != null) {
			pos2 = body1.posr().pos();
			R2 = body1.posr().R();
		}


		DVector3 axP = new DVector3(); // Axis of the prismatic joint in global frame
		dMultiply0_331( axP, R1, axisP1 );

		// distance between the body1 and the anchor2 in global frame
		// Calculated in the same way as the offset
		DVector3 wanchor2 = new DVector3(0,0,0), dist = new DVector3();

		if (body1 != null)
		{
			// Calculate anchor2 in world coordinate
			dMultiply0_331( wanchor2, R2, _anchor2 );
//			dist.v[0] = wanchor2.v[0] + pos2.v[0] - pos1.v[0];
//			dist.v[1] = wanchor2.v[1] + pos2.v[1] - pos1.v[1];
//			dist.v[2] = wanchor2.v[2] + pos2.v[2] - pos1.v[2];
			dist.eqSum(wanchor2, pos2).sub(pos1);
		}
		else
		{
			if (isFlagsReverse()) {
				dSubtractVectors3(dist, pos1, _anchor2); // Invert the value
			} else {
				dSubtractVectors3(dist, _anchor2, pos1); // Invert the value
			}
		}


		// ======================================================================
		// Work on the Rotoide part (i.e. row 0, 1 and maybe 4 if rotoide powered

		// Set the two rotoide rows. The rotoide axis should be the only unconstrained
		// rotational axis, the angular velocity of the two bodies perpendicular to
		// the rotoide axis should be equal. Thus the constraint equations are
		//    p*w1 - p*w2 = 0
		//    q*w1 - q*w2 = 0
		// where p and q are unit vectors normal to the rotoide axis, and w1 and w2
		// are the angular velocity vectors of the two bodies.
		DVector3 ax2 = new DVector3();
		DVector3 ax1 = new DVector3();
		dMultiply0_331( ax1, R1, axisR1 );
		dCalcVectorCross3( q, ax1, axP );

		dCopyVector3(J1A, J1Ofs + GI2__JA_MIN, axP);

		if (body1 != null) {
			dCopyNegatedVector3(J2A, J2Ofs + GI2__JA_MIN, axP);
		}

		dCopyVector3(J1A, J1Ofs + rowskip + GI2__JA_MIN, q);

		if (body1 != null) {
			dCopyNegatedVector3(J2A, J2Ofs + rowskip + GI2__JA_MIN, q);
		}

		// Compute the right hand side of the constraint equation set. Relative
		// body velocities along p and q to bring the rotoide back into alignment.
		// ax1,ax2 are the unit length rotoide axes of body1 and body2 in world frame.
		// We need to rotate both bodies along the axis u = (ax1 x ax2).
		// if `theta' is the angle between ax1 and ax2, we need an angular velocity
		// along u to cover angle erp*theta in one step :
		//   |angular_velocity| = angle/time = erp*theta / stepsize
		//                      = (erp*fps) * theta
		//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
		//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
		// ...as ax1 and ax2 are unit length. if theta is smallish,
		// theta ~= sin(theta), so
		//    angular_velocity  = (erp*fps) * (ax1 x ax2)
		// ax1 x ax2 is in the plane space of ax1, so we project the angular
		// velocity to p and q to find the right hand side.

		if (body1 != null) {
			dMultiply0_331(ax2, R2, axisR2);
		} else {
			dCopyVector3(ax2, axisR2);
		}

		DVector3 b = new DVector3();
		dCalcVectorCross3(b, ax1, ax2);
		pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = k * dCalcVectorDot3(b, axP);
		pairRhsCfmA[pairRhsCfmOfs + pairskip + GI2_RHS] = k * dCalcVectorDot3(b, q);


		// ==========================
			// Work on the Prismatic part (i.e row 2,3 and 4 if only the prismatic is powered
		// or 5 if rotoide and prismatic powered

		// two rows. we want: vel2 = vel1 + w1 x c ... but this would
		// result in three equations, so we project along the planespace vectors
		// so that sliding along the prismatic axis is disregarded. for symmetry we
		// also substitute (w1+w2)/2 for w1, as w1 is supposed to equal w2.

		// p1 + R1 dist' = p2 + R2 anchor2' ## OLD ## p1 + R1 anchor1' = p2 + R2 dist'
		// v1 + w1 x R1 dist' + v_p = v2 + w2 x R2 anchor2'## OLD  v1 + w1 x R1 anchor1' = v2 + w2 x R2 dist' + v_p
		// v_p is speed of prismatic joint (i.e. elongation rate)
		// Since the constraints are perpendicular to v_p we have:
		// p dot v_p = 0 and q dot v_p = 0
		// ax1 dot ( v1 + w1 x dist = v2 + w2 x anchor2 )
		// q dot ( v1 + w1 x dist = v2 + w2 x anchor2 )
		// ==
		// ax1 . v1 + ax1 . w1 x dist = ax1 . v2 + ax1 . w2 x anchor2 ## OLD ## ax1 . v1 + ax1 . w1 x anchor1 = ax1 . v2 + ax1 . w2 x dist
		// since a . (b x c) = - b . (a x c) = - (a x c) . b
		// and a x b = - b x a
		// ax1 . v1 - ax1 x dist . w1 - ax1 . v2 - (- ax1 x anchor2 . w2) = 0
		// ax1 . v1 + dist x ax1 . w1 - ax1 . v2 - anchor2 x ax1 . w2 = 0
		// Coeff for 1er line of: J1l => ax1, J2l => -ax1
		// Coeff for 2er line of: J1l => q, J2l => -q
		// Coeff for 1er line of: J1a => dist x ax1, J2a => - anchor2 x ax1
		// Coeff for 2er line of: J1a => dist x q,   J2a => - anchor2 x q

		int currRowSkip = 2 * rowskip;
		{
			dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JL_MIN, ax1);
			dCalcVectorCross3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, dist, ax1);

			if (body1 != null) {
				dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JL_MIN, ax1);
				// ax2 x anchor2 instead of anchor2 x ax2 since we want the negative value
				dCalcVectorCross3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, ax2, wanchor2);   // since ax1 == ax2
			}
		}

		currRowSkip += rowskip;
		{
			dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JL_MIN, q);
			dCalcVectorCross3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, dist, q);

			if (body1 != null) {
				dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JL_MIN, q);
				// The cross product is in reverse order since we want the negative value
				dCalcVectorCross3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, q, wanchor2);
			}
		}

		// We want to make correction for motion not in the line of the axisP
		// We calculate the displacement w.r.t. the anchor pt.
		//
		// compute the elements 2 and 3 of right hand side.
		// we want to align the offset point (in body 2's frame) with the center of body 1.
		// The position should be the same when we are not along the prismatic axis
		DVector3 err = new DVector3();
		dMultiply0_331( err, R1, offset );
		dSubtractVectors3(err, dist, err);

		int currPairSkip = 2 * pairskip;
		{
			pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * dCalcVectorDot3(ax1, err);
		}

		currPairSkip += pairskip;
		{
			pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * dCalcVectorDot3(q, err);
		}

		currRowSkip += rowskip;
		currPairSkip += pairskip;

		if (body1 != null || !isFlagsReverse()) {
			if (limotP.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, axP, false)) {
				currRowSkip += rowskip;
				currPairSkip += pairskip;
			}
		} else {
			DVector3 rAxP = new DVector3();
			dCopyNegatedVector3(rAxP, axP);

			if (limotP.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, rAxP, false)) {
				currRowSkip += rowskip;
				currPairSkip += pairskip;
			}
		}

		limotR.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
				pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax1, true);
	}


	// compute initial relative rotation body1 -> body2, or env -> body1
	void
	computeInitialRelativeRotation()
	{
		if ( node[0].body!=null )
		{
			if ( node[1].body!=null )
			{
				dQMultiply1( qrel, node[0].body._q, node[1].body._q );
			}
			else
			{
				// set joint->qrel to the transpose of the first body q
				qrel.set(0, node[0].body._q.get(0) );
				for ( int i = 1; i < 4; i++ )
					qrel.set(i, -node[0].body._q.get(i) );
				// WARNING do we need the - in -joint->node[0].body->q[i]; or not
			}
		}
	}

	public void dJointSetPRAnchor( double x, double y, double z )
	{
		dJointSetPRAnchor( new DVector3(x, y, z) );
	}
	public void dJointSetPRAnchor( DVector3C xyz )
	{
		setAnchors( xyz, offset, _anchor2 );
	}


	//void dJointSetPRAxis1( dJoint j, double x, double y, double z )
	public void dJointSetPRAxis1( double x, double y, double z )
	{
		setAxes( x, y, z, axisP1, null );

		computeInitialRelativeRotation();
	}


	//void dJointSetPRAxis2( dJoint j, double x, double y, double z )
	public void dJointSetPRAxis2( double x, double y, double z )
	{
		setAxes( x, y, z, axisR1, axisR2 );
		computeInitialRelativeRotation();
	}


	public void dJointSetPRParam( PARAM_N parameter, double value )
	{
		if ( parameter.isGroup2())//and( 0xff00 ).eq( 0x100 ))
		{
			limotR.set( parameter.toSUB(), value);     // Take only lower part of the
		}                                              // parameter alue
		else
		{
			limotP.set( parameter.toSUB(), value );
		}
	}

	//void dJointGetPRAnchor( dJoint j, dVector3 result )
	public void dJointGetPRAnchor( DVector3 result )
	{
		if ( node[1].body!=null )
			getAnchor2( result, _anchor2 );
		else
		{
			//        result.v[0] = anchor2.v[0];
			//        result.v[1] = anchor2.v[1];
			//        result.v[2] = anchor2.v[2];
			result.set(_anchor2);
		}
	}

//	void dJointGetPRAxis1( dJoint j, dVector3 result )
	void dJointGetPRAxis1( DVector3 result )
	{
		getAxis( result, axisP1 );
	}

//	void dJointGetPRAxis2( dJoint j, dVector3 result )
	void dJointGetPRAxis2( DVector3 result )
	{
		getAxis( result, axisR1 );
	}

	public double dJointGetPRParam( PARAM_N parameter )
	{
		if ( parameter.isGroup2())//and( 0xff00 ).eq( 0x100 ))
		{
			return limotR.get( parameter.toSUB());
		}
		else
		{
			return limotP.get( parameter.toSUB() );
		}
	}

//	void dJointAddPRTorque( dJoint j, double torque )
	void dJointAddPRTorque( double torque )
	{
		DVector3 axis = new DVector3();

		if ( isFlagsReverse() )
			torque = -torque;

		getAxis( axis, axisR1 );
//		axis.v[0] *= torque;
//		axis.v[1] *= torque;
//		axis.v[2] *= torque;
		axis.scale(torque);

		if ( node[0].body != null )
			node[0].body.dBodyAddTorque( axis );
		if ( node[1].body != null )
			node[1].body.dBodyAddTorque( axis.scale(-1) );
	}


	@Override
	void setRelativeValues()
	{
	    DVector3 anchor = new DVector3();
	    dJointGetPRAnchor(anchor);
	    setAnchors( anchor, offset, _anchor2 );

	    DVector3 axis = new DVector3();
	    dJointGetPRAxis1(axis);
	    setAxes( axis, axisP1, null );

	    dJointGetPRAxis2(axis);
	    setAxes( axis, axisR1, axisR2 );

	    computeInitialRelativeRotation();
	}


	// ******************************
	// API dPRJoint
	// ******************************

	@Override
	public void setAnchor (double x, double y, double z)
	{ dJointSetPRAnchor (x, y, z); }
	@Override
	public void setAnchor (DVector3C a)
	{ dJointSetPRAnchor (a); }
	@Override
	public void setAxis1 (double x, double y, double z)
	{ dJointSetPRAxis1 (x, y, z); }
	@Override
	public void setAxis1 (DVector3C a)
	//TODO use dVector3
	{ setAxis1(a.get0(), a.get1(), a.get2()); }
	@Override
	public void setAxis2 (double x, double y, double z)
	{ dJointSetPRAxis2 (x, y, z); }
	@Override
	public void setAxis2 (DVector3C a)
	//TODO use dVector3
	{ setAxis2(a.get0(), a.get1(), a.get2()); }

	@Override
	public void getAnchor (DVector3 result)
	{ dJointGetPRAnchor (result); }
	@Override
	public void getAxis1 (DVector3 result)
	{ dJointGetPRAxis1 (result); }
	@Override
	public void getAxis2 (DVector3 result)
	{ dJointGetPRAxis2 (result); }

	@Override
	public double getAngle() {
		return dJointGetPRAngle();
	}

	@Override
	public double getAngleRate() {
		return dJointGetPRAngleRate();
	}

	@Override
	public double getPosition()
	{ return dJointGetPRPosition (); }
	@Override
	public double getPositionRate()
	{ return dJointGetPRPositionRate (); }

	@Override
	public void setParam (PARAM_N parameter, double value)
	{ dJointSetPRParam (parameter, value); }
	@Override
	public double getParam (PARAM_N parameter)
	{ return dJointGetPRParam (parameter); }


	@Override
	public void setParamHiStop(double d) {
		dJointSetPRParam(PARAM_N.dParamHiStop1, d);
	}


	@Override
	public void setParamLoStop(double d) {
		dJointSetPRParam(PARAM_N.dParamLoStop1, d);
	}


	@Override
	public void setParamHiStop2(double d) {
		dJointSetPRParam(PARAM_N.dParamHiStop2, d);
	}


	@Override
	public void setParamLoStop2(double d) {
		dJointSetPRParam(PARAM_N.dParamLoStop2, d);
	}


	@Override
	public void setParamFMax2(double d) {
		dJointSetPRParam(PARAM_N.dParamFMax2, d);
	}


	@Override
	public void setParamVel2(double d) {
		dJointSetPRParam(PARAM_N.dParamVel2, d);
	}


	@Override
	public void addTorque(double torque) {
		dJointAddPRTorque(torque);
	}
}



