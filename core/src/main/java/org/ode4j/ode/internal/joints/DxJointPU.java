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

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DPUJoint;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;


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
 * &lt;--------|      x       | .....|.......(__)     ..
 * axisP    +--------------+    --|         ^     &lt;
 *                 |-----------------------&gt;|
 *                     anchor1              |--- Universal articulation
 *                                               axis1 going out of the plane
 *                                               axis2 is perpendicular to axis1
 *                                               (i.e. 2 rotoides)
 * </PRE>
 */
public class DxJointPU extends DxJointUniversal implements DPUJoint
{
	/** 
	 * Axis for the prismatic articulation w.r.t first body.
	 * <p>NOTE: This is considered as axis2 from the parameter view
	 */
	DVector3 axisP1 = new DVector3();

	/** limit and motor information for the prismatic articulation. */
	public DxJointLimitMotor limotP = new DxJointLimitMotor(); 


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
//		_axis1.dSetZero();//dSetZero( _axis1, 4 );
//		_axis1.v[1] = 1;
		_axis1.set(0, 1, 0);

		// Setting member variables which are w.r.t body2
		_anchor2.setZero();//dSetZero( _anchor2, 4 );
//		_axis2.dSetZero();//dSetZero( _axis2, 4 );
//		_axis2.v[2] = 1;
		_axis2.set(0, 0, 1);

//		axisP1.dSetZero();//dSetZero( axisP1, 4 );
//		axisP1.v[0] = 1;
		axisP1.set(1, 0, 0);

		qrel1.setZero();//dSetZero( qrel1, 4 );
		qrel2.setZero();//dSetZero( qrel2, 4 );


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
		dMultiply0_331( q, node[0].body.posr().R(), _anchor1 );

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
			q.add( node[0].body.posr().pos() );
			q.sub( node[1].body.posr().pos() );
			q.sub( anchor2 );
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
			q.add( node[0].body.posr().pos() );
			q.sub( _anchor2 );

	        if ( isFlagsReverse() )
	        {
//	            q[0] = -q[0];
//	            q[1] = -q[1];
//	            q[2] = -q[2];
	        	q.scale( -1 );
	        }		
	    }

		DVector3 axP = new DVector3();
		// get prismatic axis in global coordinates
		dMultiply0_331( axP, node[0].body.posr().R(), axisP1 );

		return dCalcVectorDot3( axP, q );
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

				// NOTE! anchor2 needs a volatile assignment on the multiplication to discard computation errors.
				// Otherwise, tests fail for single type on x86.
				// dxTruncToType::dMultiply0_331(anchor2, joint->node[1].body->posr.R, joint->anchor2);
				dMultiply0_331( anchor2, node[1].body.posr().R(), _anchor2 );

				//				r.v[0] = ( node[0].body._posr.pos.v[0] -
				//						( anchor2.v[0] + node[1].body._posr.pos.v[0] ) );
				//				r.v[1] = ( node[0].body._posr.pos.v[1] -
				//						( anchor2.v[1] + node[1].body._posr.pos.v[1] ) );
				//				r.v[2] = ( node[0].body._posr.pos.v[2] -
				//						( anchor2.v[2] + node[1].body._posr.pos.v[2] ) );
				r.eqDiff(node[0].body.posr().pos(), node[1].body.posr().pos()).sub(anchor2);
			}
			else
			{
				//N.B. When there is no body 2 the joint->anchor2 is already in
				//     global coordinates
				// r = joint->node[0].body->posr.pos -  joint->anchor2;
				//dOP( r.v, OP.SUB, node[0].body._posr.pos.v, _anchor2.v );
				r.eqDiff(node[0].body.posr().pos(), _anchor2);
			}

			// The body1 can have velocity coming from the rotation of
			// the rotoide axis. We need to remove this.

			// N.B. We do vel = r X w instead of vel = w x r to have vel negative
			//      since we want to remove it from the linear velocity of the body
			DVector3 lvel1 = new DVector3();
			dCalcVectorCross3( lvel1, r, node[0].body.avel );

			// lvel1 += joint->node[0].body->lvel;
			//dOPE( lvel1.v, 0, OP.ADD_EQ , node[0].body.lvel.v );
			lvel1.add(node[0].body.lvel);

	        // Since we want rate of change along the prismatic axis
	        // get axisP1 in global coordinates and get the component
	        // along this axis only
	        DVector3 axP1 = new DVector3();
	        dMultiply0_331( axP1, node[0].body.posr().R(), axisP1 );

			if ( node[1].body!=null )
			{
				// Find the contribution of the angular rotation to the linear speed
				// N.B. We do vel = r X w instead of vel = w x r to have vel negative
				//      since we want to remove it from the linear velocity of the body
				DVector3 lvel2 = new DVector3();
				dCalcVectorCross3( lvel2, anchor2, node[1].body.avel );

				// lvel1 -=  lvel2 + joint->node[1].body->lvel;
				//  dVector3 tmp;
				//  dAddVectors3( tmp, lvel2, joint->node[1].body->lvel );
				//  dSubtractVectors3( lvel1, lvel1, tmp );
				lvel1.sub( lvel2 );
				lvel1.sub( node[1].body.lvel );

	            return dCalcVectorDot3( axP1, lvel1 );
	        }
	        else
	        {
	            double rate = axP1.dot( lvel1 );
	            return isFlagsReverse() ? -rate : rate;
			}
		}

		return 0.0;
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


	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		final double k = worldFPS * worldERP;

	    // ======================================================================
	    // The angular constraint
	    //
	    DVector3 ax1 = new DVector3(), ax2 = new DVector3(); // Global axes of rotation
	    getAxis(ax1, _axis1);
	    getAxis2(ax2, _axis2);

	    DVector3 uniPerp = new DVector3();  // Axis perpendicular to axes of rotation
	    dCalcVectorCross3(uniPerp,ax1,ax2);
	    dNormalize3( uniPerp );

		dCopyVector3( J1A, J1Ofs + GI2__JA_MIN, uniPerp );

		DxBody body1 = node[1].body;

		if ( body1 != null ) {
			dCopyNegatedVector3( J2A, J2Ofs + GI2__JA_MIN , uniPerp );
		}
	    // Corrective velocity attempting to keep uni axes perpendicular
	    double val = dCalcVectorDot3( ax1, ax2 );
	    // Small angle approximation : 
	    // theta = asin(val)
	    // theta is approximately val when val is near zero.
		pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = -k * val;

	    // ==========================================================================
	    // Handle axes orthogonal to the prismatic 
	    DVector3 an1 = new DVector3(), an2 = new DVector3(); // Global anchor positions
	    DVector3 axP = new DVector3(), sep = new DVector3(); // Prismatic axis and separation vector
		getAnchor(an1, _anchor1);
		getAnchor2(an2, _anchor2);

	    if ((flags & dJOINT_REVERSE)!=0) {
	        getAxis2(axP, axisP1);
	    } else {
	        getAxis(axP, axisP1);
	    }
		dSubtractVectors3(sep, an2, an1);

		DVector3 p = new DVector3(), q = new DVector3();
		dPlaneSpace(axP, p, q);

		dCopyVector3( J1A, J1Ofs + rowskip + GI2__JL_MIN, p );
		dCopyVector3( J1A, J1Ofs + 2 * rowskip + GI2__JL_MIN, q );
		// Make the anchors be body local
		// Aliasing isn't a problem here.
		dSubtractVectors3(an1, an1, node[0].body.posr().pos());
		dCalcVectorCross3( J1A, J1Ofs + rowskip + GI2__JA_MIN, an1, p );
		dCalcVectorCross3( J1A, J1Ofs + 2 * rowskip + GI2__JA_MIN, an1, q );

		if (body1 != null) {
			dCopyNegatedVector3( J2A, J2Ofs + rowskip + GI2__JL_MIN, p );
			dCopyNegatedVector3( J2A, J2Ofs + 2 * rowskip + GI2__JL_MIN, q );
			dSubtractVectors3(an2, an2, body1.posr().pos());
			dCalcVectorCross3( J2A, J2Ofs + rowskip + GI2__JA_MIN, p, an2 );
			dCalcVectorCross3( J2A, J2Ofs + 2 * rowskip + GI2__JA_MIN, q, an2 );
		}

		pairRhsCfmA[pairRhsCfmOfs + pairskip + GI2_RHS] = k * dCalcVectorDot3( p, sep );
		pairRhsCfmA[pairRhsCfmOfs + 2 * pairskip + GI2_RHS] = k * dCalcVectorDot3( q, sep );
	    
	    // ==========================================================================
	    // Handle the limits/motors
		int currRowSkip = 3 * rowskip, currPairSkip = 3 * pairskip;

		if (limot1.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
				pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax1, true)) {
			currRowSkip += rowskip;
			currPairSkip += pairskip;
		}

		if (limot2.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
				pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax2, true)) {
			currRowSkip += rowskip;
			currPairSkip += pairskip;
		}

		if (body1 != null || !isFlagsReverse()) {
			limotP.addTwoPointLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, axP, an1, an2);
		} else {
			dNegateVector3(axP);
			limotP.addTwoPointLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, axP, an1, an2);
		}
	}

	public void dJointSetPUAnchor( double x, double y, double z )
	{
		dJointSetPUAnchor( new DVector3(x, y, z) );
	}
	public void dJointSetPUAnchor( DVector3C xyz )
	{
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
	 * <p>NOTE: Should have the same meaning as dJointSetSliderAxisDelta
	 *
	 * @param x The X position of the anchor point in world frame
	 * @param y The Y position of the anchor point in world frame
	 * @param z The Z position of the anchor point in world frame
	 * @param dx A delta to be added to the X position as if the anchor was set
	 *           when body1 was at current_position[X] + dx
	 * @param dy A delta to be added to the Y position as if the anchor was set
	 *           when body1 was at current_position[Y] + dy
	 * @param dz A delta to be added to the Z position as if the anchor was set
	 *           when body1 was at current_position[Z] + dz
	 */
	void dJointSetPUAnchorDelta( double x, double y, double z,
			double dx, double dy, double dz )
	{
		if ( node[0].body!=null )
		{
//			node[0].body._posr.pos.v[0] += dx;
//			node[0].body._posr.pos.v[1] += dy;
//			node[0].body._posr.pos.v[2] += dz;
			node[0].body._posr.pos.add(dx, dy, dz);
		}

		setAnchors( new DVector3(x, y, z), _anchor1, _anchor2 );

		if ( node[0].body!=null )
		{
//			node[0].body._posr.pos.v[0] -= dx;
//			node[0].body._posr.pos.v[1] -= dy;
//			node[0].body._posr.pos.v[2] -= dz;
			node[0].body._posr.pos.sub(dx, dy, dz);
		}

		computeInitialRelativeRotations();
	}



	/**
	 * This function initialize the anchor and the relative position of each body
	 * such that dJointGetPUPosition will return the dot product of axis and [dx,dy,dy].
	 *
	 * The body 1 is moved to [-dx, -dy, -dx] then the anchor is set. This will be the
	 * position 0 for the prismatic part of the joint. Then the body 1 is moved to its
	 * original position.
	 *
	 * Ex:
	 * <PRE>
	 * dReal offset = 1;
	 * dVector3 dir;
	 * dJointGetPUAxis3(jId, dir);
	 * dJointSetPUAnchor(jId, 0, 0, 0);
	 * // If you request the position you will have: dJointGetPUPosition(jId) == 0
	 * dJointSetPUAnchorDelta(jId, 0, 0, 0, dir[X]*offset, dir[Y]*offset, dir[Z]*offset);
	 * // If you request the position you will have: dJointGetPUPosition(jId) == offset
	 * </PRE>
	 * <p>NOTE: Should have the same meaning as dJointSetSliderAxisDelta
	 *
	 * @param x The X position of the anchor point in world frame
	 * @param y The Y position of the anchor point in world frame
	 * @param z The Z position of the anchor point in world frame
	 * @param dx A delta to be added to the X position as if the anchor was set
	 *           when body1 was at current_position[X] + dx
	 * @param dy A delta to be added to the Y position as if the anchor was set
	 *           when body1 was at current_position[Y] + dy
	 * @param dz A delta to be added to the Z position as if the anchor was set
	 *           when body1 was at current_position[Z] + dz
	 */
	void dJointSetPUAnchorOffset( double x, double y, double z,
			double dx, double dy, double dz )
	{
		DVector3 dxyz = new DVector3(dx, dy, dz);
		
	    if ( isFlagsReverse() )
	    {
//	        dx = -dx;
//	        dy = -dy;
//	        dz = -dz;
	    	dxyz.scale( -1 );
	    }

	    if ( node[0].body != null )
	    {
//	        node[0].body._posr.pos[0] -= dx;
//	        node[0].body._posr.pos[1] -= dy;
//	        node[0].body._posr.pos[2] -= dz;
	        node[0].body._posr.pos.sub(dxyz);
	    }

	    setAnchors( dxyz, _anchor1, _anchor2 );

	    if ( node[0].body != null )
	    {
//	        node[0].body->posr.pos[0] += dx;
//	        node[0].body->posr.pos[1] += dy;
//	        node[0].body->posr.pos[2] += dz;
	        node[0].body._posr.pos.add(dxyz);
	    }

	    computeInitialRelativeRotations();
	}


	public void dJointSetPUAxis1( double x, double y, double z )
	{
		if ( isFlagsReverse() )
			setAxes( x, y, z, null, _axis2 );
		else
			setAxes( x, y, z, _axis1, null );
		computeInitialRelativeRotations();
	}

	public void dJointSetPUAxis2( double x, double y, double z )
	{
		if ( isFlagsReverse() )
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
		setAxes( x, y, z, axisP1, null );

		computeInitialRelativeRotations();
	}


	//void dJointGetPUAngles( dJoint j, double *angle1, double *angle2 )
	void dJointGetPUAngles( RefDouble angle1, RefDouble angle2 )
	{
		if ( isFlagsReverse() )
			getAngles( angle2, angle1 );
		else
			getAngles( angle1, angle2 );
	}


	double dJointGetPUAngle1()
	{
		if ( isFlagsReverse() )
			return getAngle2Internal();
		else
			return getAngle1Internal();
	}


	double dJointGetPUAngle2()
	{
		if ( isFlagsReverse() )
			return getAngle1Internal();
		else
			return getAngle2Internal();
	}


	double dJointGetPUAngle1Rate()
	{
		if ( node[0].body!=null )
		{
			DVector3 axis = new DVector3();

			if ( isFlagsReverse() )
				getAxis2( axis, _axis2 );
			else
				getAxis( axis, _axis1 );

			double rate = dCalcVectorDot3( axis, node[0].body.avel );
			if ( node[1].body!=null ) rate -= dCalcVectorDot3( axis, node[1].body.avel );
			return rate;
		}
		return 0;
	}


	double dJointGetPUAngle2Rate()
	{
		if ( node[0].body!=null )
		{
			DVector3 axis = new DVector3();

			if ( isFlagsReverse() )
				getAxis( axis, _axis1 );
			else
				getAxis2( axis, _axis2 );

			double rate = dCalcVectorDot3( axis, node[0].body.avel );
			if ( node[1].body!=null ) rate -= dCalcVectorDot3( axis, node[1].body.avel );
			return rate;
		}
		return 0;
	}


	public void dJointSetPUParam( PARAM_N parameter, double value )
	{
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
		default:
			throw new IllegalArgumentException(parameter.name());
		}
	}

	//	void dJointGetPUAnchor( dJoint j, dVector3 result )
	public void dJointGetPUAnchor( DVector3 result )
	{
		if ( node[1].body!=null )
			getAnchor2( result, _anchor2 );
		else
		{
			// result[i] = joint->anchor2[i];
			//dCopyVector3( result, joint->anchor2 );
			result.set( _anchor2 );
		}
	}

	void dJointGetPUAxis1( DVector3 result )
	{
	    if ( isFlagsReverse() )
	        getAxis2( result, _axis2 );
	    else
	        getAxis( result, _axis1 );
	}

	void dJointGetPUAxis2( DVector3 result )
	{
	    if ( isFlagsReverse() )
	        getAxis( result, _axis1 );
	    else
	        getAxis2( result, _axis2 );
	}

	/**
	 * Get the prismatic axis.
	 *
	 * NOTE: This function was added for convenience it is the same as
	 *       dJointGetPUAxis3
	 */
	void dJointGetPUAxisP( DVector3 result )
	{
		dJointGetPUAxis3( result );
	}


	void dJointGetPUAxis3( DVector3 result )
	{
		getAxis( result, axisP1 );
	}

	public double dJointGetPUParam( PARAM_N parameter )
	{
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

	@Override
	void setRelativeValues()
	{
	    DVector3 anchor = new DVector3();
	    dJointGetPUAnchor(anchor);
	    setAnchors( anchor, _anchor1, _anchor2 );

	    DVector3 ax1 = new DVector3(), ax2 = new DVector3(), ax3 = new DVector3();
	    dJointGetPUAxis1(ax1);
	    dJointGetPUAxis2(ax2);
	    dJointGetPUAxis3(ax3);

	    if ( isFlagsReverse() )
	    {
	        setAxes( ax1, null, _axis2 );
	        setAxes( ax2, _axis1, null );
	    }
	    else
	    {
	        setAxes( ax1, _axis1, null );
	        setAxes( ax2, null, _axis2 );
	    }


	    setAxes( ax3, axisP1, null );

	    computeInitialRelativeRotations();
	}


	// **********************************
	// API PUJoint
	// **********************************

	@Override
	public final void setAnchor (double x, double y, double z)
	{ dJointSetPUAnchor (x, y, z); }
	@Override
	public final void setAnchor (DVector3C a)
	{ dJointSetPUAnchor (a); }
	@Override
	public final void setAxis1 (double x, double y, double z)
	{ dJointSetPUAxis1 (x, y, z); }
	@Override
	public final void setAxis1 (DVector3C a)
	//TODO use dVector3
	{ setAxis1(a.get0(), a.get1(), a.get2()); }
	@Override
	public final void setAxis2 (double x, double y, double z)
	{ dJointSetPUAxis2 (x, y, z); }
	@Override
	public final void setAxis3 (double x, double y, double z)
	{ dJointSetPUAxis3 (x, y, z); }
	@Override
	public final void setAxis3 (DVector3C a)
	//TODO use dVector3
	{ setAxis3(a.get0(), a.get1(), a.get2()); }
	@Override
	public final void setAxisP (double x, double y, double z)
	{ dJointSetPUAxis3 (x, y, z); }
	@Override
	public final void setAxisP (DVector3C a)
	//TODO use dVector3
	{ setAxisP(a.get0(), a.get1(), a.get2()); }

	//TZ not final: 'virtual'
	@Override
	public void getAnchor (DVector3 result)
	{ dJointGetPUAnchor (result); }
	@Override
	public final void getAxis1 (DVector3 result)
	{ dJointGetPUAxis1 (result); }
	@Override
	public final void getAxis2 (DVector3 result)
	{ dJointGetPUAxis2 (result); }
	@Override
	public final void getAxis3 (DVector3 result)
	{ dJointGetPUAxis3 (result); }
	@Override
	public final void getAxisP (DVector3 result)
	{ dJointGetPUAxis3 (result); }

	/** TZ Take care to call getAngle1Internal() from dx-classes.*/
	@Override
	public final double getAngle1()
	{ return dJointGetPUAngle1 (); }
	@Override
	public final double getAngle1Rate()
	{ return dJointGetPUAngle1Rate (); }
	/** TZ Take care to call getAngle2Internal() from dx-classes.*/
	@Override
	public final double getAngle2()
	{ return dJointGetPUAngle2 (); }
	@Override
	public final double getAngle2Rate()
	{ return dJointGetPUAngle2Rate (); }

	@Override
	public final double getPosition()
	{ return dJointGetPUPosition (); }
	@Override
	public final double getPositionRate()
	{ return dJointGetPUPositionRate (); }

	@Override
	public final void setParam (PARAM_N parameter, double value)
	{ dJointSetPUParam (parameter, value); }
	@Override
	public final double getParam (PARAM_N parameter)
	{ return dJointGetPUParam (parameter); }


	@Override
	public void setAnchorOffset(double x, double y, double z, double dx,
			double dy, double dz) {
		dJointSetPUAnchorOffset(x, y, z, dx, dy, dz);
	}


}

