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

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.dUASSERT;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DDoubleBallJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.DxWorld;

public class DxJointDBall extends DxJoint implements DDoubleBallJoint {
	private final DVector3 anchor1 = new DVector3();   // anchor w.r.t first body
	private final DVector3 anchor2 = new DVector3();   // anchor w.r.t second body
	private double erp;          // error reduction
	private double cfm;          // constraint force mix in
	private double targetDistance;



	DxJointDBall(DxWorld w) {
		super(w);
		targetDistance = 0;
		erp = world.getERP();
		cfm = world.getCFM();
	}

	@Override
	void
	getSureMaxInfo( SureMaxInfo info )
	{
		info.max_m = 1;
	}

	@Override
	public void
	getInfo1( DxJoint.Info1 info )
	{
		info.m = 1;
		info.nub = 1;
	}

	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		DVector3 globalA1 = new DVector3(), globalA2 = new DVector3();
		//dBodyGetRelPointPos(node[0].body, anchor1[0], anchor1[1], anchor1[2], globalA1);
		node[0].body.getRelPointPos(anchor1, globalA1);

		if (node[1].body != null) {
			//dBodyGetRelPointPos(node[1].body, anchor2[0], anchor2[1], anchor2[2], globalA2);
			node[1].body.getRelPointPos(anchor2, globalA2);
		} else {
			//dCopyVector3(globalA2, anchor2);
			globalA2.set(anchor2);
		}

		DVector3 q = new DVector3();
		//dSubtractVectors3(q, globalA1, globalA2);
		q.eqDiff(globalA1, globalA2);

		//    #ifdef dSINGLE
		//        const dReal MIN_LENGTH = REAL(1e-7);
		//    #else
		final double MIN_LENGTH = 1e-12;
		//    #endif

		if (q.length() < MIN_LENGTH) {
			// too small, let's choose an arbitrary direction
			// heuristic: difference in velocities at anchors
			DVector3 v1 = new DVector3(), v2 = new DVector3();
			//dBodyGetPointVel(node[0].body, globalA1[0], globalA1[1], globalA1[2], v1);
			node[0].body.getPointVel(globalA1, v1);

			if (node[1].body != null) {
				//dBodyGetPointVel(node[1].body, globalA2[0], globalA2[1], globalA2[2], v2);
				node[1].body.getPointVel(globalA2, v2);
			} else {
				v2.setZero();
			}

			//dSubtractVectors3(q, v1, v2);
			q.eqDiff(v1, v2);

			if (q.length() < MIN_LENGTH) {
				// this direction is as good as any
				q.set(1, 0, 0);
			}
		}
		OdeMath.dNormalize3(q);

		dCopyVector3(J1A, J1Ofs + GI2__JL_MIN, q);

		DVector3 relA1 = new DVector3();
		//        dBodyVectorToWorld(node[0].body,
		//                           anchor1[0], anchor1[1], anchor1[2],
		//                           relA1);
		node[0].body.vectorToWorld(anchor1, relA1);

		DMatrix3 a1m = new DMatrix3();
		//dSetZero(a1m, 12);
		dSetCrossMatrixMinus(a1m, relA1);//, 4);

		dMultiply1_331(J1A, J1Ofs + GI2__JA_MIN, a1m, q);

		if (node[1].body != null) {
			dCopyNegatedVector3(J2A, J2Ofs + GI2__JL_MIN, q);

			DVector3 relA2 = new DVector3();
			//            dBodyVectorToWorld(node[1].body,
			//                               anchor2[0], anchor2[1], anchor2[2],
			//                               relA2);
			node[1].body.vectorToWorld(anchor2, relA2);
			DMatrix3 a2m = new DMatrix3();
			//dSetZero(a2m, 12);
			dSetCrossMatrixPlus(a2m, relA2);//, 4);
			dMultiply1_331(J2A, J2Ofs + GI2__JA_MIN, a2m, q);
		}

		final double k = worldFPS * this.erp;
		pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = k * (targetDistance - globalA1.distance(globalA2));
		pairRhsCfmA[pairRhsCfmOfs + GI2_CFM] = this.cfm;
	}


	void
	updateTargetDistance()
	{
		DVector3 p1 = new DVector3(), p2 = new DVector3();

		if (node[0].body != null)
			//dBodyGetRelPointPos(node[0].body, anchor1[0], anchor1[1], anchor1[2], p1);
			node[0].body.getRelPointPos(anchor1, p1);
		else
			//dCopyVector3(p1, anchor1);
			p1.set(anchor1);
		if (node[1].body != null)
			//dBodyGetRelPointPos(node[1].body, anchor2[0], anchor2[1], anchor2[2], p2);
			node[1].body.getRelPointPos(anchor2, p2);
		else
			//dCopyVector3(p2, anchor2);
			p2.set(anchor2);

		targetDistance = p1.distance(p2);//dCalcPointsDistance3(p1, p2);
	}


	void dJointSetDBallAnchor1( DVector3C xyz )
	{
		if ( (flags & dJOINT_REVERSE) != 0 ) {
			if (node[1].body != null)
				//dBodyGetPosRelPoint(node[1].body, xyz, anchor2);
				node[1].body.getPosRelPoint(xyz, anchor2);
			else {
				//                anchor2[0] = x;
				//                anchor2[1] = y;
				//                anchor2[2] = z;
				anchor2.set(xyz);
			}
		} else {
			if (node[0].body != null)
				//dBodyGetPosRelPoint(node[0].body, x, y, z, anchor1);
				node[0].body.getPosRelPoint(xyz, anchor1);
			else {
				//                anchor1[0] = x;
				//                anchor1[1] = y;
				//                anchor1[2] = z;
				anchor1.set(xyz);
			}
		}

		updateTargetDistance();
	}


	void dJointSetDBallAnchor2( DVector3C xyz )
	{
		if ( (flags & dJOINT_REVERSE) != 0 ) {
			if (node[0].body != null)
				//dBodyGetPosRelPoint(node[0].body, x, y, z, anchor1);
				node[0].body.getPosRelPoint(xyz, anchor1);
			else {
				//                anchor1[0] = x;
				//                anchor1[1] = y;
				//                anchor1[2] = z;
				anchor1.set(xyz);
			}
		} else {
			if (node[1].body != null)
				//dBodyGetPosRelPoint(node[1].body, x, y, z, anchor2);
				node[1].body.getPosRelPoint(xyz, anchor2);
			else {
				//                anchor2[0] = x;
				//                anchor2[1] = y;
				//                anchor2[2] = z;
				anchor2.set(xyz);
			}
		}

		updateTargetDistance();
	}

	double dJointGetDBallDistance()
	{
		return targetDistance;
	}

	void dJointSetDBallDistance(double dist) {
		dUASSERT(dist >= 0, "target distance must be non-negative");
		targetDistance = dist;
	}

	void dJointGetDBallAnchor1( DVector3 result )
	{
		dUASSERT( result, "bad result argument" );

		if ( (flags & dJOINT_REVERSE)!=0 ) {
			if (node[1].body != null)
				//dBodyGetRelPointPos(node[1].body, anchor2[0], anchor2[1], anchor2[2], result);
				node[1].body.getRelPointPos(anchor2, result);
			else
				//dCopyVector3(result, anchor2);
				result.set(anchor2);
		} else {
			if (node[0].body != null)
				//dBodyGetRelPointPos(node[0].body, anchor1[0], anchor1[1], anchor1[2], result);
				node[0].body.getRelPointPos(anchor1, result);
			else
				//dCopyVector3(result, anchor1);
				result.set(anchor1);
		}
	}


	void dJointGetDBallAnchor2( DVector3 result )
	{
		dUASSERT( result, "bad result argument" );

		if ( (flags & dJOINT_REVERSE)!=0 ) {
			if (node[0].body != null)
				node[0].body.getRelPointPos(anchor1, result);
			else
				//dCopyVector3(result, anchor1);
				result.set(anchor1);
		} else {
			if (node[1].body != null)
				//dBodyGetRelPointPos(node[1].body, anchor2[0], anchor2[1], anchor2[2], result);
				node[1].body.getRelPointPos(anchor2, result);
			else
				//dCopyVector3(result, anchor2);
				result.set(anchor2);
		}
	}


	void dJointSetDBallParam(DJoint.PARAM parameter, double value )
	{
		switch ( parameter ) {
		case dParamCFM:
			cfm = value;
			break;
		case dParamERP:
			erp = value;
			break;
		default:
			//ignore
		}
	}


	double dJointGetDBallParam(DJoint.PARAM parameter )
	{
		switch ( parameter ) {
		case dParamCFM:
			return cfm;
		case dParamERP:
			return erp;
		default:
			return 0;
		}
	}


	@Override
	void
	setRelativeValues()
	{
		updateTargetDistance();
	}

	// *******************************
	// API dBallJoint
	// *******************************

	@Override
	public final void setAnchor1 (double x, double y, double z)
	{ dJointSetDBallAnchor1 (new DVector3(x, y, z)); }
	@Override
	public final void setAnchor1 (DVector3C a)
	{ dJointSetDBallAnchor1 (a); }
	@Override
	public final void setAnchor2 (double x, double y, double z)
	{ dJointSetDBallAnchor2 (new DVector3(x, y, z)); }
	@Override
	public final void setAnchor2 (DVector3C a)
	{ dJointSetDBallAnchor2 (a); }
	@Override
	public final void getAnchor1 (DVector3 result)
	{ dJointGetDBallAnchor1 (result); }
	@Override
	public final void getAnchor2 (DVector3 result)
	{ dJointGetDBallAnchor2 (result); }
	@Override
	public final void setParam (PARAM_N parameter, double value)
	{ 
		if (!parameter.isGroup1()) 
			throw new IllegalArgumentException("Only Group #1 allowed, but got: " + parameter.name());
		dJointSetDBallParam (parameter.toSUB(), value); 
	}
	@Override
	public final double getParam (PARAM_N parameter)
	{ 	
		if (!parameter.isGroup1()) 
			throw new IllegalArgumentException("Only Group #1 allowed, but got: " + parameter.name());
		return dJointGetDBallParam (parameter.toSUB()); 
	}

	@Override
	public double getDistance() {
		return dJointGetDBallDistance();
	}

	@Override
	public void setDistance(double distance) {
		dJointSetDBallDistance(distance);
	}

	protected final double erp() {
		return erp;
	}

	protected final DVector3C anchor1() {
		return anchor1;
	}

	protected final DVector3C anchor2() {
		return anchor2;
	}


}
