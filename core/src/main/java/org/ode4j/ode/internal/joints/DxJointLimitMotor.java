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
import static org.ode4j.ode.internal.CommonEnums.*;
import static org.ode4j.ode.internal.joints.JointEnums.*;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DJoint.PARAM;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.processmem.DxWorldProcessContext;

/**
 * common limit and motor information for a single joint axis of movement
 *
 */
public class DxJointLimitMotor {
	public double vel;        // powered joint: velocity, max force
	public double fmax;
	public double lostop;   // joint limits, relative to initial position
	public double histop;
	public double fudge_factor;     // when powering away from joint limits
	public double normal_cfm;       // cfm to use when not at a stop
	public double stop_erp; // erp and cfm for when at joint limit
	double stop_cfm;
	public double bounce;           // restitution factor
	// variables used between getInfo1() and getInfo2()
	public int limit;          // 0=free, 1=at lo limit, 2=at hi limit
	double limit_err;    // if at limit, amount over limit

	//****************************************************************************
	// dxJointLimitMotor

	public DxJointLimitMotor() {}

	public void init( DxWorld world )
	{
		vel = 0;
		fmax = 0;
		lostop = -dInfinity;
		histop = dInfinity;
		fudge_factor = 1;
		normal_cfm = world.getCFM();
		stop_erp = world.getERP();
		stop_cfm = world.getCFM();
		bounce = 0;
		limit = 0;
		limit_err = 0;
	}


	public void set( PARAM num, double value )
	{
		switch ( num )
		{
		case dParamLoStop:
			lostop = value;
			break;
		case dParamHiStop:
			histop = value;
			break;
		case dParamVel:
			vel = value;
			break;
		case dParamFMax:
			if ( value >= 0 ) fmax = value;
			break;
		case dParamFudgeFactor:
			if ( value >= 0 && value <= 1 ) fudge_factor = value;
			break;
		case dParamBounce:
			bounce = value;
			break;
		case dParamCFM:
			normal_cfm = value;
			break;
		case dParamStopERP:
			stop_erp = value;
			break;
		case dParamStopCFM:
			stop_cfm = value;
			break;
			default:
				throw new IllegalArgumentException(num.name());
		}
	}


	public double get( PARAM num )
	{
		switch ( num )
		{
		case dParamLoStop:
			return lostop;
		case dParamHiStop:
			return histop;
		case dParamVel:
			return vel;
		case dParamFMax:
			return fmax;
		case dParamFudgeFactor:
			return fudge_factor;
		case dParamBounce:
			return bounce;
		case dParamCFM:
			return normal_cfm;
		case dParamStopERP:
			return stop_erp;
		case dParamStopCFM:
			return stop_cfm;
		default:
			//return 0;
			throw new IllegalArgumentException("" + num.name());
		}
	}


	public boolean testRotationalLimit( double angle )
	{
		if ( angle <= lostop )
		{
			limit = 1;
			limit_err = angle - lostop;
			return true;
		}
		else if ( angle >= histop )
		{
			limit = 2;
			limit_err = angle - histop;
			return true;
		}
		else
		{
			limit = 0;
			return false;
		}
	}


	//	bool addLimot( dxJoint *joint, dReal fps,
	//				   dReal *J1, dReal *J2, dReal *pairRhsCfm, dReal *pairLoHi,
	//        const dVector3 ax1, int rotational );
	public boolean addLimot(DxJoint joint, double fps, double[] J1A, int J1Ofs, double[] J2A, int J2Ofs,
							double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA, int pairLoHiOfs,
							final DVector3C ax1, boolean rotational) {
		// if the joint is powered, or has joint limits, add in the extra row
		boolean powered = fmax > 0;
		if ( powered || limit != 0)
		{
			int J1UsedOfs = rotational ? J1Ofs + GI2__JA_MIN : J1Ofs + GI2__JL_MIN;
			int J2UsedOfs = rotational ? J2Ofs + GI2__JA_MIN : J2Ofs + GI2__JL_MIN;

			dCopyVector3(J1A, J1UsedOfs, ax1);

			DxBody b1 = joint.node[1].body;
			if (b1 != null) {
				dCopyNegatedVector3(J2A, J2UsedOfs, ax1);
			}

			// linear limot torque decoupling step:
			//
			// if this is a linear limot (e.g. from a slider), we have to be careful
			// that the linear constraint forces (+/- ax1) applied to the two bodies
			// do not create a torque couple. in other words, the points that the
			// constraint force is applied at must lie along the same ax1 axis.
			// a torque couple will result in powered or limited slider-jointed free
			// bodies from gaining angular momentum.
			// the solution used here is to apply the constraint forces at the point
			// halfway between the body centers. there is no penalty (other than an
			// extra tiny bit of computation) in doing this adjustment. note that we
			// only need to do this if the constraint connects two bodies.

			DVector3 ltd = new DVector3(0, 0, 0); // Linear Torque Decoupling vector (a torque)
			if ( (!rotational) && (b1 != null))
			{
				DxBody b0 = joint.node[0].body;
				DVector3 c = new DVector3();
				//	c.v[0] = 0.5 * ( joint.node[1].body._posr.pos.v[0] - joint.node[0].body._posr.pos.v[0] );
				//	c.v[1] = 0.5 * ( joint.node[1].body._posr.pos.v[1] - joint.node[0].body._posr.pos.v[1] );
				//	c.v[2] = 0.5 * ( joint.node[1].body._posr.pos.v[2] - joint.node[0].body._posr.pos.v[2] );
				c.eqDiff(b1.posr().pos(), b0.posr().pos()).scale(0.5);
				dCalcVectorCross3( ltd, c, ax1 );
				dCopyVector3(J1A, J1Ofs + DxJoint.GI2__JA_MIN, ltd);
				dCopyVector3(J2A, J2Ofs + DxJoint.GI2__JA_MIN, ltd);
			}

			// if we're limited low and high simultaneously, the joint motor is
			// ineffective
			if ( limit!=0 && ( lostop == histop ) ) powered = false;

			if ( powered )
			{
				pairRhsCfmA[pairRhsCfmOfs + GI2_CFM] = normal_cfm;
				if ( limit == 0 )
				{
					pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = vel;
					pairLoHiA[pairLoHiOfs + GI2_LO] = -fmax;
					pairLoHiA[pairLoHiOfs + GI2_HI] = fmax;
				}
				else
				{
					// the joint is at a limit, AND is being powered. if the joint is
					// being powered into the limit then we apply the maximum motor force
					// in that direction, because the motor is working against the
					// immovable limit. if the joint is being powered away from the limit
					// then we have problems because actually we need *two* lcp
					// constraints to handle this case. so we fake it and apply some
					// fraction of the maximum force. the fraction to use can be set as
					// a fudge factor.

					double fm = fmax;
					if (( vel > 0 ) || ( vel == 0 && limit == 2 ) ) fm = -fm;

					// if we're powering away from the limit, apply the fudge factor
					if (( limit == 1 && vel > 0 ) || ( limit == 2 && vel < 0 ) ) fm *= fudge_factor;

	                
					double fm_ax1_0 = fm*ax1.get0(), fm_ax1_1 = fm*ax1.get1(), fm_ax1_2 = fm*ax1.get2();
	                
	                DxBody b0 = joint.node[0].body;
	                DxWorldProcessContext world_process_context = b0.world.UnsafeGetWorldProcessingContext(); 

	                world_process_context.LockForAddLimotSerialization();

					if ( rotational )
					{
	                    b1 = joint.node[1].body;
						if ( b1 != null) {
							b1.dBodyAddTorque( fm_ax1_0, fm_ax1_1, fm_ax1_2 );
						}
	                    b0.dBodyAddTorque( -fm_ax1_0, -fm_ax1_1, -fm_ax1_2 );
					}
					else
					{
	                    b1 = joint.node[1].body;
						if ( b1 != null)
						{
	                        DVector3 neg_fm_ltd_0 = ltd.scale(-fm); 
							// linear limot torque decoupling step: refer to above discussion
							b0.dBodyAddTorque( neg_fm_ltd_0 );
							b1.dBodyAddTorque( neg_fm_ltd_0 );

							b1.dBodyAddForce( fm_ax1_0, fm_ax1_1, fm_ax1_2 );
						}
	                    b0.dBodyAddForce( -fm_ax1_0, -fm_ax1_1, -fm_ax1_2 );
					}
	                world_process_context.UnlockForAddLimotSerialization();
				}
			}

			if ( limit != 0 )
			{
				double k = fps * stop_erp;
				pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = -k * limit_err;
				pairRhsCfmA[pairRhsCfmOfs + GI2_CFM] = stop_cfm;

				if ( lostop == histop )
				{
					// limited low and high simultaneously
					pairLoHiA[pairLoHiOfs + GI2_LO] = -dInfinity;
					pairLoHiA[pairLoHiOfs + GI2_HI] = dInfinity;
				}
				else
				{
					if ( limit == 1 )
					{
						// low limit
						pairLoHiA[pairLoHiOfs + GI2_LO] = 0;
						pairLoHiA[pairLoHiOfs + GI2_HI] = dInfinity;
					}
					else
					{
						// high limit
						pairLoHiA[pairLoHiOfs + GI2_LO] = -dInfinity;
						pairLoHiA[pairLoHiOfs + GI2_HI] = 0;
					}

					// deal with bounce
					if ( bounce > 0 )
					{
						// calculate joint velocity
						double vel;
						if ( rotational )
						{
							vel = joint.node[0].body.avel.dot( ax1 );
							if ( joint.node[1].body != null)
								vel -= joint.node[1].body.avel.dot( ax1 );
						}
						else
						{
							vel = joint.node[0].body.lvel.dot( ax1 );
							if ( joint.node[1].body != null)
								vel -= joint.node[1].body.lvel.dot( ax1 );
						}

						// only apply bounce if the velocity is incoming, and if the
						// resulting c[] exceeds what we already have.
						if ( limit == 1 )
						{
							// low limit
							if ( vel < 0 )
							{
								double newc = -bounce * vel;
								if (newc > pairRhsCfmA[pairRhsCfmOfs + GI2_RHS])
									pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = newc;
							}
						}
						else
						{
							// high limit - all those computations are reversed
							if ( vel > 0 )
							{
								double newc = -bounce * vel;
								if (newc < pairRhsCfmA[pairRhsCfmOfs + GI2_RHS])
									pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = newc;
							}
						}
					}
				}
			}
			return true;
		}
		else return false;
	}

	// bool addTwoPointLimot( dxJoint *joint, dReal fps,
	// 					   dReal *J1, dReal *J2, dReal *pairRhsCfm, dReal *pairLoHi,
    //    const dVector3 ax1, const dVector3 pt1, const dVector3 pt2 );
	/**
	 * This function generalizes the "linear limot torque decoupling"
	 * in addLimot to use anchor points provided by the caller.
	 * 
	 * This makes it so that the appropriate torques are applied to
	 * a body when it's being linearly motored or limited using anchor points
	 * that aren't at the center of mass.
	 * 
	 * pt1 and pt2 are centered in body coordinates but use global directions.
	 * I.e., they are conveniently found within joint code with:
	 *   getAxis(joint,pt1,anchor1);
	 *   getAxis2(joint,pt2,anchor2);
	 */
	boolean addTwoPointLimot(DxJoint joint, double fps, double[] J1A, int J1Ofs, double[] J2A, int J2Ofs,
						 double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA, int pairLoHiOfs, DVector3C ax1,
						 DVector3C pt1, DVector3C pt2) {
		// if the joint is powered, or has joint limits, add in the extra row
		boolean powered = fmax > 0;
		DVector3 j1a = new DVector3();
		DVector3 j2l = new DVector3();
		DVector3 j2a = new DVector3();
		if ( powered || limit != 0)
		{
			// Set the linear portion
			dCopyVector3(J1A, J1Ofs + GI2__JL_MIN, ax1);
			// Set the angular portion (to move the linear constraint
			// away from the center of mass).
			dCalcVectorCross3(J1A, J1Ofs + GI2__JA_MIN, pt1, ax1);
			// Set the constraints for the second body
			if ( joint.node[1].body != null ) {
				dCopyNegatedVector3(J2A, J2Ofs + GI2__JL_MIN, ax1);
				dCalcVectorCross3(J2A, J2Ofs + GI2__JA_MIN, pt2, J2A, J2Ofs + GI2__JL_MIN);
			}

			// if we're limited low and high simultaneously, the joint motor is
			// ineffective
			if ( limit != 0 && ( lostop == histop ) ) powered = false;

			if ( powered )
			{
				pairRhsCfmA[pairRhsCfmOfs + GI2_CFM] = normal_cfm;
				if ( limit==0 )
				{
					pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = vel;
					pairLoHiA[pairLoHiOfs + GI2_LO] = -fmax;
					pairLoHiA[pairLoHiOfs + GI2_HI] = fmax;
				}
				else
				{
					// the joint is at a limit, AND is being powered. if the joint is
					// being powered into the limit then we apply the maximum motor force
					// in that direction, because the motor is working against the
					// immovable limit. if the joint is being powered away from the limit
					// then we have problems because actually we need *two* lcp
					// constraints to handle this case. so we fake it and apply some
					// fraction of the maximum force. the fraction to use can be set as
					// a fudge factor.

					double fm = fmax;
					if (( vel > 0 ) || ( vel == 0 && limit == 2 ) ) fm = -fm;

					// if we're powering away from the limit, apply the fudge factor
					if (( limit == 1 && vel > 0 ) || ( limit == 2 && vel < 0 ) ) fm *= fudge_factor;


					DxBody b0 = joint.node[0].body;
					int tAx1Ofs = J1Ofs + GI2__JA_MIN;
					b0.dBodyAddForce(-fm * ax1.get(dSA_X), -fm * ax1.get(dSA_Y), -fm * ax1.get(dSA_Z));
					b0.dBodyAddTorque(-fm * J1A[tAx1Ofs + dSA_X], -fm * J1A[tAx1Ofs + dSA_Y],
							-fm * J1A[tAx1Ofs + dSA_Z]);

					DxBody b1 = joint.node[1].body;
					if (b1 != null) {
						final int tAx2 = J2Ofs + GI2__JA_MIN;
						b1.dBodyAddForce(fm * ax1.get(dSA_X), fm * ax1.get(dSA_Y), fm * ax1.get(dSA_Z));
						b1.dBodyAddTorque(-fm * J2A[tAx2 + dSA_X], -fm * J2A[tAx2 + dSA_Y], -fm * J2A[tAx2 + dSA_Z]);
					}

				}
			}

			if ( limit!=0 )
			{
				double k = fps * stop_erp;
				pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = -k * limit_err;
				pairRhsCfmA[pairRhsCfmOfs + GI2_CFM] = stop_cfm;

				if ( lostop == histop )
				{
					// limited low and high simultaneously
					pairLoHiA[pairLoHiOfs + GI2_LO] = -dInfinity;
					pairLoHiA[pairLoHiOfs + GI2_HI] = dInfinity;
				}
				else
				{
					if ( limit == 1 )
					{
						// low limit
						pairLoHiA[pairLoHiOfs + GI2_LO] = 0;
						pairLoHiA[pairLoHiOfs + GI2_HI] = dInfinity;
					}
					else
					{
						// high limit
						pairLoHiA[pairLoHiOfs + GI2_LO] = -dInfinity;
						pairLoHiA[pairLoHiOfs + GI2_HI] = 0;
					}

					// deal with bounce
					if ( bounce > 0 )
					{
						// calculate relative velocity of the two anchor points
						DxBody b0 = joint.node[0].body;
						double vel =
								dCalcVectorDot3( b0.lvel, J1A, J1Ofs + GI2__JL_MIN ) +
								dCalcVectorDot3( b0.avel, J1A, J1Ofs + GI2__JA_MIN );
						DxBody b1 = joint.node[1].body;
						if (joint.node[1].body != null) {
							vel +=
								dCalcVectorDot3( b1.lvel, J2A, J2Ofs + GI2__JL_MIN ) +
								dCalcVectorDot3( b1.avel, J2A, J2Ofs + GI2__JA_MIN );
						}

						// only apply bounce if the velocity is incoming, and if the
						// resulting c[] exceeds what we already have.
						if ( limit == 1 )
						{
							// low limit
							if ( vel < 0 )
							{
								double newc = -bounce * vel;
								if (newc > pairRhsCfmA[pairRhsCfmOfs + GI2_RHS])
									pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = newc;
							}
						}
						else
						{
							// high limit - all those computations are reversed
							if ( vel > 0 )
							{
								double newc = -bounce * vel;
								if (newc < pairRhsCfmA[pairRhsCfmOfs + GI2_RHS])
									pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = newc;
							}
						}
					}
				}
			}
			return true;
		}
		else return false;
	}

	
	//************** TZ stuff  ****************
	
	
	public int getLimit() {
		return limit;
	}


	public void setLimit(int i) {
		limit = i;
	}


	public double getLostop() {
		return lostop;
	}


}
