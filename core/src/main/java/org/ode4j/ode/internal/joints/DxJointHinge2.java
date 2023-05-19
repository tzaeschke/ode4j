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
import static org.ode4j.ode.internal.Common.*;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.internal.CommonEnums;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;


/**
 * **************************************************************************
 * hinge 2. note that this joint must be attached to two bodies for it to work
 */
public class DxJointHinge2 extends DxJoint implements DHinge2Joint {

	private final DVector3 anchor1;   // anchor w.r.t first body
	private final DVector3 anchor2;   // anchor w.r.t second body
	private final DVector3 _axis1;     // axis 1 w.r.t first body
	private final DVector3 _axis2;     // axis 2 w.r.t second body
	double c0, s0;       // cos,sin of desired angle between axis 1,2
	private final DVector3 v1, v2;    // angle ref vectors embedded in first body
	private final DVector3 w1, w2;    // angle ref vectors embedded in second body
	DxJointLimitMotor limot1; // limit+motor info for axis 1
	DxJointLimitMotor limot2; // limit+motor info for axis 2
	double susp_erp, susp_cfm; // suspension parameters (erp,cfm)


	private double measureAngle1() {
	    // bring axis 2 into first body's reference frame
	    DVector3 p = new DVector3(), q = new DVector3();
	    if (node[1].body != null)
	        dMultiply0_331( p, node[1].body.posr().R(), _axis2 );
	    else
	        p.set(_axis2);

	    if (node[0].body != null)
	        dMultiply1_331( q, node[0].body.posr().R(), p );
	    else
	        q.set(p);

	    double x = dCalcVectorDot3( v1, q );
	    double y = dCalcVectorDot3( v2, q );
	    return -dAtan2( y, x );
	}

	private double measureAngle2() {
	    // bring axis 1 into second body's reference frame
	    DVector3 p = new DVector3(), q = new DVector3();
	    if (node[0].body != null)
	        dMultiply0_331( p, node[0].body.posr().R(), _axis1 );
	    else
	        p.set(_axis1);

	    if (node[1].body != null)
	        dMultiply1_331( q, node[1].body.posr().R(), p );
	    else
	        q.set(p);

	    double x = dCalcVectorDot3( w1, q );
	    double y = dCalcVectorDot3( w2, q );
	    return -dAtan2( y, x );
	}

	DxJointHinge2( DxWorld w ) {
		super(w);

		anchor1 = new DVector3();
		anchor2 = new DVector3();
		_axis1 = new DVector3(1, 0, 0);
		
		_axis2 = new DVector3(0, 1, 0);
		c0 = 0;
		s0 = 0;

		v1 = new DVector3(1, 0, 0);
		v2 = new DVector3(0, 1, 0);

		//TZ
		w1 = new DVector3();
		w2 = new DVector3();
		
		limot1 = new DxJointLimitMotor();
		limot1.init( world );
		limot2 = new DxJointLimitMotor();
		limot2.init( world );

		susp_erp = world.getERP();
		susp_cfm = world.getCFM();

		//flags |= dJOINT_TWOBODIES;
		setFlagsTwoBodies();
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
		info.setM(4);
		info.setNub(4);

		// see if we're powered or at a joint limit for axis 1
		limot1.setLimit(0);
		if (( limot1.getLostop() >= -M_PI || limot1.histop <= M_PI ) &&
				limot1.getLostop() <= limot1.histop )
		{
			double angle = measureAngle1();
			limot1.testRotationalLimit( angle );
		}
		if ( limot1.getLimit()!=0 || limot1.fmax > 0 ) info.incM();

		// see if we're powering axis 2 (we currently never limit this axis)
		limot2.setLimit(0);
		if ( limot2.fmax > 0 ) info.incM();
	}


	/**
	 * Function that computes ax1,ax2 = axis 1 and 2 in global coordinates (they are
	 * relative to body 1 and 2 initially) and then computes the constrained
	 * rotational axis as the cross product of ax1 and ax2.
	 * the sin and cos of the angle between axis 1 and 2 is computed, this comes
	 * from dot and cross product rules.
	 * 
	 * @param ax1 Will contain the joint axis1 in world frame
	 * @param ax2 Will contain the joint axis2 in world frame
	 * @param axCross Will contain the cross product of ax1 x ax2
	 * @param sin_angle result sin angle
	 * @param cos_angle result cos angle
	 */
	private void getAxisInfo(DVector3 ax1, DVector3 ax2, DVector3 axCross,
	                           RefDouble sin_angle, RefDouble cos_angle)
	{
	    dMultiply0_331 (ax1, node[0].body.posr().R(), _axis1);
	    dMultiply0_331 (ax2, node[1].body.posr().R(), _axis2);
	    dCalcVectorCross3 (axCross ,ax1, ax2);
	    sin_angle.d = axCross.length();//dSqrt (axCross[0]*axCross[0] + axCross[1]*axCross[1] + axCross[2]*axCross[2]);
	    cos_angle.d = ax1.dot(ax2);//dDOT (ax1,ax2);
	}


	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		// get information we need to set the hinge row
		RefDouble s = new RefDouble(0), c = new RefDouble(0);
		DVector3 q = new DVector3();

		DVector3 ax1 = new DVector3(), ax2 = new DVector3();
		getAxisInfo( ax1, ax2, q, s, c);
		dNormalize3( q );   // @@@ quicker: divide q by s ?

		// set the three ball-and-socket rows (aligned to the suspension axis ax1)
		setBall2(this, worldFPS, worldERP, rowskip, J1A, J1Ofs, J2A, J2Ofs, pairskip, pairRhsCfmA, pairRhsCfmOfs,
				anchor1, anchor2, ax1, susp_erp);
		// set parameter for the suspension
		pairRhsCfmA[pairRhsCfmOfs + GI2_CFM] = susp_cfm;

		// set the hinge row
		int currRowSkip = 3 * rowskip;
		dCopyVector3(J1A, J1Ofs + currRowSkip + GI2__JA_MIN, q);
		if (node[1].body != null) {
			dCopyNegatedVector3(J2A, J2Ofs + currRowSkip + GI2__JA_MIN, q);
		}

		// compute the right hand side for the constrained rotational DOF.
		// axis 1 and axis 2 are separated by an angle `theta'. the desired
		// separation angle is theta0. sin(theta0) and cos(theta0) are recorded
		// in the joint structure. the correcting angular velocity is:
		//   |angular_velocity| = angle/time = erp*(theta0-theta) / stepsize
		//                      = (erp*fps) * (theta0-theta)
		// (theta0-theta) can be computed using the following small-angle-difference
		// approximation:
		//   theta0-theta ~= tan(theta0-theta)
		//                 = sin(theta0-theta)/cos(theta0-theta)
		//                 = (c*s0 - s*c0) / (c*c0 + s*s0)
		//                 = c*s0 - s*c0         assuming c*c0 + s*s0 ~= 1
		// where c = cos(theta), s = sin(theta)
		//       c0 = cos(theta0), s0 = sin(theta0)

		double k = worldFPS * worldERP;

		int currPairSkip = 3 * pairskip;
		pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * (c0 * s.get() - this.s0 * c.get());

		currRowSkip += rowskip;
		currPairSkip += pairskip;
		// if the axis1 hinge is powered, or has joint limits, add in more stuff
		if (limot1.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
				pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax1, true)) {
			currRowSkip += rowskip;
			currPairSkip += pairskip;
		}

		// if the axis2 hinge is powered, add in more stuff
		limot2.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
				pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax2, true);
	}


	// compute vectors v1 and v2 (embedded in body1), used to measure angle
	// between body 1 and body 2

	private void makeV1andV2()
	{
		if ( node[0].body != null)
		{
			// get axis 1 and 2 in global coords
			DVector3 ax1 = new DVector3(), ax2 = new DVector3(), v = new DVector3();
			dMultiply0_331( ax1, node[0].body.posr().R(), _axis1 );
			dMultiply0_331( ax2, node[1].body.posr().R(), _axis2 );

			// modify axis 2 so it's perpendicular to axis 1
			double k = dCalcVectorDot3(ax1, ax2);
			dAddVectorScaledVector3(ax2, ax2, ax1, -k);

			if (dxSafeNormalize3(ax2)) {
				// make v1 = modified axis2, v2 = axis1 x (modified axis2)
				dCalcVectorCross3(v, ax1, ax2);
				dMultiply1_331(v1, node[0].body.posr().R(), ax2);
				dMultiply1_331(v2, node[0].body.posr().R(), v);
			} else {
				dUASSERT(false, "Hinge2 axes must be chosen to be linearly independent");
			}
		}
	}

	// same as above, but for the second axis
	private void makeW1andW2()
	{
	    if ( node[1].body != null )
	    {
	        // get axis 1 and 2 in global coords
	        DVector3 ax1 = new DVector3(), ax2 = new DVector3(), w = new DVector3();
	        dMultiply0_331( ax1, node[0].body.posr().R(), _axis1 );
	        dMultiply0_331( ax2, node[1].body.posr().R(), _axis2 );

	        // modify axis 1 so it's perpendicular to axis 2
	        double k = dCalcVectorDot3( ax2, ax1 );
			dAddVectorScaledVector3(ax1, ax1, ax2, -k);

			if (dxSafeNormalize3(ax1)) {
				// make w1 = modified axis1, w2 = axis2 x (modified axis1)
				dCalcVectorCross3(w, ax2, ax1);
				dMultiply1_331(w1, node[1].body.posr().R(), ax1);
				dMultiply1_331(w2, node[1].body.posr().R(), w);
			} else {
				dUASSERT(false, "Hinge2 axes must be chosen to be linearly independent");
			}
	    }
	}

	public void dJointSetHinge2Anchor( DVector3C xyz ) {
		setAnchors( xyz, anchor1, anchor2 );
		makeV1andV2();
		makeW1andW2();
	}

	/*ODE_API */
	void dJointSetHinge2Axes(final DVector3C axis1/*=[dSA__MAX],=NULL*/, final DVector3C axis2/*=[dSA__MAX],
	=NULL*/) {
		dAASSERT(axis1 != null || axis2 != null);
		dAASSERT(node[0].body != null || axis1 == null);
		dAASSERT(node[1].body != null || axis2 == null);

		CommonEnums.dAssertVec3Element();
		if (axis1 != null) {
			// setAxes(axis1[dSA_X], axis1[dSA_Y], axis1[dSA_Z], axis1, null);
			setAxes(axis1.get0(), axis1.get1(), axis1.get2(), this._axis1, null);
		}

		if (axis2 != null) {
			// setAxes(axis2[dSA_X], axis2[dSA_Y], axis2[dSA_Z], null, axis2);
			setAxes(axis2.get0(), axis2.get1(), axis2.get2(), null, this._axis2);
		}

		// compute the sin and cos of the angle between axis 1 and axis 2
		DVector3 ax1 = new DVector3(), ax2 = new DVector3(), ax = new DVector3();
		RefDouble s0Ref = new RefDouble(s0), c0Ref = new RefDouble(c0);
		getAxisInfo(ax1, ax2, ax, s0Ref, c0Ref);
		s0 = s0Ref.get();
		c0 = c0Ref.get();

		makeV1andV2();
		makeW1andW2();
	}


	/*ODE_API_DEPRECATED ODE_API */
	public void dJointSetHinge2Axis1(double x, double y, double z) {
		CommonEnums.dAssertVec3Element();
		DVector3 axis1 = new DVector3(x, y, z);
		//		axis1[dSA_X] = x;
		//		axis1[dSA_Y] = y;
		//		axis1[dSA_Z] = z;
		dJointSetHinge2Axes(axis1, null);
	}

	/*ODE_API_DEPRECATED ODE_API */
	void dJointSetHinge2Axis2(double x, double y, double z) {
		CommonEnums.dAssertVec3Element();
		DVector3 axis2 = new DVector3(x, y, z);
		//		axis2[dSA_X] = x;
		//		axis2[dSA_Y] = y;
		//		axis2[dSA_Z] = z;
		dJointSetHinge2Axes(null, axis2);
	}


	//	public void dJointSetHinge2Param( dxJointHinge2 j,
//			D_PARAM_NAMES parameter, double value )
	public void dJointSetHinge2Param( PARAM_N parameter, double value )
	{
		//if ( parameter.and(0xff00).eq(0x100) )
		if (parameter.isGroup2())
		{
			limot2.set( parameter.toSUB(), value );
		}
		else
		{
			if ( parameter.toSUB() == PARAM.dParamSuspensionERP ) susp_erp = value;
			else if ( parameter.toSUB() == PARAM.dParamSuspensionCFM ) susp_cfm = value;
			else limot1.set( parameter.toSUB(), value );
		}
	}


//	private void dJointGetHinge2Anchor( dJoint j, dVector3 result )
	private void dJointGetHinge2Anchor( DVector3 result )
	{
		if ( isFlagsReverse() )
			getAnchor2( result, anchor2 );
		else
			getAnchor( result, anchor1 );
	}


//	private void dJointGetHinge2Anchor2( dJoint j, dVector3 result )
	private void dJointGetHinge2Anchor2( DVector3 result )
	{
		if ( isFlagsReverse() )
			getAnchor( result, anchor1 );
		else
			getAnchor2( result, anchor2 );
	}


//	private void dJointGetHinge2Axis1( dJoint j, dVector3 result )
	private void dJointGetHinge2Axis1( DVector3 result )
	{
		if ( node[0].body != null)
		{
			dMultiply0_331( result, node[0].body.posr().R(), _axis1 );
		}
		else
		{
			// dZeroVector3(result);
			result.setZero();
			dUASSERT( false, "the joint does not have first body attached" );
		}
	}


//	private void dJointGetHinge2Axis2( dJoint j, dVector3 result )
	private void dJointGetHinge2Axis2( DVector3 result )
	{
		if ( node[1].body!= null )
		{
			dMultiply0_331( result, node[1].body.posr().R(), _axis2 );
		}
		else
		{
			// dZeroVector3(result);
			result.setZero();
			dUASSERT( false, "the joint does not have second body attached" );
		}
	}


//	private double dJointGetHinge2Param( dJoint j, D_PARAM_NAMES_N parameter )
	private double dJointGetHinge2Param( PARAM_N parameter )
	{
		if ( parameter.isGroup2())//and(0xff00).eq(0x100) )
		{
			return limot2.get( parameter.toSUB());
		}
		else
		{
			if ( parameter.toSUB() == PARAM.dParamSuspensionERP ) return susp_erp;
			else if ( parameter.toSUB() == PARAM.dParamSuspensionCFM ) return susp_cfm;
			else return limot1.get( parameter.toSUB() );
		}
	}


	public double dJointGetHinge2Angle1()
	{
		return measureAngle1();
	}

	public double dJointGetHinge2Angle2()
	{
	    return measureAngle2();
	}

	public double dJointGetHinge2Angle1Rate()
	{
		if ( node[0].body != null )
		{
			DVector3 axis = new DVector3();
			dMultiply0_331( axis, node[0].body.posr().R(), _axis1 );
			double rate = dCalcVectorDot3( axis, node[0].body.avel );
			if ( node[1].body != null )
				rate -= dCalcVectorDot3( axis, node[1].body.avel );
			return rate;
		}
		else return 0;
	}


	public double dJointGetHinge2Angle2Rate()
	{
		if ( node[0].body != null && node[1].body != null )
		{
			DVector3 axis = new DVector3();
			dMultiply0_331( axis, node[1].body.posr().R(), _axis2 );
			double rate = dCalcVectorDot3( axis, node[0].body.avel );
			if ( node[1].body != null )
				rate -= dCalcVectorDot3( axis, node[1].body.avel );
			return rate;
		}
		else return 0;
	}


//	private void dJointAddHinge2Torques( dJoint j, double torque1, double torque2 )
	private void dJointAddHinge2Torques( double torque1, double torque2 )
	{
		if ( node[0].body != null && node[1].body != null)
		{
			DVector3 axis1 = new DVector3(), axis2 = new DVector3();
			dMultiply0_331( axis1, node[0].body.posr().R(), _axis1 );
			dMultiply0_331( axis2, node[1].body.posr().R(), _axis2 );
//			axis1.v[0] = axis1.v[0] * torque1 + axis2.v[0] * torque2;
//			axis1.v[1] = axis1.v[1] * torque1 + axis2.v[1] * torque2;
//			axis1.v[2] = axis1.v[2] * torque1 + axis2.v[2] * torque2;
			axis1.eqSum(axis1, torque1, axis2, torque2);
			node[0].body.dBodyAddTorque( axis1 );
			node[1].body.dBodyAddTorque( axis1.reScale(-1) );
		}
	}


	@Override
	void setRelativeValues()
	{
	    DVector3 anchor = new DVector3();
	    dJointGetHinge2Anchor(anchor);
	    setAnchors( anchor, anchor1, anchor2 );

	    DVector3 axis = new DVector3();

	    if ( node[0].body != null )
	    {
	        dJointGetHinge2Axis1(axis);
	        setAxes( axis, _axis1, null );
	    }

	    if ( node[0].body != null )
	    {
	        dJointGetHinge2Axis2(axis);
	        setAxes( axis, null, _axis2 );
	    }

	    DVector3 ax1 = new DVector3(), ax2 = new DVector3();
	    RefDouble s0R = new RefDouble(s0), c0R = new RefDouble(c0);
	    getAxisInfo( ax1, ax2, axis, s0R, c0R );
	    s0 = s0R.d;
	    c0 = c0R.d;

	    makeV1andV2();
	    makeW1andW2();
	}

	
	public DxJointLimitMotor getLimot1() {
		return limot1;
	}

	public DxJointLimitMotor getLimot2() {
		return limot2;
	}
	
	// ****************************
	// API dHinge2Joint
	// ****************************

	@Override
	public void setAnchor (double x, double y, double z)
	{ dJointSetHinge2Anchor (new DVector3(x, y, z)); }
	@Override
	public void setAnchor (final DVector3C a)
	{ dJointSetHinge2Anchor(a); }
	@Override
	public void setAxes (final DVector3C a, final DVector3C b)
	{ dJointSetHinge2Axes(a, b); }
	@Override
	public void setAxes (double ax, double ay, double az, double bx, double by, double bz)
	{ dJointSetHinge2Axes(new DVector3(ax, ay, az), new DVector3(bx, by, bz)); }
	@Deprecated // Please use setAxes()
	@Override
	public void setAxis1 (double x, double y, double z)
	{ dJointSetHinge2Axis1 (x, y, z); }
	@Deprecated // Please use setAxes()
	@Override
	public void setAxis1 (final DVector3C a)
	{ setAxis1 (a.get0(), a.get1(), a.get2()); }
	@Deprecated // Please use setAxes()
	@Override
	public void setAxis2 (double x, double y, double z)
	{ dJointSetHinge2Axis2 (x, y, z); }
	@Deprecated // Please use setAxes()
	@Override
	public void setAxis2 (final DVector3C a)
	{ setAxis2 (a.get0(), a.get1(), a.get2()); }

	@Override
	public void getAnchor (DVector3 result)
	{ dJointGetHinge2Anchor (result); }
	@Override
	public void getAnchor2 (DVector3 result)
	{ dJointGetHinge2Anchor2 (result); }
	@Override
	public void getAxis1 (DVector3 result)
	{ dJointGetHinge2Axis1 (result); }
	@Override
	public void getAxis2 (DVector3 result)
	{ dJointGetHinge2Axis2 (result); }

	@Override
	public double getAngle1()
	{ return dJointGetHinge2Angle1 (); }
	@Override
	public double getAngle2()
	{ return dJointGetHinge2Angle2 (); }
	@Override
	public double getAngle1Rate()
	{ return dJointGetHinge2Angle1Rate (); }
	@Override
	public double getAngle2Rate()
	{ return dJointGetHinge2Angle2Rate (); }

	@Override
	public void setParam (PARAM_N parameter, double value)
	{ dJointSetHinge2Param (parameter, value); }
	@Override
	public double getParam (PARAM_N parameter)
	{ return dJointGetHinge2Param (parameter); }

	@Override
	public void addTorques(double torque1, double torque2)
	{ dJointAddHinge2Torques(torque1, torque2); }


	@Override
	public void setParamFMax2(double d) {
		dJointSetHinge2Param(PARAM_N.dParamFMax2, d);
	}


	@Override
	public void setParamFudgeFactor(double d) {
		dJointSetHinge2Param(PARAM_N.dParamFudgeFactor1, d);
	}


	@Override
	public void setParamHiStop(double d) {
		dJointSetHinge2Param(PARAM_N.dParamHiStop1, d);
	}


	@Override
	public void setParamLoStop(double d) {
		dJointSetHinge2Param(PARAM_N.dParamLoStop1, d);
	}


	@Override
	public void setParamFMax(double d) {
		dJointSetHinge2Param(PARAM_N.dParamFMax1, d);
	}


	@Override
	public void setParamSuspensionCFM(double d) {
		dJointSetHinge2Param(PARAM_N.dParamSuspensionCFM1, d);
	}


	@Override
	public void setParamSuspensionERP(double d) {
		dJointSetHinge2Param(PARAM_N.dParamSuspensionERP1, d);
	}


	@Override
	public void setParamVel(double d) {
		dJointSetHinge2Param(PARAM_N.dParamVel1, d);
	}


	@Override
	public void setParamVel2(double d) {
		dJointSetHinge2Param(PARAM_N.dParamVel2, d);
	}
}

