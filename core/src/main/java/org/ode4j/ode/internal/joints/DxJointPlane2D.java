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

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DPlane2DJoint;
import org.ode4j.ode.internal.DxWorld;

import static org.ode4j.ode.internal.joints.JointEnums.GI2_JLZ;


/**
 * Plane2D.
 * 2d joint, constrains to z == 0
 * 
 * This code is part of the Plane2D ODE joint
 * by psero@gmx.de
 * Wed Apr 23 18:53:43 CEST 2003
 */
public class DxJointPlane2D extends DxJoint implements DPlane2DJoint
{
	int                 row_motor_x;
	int                 row_motor_y;
	int                 row_motor_angle;
	DxJointLimitMotor   motor_x = new DxJointLimitMotor();
	DxJointLimitMotor   motor_y = new DxJointLimitMotor();
	DxJointLimitMotor   motor_angle = new DxJointLimitMotor();


	//private static final double[][] Midentity =
	//{
	//    {   1,  0,  0   },
	//    {   0,  1,  0   },
	//    {   0,  0,  1,  }
	//}
	private static final DVector3C Midentity0 = new DVector3( 1,  0,  0 );
	private static final DVector3C Midentity1 = new DVector3( 0,  1,  0 );
	private static final DVector3C Midentity2 = new DVector3( 0,  0,  1 );


	DxJointPlane2D( DxWorld w )// :
	//dxJoint( w )
	{
		super(w);
		motor_x.init( world );
		motor_y.init( world );
		motor_angle.init( world );
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

		if ( motor_x.fmax > 0 ) {
			row_motor_x = info.m++;
		} else {
			row_motor_x = 0;
		}
		if ( motor_y.fmax > 0 ) {
			row_motor_y = info.m++;
		} else {
			row_motor_y = 0;
		}
		if ( motor_angle.fmax > 0 ) {
			row_motor_angle = info.m++;
		} else {
			row_motor_angle = 0;
		}
	}


	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		double       eps = worldFPS * worldERP;

		/*
        v = v1, w = omega1
        (v2, omega2 not important (== static environment))

        constraint equations:
            vz = 0
            wx = 0
            wy = 0

        <=> ( 0 0 1 ) (vx)   ( 0 0 0 ) (wx)   ( 0 )
            ( 0 0 0 ) (vy) + ( 1 0 0 ) (wy) = ( 0 )
            ( 0 0 0 ) (vz)   ( 0 1 0 ) (wz)   ( 0 )
            J1/J1l           Omega1/J1a
		 */

		// fill in linear and angular coeff. for left hand side:

		J1A[J1Ofs + GI2_JLZ] = 1;
		J1A[J1Ofs + rowskip + GI2_JAX] = 1;
		J1A[J1Ofs + 2 * rowskip + GI2_JAY] = 1;

		// error correction (against drift):

		// a) linear vz, so that z (== pos[2]) == 0
		pairRhsCfmA[pairRhsCfmOfs + GI2_RHS] = eps * -node[0].body.posr().pos().get2();

		//# if 0
		//    // b) angular correction? -> left to application !!!
		//    dReal       *body_z_axis = &node[0].body->R[8];
		//		pairRhsCfm[pairskip + GI2_RHS] = eps * + atan2( body_z_axis[1], body_z_axis[2] );  // wx error
		//		pairRhsCfm[2 * pairskip + GI2_RHS] = eps * -atan2( body_z_axis[0], body_z_axis[2] );  // wy error
		//# endif

		// if the slider is powered, or has joint limits, add in the extra row:

		if (row_motor_x > 0) {
			int currRowSkip = row_motor_x * rowskip, currPairSkip = row_motor_x * pairskip;
			motor_x.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, Midentity0, false);
		}

		if (row_motor_y > 0) {
			int currRowSkip = row_motor_y * rowskip, currPairSkip = row_motor_y * pairskip;
			motor_y.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, Midentity1, false);
		}

		if (row_motor_angle > 0) {
			int currRowSkip = row_motor_angle * rowskip, currPairSkip = row_motor_angle * pairskip;
			motor_angle.addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, Midentity2, true);
		}
	}


	public void dJointSetPlane2DXParam( 
			PARAM_N parameter, double value )
	{
		motor_x.set( parameter.toSUB(), value );
	}


	public void dJointSetPlane2DYParam( 
			PARAM_N parameter, double value )
	{
		motor_y.set( parameter.toSUB(), value );
	}



	void dJointSetPlane2DAngleParam(
			PARAM_N parameter, double value )
	{
		motor_angle.set( parameter.toSUB(), value );
	}


	// ***********************************
	// API dJoint
	// ***********************************

	@Override
	public double getParam(PARAM_N parameter) {
		// use 1 for x and 2 for y ?
		switch (parameter) {
		case dParamFMax1:
		case dParamVel1:
			return motor_x.get(parameter.toSUB());
		case dParamFMax2:
		case dParamVel2:
			return motor_y.get(parameter.toSUB());

		default:
			throw new UnsupportedOperationException();
		}
	}


	@Override
	public void setParam(PARAM_N parameter, double value) {
		// use 1 for x and 2 for y ?
		switch (parameter) {
		case dParamFMax1:
		case dParamVel1:
			motor_x.set(parameter.toSUB(), value);
			break;
		case dParamFMax2:
		case dParamVel2:
			motor_y.set(parameter.toSUB(), value);
			break;

		default:
			throw new UnsupportedOperationException();
		}
	}



	@Override
	public double getXParamFMax() {
		return motor_x.get( PARAM.dParamFMax );
	}

	@Override
	public double getYParamFMax() {
		return motor_y.get( PARAM.dParamFMax );
	}

	@Override
	public void setXParamFMax(double d) {
		dJointSetPlane2DXParam(PARAM_N.dParamFMax1, d);
	}

	@Override
	public void setYParamFMax(double d) {
		dJointSetPlane2DYParam(PARAM_N.dParamFMax1, d);
	}

	@Override
	public double getXParamVel() {
		return motor_x.get( PARAM.dParamVel );
	}

	@Override
	public double getYParamVel() {
		return motor_y.get( PARAM.dParamVel );
	}

	@Override
	public void setXParamVel(double d) {
		dJointSetPlane2DXParam(PARAM_N.dParamVel1, d);
	}

	@Override
	public void setYParamVel(double d) {
		dJointSetPlane2DYParam(PARAM_N.dParamVel1, d);
	}

	@Override
	public void setAngleParam(PARAM parameter, double value) {
		motor_angle.set(parameter, value);
	}
}

