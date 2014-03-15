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


	//    # define        VoXYZ(v1, o1, x, y, z) \
	//private void VoXYZ(v1, o1, x, y, z) {
	//      (v1)[0] o1 (x), 
	//      (v1)[1] o1 (y), 
	//      (v1)[2] o1 (z)  
	//}
//	private void VoXYZeq(v1, x, y, z) {
//		(v1)[0] = (x), 
//		(v1)[1] = (y), 
//		(v1)[2] = (z)  
//	}

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



	@Override
	public void
	getInfo2( double worldFPS, double worldERP, DxJoint.Info2Descr info )
	{
		int r0 = 0;
		int r1 = info.rowskip();
		int r2 = 2 * r1;
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

		//    VoXYZ( info.J1l.v[r0], OP.EQ , 0, 0, 1 );
		//    VoXYZ( info.J1l.v[r1], OP.EQ , 0, 0, 0 );
		//    VoXYZ( info.J1l.v[r2], OP.EQ , 0, 0, 0 );
		//
		//    VoXYZ( info.J1a.v[r0], OP.EQ , 0, 0, 0 );
		//    VoXYZ( info.J1a.v[r1], OP.EQ , 1, 0, 0 );
		//    VoXYZ( info.J1a.v[r2], OP.EQ , 0, 1, 0 );
		int rx = info.J1lp + r0; 
		info._J[rx] = 0; info._J[rx+1] = 0; info._J[rx+2] = 1;
		rx = info.J1lp + r1;
		info._J[rx] = 0; info._J[rx+1] = 0; info._J[rx+2] = 0;
		rx = info.J1lp + r2;
		info._J[rx] = 0; info._J[rx+1] = 0; info._J[rx+2] = 0;
//		info.J1l.set(0, 0, 1,  0,0,0,   0,0,0);
//		info.J1a.set(0, 0, 0,  1,0,0,   0,1,0 );
		rx = info.J1ap + r0;
		info._J[rx] = 0; info._J[rx+1] = 0; info._J[rx+2] = 0;
		rx = info.J1ap + r1;
		info._J[rx] = 1; info._J[rx+1] = 0; info._J[rx+2] = 0;
		rx = info.J1ap + r2;
		info._J[rx] = 0; info._J[rx+1] = 1; info._J[rx+2] = 0;

		// error correction (against drift):

		// a) linear vz, so that z (== pos[2]) == 0
		info.setC(0, eps * -node[0].body.posr().pos().get2() );

		//# if 0
		//    // b) angular correction? -> left to application !!!
		//    dReal       *body_z_axis = &node[0].body->R[8];
		//    info->c[1] = eps * + atan2( body_z_axis[1], body_z_axis[2] );  // wx error
		//    info->c[2] = eps * -atan2( body_z_axis[0], body_z_axis[2] );  // wy error
		//# endif

		// if the slider is powered, or has joint limits, add in the extra row:

		if ( row_motor_x > 0 )
			motor_x.addLimot( this, worldFPS, info, row_motor_x, Midentity0, false );

		if ( row_motor_y > 0 )
			motor_y.addLimot( this, worldFPS, info, row_motor_y, Midentity1, false );

		if ( row_motor_angle > 0 )
			motor_angle.addLimot( this, worldFPS, info, row_motor_angle, Midentity2, true );
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
		//TODO use 1 for x and 2 for y ?
		throw new UnsupportedOperationException();
	}


	@Override
	public void setParam(PARAM_N parameter, double value) {
		// TODO use 1 for x and 2 for y ?
		throw new UnsupportedOperationException();
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

	/** @deprecated TZ do not use. */
	@Override
	public void setXParam(PARAM parameter, double value) {
		motor_x.set(parameter, value);
	}

	/** @deprecated TZ do not use. */
	@Override
	public void setYParam(PARAM parameter, double value) {
		motor_y.set(parameter, value);
	}



	@Override
	public void setAngleParam(PARAM parameter, double value) {
		motor_angle.set(parameter, value);
	}
}

