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

import org.ode4j.math.DVector3;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.Common.D_PARAM_NAMES_N;



//****************************************************************************
// Plane2D
// 2d joint, constrains to z == 0
/**
    This code is part of the Plane2D ODE joint
    by psero@gmx.de
    Wed Apr 23 18:53:43 CEST 2003
 */
public class DxJointPlane2D extends DxJoint
{
	int                 row_motor_x;
	int                 row_motor_y;
	int                 row_motor_angle;
	DxJointLimitMotor   motor_x = new DxJointLimitMotor();
	DxJointLimitMotor   motor_y = new DxJointLimitMotor();
	DxJointLimitMotor   motor_angle = new DxJointLimitMotor();


	//    dxJointPlane2D( dxWorld w );
	//    virtual void getInfo1( Info1* info );
	//    virtual void getInfo2( Info2* info );
	//    virtual dJointType type() const;
	//    virtual size_t size() const;


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
	private static final DVector3[] Midentity =
	{
		new DVector3( 1,  0,  0 ),
		new DVector3( 0,  1,  0 ),
		new DVector3( 0,  0,  1 )
	};


	DxJointPlane2D( DxWorld w )// :
	//dxJoint( w )
	{
		super(w);
		motor_x.init( world );
		motor_y.init( world );
		motor_angle.init( world );
	}



	public void
	getInfo1( DxJoint.Info1 info )
	{
	    info.setM(3);
	    info.setNub(3);

		if ( motor_x.fmax > 0 )
			row_motor_x = info.m ++;
		if ( motor_y.fmax > 0 )
			row_motor_y = info.m ++;
		if ( motor_angle.fmax > 0 )
			row_motor_angle = info.m ++;
	}



	public void
	getInfo2( DxJoint.Info2 info )
	{
		int r0 = 0;
		int r1 = info.rowskip();
		int r2 = 2 * r1;
		double       eps = info.fps * info.erp;

		/*
        v = v1, w = omega1
        (v2, omega2 not important (== static environment))

        constraint equations:
            xz = 0
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

		//TODO report TZ why only in direction 'z'??
		// a) linear vz, so that z (== pos[2]) == 0
		info.setC(0, eps * -node[0].body._posr.pos.get2() );

		//# if 0
		//    // b) angular correction? -> left to application !!!
		//    dReal       *body_z_axis = &node[0].body->R[8];
		//    info->c[1] = eps * + atan2( body_z_axis[1], body_z_axis[2] );  // wx error
		//    info->c[2] = eps * -atan2( body_z_axis[0], body_z_axis[2] );  // wy error
		//# endif

		// if the slider is powered, or has joint limits, add in the extra row:

		if ( row_motor_x > 0 )
			motor_x.addLimot( this, info, row_motor_x, Midentity[0], false );

		if ( row_motor_y > 0 )
			motor_y.addLimot( this, info, row_motor_y, Midentity[1], false );

		if ( row_motor_angle > 0 )
			motor_angle.addLimot( this, info, row_motor_angle, Midentity[2], true );
	}


//	public void dJointSetPlane2DXParam( dxJoint joint,
//			D_PARAM_NAMES parameter, double value )
	public void dJointSetPlane2DXParam( 
			D_PARAM_NAMES_N parameter, double value )
	{
		//COM.dUASSERT( joint, "bad joint argument" );
		//checktype( joint, dxJointPlane2D.class );
		//dxJointPlane2D joint2d = ( dxJointPlane2D )( joint );
		motor_x.set( parameter.toSUB(), value );
	}


//	void dJointSetPlane2DYParam( dxJoint joint,
//			D_PARAM_NAMES parameter, double value )
	public void dJointSetPlane2DYParam( 
			D_PARAM_NAMES_N parameter, double value )
	{
//		COM.dUASSERT( joint, "bad joint argument" );
//		checktype( joint, dxJointPlane2D.class );
//		dxJointPlane2D joint2d = ( dxJointPlane2D )( joint );
		motor_y.set( parameter.toSUB(), value );
	}



//	void dJointSetPlane2DAngleParam( dxJoint joint,
//			D_PARAM_NAMES_X parameter, double value )
	void dJointSetPlane2DAngleParam(
			D_PARAM_NAMES_N parameter, double value )
	{
//		COM.dUASSERT( joint, "bad joint argument" );
//		checktype( joint, dxJointPlane2D.class );
//		dxJointPlane2D joint2d = ( dxJointPlane2D )( joint );
		motor_angle.set( parameter.toSUB(), value );
	}


	// ***********************************
	// API dJoint
	// ***********************************

	@Override
	public double getParam(D_PARAM_NAMES_N parameter) {
		//TODO
		throw new UnsupportedOperationException();
	}


	@Override
	public void setParam(D_PARAM_NAMES_N parameter, double value) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException();
	}
}

