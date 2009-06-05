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

import org.cpp4j.java.RefDouble;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DUniversalJoint;
import org.ode4j.ode.internal.DxWorld;
import static org.ode4j.ode.OdeMath.*;


/** 
 * ****************************************************************************
 * universal
 * 
 * I just realized that the universal joint is equivalent to a hinge 2 joint with
 * perfectly stiff suspension.  By comparing the hinge 2 implementation to
 * the universal implementation, you may be able to improve this
 * implementation (or, less likely, the hinge2 implementation).
 */
public class DxJointUniversal extends DxJoint implements DUniversalJoint
{
	DVector3 _anchor1 = new DVector3();   // anchor w.r.t first body
	DVector3 _anchor2 = new DVector3();   // anchor w.r.t second body
	DVector3 _axis1;     // axis w.r.t first body
	DVector3 _axis2;     // axis w.r.t second body
	DQuaternion qrel1;  // initial relative rotation body1 -> virtual cross piece
	DQuaternion qrel2;  // initial relative rotation virtual cross piece -> body2
	DxJointLimitMotor limot1; // limit and motor information for axis1
	DxJointLimitMotor limot2; // limit and motor information for axis2

	DxJointUniversal( DxWorld w ) 
	{
		super(w);
		_axis1 = new DVector3(1, 0, 0);
		_axis2 = new DVector3(0, 1, 0);
		qrel1 = new DQuaternion();
		qrel2 = new DQuaternion();
		limot1 = new DxJointLimitMotor();
		limot1.init( world );
		limot2 = new DxJointLimitMotor();
		limot2.init( world );
	}


	void
	getAxes( DVector3 ax1, DVector3 ax2 )
	{
		// This says "ax1 = joint->node[0].body->posr.R * joint->axis1"
		dMULTIPLY0_331( ax1, node[0].body._posr.R, _axis1 );

		if ( node[1].body != null)
		{
			dMULTIPLY0_331( ax2, node[1].body._posr.R, _axis2 );
		}
		else
		{
//			ax2[0] = axis2[0];
//			ax2[1] = axis2[1];
//			ax2[2] = axis2[2];
			ax2.set(_axis2);
		}
	}

	void
	//getAngles( dReal *angle1, dReal *angle2 )
	getAngles( RefDouble angle1, RefDouble angle2 )
	{
		if ( node[0].body!= null )
		{
			// length 1 joint axis in global coordinates, from each body
			DVector3 ax1 = new DVector3(), ax2 = new DVector3();
			DMatrix3 R = new DMatrix3();
			DQuaternion qcross = new DQuaternion(), qq = new DQuaternion(), qrel = new DQuaternion();

			getAxes( ax1, ax2 );

			// It should be possible to get both angles without explicitly
			// constructing the rotation matrix of the cross.  Basically,
			// orientation of the cross about axis1 comes from body 2,
			// about axis 2 comes from body 1, and the perpendicular
			// axis can come from the two bodies somehow.  (We don't really
			// want to assume it's 90 degrees, because in general the
			// constraints won't be perfectly satisfied, or even very well
			// satisfied.)
			//
			// However, we'd need a version of getHingeAngleFromRElativeQuat()
			// that CAN handle when its relative quat is rotated along a direction
			// other than the given axis.  What I have here works,
			// although it's probably much slower than need be.

			dRFrom2Axes( R, ax1.get0(), ax1.get1(), ax1.get2(), ax2.get0(), ax2.get1(), ax2.get2() );

			dRtoQ( R, qcross );


			// This code is essentialy the same as getHingeAngle(), see the comments
			// there for details.

			// get qrel = relative rotation between node[0] and the cross
			dQMultiply1( qq, node[0].body._q, qcross );
			dQMultiply2( qrel, qq, qrel1 );

			//*angle1 = getHingeAngleFromRelativeQuat( qrel, axis1 );
			angle1.set( getHingeAngleFromRelativeQuat( qrel, _axis1 ) );
			
			// This is equivalent to
			// dRFrom2Axes(R, ax2[0], ax2[1], ax2[2], ax1[0], ax1[1], ax1[2]);
			// You see that the R is constructed from the same 2 axis as for angle1
			// but the first and second axis are swapped.
			// So we can take the first R and rapply a rotation to it.
			// The rotation is around the axis between the 2 axes (ax1 and ax2).
			// We do a rotation of 180deg.

			DQuaternion qcross2 = new DQuaternion();
			// Find the vector between ax1 and ax2 (i.e. in the middle)
			// We need to turn around this vector by 180deg

			// The 2 axes should be normalize so to find the vector between the 2.
			// Add and devide by 2 then normalize or simply normalize
			//    ax2
			//    ^
			//    |
			//    |
			///   *------------> ax1
			//    We want the vector a 45deg
			//
			// N.B. We don't need to normalize the ax1 and ax2 since there are
			//      normalized when we set them.

			// We set the quaternion q = [cos(theta), dir*sin(theta)] = [w, x, y, Z]
			qrel.v[0] = 0;                // equivalent to cos(Pi/2)
			qrel.v[1] = ax1.v[0] + ax2.v[0];  // equivalent to x*sin(Pi/2); since sin(Pi/2) = 1
			qrel.v[2] = ax1.v[1] + ax2.v[1];
			qrel.v[3] = ax1.v[2] + ax2.v[2];

			double l = dRecip( Math.sqrt( qrel.v[1] * qrel.v[1] + 
					qrel.v[2] * qrel.v[2] + 
					qrel.v[3] * qrel.v[3] ) );
			qrel.v[1] *= l;
			qrel.v[2] *= l;
			qrel.v[3] *= l;

			dQMultiply0( qcross2, qrel, qcross );

			if ( node[1].body != null)
			{
				dQMultiply1( qq, node[1].body._q, qcross2 );
				dQMultiply2( qrel, qq, qrel2 );
			}
			else
			{
				// pretend joint->node[1].body->q is the identity
				dQMultiply2( qrel, qcross2, qrel2 );
			}

			//*angle2 = - getHingeAngleFromRelativeQuat( qrel, axis2 );
			angle2.set(- getHingeAngleFromRelativeQuat( qrel, _axis2 ) );
		}
		else
		{
//			*angle1 = 0;
//			*angle2 = 0;
			angle1.set(0);
			angle2.set(0);
		}
	}

	protected double
	getAngle1Internal()
	{
		if ( node[0].body != null)
		{
			// length 1 joint axis in global coordinates, from each body
			DVector3 ax1 = new DVector3(), ax2 = new DVector3();
			DMatrix3 R = new DMatrix3();
			DQuaternion qcross = new DQuaternion(), qq = new DQuaternion(), qrel = new DQuaternion();

			getAxes( ax1, ax2 );

			// It should be possible to get both angles without explicitly
			// constructing the rotation matrix of the cross.  Basically,
			// orientation of the cross about axis1 comes from body 2,
			// about axis 2 comes from body 1, and the perpendicular
			// axis can come from the two bodies somehow.  (We don't really
			// want to assume it's 90 degrees, because in general the
			// constraints won't be perfectly satisfied, or even very well
			// satisfied.)
			//
			// However, we'd need a version of getHingeAngleFromRElativeQuat()
			// that CAN handle when its relative quat is rotated along a direction
			// other than the given axis.  What I have here works,
			// although it's probably much slower than need be.

			dRFrom2Axes( R, ax1.v[0], ax1.v[1], ax1.v[2], ax2.v[0], ax2.v[1], ax2.v[2] );
			dRtoQ( R, qcross );

			// This code is essential the same as getHingeAngle(), see the comments
			// there for details.

			// get qrel = relative rotation between node[0] and the cross
			dQMultiply1( qq, node[0].body._q, qcross );
			dQMultiply2( qrel, qq, qrel1 );

			return getHingeAngleFromRelativeQuat( qrel, _axis1 );
		}
		return 0;
	}


	protected double
	getAngle2Internal() 
	{
		if ( node[0].body != null)
		{
			// length 1 joint axis in global coordinates, from each body
			DVector3 ax1 = new DVector3(), ax2 = new DVector3();
			DMatrix3 R = new DMatrix3();
			DQuaternion qcross = new DQuaternion(), qq = new DQuaternion(), qrel = new DQuaternion();

			getAxes( ax1, ax2 );

			// It should be possible to get both angles without explicitly
			// constructing the rotation matrix of the cross.  Basically,
			// orientation of the cross about axis1 comes from body 2,
			// about axis 2 comes from body 1, and the perpendicular
			// axis can come from the two bodies somehow.  (We don't really
			// want to assume it's 90 degrees, because in general the
			// constraints won't be perfectly satisfied, or even very well
			// satisfied.)
			//
			// However, we'd need a version of getHingeAngleFromRElativeQuat()
			// that CAN handle when its relative quat is rotated along a direction
			// other than the given axis.  What I have here works,
			// although it's probably much slower than need be.

			dRFrom2Axes( R, ax2.v[0], ax2.v[1], ax2.v[2], ax1.v[0], ax1.v[1], ax1.v[2] );
			dRtoQ( R, qcross );

			if ( node[1].body != null )
			{
				dQMultiply1( qq, node[1].body._q, qcross );
				dQMultiply2( qrel, qq, qrel2 );
			}
			else
			{
				// pretend joint->node[1].body->q is the identity
				dQMultiply2( qrel, qcross, qrel2 );
			}

			return - getHingeAngleFromRelativeQuat( qrel, _axis2 );
		}
		return 0;
	}


	public void
	getInfo1( DxJoint.Info1 info )
	{
		info.setNub(4);
		info.setM(4);

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
			//double angle1, angle2;
			RefDouble angle1 = new RefDouble(0), angle2 = new RefDouble(0); 
			//getAngles( &angle1, &angle2 );
			getAngles(angle1, angle2);
			if ( limiting1 )
				limot1.testRotationalLimit( angle1.get() );
			if ( limiting2 )
				limot2.testRotationalLimit( angle2.get() );
		}
		if ( limot1.limit != 0 || limot1.fmax > 0 ) info.incM();
		if ( limot2.limit != 0 || limot2.fmax > 0 ) info.incM();
	}


	public void
	getInfo2( DxJoint.Info2 info )
	{
		// set the three ball-and-socket rows
		setBall( this, info, _anchor1, _anchor2 );

		// set the universal joint row. the angular velocity about an axis
		// perpendicular to both joint axes should be equal. thus the constraint
		// equation is
		//    p*w1 - p*w2 = 0
		// where p is a vector normal to both joint axes, and w1 and w2
		// are the angular velocity vectors of the two bodies.

		// length 1 joint axis in global coordinates, from each body
		DVector3 ax1 = new DVector3(), ax2 = new DVector3();
		DVector3 ax2_temp = new DVector3();
		// length 1 vector perpendicular to ax1 and ax2. Neither body can rotate
		// about this.
		DVector3 p = new DVector3();
		double k;

		// Since axis1 and axis2 may not be perpendicular
		// we find a axis2_tmp which is really perpendicular to axis1
		// and in the plane of axis1 and axis2
		getAxes( ax1, ax2 );
		k = dDOT( ax1, ax2 );
		ax2_temp.v[0] = ax2.v[0] - k * ax1.v[0];
		ax2_temp.v[1] = ax2.v[1] - k * ax1.v[1];
		ax2_temp.v[2] = ax2.v[2] - k * ax1.v[2];
		dCROSS( p, OP.EQ , ax1, ax2_temp );
		dNormalize3( p );

		int s3 = 3 * info.rowskip();

		info._J[info.J1ap+s3+0] = p.v[0];
		info._J[info.J1ap+s3+1] = p.v[1];
		info._J[info.J1ap+s3+2] = p.v[2];

		if ( node[1].body != null)
		{
			info._J[info.J2ap+s3+0] = -p.v[0];
			info._J[info.J2ap+s3+1] = -p.v[1];
			info._J[info.J2ap+s3+2] = -p.v[2];
		}

		// compute the right hand side of the constraint equation. set relative
		// body velocities along p to bring the axes back to perpendicular.
		// If ax1, ax2 are unit length joint axes as computed from body1 and
		// body2, we need to rotate both bodies along the axis p.  If theta
		// is the angle between ax1 and ax2, we need an angular velocity
		// along p to cover the angle erp * (theta - Pi/2) in one step:
		//
		//   |angular_velocity| = angle/time = erp*(theta - Pi/2) / stepsize
		//                      = (erp*fps) * (theta - Pi/2)
		//
		// if theta is close to Pi/2,
		// theta - Pi/2 ~= cos(theta), so
		//    |angular_velocity|  ~= (erp*fps) * (ax1 dot ax2)

		info.setC(3, info.fps * info.erp * - k );

		// if the first angle is powered, or has joint limits, add in the stuff
		int row = 4 + limot1.addLimot( this, info, 4, ax1, true );

		// if the second angle is powered, or has joint limits, add in more stuff
		limot2.addLimot( this, info, row, ax2, true );
	}


	void
	computeInitialRelativeRotations()
	{
		if ( node[0].body != null)
		{
			DVector3 ax1 = new DVector3(), ax2 = new DVector3();
			DMatrix3 R = new DMatrix3();
			DQuaternion qcross = new DQuaternion();

			getAxes( ax1, ax2 );

			// Axis 1.
			dRFrom2Axes( R, ax1.v[0], ax1.v[1], ax1.v[2], ax2.v[0], ax2.v[1], ax2.v[2] );
			dRtoQ( R, qcross );
			dQMultiply1( qrel1, node[0].body._q, qcross );

			// Axis 2.
			dRFrom2Axes( R, ax2.v[0], ax2.v[1], ax2.v[2], ax1.v[0], ax1.v[1], ax1.v[2] );
			dRtoQ( R, qcross );
			if ( node[1].body != null)
			{
				dQMultiply1( qrel2, node[1].body._q, qcross );
			}
			else
			{
				// set joint->qrel to qcross
				for ( int i = 0; i < 4; i++ ) qrel2.v[i] = qcross.v[i];
			}
		}
	}


//	public void dJointSetUniversalAnchor( dJoint j, double x, double y, double z )
	public void dJointSetUniversalAnchor( double x, double y, double z )
	{
		dJointSetUniversalAnchor( new DVector3(x, y, z) );
	}
	public void dJointSetUniversalAnchor( DVector3C xyz )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dUASSERT( joint, "bad joint argument" );
//		checktype( joint, dxJointUniversal.class );
		setAnchors( xyz, _anchor1, _anchor2 );
		computeInitialRelativeRotations();
	}


//	private void dJointSetUniversalAxis1( dJoint j, double x, double y, double z )
	public void dJointSetUniversalAxis1( double x, double y, double z )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dUASSERT( joint, "bad joint argument" );
//		checktype( joint, dxJointUniversal.class );
		if ( (flags & dJOINT_REVERSE) != 0 )
			setAxes( x, y, z, null, _axis2 );
		else
			setAxes( x, y, z, _axis1, null );
		computeInitialRelativeRotations();
	}


//	private void dJointSetUniversalAxis2( dJoint j, double x, double y, double z )
	public void dJointSetUniversalAxis2( double x, double y, double z )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dUASSERT( joint, "bad joint argument" );
//		checktype( joint, dxJointUniversal.class );
		if ( (flags & dJOINT_REVERSE) != 0 )
			setAxes( x, y, z, _axis1, null );
		else
			setAxes( x, y, z, null, _axis2 );
		computeInitialRelativeRotations();
	}


//	public void dJointGetUniversalAnchor( dJoint j, dVector3 result )
	public void dJointGetUniversalAnchor( DVector3 result )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dUASSERT( joint, "bad joint argument" );
//		dUASSERT( result, "bad result argument" );
//		checktype( joint, dxJointUniversal.class );
		if ( (flags & dJOINT_REVERSE) != 0 )
			getAnchor2( result, _anchor2 );
		else
			getAnchor( result, _anchor1 );
	}


//	private void dJointGetUniversalAnchor2( dJoint j, dVector3 result )
	private void dJointGetUniversalAnchor2( DVector3 result )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dUASSERT( joint, "bad joint argument" );
//		dUASSERT( result, "bad result argument" );
//		checktype( joint, dxJointUniversal.class );
		if ( (flags & dJOINT_REVERSE) != 0 )
			getAnchor( result, _anchor1 );
		else
			getAnchor2( result, _anchor2 );
	}


	public void dJointGetUniversalAxis1( DVector3 result )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dUASSERT( joint, "bad joint argument" );
//		dUASSERT( result, "bad result argument" );
//		checktype( joint, dxJointUniversal.class );
		if ( (flags & dJOINT_REVERSE) != 0)
			getAxis2( result, _axis2 );
		else
			getAxis( result, _axis1 );
	}


	public void dJointGetUniversalAxis2( DVector3 result )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dUASSERT( joint, "bad joint argument" );
//		dUASSERT( result, "bad result argument" );
//		checktype( joint, dxJointUniversal.class );
		if ( (flags & dJOINT_REVERSE) != 0 )
			getAxis( result, _axis1 );
		else
			getAxis2( result, _axis2 );
	}


//	private void dJointSetUniversalParam( dJoint j, int parameter, double value )
	public void dJointSetUniversalParam( D_PARAM_NAMES_N parameter, double value )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dUASSERT( joint, "bad joint argument" );
//		checktype( joint, dxJointUniversal.class );
		if (  parameter.isGroup2()) //and( 0xff00 ).eq( 0x100 ))
		{
			limot2.set( parameter.toSUB(), value);//parameter.and( 0xff ), value );
		}
		else
		{
			limot1.set( parameter.toSUB(), value );
		}
	}


//	private double dJointGetUniversalParam( dJoint j, D_PARAM_NAMES parameter )
	private double dJointGetUniversalParam( D_PARAM_NAMES_N parameter )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dUASSERT( joint, "bad joint argument" );
//		checktype( joint, dxJointUniversal.class );
		if ( parameter.isGroup2())//and( 0xff00 ).eq( 0x100 ))
		{
			return limot2.get( parameter.toSUB());//and( 0xff) );
		}
		else
		{
			return limot1.get( parameter.toSUB() );
		}
	}

	//TZ removed to avoid RefDouble usage
//	private void dJointGetUniversalAngles( RefDouble angle1, 
//			RefDouble angle2 )
//	{
////		dxJointUniversal joint = ( dxJointUniversal )j;
////		dUASSERT( joint, "bad joint argument" );
////		checktype( joint, dxJointUniversal.class );
//		if ( (flags & dJOINT_REVERSE) != 0 )
//			getAngles( angle2, angle1 );
//		else
//			getAngles( angle1, angle2 );
//	}


	public double dJointGetUniversalAngle1( )
	{
		if ( (flags & dJOINT_REVERSE) != 0 )
			return getAngle2Internal();
		else
			return getAngle1Internal();
	}


	public double dJointGetUniversalAngle2( )
	{
		if ( (flags & dJOINT_REVERSE) != 0 )
			return getAngle1Internal();
		else
			return getAngle2Internal();
	}


	public double dJointGetUniversalAngle1Rate( )
	{
		if ( node[0].body != null)
		{
			DVector3 axis = new DVector3();

			if ( (flags & dJOINT_REVERSE) != 0 )
				getAxis2( axis, _axis2 );
			else
				getAxis( axis, _axis1 );

			double rate = dDOT( axis, node[0].body.avel );
			if ( node[1].body != null)
				rate -= dDOT( axis, node[1].body.avel );
			return rate;
		}
		return 0;
	}


	public double dJointGetUniversalAngle2Rate( )
	{
		if ( node[0].body != null)
		{
			DVector3 axis = new DVector3();

			if ( (flags & dJOINT_REVERSE) != 0 )
				getAxis( axis, _axis1 );
			else
				getAxis2( axis, _axis2 );

			double rate = dDOT( axis, node[0].body.avel );
			if ( node[1].body != null) 
				rate -= dDOT( axis, node[1].body.avel );
			return rate;
		}
		return 0;
	}


	private void dJointAddUniversalTorques( double torque1, double torque2 )
	{
//		dxJointUniversal joint = ( dxJointUniversal )j;
//		dAASSERT( joint );
//		checktype( joint, dxJointUniversal.class );
		DVector3 axis1 = new DVector3(), axis2 = new DVector3();

		if ( (flags & dJOINT_REVERSE) != 0 )
		{
			double temp = torque1;
			torque1 = - torque2;
			torque2 = - temp;
		}

		getAxis( axis1, _axis1 );
		getAxis2( axis2, _axis2 );
//		axis1.v[0] = axis1.v[0] * torque1 + axis2.v[0] * torque2;
//		axis1.v[1] = axis1.v[1] * torque1 + axis2.v[1] * torque2;
//		axis1.v[2] = axis1.v[2] * torque1 + axis2.v[2] * torque2;
		axis1.eqSum( axis1, torque1, axis2, torque2 );

		if ( node[0].body != null )
			node[0].body.dBodyAddTorque(axis1);
		if ( node[1].body != null )
			node[1].body.dBodyAddTorque( axis1.scale(-1) );
	}


	public DxJointLimitMotor getLimot1() {
		return limot1;
	}
	
	public DxJointLimitMotor getLimot2() {
		return limot2;
	}
	
	
	// ***********************************
	// API dUniversalJoint
	// ***********************************
	
	  public void setAnchor (double x, double y, double z)
	    { dJointSetUniversalAnchor (x, y, z); }
	  public void setAnchor (DVector3C a)
	    { dJointSetUniversalAnchor(a); }
	  public void setAxis1 (double x, double y, double z)
	    { dJointSetUniversalAxis1 (x, y, z); }
	  public void setAxis1 (DVector3C a)
	  //TODO use dVector3
	    { setAxis1 (a.get0(), a.get1(), a.get2()); }
	  public void setAxis2 (double x, double y, double z)
	    { dJointSetUniversalAxis2 (x, y, z); }
	  public void setAxis2 (DVector3C a)
	  //TODO use dVector3
	    { setAxis2 (a.get0(), a.get1(), a.get2()); }

	  public void getAnchor (DVector3 result)
	    { dJointGetUniversalAnchor (result); }
	  public void getAnchor2 (DVector3 result)
	    { dJointGetUniversalAnchor2 (result); }
	  public void getAxis1 (DVector3 result)
	    { dJointGetUniversalAxis1 (result); }
	  public void getAxis2 (DVector3 result)
	    { dJointGetUniversalAxis2 (result); }

	  public void setParam (D_PARAM_NAMES_N parameter, double value)
	    { dJointSetUniversalParam (parameter, value); }
	  public double getParam (D_PARAM_NAMES_N parameter)
	    { return dJointGetUniversalParam (parameter); }

//	  public void getAngles(double *angle1, double *angle2)
//	    { dJointGetUniversalAngles (angle1, angle2); }

		/** TZ Take care to call getAngle1Internal() from dx-classes.*/
	  public double getAngle1()
	    { return dJointGetUniversalAngle1 (); }
	  public double getAngle1Rate()
	    { return dJointGetUniversalAngle1Rate (); }
		/** TZ Take care to call getAngle2Internal() from dx-classes.*/
	  public double getAngle2()
	    { return dJointGetUniversalAngle2 (); }
	  public double getAngle2Rate()
	    { return dJointGetUniversalAngle2Rate (); }

	  public void addTorques (double torque1, double torque2)
		{ dJointAddUniversalTorques(torque1, torque2); }
}

