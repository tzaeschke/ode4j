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

import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.internal.DxWorld;

import static org.ode4j.ode.OdeMath.*;


/**
 * ****************************************************************************
 * hinge
 * 
 */
public class DxJointHinge extends DxJoint implements DHingeJoint
{
	private DVector3 anchor1;   // anchor w.r.t first body
	private DVector3 anchor2;   // anchor w.r.t second body
	private DVector3 _axis1;     // axis w.r.t first body
	private DVector3 _axis2;     // axis w.r.t second body
	private DQuaternion qrel;   // initial relative rotation body1 -> body2
	private DxJointLimitMotor limot; // limit and motor information


	DxJointHinge( DxWorld w ) 
	//dxJoint( w )
	{
		super(w);
		anchor1 = new DVector3();
		anchor2 = new DVector3();
		_axis1 = new DVector3();
		_axis2 = new DVector3();
		qrel = new DQuaternion();
		limot = new DxJointLimitMotor();
//		dSetZero( anchor1, 4 );
//		dSetZero( anchor2, 4 );
//		dSetZero( _axis1, 4 );
		_axis1.set0( 1 );
//		dSetZero( _axis2, 4 );
		_axis2.set0( 1 );
//		dSetZero( qrel, 4 );
		limot.init( world );
	}


	@Override
	public void
	getInfo1( DxJoint.Info1 info )
	{
		info.setNub(5);

		// see if joint is powered
		if ( limot.fmax > 0 )
			info.setM(6); // powered hinge needs an extra constraint row
		else info.setM(5);

		// see if we're at a joint limit.
		if (( limot.lostop >= -M_PI || limot.histop <= M_PI ) &&
				limot.lostop <= limot.histop )
		{
			double angle = getHingeAngle( node[0].body,
					node[1].body,
					_axis1, qrel );
			if ( limot.testRotationalLimit( angle ) )
				info.setM(6);
		}
	}


	@Override
	public void
	getInfo2( DxJoint.Info2 info )
	{
		// set the three ball-and-socket rows
		setBall( this, info, anchor1, anchor2 );

		// set the two hinge rows. the hinge axis should be the only unconstrained
		// rotational axis, the angular velocity of the two bodies perpendicular to
		// the hinge axis should be equal. thus the constraint equations are
		//    p*w1 - p*w2 = 0
		//    q*w1 - q*w2 = 0
		// where p and q are unit vectors normal to the hinge axis, and w1 and w2
		// are the angular velocity vectors of the two bodies.

		DVector3 ax1 = new DVector3();  // length 1 joint axis in global coordinates, from 1st body
		DVector3 p = new DVector3(), q = new DVector3(); // plane space vectors for ax1
		dMULTIPLY0_331( ax1, node[0].body.posr().R(), _axis1 );
		dPlaneSpace( ax1, p, q );

		int s3 = 3 * info.rowskip();
		int s4 = 4 * info.rowskip();

//		info._J[info.J1ap+s3+0] = p.v[0];
//		info._J[info.J1ap+s3+1] = p.v[1];
//		info._J[info.J1ap+s3+2] = p.v[2];
		p.wrapSet( info._J, info.J1ap+s3 );
//		info._J[info.J1ap+s4+0] = q.v[0];
//		info._J[info.J1ap+s4+1] = q.v[1];
//		info._J[info.J1ap+s4+2] = q.v[2];
		q.wrapSet( info._J, info.J1ap+s4 );

		if ( node[1].body!= null )
		{
//			info._J[info.J2ap+s3+0] = -p.v[0];
//			info._J[info.J2ap+s3+1] = -p.v[1];
//			info._J[info.J2ap+s3+2] = -p.v[2];
			p.wrapSub( info._J, info.J2ap+s3 );
//			info._J[info.J2ap+s4+0] = -q.v[0];
//			info._J[info.J2ap+s4+1] = -q.v[1];
//			info._J[info.J2ap+s4+2] = -q.v[2];
			q.wrapSub( info._J, info.J2ap+s4 );
		}

		// compute the right hand side of the constraint equation. set relative
		// body velocities along p and q to bring the hinge back into alignment.
		// if ax1,ax2 are the unit length hinge axes as computed from body1 and
		// body2, we need to rotate both bodies along the axis u = (ax1 x ax2).
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

		DVector3 ax2 = new DVector3(), b = new DVector3();
		if ( node[1].body != null)
		{
			dMULTIPLY0_331( ax2, node[1].body.posr().R(), _axis2 );
		}
		else
		{
			ax2.set(_axis2);
			//        ax2[0] = axis2[0];
			//        ax2[1] = axis2[1];
			//        ax2[2] = axis2[2];
		}
		dCROSS( b, OP.EQ , ax1, ax2 );
		double k = info.fps * info.erp;
		info.setC(3, k * dDOT( b, p ) );
		info.setC(4, k * dDOT( b, q ) );

		// if the hinge is powered, or has joint limits, add in the stuff
		limot.addLimot( this, info, 5, ax1, true );
	}



	//void dJointSetHingeAnchor( dxJointHinge j, double x, double y, double z )
	public void dJointSetHingeAnchor( double x, double y, double z )
	{
		dJointSetHingeAnchor( new DVector3(x, y, z) );
	}
	
	public void dJointSetHingeAnchor( DVector3C xyz )
	{
//		setAnchors( x, y, z, anchor1, anchor2 );
		setAnchors( xyz, anchor1, anchor2 );
		computeInitialRelativeRotation();
	}


	void dJointSetHingeAnchorDelta( DxJointHinge joint, double x, double y, 
			double z, double dx, double dy, double dz )
	{
		dUASSERT( joint, "bad joint argument" );

		if ( joint.node[0].body != null)
		{
			//double q[4];
			DVector3 q = new DVector3();
//			q.v[0] = x - joint.node[0].body._posr.pos.v[0];
//			q.v[1] = y - joint.node[0].body._posr.pos.v[1];
//			q.v[2] = z - joint.node[0].body._posr.pos.v[2];
			//q[3] = 0;
			dMULTIPLY1_331( joint.anchor1, joint.node[0].body.posr().R(), q );

			if ( joint.node[1].body != null )
			{
//				q.v[0] = x - joint.node[1].body._posr.pos.v[0];
//				q.v[1] = y - joint.node[1].body._posr.pos.v[1];
//				q.v[2] = z - joint.node[1].body._posr.pos.v[2];
				q.set( x, y, z ).sub( joint.node[1].body.posr().pos() );
				//q[3] = 0;
				dMULTIPLY1_331( joint.anchor2, joint.node[1].body.posr().R(), q );
			}
			else
			{
				// Move the relative displacement between the passive body and the
				//  anchor in the same direction as the passive body has just moved
//				joint.anchor2.v[0] = x + dx;
//				joint.anchor2.v[1] = y + dy;
//				joint.anchor2.v[2] = z + dz;
				joint.anchor2.set( x, y, z ).add( dx, dy, dz );
			}
		}
//		joint.anchor1.v[3] = 0;
//		joint.anchor2.v[3] = 0;

		joint.computeInitialRelativeRotation();
	}



	public void dJointSetHingeAxis( double x, double y, double z )
	{
		setAxes( x, y, z, _axis1, _axis2 );
		computeInitialRelativeRotation();
	}


	//void dJointSetHingeAxisOffset( dxJointHinge j, double x, double y, double z, double dangle )
	public void dJointSetHingeAxisOffset( double x, double y, double z, double dangle )
	{
		setAxes( x, y, z, _axis1, _axis2 );
		computeInitialRelativeRotation();

		if ( isFlagsReverse() ) dangle = -dangle;

		DQuaternion qAngle = new DQuaternion(), qOffset = new DQuaternion();
		dQFromAxisAndAngle(qAngle, x, y, z, dangle);
		dQMultiply3(qOffset, qAngle, qrel);
//		qrel.v[0] = qOffset.v[0];
//		qrel.v[1] = qOffset.v[1];
//		qrel.v[2] = qOffset.v[2];
//		qrel.v[3] = qOffset.v[3];
		qrel.set(qOffset);
	}



	//void dJointGetHingeAnchor( dxJointHinge j, dVector3 result )
	void dJointGetHingeAnchor( DVector3 result )
	{
		if ( isFlagsReverse() )
			getAnchor2( result, anchor2 );
		else
			getAnchor( result, anchor1 );
	}


	//void dJointGetHingeAnchor2( dxJointHinge j, dVector3 result )
	void dJointGetHingeAnchor2( DVector3 result )
	{
		if ( isFlagsReverse() )
			getAnchor( result, anchor1 );
		else
			getAnchor2( result, anchor2 );
	}


//	void dJointGetHingeAxis( dxJointHinge j, dVector3 result )
	void dJointGetHingeAxis( DVector3 result )
	{
		getAxis( result, _axis1 );
	}


	//void dJointSetHingeParam( dxJointHinge j, D_PARAM_NAMES_X parameter, double value )
	public void dJointSetHingeParam( PARAM_N parameter, double value )
	{
		limot.set( parameter.toSUB(), value );
	}


	//double dJointGetHingeParam( dxJointHinge j, D_PARAM_NAMES parameter )
	double dJointGetHingeParam( PARAM_N parameter )
	{
		return limot.get( parameter.toSUB() );
	}


	//double dJointGetHingeAngle( dxJointHinge j )
	public double dJointGetHingeAngle( )
	{
		if ( node[0].body !=null)
		{
			double ang = getHingeAngle( node[0].body,
					node[1].body,
					_axis1,
					qrel );
			if ( isFlagsReverse() )
				return -ang;
			else
				return ang;
		}
		else return 0;
	}


	public double dJointGetHingeAngleRate()
	{
		if ( node[0].body!=null )
		{
			DVector3 axis = new DVector3();
			dMULTIPLY0_331( axis, node[0].body.posr().R(), _axis1 );
			double rate = dDOT( axis, node[0].body.avel );
			if ( node[1].body!=null ) rate -= dDOT( axis, node[1].body.avel );
			if ( isFlagsReverse() ) rate = - rate;
			return rate;
		}
		else return 0;
	}


//	void dJointAddHingeTorque( dxJointHinge j, double torque )
	void dJointAddHingeTorque( double torque )
	{
		DVector3 axis = new DVector3();

		if ( isFlagsReverse() )
			torque = -torque;

		getAxis( axis, _axis1 );
//		axis.v[0] *= torque;
//		axis.v[1] *= torque;
//		axis.v[2] *= torque;
		axis.scale(torque);

		if ( node[0].body != null )
			node[0].body.dBodyAddTorque( axis );
		if ( node[1].body != null )
			node[1].body.dBodyAddTorque( axis.reScale(-1) );
	}

	@Override
	void setRelativeValues()
	{
	    DVector3 vec = new DVector3();
	    dJointGetHingeAnchor(vec);
	    setAnchors( vec, anchor1, anchor2 );

	    dJointGetHingeAxis(vec);
	    setAxes( vec.get0(), vec.get1(), vec.get2(), _axis1, _axis2 );
	    computeInitialRelativeRotation();
	}


	/// Compute initial relative rotation body1 . body2, or env . body1
	void
	computeInitialRelativeRotation()
	{
		if ( node[0].body != null)
		{
			if ( node[1].body  != null)
			{
				dQMultiply1( qrel, node[0].body._q, node[1].body._q );
			}
			else
			{
				// set qrel to the transpose of the first body q
//				qrel.v[0] =  node[0].body._q.v[0];
//				qrel.v[1] = -node[0].body._q.v[1];
//				qrel.v[2] = -node[0].body._q.v[2];
//				qrel.v[3] = -node[0].body._q.v[3];
				qrel.set( node[0].body._q.get0(),
						 -node[0].body._q.get1(),
						 -node[0].body._q.get2(),
						 -node[0].body._q.get3() );

			}
		}
	}

	// ***********************************
	// API dHingeJoint
	// ***********************************

	public void setAnchor (double x, double y, double z)
	{ dJointSetHingeAnchor (x, y, z); }
	public void setAnchor (DVector3C a)
	{ dJointSetHingeAnchor (a); }
	public void getAnchor (DVector3 result)
	{ dJointGetHingeAnchor (result); }
	public void getAnchor2 (DVector3 result)
	{ dJointGetHingeAnchor2 (result); }

	public void setAxis (double x, double y, double z)
	{ dJointSetHingeAxis (x, y, z); }
	public void setAxis (DVector3C a)
	{ setAxis(a.get0(), a.get1(), a.get2()); }
	public void getAxis (DVector3 result)
	{ dJointGetHingeAxis (result); }
	@Override
	public void setAxisOffset(double x, double y, double z, double angle) {
		dJointSetHingeAxisOffset(x, y, z, angle); }

	public double getAngle()
	{ return dJointGetHingeAngle (); }
	public double getAngleRate()
	{ return dJointGetHingeAngleRate (); }

	@Override
	public void setParam (PARAM_N parameter, double value)
	{ dJointSetHingeParam (parameter, value); }
	@Override
	public double getParam (PARAM_N parameter)
	{ return dJointGetHingeParam (parameter); }

	public void addTorque (double torque)
	{ dJointAddHingeTorque(torque); }
	
	@Override
	public void setParamFMax(double d) {
		dJointSetHingeParam(PARAM_N.dParamFMax1, d);
	}


	@Override
	public void setParamVel(double d) {
		dJointSetHingeParam(PARAM_N.dParamVel1, d);
	}


	@Override
	public void setParamHiStop(double d) {
		dJointSetHingeParam(PARAM_N.dParamHiStop1, d);
	}


	@Override
	public void setParamLoStop(double d) {
		dJointSetHingeParam(PARAM_N.dParamLoStop1, d);
	}


	@Override
	public void setParamBounce(double d) {
		dJointSetHingeParam(PARAM_N.dParamBounce1, d);
	}

}
