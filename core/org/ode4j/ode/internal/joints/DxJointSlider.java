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

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.Common.D_PARAM_NAMES_N;

import static org.ode4j.ode.OdeMath.*;


/** 
 * **************************************************************************
 * slider
 * slider. if body2 is 0 then qrel is the absolute rotation of body1 and
 * offset is the position of body1 center along axis1.
 * 
 */
public class DxJointSlider extends DxJoint implements DSliderJoint
{
	DVector3 axis1;     // axis w.r.t first body
	public DQuaternion qrel;   // initial relative rotation body1 -> body2
	DVector3 offset;    // point relative to body2 that should be
	// aligned with body1 center along axis1
	DxJointLimitMotor limot; // limit and motor information

	DxJointSlider ( DxWorld w ) 
	//dxJoint ( w )
	{
		super(w);
		axis1 = new DVector3(1, 0, 0);
		qrel = new DQuaternion();
		offset = new DVector3();
		limot = new DxJointLimitMotor();
		limot.init ( world );
	}


	//double dJointGetSliderPosition ( dJoint j )
	public double dJointGetSliderPosition ( )
	{
		//    dxJointSlider joint = ( dxJointSlider ) j;
		//    dUASSERT ( joint, "bad joint argument" );
		//    checktype ( joint, dxJointSlider.class );

		// get axis1 in global coordinates
		DVector3 ax1 = new DVector3(), q = new DVector3();
		dMULTIPLY0_331 ( ax1, node[0].body._posr.R, axis1 );

		if ( node[1].body!= null )
		{
			// get body2 + offset point in global coordinates
			dMULTIPLY0_331 ( q, node[1].body._posr.R, offset );
//			for ( int i = 0; i < 3; i++ )
//				q.v[i] = node[0].body._posr.pos.v[i]
//				         - q.v[i] - node[1].body._posr.pos.v[i];
			q.eqDiff(node[0].body._posr.pos, q);
			q.sub(node[1].body._posr.pos);

		}
		else
		{
//			q.v[0] = node[0].body._posr.pos.v[0] - offset.v[0];
//			q.v[1] = node[0].body._posr.pos.v[1] - offset.v[1];
//			q.v[2] = node[0].body._posr.pos.v[2] - offset.v[2];
			q.eqDiff(node[0].body._posr.pos, offset);

			if ( (flags & dJOINT_REVERSE)!=0 )
			{   // N.B. it could have been simplier to only inverse the sign of
				//      the dDot result but this case is exceptional and doing
				//      the check for all case can decrease the performance.
//				ax1.v[0] = -ax1.v[0];
//				ax1.v[1] = -ax1.v[1];
//				ax1.v[2] = -ax1.v[2];
				ax1.scale(-1);
			}
		}

		return dDOT ( ax1, q );
	}


	//double dJointGetSliderPositionRate ( dJoint j )
	public double dJointGetSliderPositionRate ( )
	{
		//    dxJointSlider joint = ( dxJointSlider ) j;
		//    dUASSERT ( joint, "bad joint argument" );
		//    checktype ( joint, dxJointSlider.class );

		// get axis1 in global coordinates
		DVector3 ax1 = new DVector3();
		dMULTIPLY0_331 ( ax1, node[0].body._posr.R, axis1 );

		if ( node[1].body != null )
		{
			return dDOT ( ax1, node[0].body.lvel ) -
			dDOT ( ax1, node[1].body.lvel );
		}
		else
		{
			double rate = dDOT ( ax1, node[0].body.lvel );
			if ( (flags & dJOINT_REVERSE) !=0 ) rate = - rate;
			return rate;
		}
	}


	public void
	getInfo1 ( DxJoint.Info1 info )
	{
		info.setNub(5);

		// see if joint is powered
		if ( limot.fmax > 0 )
			info.setM(6); // powered slider needs an extra constraint row
		else info.setM(5);

		// see if we're at a joint limit.
		limot.limit = 0;
		if ( ( limot.lostop > -dInfinity || limot.histop < dInfinity ) &&
				limot.lostop <= limot.histop )
		{
			// measure joint position
			double pos = dJointGetSliderPosition ( );
			if ( pos <= limot.lostop )
			{
				limot.limit = 1;
				limot.limit_err = pos - limot.lostop;
				info.setM(6);
			}
			else if ( pos >= limot.histop )
			{
				limot.limit = 2;
				limot.limit_err = pos - limot.histop;
				info.setM(6);
			}
		}
	}


	public void
	getInfo2 ( DxJoint.Info2 info )
	{
		int i, s = info.rowskip();
		int s3 = 3 * s, s4 = 4 * s;

		// pull out pos and R for both bodies. also get the `connection'
		// vector pos2-pos1.

		//double *pos1, *pos2, *R1, *R2;
		DVector3 pos1, pos2;
		DMatrix3 R1 = new DMatrix3();
		DMatrix3 R2 = new DMatrix3();
		DVector3 c = new DVector3();
		pos1 = node[0].body._posr.pos;
		R1 = node[0].body._posr.R; //TODO clone?
		if ( node[1].body!= null )
		{
			pos2 = node[1].body._posr.pos;
			R2 = node[1].body._posr.R; //TODO clone?
//			for ( i = 0; i < 3; i++ )
//				c.v[i] = pos2[i] - pos1[i];
			c.eqDiff(pos2, pos1);
		}
		else
		{
			pos2 = null;
			R2 = null;
		}

		// 3 rows to make body rotations equal
		setFixedOrientation ( this, info, qrel, 0 );

		// remaining two rows. we want: vel2 = vel1 + w1 x c ... but this would
		// result in three equations, so we project along the planespace vectors
		// so that sliding along the slider axis is disregarded. for symmetry we
		// also substitute (w1+w2)/2 for w1, as w1 is supposed to equal w2.

		DVector3 ax1 = new DVector3(); // joint axis in global coordinates (unit length)
		DVector3 p = new DVector3(), q = new DVector3(); // plane space of ax1
		dMULTIPLY0_331 ( ax1, R1, axis1 );
		dPlaneSpace ( ax1, p, q );
		if ( node[1].body!= null )
		{
			DVector3 tmp = new DVector3();
			//dCROSS ( tmp, =  0.5 * , c, p );
			dCROSS ( tmp, OP.EQ , c, p );
			//for (int k = 0; k < 3; k++) tmp.v[k] = tmp.v[k] * 0.5;
			tmp.scale(0.5);

			for ( i = 0; i < 3; i++ ) info._J[info.J1ap+s3+i] = tmp.get(i);
			for ( i = 0; i < 3; i++ ) info._J[info.J2ap+s3+i] = tmp.get(i);
			//dCROSS ( tmp, = 0.5 * , c, q );
			dCROSS ( tmp, OP.EQ , c, p );
			//for (int k = 0; k < 3; k++) tmp.v[k] = tmp.v[k] * 0.5;
			tmp.scale(0.5);

			for ( i = 0; i < 3; i++ ) info._J[info.J1ap+s4+i] = tmp.get(i);
			for ( i = 0; i < 3; i++ ) info._J[info.J2ap+s4+i] = tmp.get(i);
			for ( i = 0; i < 3; i++ ) info._J[info.J2lp+s3+i] = -p.get(i);
			for ( i = 0; i < 3; i++ ) info._J[info.J2lp+s4+i] = -q.get(i);
		}
		for ( i = 0; i < 3; i++ ) info._J[info.J1lp+s3+i] = p.get(i);
		for ( i = 0; i < 3; i++ ) info._J[info.J1lp+s4+i] = q.get(i);

		// compute last two elements of right hand side. we want to align the offset
		// point (in body 2's frame) with the center of body 1.
		double k = info.fps * info.erp;
		if ( node[1].body != null)
		{
			DVector3 ofs = new DVector3();  // offset point in global coordinates
			dMULTIPLY0_331 ( ofs, R2, offset );
			//for ( i = 0; i < 3; i++ ) c.v[i] += ofs.v[i];
			c.add(ofs);
			info.setC(3, k * dDOT ( p, c ) );
			info.setC(4, k * dDOT ( q, c ) );
		}
		else
		{
			DVector3 ofs = new DVector3();  // offset point in global coordinates
			//for ( i = 0; i < 3; i++ ) ofs.v[i] = offset.v[i] - pos1[i];
			ofs.eqDiff(offset, pos1);
			info.setC(3, k * dDOT ( p, ofs ) );
			info.setC(4, k * dDOT ( q, ofs ) );
		}

		// if the slider is powered, or has joint limits, add in the extra row
		limot.addLimot ( this, info, 5, ax1, false );
	}


	//void dJointSetSliderAxis ( dJoint j, double x, double y, double z )
	public void dJointSetSliderAxis ( double x, double y, double z )
	{
		//    dxJointSlider joint = ( dxJointSlider ) j;
		//    dUASSERT ( joint, "bad joint argument" );
		//    checktype ( joint, dxJointSlider.class );
		setAxes ( x, y, z, axis1, null );

		// compute initial relative rotation body1 . body2, or env . body1
		// also compute center of body1 w.r.t body 2
		if ( node[1].body != null)
		{
			DVector3 c = new DVector3();
//			for ( int i = 0; i < 3; i++ )
//				c.v[i] = node[0].body._posr.pos.v[i] - node[1].body._posr.pos.v[i];
			c.eqDiff(node[0].body._posr.pos, node[1].body._posr.pos);
			dMULTIPLY1_331 ( offset, node[1].body._posr.R, c );
		}
		else
		{
			//for ( int i = 0; i < 3; i++ ) offset.v[i] = node[0].body._posr.pos.v[i];
			offset.set(node[0].body._posr.pos);
		}

		computeInitialRelativeRotation();
	}


//	void dJointSetSliderAxisDelta ( dJoint j, double x, double y, double z, 
//	double dx, double dy, double dz )
	void dJointSetSliderAxisDelta ( double x, double y, double z, 
			double dx, double dy, double dz )
	{
//		dxJointSlider joint = ( dxJointSlider ) j;
//		dUASSERT ( joint, "bad joint argument" );
//		checktype ( joint, dxJointSlider.class );
		setAxes ( x, y, z, axis1, null );

		// compute initial relative rotation body1 . body2, or env . body1
		// also compute center of body1 w.r.t body 2
		if ( node[1].body!= null )
		{
			DVector3 c = new DVector3();
//			for ( i = 0; i < 3; i++ )
//				c.v[i] = node[0].body._posr.pos.v[i] - node[1].body._posr.pos.v[i];
			c.eqDiff(node[0].body._posr.pos, node[1].body._posr.pos);
			dMULTIPLY1_331 ( offset, node[1].body._posr.R, c );
		}
		else
		{
//			offset.v[0] = node[0].body._posr.pos.v[0] + dx;
//			offset.v[1] = node[0].body._posr.pos.v[1] + dy;
//			offset.v[2] = node[0].body._posr.pos.v[2] + dz;
			offset.set(node[0].body._posr.pos).add(dx, dy, dz);
		}

		computeInitialRelativeRotation();
	}



//	void dJointGetSliderAxis ( dJoint j, dVector3 result )
	void dJointGetSliderAxis ( DVector3 result )
	{
//		dxJointSlider joint = ( dxJointSlider ) j;
//		dUASSERT ( joint, "bad joint argument" );
		dUASSERT ( result, "bad result argument" );
//		checktype ( joint, dxJointSlider.class );
		getAxis ( result, axis1 );
	}


	//void dJointSetSliderParam ( dJoint j, D_PARAM_NAMES parameter, double value )
	public void dJointSetSliderParam ( D_PARAM_NAMES_N parameter, double value )
	{
		//    dxJointSlider joint = ( dxJointSlider ) j;
		//    dUASSERT ( joint, "bad joint argument" );
		//    checktype ( joint, dxJointSlider.class );
		limot.set ( parameter.toSUB(), value );
	}


	double dJointGetSliderParam ( D_PARAM_NAMES_N parameter )
	{
		//    dxJointSlider joint = ( dxJointSlider ) j;
		//    dUASSERT ( joint, "bad joint argument" );
		//    checktype ( joint, dxJointSlider.class );
		return limot.get ( parameter.toSUB() );
	}


//	void dJointAddSliderForce ( dJoint j, double force )
	public void dJointAddSliderForce ( double force )
	{
//		dxJointSlider joint = ( dxJointSlider ) j;
		DVector3 axis = new DVector3();
//		dUASSERT ( joint, "bad joint argument" );
//		checktype ( joint, dxJointSlider.class );

		if ( (flags & dJOINT_REVERSE)!=0 )
			force -= force;

		getAxis ( axis, axis1 );
//		axis.v[0] *= force;
//		axis.v[1] *= force;
//		axis.v[2] *= force;
		axis.scale(force);

		if ( node[0].body != null )
			node[0].body.dBodyAddForce ( axis.get0(), axis.get1(), axis.get2() );
		if ( node[1].body != null )
			node[1].body.dBodyAddForce ( -axis.get0(), -axis.get1(), -axis.get2() );

		if ( node[0].body != null && node[1].body != null )
		{
			// linear torque decoupling:
			// we have to compensate the torque, that this slider force may generate
			// if body centers are not aligned along the slider axis

			DVector3 ltd = new DVector3(); // Linear Torque Decoupling vector (a torque)

			DVector3 c = new DVector3();
//			c.v[0] = 0.5 * ( joint.node[1].body._posr.pos.v[0] - joint.node[0].body._posr.pos.v[0] );
//			c.v[1] = 0.5 * ( joint.node[1].body._posr.pos.v[1] - joint.node[0].body._posr.pos.v[1] );
//			c.v[2] = 0.5 * ( joint.node[1].body._posr.pos.v[2] - joint.node[0].body._posr.pos.v[2] );
			c.eqDiff(node[1].body._posr.pos, node[0].body._posr.pos).scale(0.5);
			dCROSS ( ltd, OP.EQ , c, axis );

			node[0].body.dBodyAddTorque ( ltd.get0(), ltd.get1(), ltd.get2() );
			node[1].body.dBodyAddTorque ( ltd.get0(), ltd.get1(), ltd.get2() );
		}
	}


	/// Compute initial relative rotation body1 -> body2, or en.-> body1
	void
	computeInitialRelativeRotation()
	{
		if ( node[0].body != null)
		{
			// compute initial relative rotation body1 -> body2, or env -> body1
			// also compute center of body1 w.r.t body 2
			if ( node[1].body != null )
			{
				dQMultiply1 ( qrel, node[0].body._q, node[1].body._q );
			}
			else
			{
				// set qrel to the transpose of the first body's q
//				qrel.v[0] =  node[0].body._q.v[0];
//				qrel.v[1] = -node[0].body._q.v[1];
//				qrel.v[2] = -node[0].body._q.v[2];
//				qrel.v[3] = -node[0].body._q.v[3];
				qrel.set(node[0].body._q).scale(-1);
			}
		}
	}


	// ***********************************
	// API dSliderJoint
	// ***********************************

	public void setAxis (double x, double y, double z)
	{ dJointSetSliderAxis (x, y, z); }
	public void setAxis (DVector3C a)
	//TODO use dVector3
	{ dJointSetSliderAxis (a.get0(), a.get1(), a.get2()); }
	public void getAxis (DVector3 result)
	{ dJointGetSliderAxis (result); }

	public double getPosition()
	{ return dJointGetSliderPosition (); }
	public double getPositionRate()
	{ return dJointGetSliderPositionRate (); }

	public void setParam (D_PARAM_NAMES_N parameter, double value)
	{ dJointSetSliderParam (parameter, value); }
	public double getParam (D_PARAM_NAMES_N parameter)
	{ return dJointGetSliderParam (parameter); }
	// TODO: expose params through methods

	public void addForce (double force)
	{ dJointAddSliderForce(force); }

	
	@Override
	public void setParamFMax(double d) {
		dJointSetSliderParam(D_PARAM_NAMES_N.dParamFMax1, d);
	}


	@Override
	public void setParamHiStop(double d) {
		dJointSetSliderParam(D_PARAM_NAMES_N.dParamHiStop1, d);
	}


	@Override
	public void setParamLoStop(double d) {
		dJointSetSliderParam(D_PARAM_NAMES_N.dParamLoStop1, d);
	}


	@Override
	public void setParamVel(double d) {
		dJointSetSliderParam(D_PARAM_NAMES_N.dParamVel1, d);
	}


	@Override
	public void setParamBounce(double d) {
		dJointSetSliderParam(D_PARAM_NAMES_N.dParamBounce1, d);
	}
}
