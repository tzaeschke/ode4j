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
import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.internal.DxWorld;

import static org.ode4j.ode.OdeMath.*;


/**
 * ***************************************************************************
 * angular motor
 * 
 */
public class DxJointAMotor extends DxJoint implements DAMotorJoint
{

	private int _num;                // number of axes (0..3)
	private AMotorMode _mode;               // a dAMotorXXX constant
	private int[] _rel =new int[3];             // what the axes are relative to (global,b1,b2)
	private DVector3[] axis = new DVector3[3];       // three axes
	private DxJointLimitMotor[] limot= new DxJointLimitMotor[3]; // limit+motor info for axes
	private double[] angle = new double[3];         // user-supplied angles for axes
	// these vectors are used for calculating euler angles
	private DVector3 reference1 = new DVector3();    // original axis[2], relative to body 1
	private DVector3 reference2 = new DVector3();    // original axis[0], relative to body 2


	//    void computeGlobalAxes( dVector3 ax[3] );
	//    void computeEulerAngles( dVector3 ax[3] );
	//    void setEulerReferenceVectors();
	//
	//
	//    dxJointAMotor( dxWorld *w );
	//    virtual void getInfo1( Info1* info );
	//    virtual void getInfo2( Info2* info );
	//    virtual dJointType type() const;
	//    virtual size_t size() const;
	//};

	DxJointAMotor( DxWorld w ) 
	//  dxJoint( w )
	{
		super(w);
		int i;
		_num = 0;
		_mode = AMotorMode.dAMotorUser;
		for ( i = 0; i < 3; i++ )
		{
			_rel[i] = 0;
			//MAT.dSetZero( axis[i], 4 );
			axis[i] = new DVector3();
			limot[i] = new DxJointLimitMotor();
			limot[i].init( world );
			angle[i] = 0;
		}
		//    MAT.dSetZero( reference1, 4 );
		//    MAT.dSetZero( reference2, 4 );
	}


	// compute the 3 axes in global coordinates
	void
	computeGlobalAxes( DVector3[] ax)//[3] )
	{
		if ( _mode == AMotorMode.dAMotorEuler )
		{
			// special handling for euler mode
			dMultiply0_331( ax[0], node[0].body.posr().R(), axis[0] );
			if ( node[1].body !=null)
			{
				dMultiply0_331( ax[2], node[1].body.posr().R(), axis[2] );
			}
			else
			{
//				ax[2].v[0] = axis[2].v[0];
//				ax[2].v[1] = axis[2].v[1];
//				ax[2].v[2] = axis[2].v[2];
				ax[2].set( axis[2] );
			}
			dCalcVectorCross3( ax[1], ax[2], ax[0] );
			dNormalize3( ax[1] );
		}
		else
		{
			for ( int i = 0; i < _num; i++ )
			{
				if ( _rel[i] == 1 )
				{
					// relative to b1
					dMultiply0_331( ax[i], node[0].body.posr().R(), axis[i] );
				}
				else if ( _rel[i] == 2 )
				{
					// relative to b2
					if ( node[1].body != null)   // jds: don't assert, just ignore
					{
						dMultiply0_331( ax[i], node[1].body.posr().R(), axis[i] );
					}
	                else
	                {
	                    // global - just copy it
						ax[i].set(axis[i]);
						//ax[i][0] = axis[i][0];
	                    //ax[i][1] = axis[i][1];
	                    //ax[i][2] = axis[i][2];
	                }
				}
				else
				{
					// global - just copy it
					ax[i].set(axis[i]);
					//                ax[i].v[0] = axis[i].v[0];
					//                ax[i].v[1] = axis[i].v[1];
					//                ax[i].v[2] = axis[i].v[2];
				}
			}
		}
	}


	void
	computeEulerAngles( DVector3[] ax) //[3] )
	{
		// assumptions:
		//   global axes already calculated --> ax
		//   axis[0] is relative to body 1 --> global ax[0]
		//   axis[2] is relative to body 2 --> global ax[2]
		//   ax[1] = ax[2] x ax[0]
		//   original ax[0] and ax[2] are perpendicular
		//   reference1 is perpendicular to ax[0] (in body 1 frame)
		//   reference2 is perpendicular to ax[2] (in body 2 frame)
		//   all ax[] and reference vectors are unit length

		// calculate references in global frame
		DVector3 ref1 = new DVector3(), ref2 = new DVector3();
		dMultiply0_331( ref1, node[0].body.posr().R(), reference1 );
		if ( node[1].body!=null )
		{
			dMultiply0_331( ref2, node[1].body.posr().R(), reference2 );
		}
		else
		{
			ref2.set(reference2);
			//        ref2[0] = reference2[0];
			//        ref2[1] = reference2[1];
			//        ref2[2] = reference2[2];
		}

		// get q perpendicular to both ax[0] and ref1, get first euler angle
		DVector3 q = new DVector3();
		dCalcVectorCross3( q, ax[0], ref1 );
		angle[0] = -dAtan2( dCalcVectorDot3( ax[2], q ), dCalcVectorDot3( ax[2], ref1 ) );

		// get q perpendicular to both ax[0] and ax[1], get second euler angle
		dCalcVectorCross3( q, ax[0], ax[1] );
		angle[1] = -dAtan2( dCalcVectorDot3( ax[2], ax[0] ), dCalcVectorDot3( ax[2], q ) );

		// get q perpendicular to both ax[1] and ax[2], get third euler angle
		dCalcVectorCross3( q, ax[1], ax[2] );
		angle[2] = -dAtan2( dCalcVectorDot3( ref2, ax[1] ), dCalcVectorDot3( ref2, q ) );
	}


	// set the reference vectors as follows:
	//   * reference1 = current axis[2] relative to body 1
	//   * reference2 = current axis[0] relative to body 2
	// this assumes that:
	//    * axis[0] is relative to body 1
	//    * axis[2] is relative to body 2

	void
	setEulerReferenceVectors()
	{
		if ( node[0].body != null&& node[1].body != null)
		{
			DVector3 r = new DVector3();  // axis[2] and axis[0] in global coordinates
			dMultiply0_331( r, node[1].body.posr().R(), axis[2] );
			dMultiply1_331( reference1, node[0].body.posr().R(), r );
			dMultiply0_331( r, node[0].body.posr().R(), axis[0] );
			dMultiply1_331( reference2, node[1].body.posr().R(), r );
		} else {
			// We want to handle angular motors attached to passive geoms
	        // Replace missing node.R with identity
	        if (node[0].body != null) {
	          dMultiply1_331( reference1, node[0].body.posr().R(), axis[2] );
	          dMultiply0_331( reference2, node[0].body.posr().R(), axis[0] );
	        } else if (node[1].body != null) {
	          dMultiply0_331( reference1, node[1].body.posr().R(), axis[2] );
	          dMultiply1_331( reference2, node[1].body.posr().R(), axis[0] );
	        }
		}
	}

	
	@Override
	void 
	getSureMaxInfo( SureMaxInfo info )
	{
	    info.max_m = _num;
	}


	@Override
	public void
	getInfo1( DxJoint.Info1 info )
	{
		info.setM((byte)0);
		info.setNub(0);

		// compute the axes and angles, if in Euler mode
		if ( _mode == AMotorMode.dAMotorEuler )
		{
			DVector3[] ax = {new DVector3(), new DVector3(), new DVector3()};//new dVector3[3];
			computeGlobalAxes( ax );
			computeEulerAngles( ax );
		}

		// see if we're powered or at a joint limit for each axis
		for ( int i = 0; i < _num; i++ )
		{
			if ( limot[i].testRotationalLimit( angle[i] ) ||
					limot[i].fmax > 0 )
			{
				info.incM();
			}
		}
	}


	@Override
	public void
	getInfo2( double worldFPS, double worldERP, DxJoint.Info2Descr info )
	{
		int i;

		// compute the axes (if not global)
		DVector3[] ax = new DVector3[] {new DVector3(),new DVector3(),new DVector3()} ;
		computeGlobalAxes( ax );

		// in euler angle mode we do not actually constrain the angular velocity
		// along the axes axis[0] and axis[2] (although we do use axis[1]) :
		//
		//    to get   constrain w2-w1 along  ...not
		//    ------   ---------------------  ------
		//    d(angle[0])/dt = 0 ax[1] x ax[2]   ax[0]
		//    d(angle[1])/dt = 0 ax[1]
		//    d(angle[2])/dt = 0 ax[0] x ax[1]   ax[2]
		//
		// constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
		// to prove the result for angle[0], write the expression for angle[0] from
		// GetInfo1 then take the derivative. to prove this for angle[2] it is
		// easier to take the euler rate expression for d(angle[2])/dt with respect
		// to the components of w and set that to 0.

		//TODO why use axprt at all? Why not just use ax???
		//    dVector3 *axptr[3];
		//    axptr[0] = &ax[0];
		//    axptr[1] = &ax[1];
		//    axptr[2] = &ax[2];
		DVector3[] axptr = new DVector3[] {ax[0], ax[1], ax[2]};

		DVector3 ax0_cross_ax1 = new DVector3();
		DVector3 ax1_cross_ax2 = new DVector3();
		if ( _mode == AMotorMode.dAMotorEuler )
		{
		    dCalcVectorCross3( ax0_cross_ax1, ax[0], ax[1] );
			//        axptr[2] = &ax0_cross_ax1;
			axptr[2] = ax0_cross_ax1;
			dCalcVectorCross3( ax1_cross_ax2, ax[1], ax[2] );
			//        axptr[0] = &ax1_cross_ax2;
			axptr[0] = ax1_cross_ax2;
		}

		int row = 0;
		for ( i = 0; i < _num; i++ )
		{
			//    row += limot[i].addLimot( this, info, row, *( axptr[i] ), true );
			row += limot[i].addLimot( this, worldFPS, info, row, axptr[i] , true );
		}
	}


	//void dJointSetAMotorNumAxes( dxJointAMotor j, int num )
	public void dJointSetAMotorNumAxes( int num )
	{
		dAASSERT( num >= 0 && num <= 3 );
		if ( _mode == AMotorMode.dAMotorEuler )
		{
			_num = 3;
		}
		else
		{
			if ( num < 0 ) num = 0;
			if ( num > 3 ) num = 3;
			_num = num;
		}
	}


	//void dJointSetAMotorAxis( dxJointAMotor j, int anum, int rel, double x, double y, double z )
	public void dJointSetAMotorAxis( int anum, int rel, double x, double y, double z )
	{
		dJointSetAMotorAxis( anum, rel, new DVector3(x, y, z));
	}
	
	
	public void dJointSetAMotorAxis( int anum, int rel, DVector3C r )
	{
		dAASSERT( anum >= 0 && anum <= 2 && rel >= 0 && rel <= 2 );

		if ( anum < 0 ) anum = 0;
		if ( anum > 2 ) anum = 2;

		// adjust rel to match the internal body order
	    if ( (flags & dJOINT_REVERSE)!=0 && rel!=0 )
	        rel ^= 3; // turns 1 into 2, 2, into 1

		_rel[anum] = rel;

		// x,y,z (vector r)  is always in global coordinates regardless of rel, so we may have
		// to convert it to be relative to a body
		if ( rel > 0 )
		{
			if ( rel == 1 )
			{
				dMultiply1_331( axis[anum], node[0].body.posr().R(), r );
			}
			else // rel == 2
			{
				// don't assert; handle the case of attachment to a bodiless geom
				if ( node[1].body!=null )   // jds
				{
					dMultiply1_331( axis[anum], node[1].body.posr().R(), r );
				}
				else
				{
//					axis[anum].v[0] = r.v[0];
//					axis[anum].v[1] = r.v[1];
//					axis[anum].v[2] = r.v[2];
//					axis[anum].v[3] = r.v[3];
					axis[anum].set(r);
				}
			}
		}
		else
		{
//			axis[anum].v[0] = r.v[0];
//			axis[anum].v[1] = r.v[1];
//			axis[anum].v[2] = r.v[2];
			axis[anum].set(r);
		}
		dNormalize3( axis[anum] );
		if ( _mode == AMotorMode.dAMotorEuler ) setEulerReferenceVectors();
	}


	void dJointSetAMotorAngle( int anum, double angle )
	{
		dAASSERT( anum >= 0 && anum < 3 );
		if ( _mode == AMotorMode.dAMotorUser )
		{
			if ( anum < 0 ) anum = 0;
			if ( anum > 2 ) anum = 2;
			this.angle[anum] = angle;
		}
	}


	public void dJointSetAMotorParam( PARAM_N parameter, double value )
	{
		//TODO remove
		int anum = parameter.toGROUP().getIndex();
		//    int anum = parameter.grp();// >> 8;
		//    if ( anum < 0 ) anum = 0;
		//    if ( anum > 2 ) anum = 2;
		//parameter &= 0xff;
		limot[anum].set( parameter.toSUB(), value );
	}


	//void dJointSetAMotorMode( dxJointAMotor j, dAMotorMode mode )
	public void dJointSetAMotorMode( AMotorMode mode )
	{
		_mode = mode;
		if ( _mode == AMotorMode.dAMotorEuler )
		{
			_num = 3;
			setEulerReferenceVectors();
		}
	}


	int dJointGetAMotorNumAxes( )
	{
		return _num;
	}


	void dJointGetAMotorAxis( int anum, DVector3 result )
	{
		dAASSERT( anum >= 0 && anum < 3 );
		if ( anum < 0 ) anum = 0;
		if ( anum > 2 ) anum = 2;
	    
	    // If we're in Euler mode, joint->axis[1] doesn't
	    // have anything sensible in it.  So don't just return
	    // that, find the actual effective axis.
	    // Likewise, the actual axis of rotation for the
	    // the other axes is different from what's stored.
	    if ( _mode == AMotorMode.dAMotorEuler  ) {
	      DVector3[] axes = new DVector3[]{new DVector3(), new DVector3(), new DVector3()};
	      computeGlobalAxes(axes);
	      if (anum == 1) {
	        //result[0]=axes[1][0];
	        //result[1]=axes[1][1];
	        //result[2]=axes[1][2];
	    	result.set(axes[1]);
	      } else if (anum == 0) {
	        // This won't be unit length in general,
	        // but it's what's used in getInfo2
	        // This may be why things freak out as
	        // the body-relative axes get close to each other.
	        dCalcVectorCross3( result, axes[1], axes[2] );
	      } else if (anum == 2) {
	        // Same problem as above.
	        dCalcVectorCross3( result, axes[0], axes[1] );
	      }
	    } else if ( _rel[anum] > 0 ) {
			if ( _rel[anum] == 1 )
			{
				dMultiply0_331( result, node[0].body.posr().R(), axis[anum] );
			}
			else
			{
				if ( node[1].body!=null )   // jds
				{
					dMultiply0_331( result, node[1].body.posr().R(), axis[anum] );
				}
				else
				{
//					result.v[0] = joint.axis[anum].v[0];
//					result.v[1] = joint.axis[anum].v[1];
//					result.v[2] = joint.axis[anum].v[2];
					//TODO?
//					result.v[3] = joint.axis[anum].v[3];
					result.set(axis[anum]);
				}
			}
		}
		else
		{
//			result.v[0] = joint.axis[anum].v[0];
//			result.v[1] = joint.axis[anum].v[1];
//			result.v[2] = joint.axis[anum].v[2];
			result.set(axis[anum]);
		}
	}


	int dJointGetAMotorAxisRel( int anum )
	{
		dAASSERT( anum >= 0 && anum < 3 );
		if ( anum < 0 ) anum = 0;
		if ( anum > 2 ) anum = 2;
	    int rel = _rel[anum];
	    if ( (flags & dJOINT_REVERSE)!=0 && rel!=0)
	         rel ^= 3; // turns 1 into 2, 2 into 1
	    return rel;
	}

	//TODO use enum?
	public double dJointGetAMotorAngle( int anum )
	{
		dAASSERT( anum >= 0 && anum < 3 );
		if ( anum < 0 ) anum = 0;
		if ( anum > 2 ) anum = 2;
		return angle[anum];
	}


	double dJointGetAMotorAngleRate( int anum )
	{
	    dAASSERT( anum >= 0 && anum < 3);
	  
	    if (node[0].body != null) {
	      DVector3 axis = new DVector3();
	      dJointGetAMotorAxis (anum, axis);
	      double rate = axis.dot( node[0].body.avel );
	      if (node[1].body!=null) rate -= axis.dot( node[1].body.avel );
	      return rate;
	    }
		return 0;
	}


	double dJointGetAMotorParam( PARAM_N parameter )
	{
		int anum = parameter.toGROUP().getIndex();// >> 8;
		//    if ( anum < 0 ) anum = 0;
		//    if ( anum > 2 ) anum = 2;
		//parameter &= 0xff;
		return limot[anum].get( parameter.toSUB() );
	}


	AMotorMode dJointGetAMotorMode()
	{
		return _mode;
	}


	void dJointAddAMotorTorques( double torque1, double torque2, double torque3 )
	{
		DVector3[] axes = new DVector3[]{new DVector3(), new DVector3(), new DVector3()};

		if ( _num == 0 )
			return;
		dUASSERT( !isFlagsReverse(), 
				"dJointAddAMotorTorques not yet implemented for reverse AMotor joints" );

		computeGlobalAxes( axes );
//		axes[0].v[0] *= torque1;
//		axes[0].v[1] *= torque1;
//		axes[0].v[2] *= torque1;
		axes[0].scale(torque1);
		if ( _num >= 2 )
		{
//			axes[0].v[0] += axes[1].v[0] * torque2;
//			axes[0].v[1] += axes[1].v[1] * torque2;
//			axes[0].v[2] += axes[1].v[2] * torque2;
			axes[0].eqSum(axes[0], axes[1], torque2);
			if ( _num >= 3 )
			{
//				axes[0].v[0] += axes[2].v[0] * torque3;
//				axes[0].v[1] += axes[2].v[1] * torque3;
//				axes[0].v[2] += axes[2].v[2] * torque3;
				axes[0].eqSum(axes[0], axes[2], torque3);
			}
		}

		if ( node[0].body != null )
			node[0].body.dBodyAddTorque( axes[0] );
		if ( node[1].body != null )
			node[1].body.dBodyAddTorque( axes[0].reScale(-1) );
	}


	// **********************************
	// API dAMotorJoint
	// **********************************

	@Override
	public void setMode (AMotorMode mode)
	{ dJointSetAMotorMode (mode); }
	@Override
	public AMotorMode getMode()
	{ return dJointGetAMotorMode (); }

	@Override
	public void setNumAxes (int num)
	{ dJointSetAMotorNumAxes (num); }
	@Override
	public int getNumAxes()
	{ return dJointGetAMotorNumAxes (); }

	@Override
	public void setAxis (int anum, int rel, double x, double y, double z)
	{ dJointSetAMotorAxis (anum, rel, x, y, z); }
	@Override
	public void setAxis (int anum, int rel, DVector3C a)
	{ dJointSetAMotorAxis (anum, rel, a); }
	@Override
	public void getAxis (int anum, DVector3 result)
	{ dJointGetAMotorAxis (anum, result); }
	@Override
	public int getAxisRel (int anum)
	{ return dJointGetAMotorAxisRel (anum); }

	@Override
	public void setAngle (int anum, double angle)
	{ dJointSetAMotorAngle (anum, angle); }
	@Override
	public double getAngle (int anum)
	{ return dJointGetAMotorAngle (anum); }
	@Override
	public double getAngleRate (int anum)
	{ return dJointGetAMotorAngleRate (anum); }

	@Override
	public void setParam (PARAM_N parameter, double value)
	{ dJointSetAMotorParam (parameter, value); }
	@Override
	public double getParam (PARAM_N parameter)
	{ return dJointGetAMotorParam (parameter); }

	@Override
	public void addTorques(double torque1, double torque2, double torque3)
	{ dJointAddAMotorTorques(torque1, torque2, torque3); }


	@Override
	public void setParamFMax(double d) {
		dJointSetAMotorParam(PARAM_N.dParamFMax1, d);
	}


	@Override
	public void setParamFMax2(double d) {
		dJointSetAMotorParam(PARAM_N.dParamFMax2, d);
	}


	@Override
	public void setParamFMax3(double d) {
		dJointSetAMotorParam(PARAM_N.dParamFMax3, d);
	}


	@Override
	public void setParamHiStop(double d) {
		dJointSetAMotorParam(PARAM_N.dParamHiStop1, d);
	}


	@Override
	public void setParamHiStop2(double d) {
		dJointSetAMotorParam(PARAM_N.dParamHiStop2, d);
	}


	@Override
	public void setParamHiStop3(double d) {
		dJointSetAMotorParam(PARAM_N.dParamHiStop3, d);
	}


	@Override
	public void setParamLoStop(double d) {
		dJointSetAMotorParam(PARAM_N.dParamLoStop1, d);
	}


	@Override
	public void setParamLoStop2(double d) {
		dJointSetAMotorParam(PARAM_N.dParamLoStop2, d);
	}


	@Override
	public void setParamLoStop3(double d) {
		dJointSetAMotorParam(PARAM_N.dParamLoStop3, d);
	}


	@Override
	public void setParamVel(double d) {
		dJointSetAMotorParam(PARAM_N.dParamVel1, d);
	}


	@Override
	public void setParamVel2(double d) {
		dJointSetAMotorParam(PARAM_N.dParamVel2, d);
	}


	@Override
	public void setParamVel3(double d) {
		dJointSetAMotorParam(PARAM_N.dParamVel3, d);
	}


	@Override
	public double getParamFMax() {
		return dJointGetAMotorParam(PARAM_N.dParamFMax1);
	}


	@Override
	public double getParamFMax2() {
		return dJointGetAMotorParam(PARAM_N.dParamFMax2);
	}


	@Override
	public double getParamFMax3() {
		return dJointGetAMotorParam(PARAM_N.dParamFMax3);
	}


	@Override
	public double getParamVel() {
		return dJointGetAMotorParam(PARAM_N.dParamVel1);
	}


	@Override
	public double getParamVel2() {
		return dJointGetAMotorParam(PARAM_N.dParamVel2);
	}


	@Override
	public double getParamVel3() {
		return dJointGetAMotorParam(PARAM_N.dParamVel3);
	}

}