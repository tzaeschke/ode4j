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
import org.ode4j.ode.DBallJoint;
import org.ode4j.ode.internal.DxWorld;


/** 
 * ****************************************************************************
 * ball and socket.
 */
public class DxJointBall extends DxJoint implements DBallJoint
{
	DVector3 anchor1;   // anchor w.r.t first body
	DVector3 anchor2;   // anchor w.r.t second body
	double erp;          // error reduction
	double cfm;          // constraint force mix in

	DxJointBall( DxWorld w ) {
		super( w );
		anchor1 = new DVector3();
		anchor2 = new DVector3();
		//    MAT.dSetZero( anchor1, 4 );
		//    MAT.dSetZero( anchor2, 4 );
		erp = world.getERP();
		cfm = world.getCFM();
	}

	@Override
	void getSureMaxInfo( SureMaxInfo info )
	{
	    info.max_m = 3;
	}

	@Override
	public void
	getInfo1( DxJoint.Info1 info )
	{
		info.setM(3);
		info.setNub(3);
	}

	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		pairRhsCfmA[pairRhsCfmOfs + GI2_CFM] = cfm;
		pairRhsCfmA[pairRhsCfmOfs + pairskip + GI2_CFM] = cfm;
		pairRhsCfmA[pairRhsCfmOfs + 2 * pairskip + GI2_CFM] = cfm;
		setBall( this, worldFPS, this.erp, rowskip, J1A, J1Ofs, J2A, J2Ofs, pairskip, pairRhsCfmA, pairRhsCfmOfs, anchor1, anchor2 );
	}

	//void dJointSetBallAnchor( dJoint j, double x, double y, double z )
	public void dJointSetBallAnchor( DVector3C xyz )
	{
		setAnchors( xyz, anchor1, anchor2 );
		//TODO TZ: Why not computeInitialRelativeRotations(); ??? Like in other joints?
	}


	//void dJointSetBallAnchor2( dJoint j, double x, double y, double z )
	void dJointSetBallAnchor2( DVector3C xyz )
	{
		anchor2.set(xyz);
	}

	//void dJointGetBallAnchor( dJoint j, dVector3 result )
	void dJointGetBallAnchor( DVector3 result )
	{
		if ( isFlagsReverse() )
			getAnchor2( result, anchor2 );
		else
			getAnchor( result, anchor1 );
	}


	//void dJointGetBallAnchor2( dJoint j, dVector3 result )
	void dJointGetBallAnchor2( DVector3 result )
	{
		if ( isFlagsReverse() )
			getAnchor( result, anchor1 );
		else
			getAnchor2( result, anchor2 );
	}


	void set( PARAM num, double value )
	{
		switch ( num )
		{
		case dParamCFM:
			cfm = value;
			break;
		case dParamERP:
			erp = value;
			break;
		default:
		}
	}


	double get( PARAM num )
	{
		switch ( num )
		{
		case dParamCFM:
			return cfm;
		case dParamERP:
			return erp;
		default:
			return 0;
		}
	}


	void dJointSetBallParam( PARAM parameter, double value )
	{
		set( parameter, value );
	}


	double dJointGetBallParam( PARAM parameter )
	{
		return get( parameter );
	}

	@Override
	void setRelativeValues()
	{
	    DVector3 anchor = new DVector3();
	    dJointGetBallAnchor(anchor);
	    setAnchors( anchor, anchor1, anchor2 );
	}


	// *******************************
	// API dBallJoint
	// *******************************

	@Override
	public final void setAnchor (double x, double y, double z)
	{ dJointSetBallAnchor (new DVector3(x, y, z)); }
	@Override
	public final void setAnchor (DVector3C a)
	{ dJointSetBallAnchor (a); }
	@Override
	public final void setAnchor2 (double x, double y, double z)
	{ dJointSetBallAnchor2 (new DVector3(x, y, z)); }
	@Override
	public final void setAnchor2 (DVector3C a)
	{ dJointSetBallAnchor2 (a); }
	@Override
	public final void getAnchor (DVector3 result)
	{ dJointGetBallAnchor (result); }
	@Override
	public final void getAnchor2 (DVector3 result)
	{ dJointGetBallAnchor2 (result); }
	@Override
	public final void setParam (PARAM_N parameter, double value)
	{ 
		if (!parameter.isGroup1()) 
		throw new IllegalArgumentException("Only Group #1 allowed, but got: " + parameter.name());
		dJointSetBallParam (parameter.toSUB(), value); 
	}
	@Override
	public final double getParam (PARAM_N parameter)
	{ 	
		if (!parameter.isGroup1()) 
		throw new IllegalArgumentException("Only Group #1 allowed, but got: " + parameter.name());
		return dJointGetBallParam (parameter.toSUB()); 
	}
}
