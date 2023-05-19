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
import org.ode4j.ode.DLMotorJoint;
import org.ode4j.ode.internal.DxWorld;

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.dAASSERT;


/**
 * ****************************************************************************
 * lmotor joint
 */
public class DxJointLMotor extends DxJoint implements DLMotorJoint {
	private int num;
	private final int[] _rel = new int[3];
	private final DVector3[] axis = new DVector3[3];
	private final DxJointLimitMotor[] limot = new DxJointLimitMotor[3];


	DxJointLMotor( DxWorld w ) 
	//dxJoint( w )
	{
		super(w);
		int i;
		num = 0;
		for ( i = 0;i < 3;i++ )
		{
			//dSetZero( axis[i], 4 );
			axis[i] = new DVector3();
			limot[i] = new DxJointLimitMotor();
			limot[i].init( world );
		}
	}

	void
	//computeGlobalAxes( dVector3 ax[3] )
	computeGlobalAxes( DVector3[] ax )
	{
		for ( int i = 0; i < num; i++ )
		{
			if ( _rel[i] == 1 )
			{
				dMultiply0_331( ax[i], node[0].body.posr().R(), axis[i] );
			}
			else if ( _rel[i] == 2 )
			{
				if ( node[1].body!= null )   // jds: don't assert, just ignore
				{
					dMultiply0_331( ax[i], node[1].body.posr().R(), axis[i] );
				}
			}
			else
			{
				ax[i].set(axis[i]);
				//            ax[i][0] = axis[i][0];
				//            ax[i][1] = axis[i][1];
				//            ax[i][2] = axis[i][2];
			}
		}
	}

	@Override
	void getSureMaxInfo( SureMaxInfo info )
	{
	    info.max_m = num;
	}

	@Override
	public void
	getInfo1( DxJoint.Info1 info )
	{
		info.setM(0);
		info.setNub(0);
		for ( int i = 0; i < num; i++ )
		{
			if ( limot[i].fmax > 0 )
			{
				info.incM();
			}
		}
	}

	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		DVector3[] ax = new DVector3[]{new DVector3(), new DVector3(), new DVector3()};
		computeGlobalAxes(ax);

		int currRowSkip = 0, currPairSkip = 0;
		for (int i = 0; i < num; ++i) {
			if (limot[i].addLimot(this, worldFPS, J1A, J1Ofs + currRowSkip, J2A, J2Ofs + currRowSkip, pairRhsCfmA,
					pairRhsCfmOfs + currPairSkip, pairLoHiA, pairLoHiOfs + currPairSkip, ax[i], false)) {
				currRowSkip += rowskip;
				currPairSkip += pairskip;
			}
		}
	}

	//void dJointSetLMotorAxis( dJoint j, int anum, int rel, dReal x, dReal y, dReal z )
	public void dJointSetLMotorAxis( int anum, int rel, double x, double y, double z )
	{
		dJointSetLMotorAxis(anum, rel, new DVector3(x, y, z));
	}
	
	public void dJointSetLMotorAxis( int anum, int rel, DVector3C r )
	{
		//    dxJointLMotor joint = ( dxJointLMotor )j;
		//for now we are ignoring rel!
		//    dAASSERT( joint != null && anum >= 0 && anum <= 2 && rel >= 0 && rel <= 2 );
		dAASSERT( anum >= 0 && anum <= 2 && rel >= 0 && rel <= 2 );
		//    checktype( joint, dxJointLimitMotor.class );

		if ( anum < 0 ) anum = 0;
		if ( anum > 2 ) anum = 2;

		//    if ( !joint.node[1].body && rel == 2 ) rel = 1; //ref 1
		if ( node[1].body == null && rel == 2 ) rel = 1; //ref 1

		_rel[anum] = rel;

//		dVector3 r = new dVector3(x, y, z, 0);
		//    r.v[0] = x;
		//    r.v[1] = y;
		//    r.v[2] = z;
		//    r.v[3] = 0;
		if ( rel > 0 )
		{
			if ( rel == 1 )
			{
				dMultiply1_331( axis[anum], node[0].body.posr().R(), r );
			}
			else
			{
				//second body has to exists thanks to ref 1 line
				dMultiply1_331( axis[anum], node[1].body.posr().R(), r );
			}
		}
		else
		{
			axis[anum].set(r);
			//        joint.axis[anum][0] = r[0];
			//        joint.axis[anum][1] = r[1];
			//        joint.axis[anum][2] = r[2];
		}

		dNormalize3( axis[anum] );
	}

	//void dJointSetLMotorNumAxes( dJoint j, int num )
	public void dJointSetLMotorNumAxes( int num )
	{
		if ( num < 0 ) num = 0;
		if ( num > 3 ) num = 3;
		this.num = num;
	}

	//void dJointSetLMotorParam( dJoint j, int parameter, dReal value )
	public void dJointSetLMotorParam( PARAM_N parameter, double value )
	{
		int anum = parameter.toGROUP().getIndex();//val() >> 8;  //TODO use >>> ????
		if ( anum < 0 ) anum = 0;
		if ( anum > 2 ) anum = 2;
		//parameter &= 0xff;
		limot[anum].set( parameter.toSUB(), value );
	}

	//int dJointGetLMotorNumAxes( dJoint j )
	int dJointGetLMotorNumAxes( )
	{
		return num;
	}


	//void dJointGetLMotorAxis( dJoint j, int anum, dVector3 result )
	void dJointGetLMotorAxis( int anum, DVector3 result )
	{
		dAASSERT( anum >= 0 && anum < 3 );
		if ( anum < 0 ) anum = 0;
		if ( anum > 2 ) anum = 2;
		result.set(axis[anum]);
	}

	//dReal dJointGetLMotorParam( dJoint j, int parameter )
	public double dJointGetLMotorParam( PARAM_N parameter )
	{
		int anum = parameter.toGROUP().getIndex();//val() >> 8;
		if ( anum < 0 ) anum = 0;
		if ( anum > 2 ) anum = 2;
		//parameter &= 0xff;
		return limot[anum].get( parameter.toSUB() );
	}


	// ******************************
	// API dLMotorJoint
	// ******************************

	@Override
	public void setNumAxes (int num)
	{ dJointSetLMotorNumAxes (num); }
	@Override
	public int getNumAxes()
	{ return dJointGetLMotorNumAxes (); }

	@Override
	public void setAxis (int anum, int rel, double x, double y, double z)
	{ dJointSetLMotorAxis (anum, rel, x, y, z); }
	@Override
	public void setAxis (int anum, int rel, DVector3C a)
	{ dJointSetLMotorAxis (anum, rel, a); }
	@Override
	public void getAxis (int anum, DVector3 result)
	{ dJointGetLMotorAxis (anum, result); }

	@Override
	public void setParam (PARAM_N parameter, double value)
	{ dJointSetLMotorParam (parameter, value); }
	@Override
	public double getParam (PARAM_N parameter)
	{ return dJointGetLMotorParam (parameter); }

	@Override
	public void setParamFMax(double d) {
		dJointSetLMotorParam(PARAM_N.dParamFMax1, d);
	}

	@Override
	public void setParamFMax2(double d) {
		dJointSetLMotorParam(PARAM_N.dParamFMax2, d);
	}

	@Override
	public void setParamFMax3(double d) {
		dJointSetLMotorParam(PARAM_N.dParamFMax3, d);
	}

	@Override
	public void setParamVel(double d) {
		dJointSetLMotorParam(PARAM_N.dParamVel1, d);
	}

	@Override
	public void setParamVel2(double d) {
		dJointSetLMotorParam(PARAM_N.dParamVel2, d);
	}

	@Override
	public void setParamVel3(double d) {
		dJointSetLMotorParam(PARAM_N.dParamVel3, d);
	}

	@Override
	public double getParamFMax() {
		return dJointGetLMotorParam(PARAM_N.dParamFMax1);
	}

	@Override
	public double getParamFMax2() {
		return dJointGetLMotorParam(PARAM_N.dParamFMax2);
	}

	@Override
	public double getParamFMax3() {
		return dJointGetLMotorParam(PARAM_N.dParamFMax3);
	}

	@Override
	public double getParamVel() {
		return dJointGetLMotorParam(PARAM_N.dParamVel1);
	}

	@Override
	public double getParamVel2() {
		return dJointGetLMotorParam(PARAM_N.dParamVel2);
	}

	@Override
	public double getParamVel3() {
		return dJointGetLMotorParam(PARAM_N.dParamVel3);
	}
}

