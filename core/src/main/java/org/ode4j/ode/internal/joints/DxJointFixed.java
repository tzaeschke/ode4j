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

import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.CommonEnums.dSA__MAX;
import static org.ode4j.ode.internal.joints.JointEnums.*;


/**
 * WARNING:
 *
 * Should not be used anymore, see forums.
 *
 */
public class DxJointFixed extends DxJoint implements DFixedJoint {

	//****************************************************************************
	// fixed joint

	public DQuaternion qrel;   // initial relative rotation body1 -> body2
	DVector3 offset;    // relative offset between the bodies
	double erp;          // error reduction parameter
	double cfm;          // constraint force mix-in
	//    void  set ( int num, double value );
	//    double get ( int num );

	//    dxJointFixed ( dxWorld w );
	//    abstract void getInfo1 ( Info1 info );
	//    abstract void getInfo2 ( Info2 info );
	//    abstract dJointType type();
	//    abstract size_t size();
	//
	//    public abstract void computeInitialRelativeRotation();


	DxJointFixed ( DxWorld w )
	{
		super(w);
		offset = new DVector3();
		//dSetZero ( offset.v, 4 );
		qrel = new DQuaternion();
		//dSetZero ( qrel.v, 4 );
		erp = world.getERP();
		cfm = world.getCFM();
	}


	@Override
	void getSureMaxInfo( SureMaxInfo info )
	{
	    info.max_m = 6;
	}


	@Override
	public void	getInfo1 ( Info1 info )
	{
		info.setM(6);
		info.setNub(6);
	}


	/**
	 * @see DxJoint#getInfo2(double, double, int, double[], int, double[], int, int, double[], int, double[], int, int[], int)
	 */
	@Override
	public void getInfo2(double worldFPS, double worldERP, int rowskip, double[] J1A, int J1Ofs, double[] J2A,
						 int J2Ofs, int pairskip, double[] pairRhsCfmA, int pairRhsCfmOfs, double[] pairLoHiA,
						 int pairLoHiOfs, int[] findexA, int findexOfs) {
		// Three rows for orientation
		setFixedOrientation ( this, worldFPS, worldERP,
				rowskip, J1A, J1Ofs + dSA__MAX * rowskip, J2A, J2Ofs + dSA__MAX * rowskip,
				pairskip, pairRhsCfmA, pairRhsCfmOfs + dSA__MAX * pairskip, qrel );

		// Three rows for position.
		// set Jacobian
		J1A[J1Ofs + GI2_JLX] = 1;
		J1A[J1Ofs + rowskip + GI2_JLY] = 1;
		J1A[J1Ofs + 2 * rowskip + GI2_JLZ] = 1;

		double k = worldFPS * this.erp;
	    DxBody b0 = node[0].body, b1 = node[1].body;

	    DVector3 ofs = new DVector3();
		dMultiply0_331 ( ofs, b0.posr().R(), offset );
		if ( b1 != null )
		{
			dSetCrossMatrixPlus( J1A, J1Ofs + GI2__JA_MIN, ofs, rowskip );

			J2A[J2Ofs + GI2_JLX] = -1;
			J2A[J2Ofs + rowskip + GI2_JLY] = -1;
			J2A[J2Ofs + 2 * rowskip + GI2_JLZ] = -1;
		}

		// set right hand side for the first three rows (linear)
		if (b1 != null) {
			for (int j = 0, currPairSkip = 0; j < 3; currPairSkip += pairskip, ++j) {
				pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] =
						k * (b1.posr().pos().get(j) - b0.posr().pos().get(j) + ofs.get(j));
			}
		} else {
			for (int j = 0, currPairSkip = 0; j < 3; currPairSkip += pairskip, ++j) {
				pairRhsCfmA[pairRhsCfmOfs + currPairSkip + GI2_RHS] = k * (offset.get(j) - b0.posr().pos().get(j));
			}
		}

		double cfm = this.cfm;
		pairRhsCfmA[pairRhsCfmOfs + GI2_CFM] = cfm;
		pairRhsCfmA[pairRhsCfmOfs + pairskip + GI2_CFM] = cfm;
		pairRhsCfmA[pairRhsCfmOfs + 2 * pairskip + GI2_CFM] = cfm;
	}

	//void dJointSetFixed ( dJoint j )
	public void dJointSetFixed ()
	{
		//    dxJointFixed joint = (dxJointFixed) j;//( dxJointFixed* ) j;
		//    COM.dUASSERT ( joint, "bad joint argument" );
		//    checktype ( joint, dxJointFixed.class );

		// This code is taken from dJointSetSliderAxis(), we should really put the
		// common code in its own function.
		// compute the offset between the bodies
		if ( node[0].body != null)
		{
			if ( node[1].body != null)
			{
				//double[] ofs = new double[4];
				DVector3 ofs = new DVector3();
//				for ( int i = 0; i < 4; i++ )
//					ofs.v[i] = node[0].body._posr.pos.v[i] - 
//					node[1].body._posr.pos.v[i];
				ofs.eqDiff( node[0].body.posr().pos(), node[1].body.posr().pos() );

				dMultiply1_331 ( offset, node[0].body.posr().R(), ofs );
			}
			else
			{
				//				offset.v[0] = node[0].body._posr.pos.v[0];
				//				offset.v[1] = node[0].body._posr.pos.v[1];
				//				offset.v[2] = node[0].body._posr.pos.v[2];
				offset.set( node[0].body.posr().pos() );
			}
		}

		computeInitialRelativeRotation();
	}

	//void set ( int num, double value )
	void set (PARAM num, double value)
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


	//double get ( int num )
	double get ( PARAM num )
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


	//void dJointSetFixedParam ( dJoint j, int parameter, double value )
	public void dJointSetFixedParam ( PARAM_N parameter, double value )
	{
		//    dxJointFixed joint = (dxJointFixed) j; //( dxJointFixed* ) j;
		//    COM.dUASSERT ( joint, "bad joint argument" );
		//    checktype ( joint, dxJointFixed.class );
		set ( parameter.toSUB(), value );
	}


	//double dJointGetFixedParam ( dJoint j, int parameter )
	private double dJointGetFixedParam ( PARAM_N parameter )
	{
		//		dxJointFixed joint = (dxJointFixed) j; //( dxJointFixed* ) j;
		//		COM.dUASSERT ( joint, "bad joint argument" );
		//		checktype ( joint, dxJointFixed.class );
		return get ( parameter.toSUB() );
	}


	void
	computeInitialRelativeRotation()
	{
		if (node[0].body != null)
		{
			if (node[1].body != null)
			{
				dQMultiply1 (qrel, node[0].body._q, node[1].body._q );
			}
			else
			{
				// set qrel to the transpose of the first body q
				qrel.set0(  node[0].body._q.get0() );
				qrel.set1( -node[0].body._q.get1() );
				qrel.set2( -node[0].body._q.get2() );
				qrel.set3( -node[0].body._q.get3() );
			}
		}
	}


	// **********************************
	// API dFixedJoint
	// **********************************

	@Override
	public void set()
	{ dJointSetFixed (); }

	@Override
	public void setParam (PARAM_N parameter, double value)
	{ dJointSetFixedParam (parameter, value); }

	@Override
	public double getParam (PARAM_N parameter)
	{ return dJointGetFixedParam (parameter); }


	@Override
	public void setFixed() {
		dJointSetFixed();
	}
}