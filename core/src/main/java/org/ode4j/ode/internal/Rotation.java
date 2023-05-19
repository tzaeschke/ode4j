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
package org.ode4j.ode.internal;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.OdeMath;

import static org.ode4j.ode.internal.Common.*;


/**
 * Quaternions have the format: (s,vx,vy,vz) where (vx,vy,vz) is the
 * "rotation axis" and s is the "rotation angle".
 *
 */
public class Rotation extends Matrix {

	/** 
	 * @param R R 
	 * @deprecated Use DMatrix.setIdentity() instead. 
	 */
	@Deprecated
    public static void dRSetIdentity (DMatrix3 R)
	{
		//dAASSERT (R); 
		//SET_3x3_IDENTITY;
		R.setIdentity();
	}

	public static void dRFromAxisAndAngle (DMatrix3 R, double ax, 
			double ay, double az, double angle)
	{
		//dAASSERT (R);
		DQuaternion q = new DQuaternion();
		dQFromAxisAndAngle (q, ax, ay, az, angle);
		dRfromQ (R, q);
	}
	public static void dRFromAxisAndAngle (DMatrix3 R, DVector3C axyz, double angle)
	{
		//dAASSERT (R);
		DQuaternion q = new DQuaternion();
		dQFromAxisAndAngle (q, axyz, angle);
		dRfromQ (R, q);
	}


	public static void dRFromEulerAngles (DMatrix3 R, double phi, double theta, 
			double psi)
	{
		double sphi,cphi,stheta,ctheta,spsi,cpsi;
		//dAASSERT (R);
		sphi = dSin(phi);
		cphi = dCos(phi);
		stheta = dSin(theta);
		ctheta = dCos(theta);
		spsi = dSin(psi);
		cpsi = dCos(psi);
		R.set00( cpsi*ctheta );
		R.set01( spsi*ctheta );
		R.set02( -stheta );
		//R.v[_R(0,3)] = 0.0 );
		R.set10( cpsi*stheta*sphi - spsi*cphi );
		R.set11( spsi*stheta*sphi + cpsi*cphi );
		R.set12( ctheta*sphi );
		//R.v[_R(1,3)] = 0.0 );
		R.set20( cpsi*stheta*cphi + spsi*sphi );
		R.set21( spsi*stheta*cphi - cpsi*sphi );
		R.set22( ctheta*cphi );
		//R.v[_R(2,3)] = 0.0 );
	}


	public static void dRFrom2Axes (DMatrix3 R, DVector3C a, DVector3C b) {
		dRFrom2Axes(R, 
				a.get0(), a.get1(), a.get2(), 
				b.get0(), b.get1(), b.get2());
	}
	
	/** 
	 * @param R R
	 * @param ax ax
	 * @param ay ay
	 * @param az az
	 * @param bx bx
	 * @param by by
	 * @param bz bz
	 */
    public static void dRFrom2Axes (DMatrix3 R, double ax, double ay, double az,
                                    double bx, double by, double bz)
	{
		double l,k;
		//dAASSERT (R);
		l = dSqrt (ax*ax + ay*ay + az*az);
		if (l <= 0.0) {
			dDEBUGMSG ("zero length vector");
			return;
		}
		l = dRecip(l);
		ax *= l;
		ay *= l;
		az *= l;
		k = ax*bx + ay*by + az*bz;
		bx -= k*ax;
		by -= k*ay;
		bz -= k*az;
		l = dSqrt (bx*bx + by*by + bz*bz);
		if (l <= 0.0) {
			dDEBUGMSG ("zero length vector");
			return;
		}
		l = dRecip(l);
		bx *= l;
		by *= l;
		bz *= l;
//		R.v[_R(0,0)] = ax;
//		R.v[_R(1,0)] = ay;
//		R.v[_R(2,0)] = az;
//		R.v[_R(0,1)] = bx;
//		R.v[_R(1,1)] = by;
//		R.v[_R(2,1)] = bz;
//		R.v[_R(0,2)] = - by*az + ay*bz;
//		R.v[_R(1,2)] = - bz*ax + az*bx;
//		R.v[_R(2,2)] = - bx*ay + ax*by;
//		R.v[_R(0,3)] = 0.0;
//		R.v[_R(1,3)] = 0.0;
//		R.v[_R(2,3)] = 0.0;
		R.set00( ax );
		R.set10( ay );
		R.set20( az );
		R.set01( bx );
		R.set11( by );
		R.set21( bz );
		R.set02( - by*az + ay*bz );
		R.set12( - bz*ax + az*bx );
		R.set22( - bx*ay + ax*by );
	}


	public static void dRFromZAxis (DMatrix3 R, double ax, double ay, double az)
	{
		DVector3 n = new DVector3(),p = new DVector3(),q = new DVector3();
		n.set( ax, ay, az );
		OdeMath.dNormalize3 (n);
		OdeMath.dPlaneSpace (n,p,q);
//		R.v[_R(0,0)] = p.get0();
//		R.v[_R(1,0)] = p.get1();
//		R.v[_R(2,0)] = p.get2();
		R.setCol(0, p);
//		R.v[_R(0,1)] = q.get0();
//		R.v[_R(1,1)] = q.get1();
//		R.v[_R(2,1)] = q.get2();
		R.setCol(1, q);
//		R.v[_R(0,2)] = n.get0();
//		R.v[_R(1,2)] = n.get1();
//		R.v[_R(2,2)] = n.get2();
		R.setCol(2, n);
//		R.v[_R(0,3)] = 0.0;
//		R.v[_R(1,3)] = 0.0;
//		R.v[_R(2,3)] = 0.0;
	}


	public static void dRFromZAxis (DMatrix3 R, DVector3C xyz)
	{
		DVector3 n = new DVector3(),p = new DVector3(),q = new DVector3();
		n.set(xyz);
		n.normalize();
		OdeMath.dPlaneSpace (n,p,q);
		R.set( p.get0(), q.get0(), n.get0(), 
				p.get1(), q.get1(), n.get1(),
				p.get2(), q.get2(), n.get2());
	}

	/** 
	 * @param q q
	 * @deprecated 
	 */
	@Deprecated
    public static void dQSetIdentity (DQuaternion q)
	{
		//dAASSERT (q);
		q.setIdentity();
	}


	public static void dQFromAxisAndAngle (DQuaternion q, 
			double ax, double ay, double az, double angle)
	{
		//dAASSERT (q);
		double l = ax*ax + ay*ay + az*az;
		if (l > 0.0) {
			angle *= 0.5;
			l = dSin(angle) * dRecipSqrt(l);
//			q.v[0] = dCos (angle);
//			q.v[1] = ax*l;
//			q.v[2] = ay*l;
//			q.v[3] = az*l;
			q.set( Math.cos(angle), ax*l, ay*l, az*l );
		}
		else {
			q.set( 1, 0, 0, 0);
		}
	}
	public static void dQFromAxisAndAngle (DQuaternion q, 
			DVector3C axyz, double angle)
	{
		//dAASSERT (q);
		double l = axyz.lengthSquared();//ax*ax + ay*ay + az*az;
		if (l > 0.0) {
			angle *= 0.5;
			l = dSin(angle) * dRecipSqrt(l);
//			q.v[0] = dCos (angle);
//			q.v[1] = ax*l;
//			q.v[2] = ay*l;
//			q.v[3] = az*l;
			q.set( Math.cos(angle), axyz.get0()*l, axyz.get1()*l, axyz.get2()*l );
		}
		else {
			q.set( 1, 0, 0, 0);
		}
	}


	public static void dQMultiply0 (DQuaternion qa, final DQuaternionC qb, 
			final DQuaternionC qc)
	{
		//dAASSERT (qa, qb, qc);
		qa.set0( qb.get0()*qc.get0() - qb.get1()*qc.get1() - qb.get2()*qc.get2() - qb.get3()*qc.get3() );
		qa.set1( qb.get0()*qc.get1() + qb.get1()*qc.get0() + qb.get2()*qc.get3() - qb.get3()*qc.get2() );
		qa.set2( qb.get0()*qc.get2() + qb.get2()*qc.get0() + qb.get3()*qc.get1() - qb.get1()*qc.get3() );
		qa.set3( qb.get0()*qc.get3() + qb.get3()*qc.get0() + qb.get1()*qc.get2() - qb.get2()*qc.get1() );
	}


	public static void dQMultiply1 (DQuaternion qa, final DQuaternionC qb, 
			final DQuaternionC qc)
	{
		//dAASSERT (qa, qb, qc);
		qa.set0( qb.get0()*qc.get0() + qb.get1()*qc.get1() + qb.get2()*qc.get2() + qb.get3()*qc.get3() );
		qa.set1( qb.get0()*qc.get1() - qb.get1()*qc.get0() - qb.get2()*qc.get3() + qb.get3()*qc.get2() );
		qa.set2( qb.get0()*qc.get2() - qb.get2()*qc.get0() - qb.get3()*qc.get1() + qb.get1()*qc.get3() );
		qa.set3( qb.get0()*qc.get3() - qb.get3()*qc.get0() - qb.get1()*qc.get2() + qb.get2()*qc.get1() );
	}


	public static void dQMultiply2 (DQuaternion qa, final DQuaternionC qb, 
			final DQuaternionC qc)
	{
		//dAASSERT (qa, qb, qc);
		qa.set0(  qb.get0()*qc.get0() + qb.get1()*qc.get1() + qb.get2()*qc.get2() + qb.get3()*qc.get3() );
		qa.set1( -qb.get0()*qc.get1() + qb.get1()*qc.get0() - qb.get2()*qc.get3() + qb.get3()*qc.get2() );
		qa.set2( -qb.get0()*qc.get2() + qb.get2()*qc.get0() - qb.get3()*qc.get1() + qb.get1()*qc.get3() );
		qa.set3( -qb.get0()*qc.get3() + qb.get3()*qc.get0() - qb.get1()*qc.get2() + qb.get2()*qc.get1() );
	}


	public static void dQMultiply3 (DQuaternion qa, final DQuaternionC qb, 
			final DQuaternionC qc)
	{
		//dAASSERT (qa, qb, qc);
		qa.set0(  qb.get0()*qc.get0() - qb.get1()*qc.get1() - qb.get2()*qc.get2() - qb.get3()*qc.get3() );
		qa.set1( -qb.get0()*qc.get1() - qb.get1()*qc.get0() + qb.get2()*qc.get3() - qb.get3()*qc.get2() );
		qa.set2( -qb.get0()*qc.get2() - qb.get2()*qc.get0() + qb.get3()*qc.get1() - qb.get1()*qc.get3() );
		qa.set3( -qb.get0()*qc.get3() - qb.get3()*qc.get0() + qb.get1()*qc.get2() - qb.get2()*qc.get1() );
	}


	// dRfromQ(), dQfromR() and dDQfromW() are derived from equations in "An Introduction
	// to Physically Based Modeling: Rigid Body Simulation - 1: Unconstrained
	// Rigid Body Dynamics" by David Baraff, Robotics Institute, Carnegie Mellon
	// University, 1997.

	public static void dRfromQ (DMatrix3 R, final DQuaternionC q)
	{
		//dAASSERT (q, R);
		// q = (s,vx,vy,vz)
		double q0 = q.get0();
		double q1 = q.get1();
		double q2 = q.get2();
		double q3 = q.get3();
		double qq1 = 2*q1*q1;
		double qq2 = 2*q2*q2;
		double qq3 = 2*q3*q3;
		R.set00( 1 - qq2 - qq3 );
		R.set01( 2*(q1*q2 - q0*q3) );
		R.set02( 2*(q1*q3 + q0*q2) );
		//R.v[_R(0,3)] = 0.0 );
		R.set10( 2*(q1*q2 + q0*q3) );
		R.set11( 1 - qq1 - qq3 );
		R.set12( 2*(q2*q3 - q0*q1) );
		//R.v[_R(1,3)] = 0.0 );
		R.set20( 2*(q1*q3 - q0*q2) );
		R.set21( 2*(q2*q3 + q0*q1) );
		R.set22( 1 - qq1 - qq2 );
		//R.v[_R(2,3)] = 0.0 );
	}

	/**
	 * @deprecated Please use {@link #dRfromQ(DMatrix3, DQuaternionC)} instead
	 * @param q q
	 * @param R R
	 */
	@Deprecated
    public static void dQtoR(final DQuaternionC q, DMatrix3 R) {
		dRfromQ(R, q);
	}

	/**
	 * @deprecated Please use {@link #dQfromR(DQuaternion, DMatrix3C)} instead
	 * @param q q
	 * @param R R
	 */
	@Deprecated
    public static void dRtoQ(final DMatrix3C R, DQuaternion q) {
		dQfromR(q, R);
	}

	public static void dQfromR (DQuaternion q, final DMatrix3C R)
	{
		//dAASSERT (q, R);
		double tr,s;
		tr = R.get00() + R.get11() + R.get22();
		if (tr >= 0) {
			s = dSqrt (tr + 1);
			q.set0( 0.5 * s );
			s = 0.5 * dRecip(s);
			q.set1( (R.get21() - R.get12()) * s );
			q.set2( (R.get02() - R.get20()) * s );
			q.set3( (R.get10() - R.get01()) * s );
		}
		else {
			// find the largest diagonal element and jump to the appropriate case
			if (R.get11() > R.get00()) {
				if (R.get22() > R.get11()) { 
					DQFR_case_2(q, R);//goto case_2;
					return;
				}
				DQFR_case_1(q, R);//goto case_1;
				return;
			}
			if (R.get22() > R.get00()) {
				DQFR_case_2(q, R);//goto case_2;
				return;
			}
			DQFR_case_0(q, R);//goto case_0;
		}
	}
	
	private static void DQFR_case_0(DQuaternion q, final DMatrix3C R) {
		//case_0:
		double s = dSqrt((R.get00() - (R.get11() + R.get22())) + 1.);
		q.set1( 0.5 * s );
		s = 0.5 * dRecip(s);
		q.set2( (R.get01() + R.get10()) * s );
		q.set3( (R.get20() + R.get02()) * s );
		q.set0( (R.get21() - R.get12()) * s );
		return;
	}

	private static void DQFR_case_1(DQuaternion q, final DMatrix3C R) {
		//case_1:
		double s = dSqrt((R.get11() - (R.get22() + R.get00())) + 1.);
		q.set2( 0.5 * s );
		s = 0.5 * dRecip(s);
		q.set3( (R.get12() + R.get21()) * s );
		q.set1( (R.get01() + R.get10()) * s );
		q.set0( (R.get02() - R.get20()) * s );
		return;
	}

	private static void DQFR_case_2(DQuaternion q, final DMatrix3C R) {
		//case_2:
		double s = dSqrt((R.get22() - (R.get00() + R.get11())) + 1.);
		q.set3( 0.5 * s );
		s = 0.5 * dRecip(s);
		q.set1( (R.get20() + R.get02()) * s );
		q.set2( (R.get12() + R.get21()) * s );
		q.set0( (R.get10() - R.get01()) * s );
		return;
	}


	/**
	 * @param w w
	 * @param q q
	 * @param dq dq
	 * @deprecated
	 */
	@Deprecated
    public static void dWtoDQ(final DVector3C w, final DQuaternionC q, DQuaternion dq)
	{
		dDQfromW(dq, w, q);
	}

	public static void dDQfromW (DQuaternion dq, final DVector3C w, final DQuaternionC q)
	{
		//dAASSERT (w, q, dq);
		dq.set(0, 0.5*(- w.get0()*q.get1() - w.get1()*q.get2() - w.get2()*q.get3()) );
		dq.set(1, 0.5*(  w.get0()*q.get0() + w.get1()*q.get3() - w.get2()*q.get2()) );
		dq.set(2, 0.5*(- w.get0()*q.get3() + w.get1()*q.get0() + w.get2()*q.get1()) );
		dq.set(3, 0.5*(  w.get0()*q.get2() - w.get1()*q.get1() + w.get2()*q.get0()) );
	}

	private Rotation() {}
}
