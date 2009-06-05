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
package org.ode4j.ode.internal;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.OdeMath;


/**
 * quaternions have the format: (s,vx,vy,vz) where (vx,vy,vz) is the
 * "rotation axis" and s is the "rotation angle".
 *
 */
public class Rotation extends Matrix {

	//#define _R(i,j) R[(i)*4+(j)]
	private static final int _R(int i, int j) {
		return i*4 + j;
	}
	//
	//#define SET_3x3_IDENTITY \
	//  _R(0,0) = REAL(1.0); \
	//  _R(0,1) = REAL(0.0); \
	//  _R(0,2) = REAL(0.0); \
	//  _R(0,3) = REAL(0.0); \
	//  _R(1,0) = REAL(0.0); \
	//  _R(1,1) = REAL(1.0); \
	//  _R(1,2) = REAL(0.0); \
	//  _R(1,3) = REAL(0.0); \
	//  _R(2,0) = REAL(0.0); \
	//  _R(2,1) = REAL(0.0); \
	//  _R(2,2) = REAL(1.0); \
	//  _R(2,3) = REAL(0.0);
	//	private final double[][] _R = { {1.0, 0.0, 0,0, 0.0}, 
	//			{0.0, 1.0, 0,0, 0.0}, 
	//			{0.0, 0.0, 1,0, 0.0}, 
	//			{0.0, 0.0, 0,0, 1.0}};
	//	private void _R(int i, int j, double x) {
	//		
	//	}
	private static void setIdentity(double[] R) {
		R[_R(0,0)] = 1;
		R[_R(0,1)] = 0;
		R[_R(0,2)] = 0;
		R[_R(0,3)] = 0;
		R[_R(1,0)] = 0;
		R[_R(1,1)] = 1;
		R[_R(1,2)] = 0;
		R[_R(1,3)] = 0;
		R[_R(2,0)] = 0;
		R[_R(2,1)] = 0;
		R[_R(2,2)] = 1;
		R[_R(2,3)] = 0;
	}

	public static void dRSetIdentity (DMatrix3 R)
	{
		dAASSERT (R); 
		//SET_3x3_IDENTITY;
		setIdentity(R.v);
	}

	public static void dRFromAxisAndAngle (DMatrix3 R, double ax, 
			double ay, double az, double angle)
	{
		dAASSERT (R);
		DQuaternion q = new DQuaternion();
		dQFromAxisAndAngle (q, ax, ay, az, angle);
		dRfromQ (R, q);
	}


	public static void dRFromEulerAngles (DMatrix3 R, double phi, double theta, 
			double psi)
	{
		double sphi,cphi,stheta,ctheta,spsi,cpsi;
		dAASSERT (R);
		sphi = dSin(phi);
		cphi = dCos(phi);
		stheta = dSin(theta);
		ctheta = dCos(theta);
		spsi = dSin(psi);
		cpsi = dCos(psi);
		R.v[_R(0,0)] = cpsi*ctheta;
		R.v[_R(0,1)] = spsi*ctheta;
		R.v[_R(0,2)] = -stheta;
		R.v[_R(0,3)] = 0.0;
		R.v[_R(1,0)] = cpsi*stheta*sphi - spsi*cphi;
		R.v[_R(1,1)] = spsi*stheta*sphi + cpsi*cphi;
		R.v[_R(1,2)] = ctheta*sphi;
		R.v[_R(1,3)] = 0.0;
		R.v[_R(2,0)] = cpsi*stheta*cphi + spsi*sphi;
		R.v[_R(2,1)] = spsi*stheta*cphi - cpsi*sphi;
		R.v[_R(2,2)] = ctheta*cphi;
		R.v[_R(2,3)] = 0.0;
	}


	public static void dRFrom2Axes (DMatrix3 R, double ax, double ay, double az,
			double bx, double by, double bz)
	{
		double l,k;
		dAASSERT (R);
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
		n.v[0] = ax;
		n.v[1] = ay;
		n.v[2] = az;
		OdeMath.dNormalize3 (n);
		OdeMath.dPlaneSpace (n,p,q);
		R.v[_R(0,0)] = p.v[0];
		R.v[_R(1,0)] = p.v[1];
		R.v[_R(2,0)] = p.v[2];
		R.v[_R(0,1)] = q.v[0];
		R.v[_R(1,1)] = q.v[1];
		R.v[_R(2,1)] = q.v[2];
		R.v[_R(0,2)] = n.v[0];
		R.v[_R(1,2)] = n.v[1];
		R.v[_R(2,2)] = n.v[2];
		R.v[_R(0,3)] = 0.0;
		R.v[_R(1,3)] = 0.0;
		R.v[_R(2,3)] = 0.0;
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


	public static void dQSetIdentity (DQuaternion q)
	{
		//dAASSERT (q);
		q.set(1, 0, 0, 0);
	}


	public static void dQFromAxisAndAngle (DQuaternion q, 
			double ax, double ay, double az, double angle)
	{
		dAASSERT (q);
		double l = ax*ax + ay*ay + az*az;
		if (l > 0.0) {
			angle *= 0.5;
			q.v[0] = dCos (angle);
			l = dSin(angle) * dRecipSqrt(l);
			q.v[1] = ax*l;
			q.v[2] = ay*l;
			q.v[3] = az*l;
		}
		else {
			q.v[0] = 1;
			q.v[1] = 0;
			q.v[2] = 0;
			q.v[3] = 0;
		}
	}


	public static void dQMultiply0 (DQuaternion qa, final DQuaternion qb, 
			final DQuaternion qc)
	{
		dAASSERT (qa, qb, qc);
		qa.v[0] = qb.v[0]*qc.v[0] - qb.v[1]*qc.v[1] - qb.v[2]*qc.v[2] - qb.v[3]*qc.v[3];
		qa.v[1] = qb.v[0]*qc.v[1] + qb.v[1]*qc.v[0] + qb.v[2]*qc.v[3] - qb.v[3]*qc.v[2];
		qa.v[2] = qb.v[0]*qc.v[2] + qb.v[2]*qc.v[0] + qb.v[3]*qc.v[1] - qb.v[1]*qc.v[3];
		qa.v[3] = qb.v[0]*qc.v[3] + qb.v[3]*qc.v[0] + qb.v[1]*qc.v[2] - qb.v[2]*qc.v[1];
	}


	public static void dQMultiply1 (DQuaternion qa, final DQuaternion qb, 
			final DQuaternionC qc)
	{
		dAASSERT (qa, qb, qc);
		qa.v[0] = qb.get0()*qc.get0() + qb.get1()*qc.get1() + qb.get2()*qc.get2() + qb.get3()*qc.get3();
		qa.v[1] = qb.get0()*qc.get1() - qb.get1()*qc.get0() - qb.get2()*qc.get3() + qb.get3()*qc.get2();
		qa.v[2] = qb.get0()*qc.get2() - qb.get2()*qc.get0() - qb.get3()*qc.get1() + qb.get1()*qc.get3();
		qa.v[3] = qb.get0()*qc.get3() - qb.get3()*qc.get0() - qb.get1()*qc.get2() + qb.get2()*qc.get1();
	}


	public static void dQMultiply2 (DQuaternion qa, final DQuaternion qb, 
			final DQuaternion qc)
	{
		dAASSERT (qa, qb, qc);
		qa.v[0] =  qb.v[0]*qc.v[0] + qb.v[1]*qc.v[1] + qb.v[2]*qc.v[2] + qb.v[3]*qc.v[3];
		qa.v[1] = -qb.v[0]*qc.v[1] + qb.v[1]*qc.v[0] - qb.v[2]*qc.v[3] + qb.v[3]*qc.v[2];
		qa.v[2] = -qb.v[0]*qc.v[2] + qb.v[2]*qc.v[0] - qb.v[3]*qc.v[1] + qb.v[1]*qc.v[3];
		qa.v[3] = -qb.v[0]*qc.v[3] + qb.v[3]*qc.v[0] - qb.v[1]*qc.v[2] + qb.v[2]*qc.v[1];
	}


	public static void dQMultiply3 (DQuaternion qa, final DQuaternion qb, 
			final DQuaternion qc)
	{
		dAASSERT (qa, qb, qc);
		qa.v[0] =  qb.v[0]*qc.v[0] - qb.v[1]*qc.v[1] - qb.v[2]*qc.v[2] - qb.v[3]*qc.v[3];
		qa.v[1] = -qb.v[0]*qc.v[1] - qb.v[1]*qc.v[0] + qb.v[2]*qc.v[3] - qb.v[3]*qc.v[2];
		qa.v[2] = -qb.v[0]*qc.v[2] - qb.v[2]*qc.v[0] + qb.v[3]*qc.v[1] - qb.v[1]*qc.v[3];
		qa.v[3] = -qb.v[0]*qc.v[3] - qb.v[3]*qc.v[0] + qb.v[1]*qc.v[2] - qb.v[2]*qc.v[1];
	}


	// dRfromQ(), dQfromR() and dDQfromW() are derived from equations in "An Introduction
	// to Physically Based Modeling: Rigid Body Simulation - 1: Unconstrained
	// Rigid Body Dynamics" by David Baraff, Robotics Institute, Carnegie Mellon
	// University, 1997.

	public static void dRfromQ (DMatrix3 R, final DQuaternionC q)
	{
		dAASSERT (q, R);
		// q = (s,vx,vy,vz)
		double q0 = q.get0();
		double q1 = q.get1();
		double q2 = q.get2();
		double q3 = q.get3();
		double qq1 = 2*q1*q1;
		double qq2 = 2*q2*q2;
		double qq3 = 2*q3*q3;
		R.v[_R(0,0)] = 1 - qq2 - qq3;
		R.v[_R(0,1)] = 2*(q1*q2 - q0*q3);
		R.v[_R(0,2)] = 2*(q1*q3 + q0*q2);
		R.v[_R(0,3)] = 0.0;
		R.v[_R(1,0)] = 2*(q1*q2 + q0*q3);
		R.v[_R(1,1)] = 1 - qq1 - qq3;
		R.v[_R(1,2)] = 2*(q2*q3 - q0*q1);
		R.v[_R(1,3)] = 0.0;
		R.v[_R(2,0)] = 2*(q1*q3 - q0*q2);
		R.v[_R(2,1)] = 2*(q2*q3 + q0*q1);
		R.v[_R(2,2)] = 1 - qq1 - qq2;
		R.v[_R(2,3)] = 0.0;
	}

	/**
	 * @deprecated
	 * @param q
	 * @param R
	 */
	public static void dQtoR(final DQuaternionC q, DMatrix3 R) {
		dRfromQ(R, q);
	}

	/**
	 * @deprecated
	 * @param q
	 * @param R
	 */
	public static void dRtoQ(final DMatrix3 R, DQuaternion q) {
		dQfromR(q, R);
	}

	public static void dQfromR (DQuaternion q, final DMatrix3C R)
	{
		dAASSERT (q, R);
		double tr,s;
		tr = R.get00() + R.get11() + R.get22();
		if (tr >= 0) {
			s = dSqrt (tr + 1);
			q.v[0] = 0.5 * s;
			s = 0.5 * dRecip(s);
			q.v[1] = (R.get21() - R.get12()) * s;
			q.v[2] = (R.get02() - R.get20()) * s;
			q.v[3] = (R.get10() - R.get01()) * s;
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
		q.v[1] = 0.5 * s;
		s = 0.5 * dRecip(s);
		q.v[2] = (R.get01() + R.get10()) * s;
		q.v[3] = (R.get20() + R.get02()) * s;
		q.v[0] = (R.get21() - R.get12()) * s;
		return;
	}

	private static void DQFR_case_1(DQuaternion q, final DMatrix3C R) {
		//case_1:
		double s = dSqrt((R.get11() - (R.get22() + R.get00())) + 1.);
		q.v[2] = 0.5 * s;
		s = 0.5 * dRecip(s);
		q.v[3] = (R.get12() + R.get21()) * s;
		q.v[1] = (R.get01() + R.get10()) * s;
		q.v[0] = (R.get02() - R.get20()) * s;
		return;
	}

	private static void DQFR_case_2(DQuaternion q, final DMatrix3C R) {
		//case_2:
		double s = dSqrt((R.get22() - (R.get00() + R.get11())) + 1.);
		q.v[3] = 0.5 * s;
		s = 0.5 * dRecip(s);
		q.v[1] = (R.get20() + R.get02()) * s;
		q.v[2] = (R.get12() + R.get21()) * s;
		q.v[0] = (R.get10() - R.get01()) * s;
		return;
	}


	/**
	 * @param w
	 * @param q
	 * @param dq
	 * @deprecated
	 */
	public static void dWtoDQ(final DVector3 w, final DQuaternion q, DQuaternion dq)
	{
		dDQfromW(dq, w, q);
	}

	//TODO void dDQfromW (double dq[4], final dVector3 w, final dQuaternion q)
	public static void dDQfromW (DQuaternion dq, final DVector3 w, final DQuaternion q)
	{
		dAASSERT (w, q, dq);
		dq.set(0, 0.5*(- w.v[0]*q.v[1] - w.v[1]*q.v[2] - w.v[2]*q.v[3]) );
		dq.set(1, 0.5*(  w.v[0]*q.v[0] + w.v[1]*q.v[3] - w.v[2]*q.v[2]) );
		dq.set(2, 0.5*(- w.v[0]*q.v[3] + w.v[1]*q.v[0] + w.v[2]*q.v[1]) );
		dq.set(3, 0.5*(  w.v[0]*q.v[2] - w.v[1]*q.v[1] + w.v[2]*q.v[0]) );
	}
}
