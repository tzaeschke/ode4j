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
package org.ode4j.cpp.internal;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.Rotation;


@SuppressWarnings("deprecation")
public abstract class ApiCppMathRotation extends ApiCppMathMisc {

	//ODE_API 
	public static void dRSetIdentity (DMatrix3 R) {
		R.setIdentity();
	}

	//ODE_API 
	public static void dRFromAxisAndAngle (DMatrix3 R, double ax, double ay, double az,
			double angle) {
		OdeMath.dRFromAxisAndAngle(R, ax, ay, az, angle);
	}

	//ODE_API 
	public static void dRFromEulerAngles (DMatrix3 R, double phi, double theta, double psi) {
		OdeMath.dRFromEulerAngles(R, phi, theta, psi);
	}

	//ODE_API 
	public static void dRFrom2Axes (DMatrix3 R, double ax, double ay, double az,
			double bx, double by, double bz) {
		Rotation.dRFrom2Axes(R, ax, ay, az, bx, by, bz);
	}

	//ODE_API 
	public static void dRFromZAxis (DMatrix3 R, double ax, double ay, double az) {
	    Rotation.dRFromZAxis(R, ax, ay, az);
	}

	//ODE_API 
	public static void dQSetIdentity (DQuaternion q) {
		q.setIdentity();
	}

	//ODE_API 
	public static void dQFromAxisAndAngle (DQuaternion q, double ax, double ay, double az,
			double angle) {
		OdeMath.dQFromAxisAndAngle(q, ax, ay, az, angle);
	}

	/** Quaternion multiplication, analogous to the matrix multiplication routines.
	 * qa = rotate by qc, then qb 
	 * @param qa qa
	 * @param qb qb
	 * @param qc qc
	 */
	//ODE_API 
	public static void dQMultiply0 (DQuaternion qa, final DQuaternion qb, final DQuaternion qc) {
		OdeMath.dQMultiply0(qa, qb, qc);
	}
	/** 
	 * Quaternion multiplication, analogous to the matrix multiplication routines.
	 * qa = rotate by qc, then by inverse of qb 
	 * @param qa qa
	 * @param qb qb
	 * @param qc qc
	 */
	//ODE_API 
	public static void dQMultiply1 (DQuaternion qa, final DQuaternion qb, final DQuaternion qc) {
		OdeMath.dQMultiply1(qa, qb, qc);
	}
	/** Quaternion multiplication, analogous to the matrix multiplication routines.
	 * qa = rotate by inverse of qc, then by qb 
	 * @param qa qa
	 * @param qb qb
	 * @param qc qc
	 */
	//ODE_API 
	public static void dQMultiply2 (DQuaternion qa, final DQuaternion qb, final DQuaternion qc) {
		OdeMath.dQMultiply2(qa, qb, qc);
	}
	/** qa = rotate by inverse of qc, then by inverse of qb 
	 * @param qa qa
	 * @param qb qb
	 * @param qc qc 
	 */
	//ODE_API 
	public static void dQMultiply3 (DQuaternion qa, final DQuaternion qb, final DQuaternion qc) {
		OdeMath.dQMultiply3(qa, qb, qc);
	}

	//ODE_API 
	public static void dRfromQ (DMatrix3 R, final DQuaternion q) {
		OdeMath.dRfromQ(R, q);
	}
	/** 
	 * @param q q
	 * @param R R
	 * @deprecated 
	 */
	@Deprecated
    public static void dQtoR (final DQuaternion q, DMatrix3 R) {
		OdeMath.dRfromQ(R, q);
	}
	//ODE_API 
	public static void dQfromR (DQuaternion q, final DMatrix3 R) {
		OdeMath.dQfromR(q, R);
	}
	//ODE_API 
	// void dDQfromW (double dq[4], final dVector3 w, final dQuaternion q);
	public static void dDQfromW (DQuaternion dq, final DVector3 w, final DQuaternion q) {
		Rotation.dDQfromW(dq, w, q);
	}

	protected ApiCppMathRotation() {}
}