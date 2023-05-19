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
package org.ode4j.ode;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.internal.Rotation;

/**
 * Port of rotation.h in ode API.
 *
 * @author Tilmann Zaeschke
 */
public class DRotation extends DMatrix {

    public static void dRSetIdentity (DMatrix3 R) {
        R.setIdentity();
    }

    public static void dRFromAxisAndAngle (DMatrix3 R, double ax, double ay, double az,
            double angle) {
        Rotation.dRFromAxisAndAngle(R, ax, ay, az, angle);
    }
    public static void dRFromAxisAndAngle (DMatrix3 R, DVector3C a, double angle) {
        Rotation.dRFromAxisAndAngle(R, a, angle);
    }

    public static void dRFromEulerAngles (DMatrix3 R, double phi, double theta, 
            double psi) {
        Rotation.dRFromEulerAngles(R, phi, theta, psi);
    }

//    public static void dRFrom2Axes (dMatrix3 R, dReal ax, dReal ay, dReal az,
//              dReal bx, dReal by, dReal bz);
//
//    public static void dRFromZAxis (DMatrix3 R, double ax, double ay, double az) {
//        Rotation.dRFromZAxis(R, ax, ay, az);
//    }
    public static void dRFromZAxis (DMatrix3 R, DVector3C a) {
        Rotation.dRFromZAxis(R, a);
    }

//    public static void dQSetIdentity (dQuaternion q);
//
    public static void dQFromAxisAndAngle (DQuaternion q, double ax, double ay,
            double az, double angle) {
        Rotation.dQFromAxisAndAngle(q, ax, ay, az, angle);
    }
    public static void dQFromAxisAndAngle (DQuaternion q, DVector3C axyz, double angle) {
        Rotation.dQFromAxisAndAngle(q, axyz, angle);
    }

    /**
     * Quaternion multiplication, analogous to the matrix multiplication routines.
     * qa = rotate by qc, then qb
     * @param qa qa
     * @param qb qb
     * @param qc qc
     */
    public static void dQMultiply0 (DQuaternion qa, DQuaternionC qb, DQuaternionC qc) {
        Rotation.dQMultiply0(qa, qb, qc);
    }
    /**
     * Quaternion multiplication, analogous to the matrix multiplication routines. 
     * qa = rotate by qc, then by inverse of qb 
     * @param qa qa
     * @param qb qb
     * @param qc qc
     */
    public static void dQMultiply1 (DQuaternion qa, DQuaternionC qb, DQuaternionC qc) {
        Rotation.dQMultiply1(qa, qb, qc);
    }
    /**
     * Quaternion multiplication, analogous to the matrix multiplication routines. 
     * qa = rotate by inverse of qc, then by qb 
     * @param qa qa
     * @param qb qb
     * @param qc qc
     */
    public static void dQMultiply2 (DQuaternion qa, DQuaternionC qb, DQuaternionC qc) {
        Rotation.dQMultiply2(qa, qb, qc);
    }
    /**
     * Quaternion multiplication, analogous to the matrix multiplication routines. 
     * qa = rotate by inverse of qc, then by inverse of qb 
     * @param qa qa
     * @param qb qb
     * @param qc qc
     */
    public static void dQMultiply3 (DQuaternion qa, DQuaternionC qb, DQuaternionC qc) {
        Rotation.dQMultiply3(qa, qb, qc);
    }

    public static void dRfromQ (DMatrix3 R, DQuaternionC q) {
        Rotation.dRfromQ(R, q);
    }
    public static void dQfromR (DQuaternion q, DMatrix3C R) {
        Rotation.dQfromR(q, R);
    }
    
//    public static void dDQfromW (dReal dq[4], const dVector3 w, const dQuaternion q);

    protected DRotation() {}
}
