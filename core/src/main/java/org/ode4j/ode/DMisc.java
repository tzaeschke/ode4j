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
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.internal.Common;
import org.ode4j.ode.internal.Misc;

/**
 * Port of misc.h from ODE API.
 *
 * @author Tilmann Zaeschke
 */
public class DMisc extends Common {
    /** 
     * @return 'true' if the random number generator is working. 
     */
    public static boolean dTestRand() {
        return Misc.dTestRand();
    }

    /**
     * @return next 32 bit random number. this uses a not-very-random linear
     * congruential method.
     */
    public static long dRand() {
        return Misc.dRand();
    }

    /** 
     * Get and set the current random number seed. 
     * @return seed
     */
    public static long dRandGetSeed() {
        return Misc.dRandGetSeed();
    }
    /**
     * @param s seed
     */
    public static void dRandSetSeed (long s) {
        Misc.dRandSetSeed(s);
    }

    /**
     * @param n max
     * @return a random integer between 0..n-1. the distribution will get worse
     * as n approaches 2^32.
     */
    public static int dRandInt (int n) {
        return Misc.dRandInt(n);
    }

    /**
     * @return a random real number between 0..1 
     */
    public static double dRandReal() {
        return Misc.dRandReal();
    }

    /**
     * Print out a matrix. 
     * @param A Matrix
     * @return String
     */
    public static String dPrintMatrix(DMatrix3C A) {
        return A.toString();
    }
//        public static void dPrintMatrix (const dReal *A, int n, int m, char *fmt, FILE *f);

    /**
     * Make a random vector with entries between +/- range. A has n elements. 
     * @param A Vector
     * @param range range
     */
    public static void dMakeRandomVector (DVector3 A, double range) {
        Misc.dMakeRandomVector(A, range);
    }
    /**
     * Make a random vector with entries between +/- range. A has n elements. 
     * @param A Quaternion
     * @param range range
     */
    public static void dMakeRandomVector (DQuaternion A, double range) {
        Misc.dMakeRandomVector(A, range);
    }
    /**
     * Make a random vector with entries between +/- range. A has n elements. 
     * @param A vector
     * @param n n
     * @param range range 
     */
    public static void dMakeRandomVector (double[] A, int n, double range) {
        Misc.dMakeRandomVector(A, n, range);
    }

    /**
     * Make a random matrix with entries between +/- range. A has size n*m. 
     * @param A Matrix
     * @param range range
     */
    public static void dMakeRandomMatrix (DMatrix3 A, double range) {
        Misc.dMakeRandomMatrix(A, range);
    }
    /**
     * Make a random matrix with entries between +/- range. A has size n*m. 
     * @param A Matrix
     * @param n n
     * @param m m
     * @param range range 
     */
    public static void dMakeRandomMatrix (double[] A, int n, int m, double range) {
        Misc.dMakeRandomMatrix(A, n, m, range);
    }

    /**
     * Clear the upper triangle of a square matrix. 
     * @param A Matrix
     */
    public static void dClearUpperTriangle (DMatrix3 A) {
        Misc.dClearUpperTriangle(A);
    }
    /**
     * Clear the upper triangle of a square matrix. 
     * @param A Matrix
     * @param n n
     */
    public static void dClearUpperTriangle (double[] A, int n) {
        Misc.dClearUpperTriangle(A, n);
    }

    /**
     * @param A Matrix A
     * @param B Matrix B
     * @return the maximum element difference between the two n*m matrices 
     */
    public static double dMaxDifference (DMatrix3C A, DMatrix3C B) {
        return Misc.dMaxDifference(A, B);
    }
    /**
     * @param A Matrix A 
     * @param B Matrix B
     * @return the maximum element difference between the two n*m matrices 
     */
    public static double dMaxDifference (DVector3C A, DVector3C B) {
        return Misc.dMaxDifference(A, B);
    }
    /**
     * @param A Quaternion A
     * @param B Quaternion B
     * @param n n
     * @param m m
     * @return the maximum element difference between the two n*m matrices 
     */
    public static double dMaxDifference (DQuaternionC A, DQuaternionC B, int n, int m) {
        return Misc.dMaxDifference(A, B, n, m);
    }
    /**
     * @param A Matrix A
     * @param B Matrix B
     * @param n n
     * @param m m
     * @return the maximum element difference between the two n*m matrices 
     */
    public static double dMaxDifference (double[] A, double[] B, int n, int m) {
        return Misc.dMaxDifference(A, B, n, m);
    }

    /**
     * @param A Matrix A
     * @param B Matrix B
     * @param n n
     * @return the maximum element difference between the lower triangle of two
     * n*n matrices 
     */
    public static double dMaxDifferenceLowerTriangle (double[] A, double[] B, int n) {
        return Misc.dMaxDifferenceLowerTriangle(A, B, n);
    }

    protected DMisc() {}
}
