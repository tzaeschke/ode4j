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

import java.util.Arrays;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.internal.Matrix;

/**
 * Matrix math functions. Ported from matrix.h in C-interface.
 *
 * @author Tilmann Zaeschke
 */
public class DMatrix extends DMisc {
    
    // from matrix.h
    
    /** 
     * Set a vector/matrix to all zeros. 
     * @param a a
     */
    public static void dSetZero (double[] a) {
        Arrays.fill(a,0);
    }
    /** 
     * Set a vector/matrix to all zeros. 
     * @param a a
     * @param aOfs offset 
     * @param aLen length
     */
    public static void dSetZero (double[] a, int aOfs, int aLen) {
        //Arrays.fill(a, aOfs, aOfs+aLen, 0);
        for (int i = aOfs; i < aOfs+aLen; i++) {
            a[i] = 0;
        }
    }
    /** 
     * Set a vector/matrix to a specific value. 
     * @param a a
     * @param value value 
     */
    public static void dSetValue (DVector3 a, double value) {
        a.set(value, value, value);
    }

    public static void dSetValue (int[] a, int pos, int len, int value) {
    	for (int i = pos; i < len+pos; i++) {
    		a[i] = value;
    	}
    }


//    /** 
//     * Get the dot product of two n*1 vectors. if n <= 0 then
//     * zero will be returned (in which case a and b need not be valid).
//     */
//    public static double dDot (DVector3C a, DVector3C b, int n) {
//        return a.dot(b);
//    }


//    /** 
//     * Get the dot products of (a0,b), (a1,b), etc and return them in outsum.
//     * all vectors are n*1. if n <= 0 then zeroes will be returned (in which case
//     * the input vectors need not be valid). this function is somewhat faster
//     * than calling dDot() for all of the combinations separately.
//     */
//
//    /* NOT INCLUDED in the library for now.
//    void static dMultidot2 (const dReal *a0, const dReal *a1,
//             const dReal *b, dReal *outsum, int n);
//    */


    /** 
     * Matrix multiplication. all matrices are stored in standard row format.
     * the digit refers to the argument that is transposed:
     *   0:   A = B  * C   (sizes: A:p*r B:p*q C:q*r)
     *   1:   A = B' * C   (sizes: A:p*r B:q*p C:q*r)
     *   2:   A = B  * C'  (sizes: A:p*r B:p*q C:r*q)
     * case 1,2 are equivalent to saying that the operation is A=B*C but
     * B or C are stored in standard column format.
     * @param a a
     * @param b b
     * @param C C
     */
    public static void dMultiply0 (DVector3 a, DVector3C b, DMatrix3C C) {
        Matrix.dMultiply0(a, b, C);
    }
    /** 
     * @param a a
     * @param B B
     * @param c c
     * @see DMatrix#dMultiply0(DVector3, DVector3C, DMatrix3C)
     */
    public static void dMultiply0 (DVector3 a, DMatrix3C B, DVector3C c) {
        Matrix.dMultiply0(a, B, c);
    }
    /** 
     * @param A A
     * @param B B
     * @param C C
     * @see DMatrix#dMultiply0(DVector3, DVector3C, DMatrix3C)
     */
    public static void dMultiply0 (DMatrix3 A, DMatrix3C B, DMatrix3C C) {
        Matrix.dMultiply0(A, B, C);
    }
    /** 
     * @param A A
     * @param B B
     * @param C C
     * @param p p
     * @param q q
     * @param r r
     * @see DMatrix#dMultiply0(DVector3, DVector3C, DMatrix3C)
     */
    public static void dMultiply0 (double[] A, final double[] B, final double[] C, int p, int q, int r) {
        Matrix.dMultiply0(A, B, C, p, q, r);
    }
    /** 
     * @param a a
     * @param B B
     * @param c c
     * @see DMatrix#dMultiply0(DVector3, DVector3C, DMatrix3C)
     */
    public static void dMultiply1 (DVector3 a, DMatrix3C B, DVector3C c) {
        Matrix.dMultiply1(a, B, c);
    }
    /** 
     * @param A A
     * @param B B
     * @param C C
     * @see DMatrix#dMultiply0(DVector3, DVector3C, DMatrix3C)
     */
    public static void dMultiply1 (DMatrix3 A, DMatrix3C B, DMatrix3C C) {
        Matrix.dMultiply1(A, B, C);
    }
    /** 
     * @param A A
     * @param B B
     * @param C C
     * @param p p
     * @param q q
     * @param r r
     * @see DMatrix#dMultiply0(DVector3, DVector3C, DMatrix3C)
     */
    public static void dMultiply1 (double[] A, final double[] B, final double[] C, int p, int q, int r) {
        Matrix.dMultiply1(A, B, C, p, q, r);
    }
    /** 
     * @param a a
     * @param B B
     * @param c c
     * @see DMatrix#dMultiply0(DVector3, DVector3C, DMatrix3C)
     */
    public static void dMultiply2 (DVector3 a, DMatrix3C B, DVector3C c) {
        Matrix.dMultiply2(a, B, c);
    }
    /** 
     * @param A A
     * @param B B
     * @param C C
     * @see DMatrix#dMultiply0(DVector3, DVector3C, DMatrix3C)
     */
    public static void dMultiply2 (DMatrix3 A, DMatrix3C B, DMatrix3C C) {
        Matrix.dMultiply2(A, B, C);
    }
    /** 
     * @param A A
     * @param B B
     * @param C C
     * @param p p
     * @param q q
     * @param r r
     * @see DMatrix#dMultiply0(DVector3, DVector3C, DMatrix3C)
     */
    public static void dMultiply2 (double[] A, final double[] B, final double[] C, int p, int q, int r) {
        Matrix.dMultiply2(A, B, C, p, q, r);
    }


    /**
     * Do an in-place cholesky decomposition on the lower triangle of the n*n
     * symmetric matrix A (which is stored by rows). the resulting lower triangle
     * will be such that L*L'=A. return 'true' on success and 'false' on failure (on failure
     * the matrix is not positive definite).
     * @param A matrix
     * @return 'false' if it failed
     */
    public static boolean dFactorCholesky (DMatrix3 A) {
        return Matrix.dFactorCholesky(A);
    }
    /**
     * Do an in-place cholesky decomposition on the lower triangle of the n*n
     * symmetric matrix A (which is stored by rows). the resulting lower triangle
     * will be such that L*L'=A. return 'true' on success and 'false' on failure (on failure
     * the matrix is not positive definite).
     * @param A Matrix
     * @param n n
     * @return false if it failed
     */
    public static boolean dFactorCholesky (double[] A, int n) {
        return Matrix.dFactorCholesky(A, n);
    }


    /**
     * Solve for x: L*L'*x = b, and put the result back into x.
     * L is size n*n, b is size n*1. only the lower triangle of L is considered.
     * @param L Matrix
     * @param x vector
     */
    public static void dSolveCholesky (DMatrix3C L, DVector3 x) {
        Matrix.dSolveCholesky(L, x);
    }
    /**
     * Solve for x: L*L'*x = b, and put the result back into x.
     * L is size n*n, b is size n*1. only the lower triangle of L is considered.
     * @param L matrix
     * @param x vector
     * @param n n
     */
    public static void dSolveCholesky (double[] L, double[] x, int n) {
        Matrix.dSolveCholesky(L, x, n);
    }


    /**
     * Compute the inverse of the n*n positive definite matrix A and put it in
     * Ainv. this is not especially fast. this returns 'true' on success (A was
     * positive definite) or 'false' on failure (not PD).
     * @param A A
     * @param Ainv result  
     * @return 'false' on failure
     */
    public static boolean dInvertPDMatrix (DMatrix3C A, DMatrix3 Ainv) {
        return Matrix.dInvertPDMatrix(A, Ainv);
    }
    public static boolean dInvertPDMatrix (double[] A, double[] Ainv, int n) {
        return Matrix.dInvertPDMatrix(A, Ainv, n);
    }


    /**
     * Check whether an n*n matrix A is positive definite, return 1/0 (yes/no).
     * positive definite means that x'*A*x &gt; 0 for any x. this performs a
     * cholesky decomposition of A. if the decomposition fails then the matrix
     * is not positive definite. A is stored by rows. A is not altered.
     * @param A matrix
     * @return false on failure
     */
    public static boolean dIsPositiveDefinite (DMatrix3C A) {
        return Matrix.dIsPositiveDefinite(A);
    }
    public static boolean dIsPositiveDefinite (double[] A, int n) {
        return Matrix.dIsPositiveDefinite(A, n);
    }


    /**
     * Factorize a matrix A into L*D*L', where L is lower triangular with ones on
     * the diagonal, and D is diagonal.
     * A is an n*n matrix stored by rows, with a leading dimension of n rounded
     * up to 4. L is written into the strict lower triangle of A (the ones are not
     * written) and the reciprocal of the diagonal elements of D are written into
     * d.
     * @param A A
     * @param d d
     * @param n n
     * @param nskip nskip 
     */
    public static void dFactorLDLT (double[] A, double[] d, int n, int nskip) {
        Matrix.dFactorLDLT(A, d, n, nskip);
    }


//    /**
//     * Solve L*x=b, where L is n*n lower triangular with ones on the diagonal,
//     * and x,b are n*1. b is overwritten with x.
//     * the leading dimension of L is `nskip'.
//     */
//    public static void dSolveL1 (const dReal *L, dReal *b, int n, int nskip) {
//        Matrix.dSolveL1;
//    }


//    /**
//     * Solve L'*x=b, where L is n*n lower triangular with ones on the diagonal,
//     * and x,b are n*1. b is overwritten with x.
//     * the leading dimension of L is `nskip'.
//     */
//    public void dSolveL1T (const dReal *L, dReal *b, int n, int nskip);


    /* in matlab syntax: a(1:n) = a(1:n) .* d(1:n)
     */

    /**
     * In matlab syntax: a(1:n) = a(1:n) .* d(1:n)
     * @param a a
     * @param d d
     */
    // ODE_API
    public static void dScaleVector (DVector3 a, DVector3C d) {
        a.scale(d);
    }

    /**
     * The function is an alias for @c dScaleVector.
     *  It has been deprecated because of a wrong naming schema used.
     * @param a a
     * @param d d
     */
    @Deprecated // deprecated in ODE
    public static void dVectorScale (DVector3 a, DVector3C d) {
        a.scale(d);
    }


    /**
     * Given `L', a n*n lower triangular matrix with ones on the diagonal,
     * and `d', a n*1 vector of the reciprocal diagonal elements of an n*n matrix
     * D, solve L*D*L'*x=b where x,b are n*1. x overwrites b.
     * the leading dimension of L is `nskip'.
     * @param L L
     * @param d d
     * @param b b
     * @param n n
     * @param nskip nskip 
     */
    public static void dSolveLDLT (double[] L, double[] d, double[] b, int n, int nskip) {
        Matrix.dSolveLDLT(L, d, b, n, nskip);
    }


    /**
     * Given an L*D*L' factorization of an n*n matrix A, return the updated
     * factorization L2*D2*L2' of A plus the following "top left" matrix:
     *
     *    [ b a' ]     &lt;-- b is a[0]
     *    [ a 0  ]     &lt;-- a is a[1..n-1]
     * 
     * <ul>
     * <li> L has size n*n, its leading dimension is nskip. L is lower triangular
     *     with ones on the diagonal. only the lower triangle of L is referenced.</li>
     * <li> d has size n. d contains the reciprocal diagonal elements of D.</li>
     * <li> a has size n.</li>
     * </ul>
     * the result is written into L, except that the left column of L and d[0]
     * are not actually modified. see ldltaddTL.m for further comments. 
     * @param L L
     * @param d d
     * @param a a
     * @param n n
     * @param nskip nskip 
     */
    public static void dLDLTAddTL (double[] L, double[] d, double[] a, int n, int nskip) {
        Matrix.dLDLTAddTL(L, d, a, n, nskip);
    }


    /**
     * Given an L*D*L' factorization of a permuted matrix A, produce a new
     * factorization for row and column `r' removed.
     * <ul>
     * <li> A has size n1*n1, its leading dimension in nskip. A is symmetric and
     *     positive definite. only the lower triangle of A is referenced.
     *     A itself may actually be an array of row pointers. </li>
     * <li> L has size n2*n2, its leading dimension in nskip. L is lower triangular
     *     with ones on the diagonal. only the lower triangle of L is referenced.</li>
     * <li> d has size n2. d contains the reciprocal diagonal elements of D. </li>
     * <li> p is a permutation vector. it contains n2 indexes into A. each index
     *     must be in the range 0..n1-1. </li>
     * <li> r is the row/column of L to remove. </li>
     * </ul>
     * the new L will be written within the old L, i.e. will have the same leading
     * dimension. the last row and column of L, and the last element of d, are
     * undefined on exit.
     *
     * a fast O(n^2) algorithm is used. see ldltremove.m for further comments.
     * @param A A
     * @param p p
     * @param L L
     * @param d d
     * @param n1 n1
     * @param n2 n2
     * @param r r
     * @param nskip nskip 
     */
    public static void dLDLTRemove (double[] A, int[] p, double[] L, double[] d,
              int n1, int n2, int r, int nskip) {
        Matrix.dLDLTRemove(A, p, L, d, n1, n2, r, nskip);
    }


    /**
     * Given an n*n matrix A (with leading dimension nskip), remove the r'th row
     * and column by moving elements. the new matrix will have the same leading
     * dimension. the last row and column of A are untouched on exit.
     * @param A A
     * @param n n
     * @param nskip nskip 
     * @param r r
     */
    public static void dRemoveRowCol (double[] A, int n, int nskip, int r) {
        Matrix.dRemoveRowCol(A, n, nskip, r);
    }

    protected DMatrix() {}
}
