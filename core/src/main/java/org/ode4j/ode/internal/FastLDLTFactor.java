/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2021 Tilmann Zaeschke     *
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
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal;

import static org.ode4j.ode.internal.Common.*;

// Code style improvements and optimizations by Oleh Derevenko ????-2017

/**
 * Implementation of FastDLTImpl
 */
class FastLDLTFactor {
//#ifndef _ODE_FASTLDLT_IMPL_H_
//#define _ODE_FASTLDLT_IMPL_H_

//    static void dxSolveL1_2 (const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip);
//    template<unsigned int d_stride>
//    void dxScaleAndFactorizeL1_2(dReal *ARow, dReal *d, unsigned rowIndex, unsigned rowSkip);
//    template<unsigned int d_stride>
//    inline void dxScaleAndFactorizeFirstL1Row_2(dReal *ARow, dReal *d, unsigned rowSkip);
//
//    static void dxSolveL1_1 (const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip);
//    template<unsigned int d_stride>
//    void dxScaleAndFactorizeL1_1(dReal *ARow, dReal *d, unsigned rowIndex);
//    template<unsigned int d_stride>
//    inline void dxScaleAndFactorizeFirstL1Row_1(dReal *ARow, dReal *d);


    //    template<unsigned int d_stride>
    public static void factorMatrixAsLDLT(double[] A, double[] d, int rowCount, int rowSkip, int d_stride) {
        if (rowCount < 1) return;

        final int lastRowIndex = rowCount - 1;

        int ARow_pos = 0;//A;
        int blockStartRow = 0;
        /* compute blocks of 2 rows */
        boolean subsequentPass = false;
        for (; blockStartRow < lastRowIndex; subsequentPass = true, ARow_pos += 2 * rowSkip, blockStartRow += 2) {
            if (subsequentPass) {
                /* solve L*(D*l)=a, l is scaled elements in 2 x i block at A(i,0) */
                solveL1Stripe_2(A, A, ARow_pos, blockStartRow, rowSkip);
                scaleAndFactorizeL1Stripe_2(A, ARow_pos, d, blockStartRow, rowSkip, d_stride);
            } else {
                scaleAndFactorizeL1FirstRowStripe_2(A, ARow_pos, d, rowSkip, d_stride);
            }
            /* done factorizing 2 x 2 block */
        }

        /* compute the (less than 2) rows at the bottom */
        if (!subsequentPass || blockStartRow == lastRowIndex) {
            if (subsequentPass) {
                solveStripeL1_1(A, A, ARow_pos, blockStartRow, rowSkip);
                scaleAndFactorizeL1Stripe_1(A, ARow_pos, d, blockStartRow, d_stride);
            } else {
                scaleAndFactorizeL1FirstRowStripe_1(A, ARow_pos, d, d_stride);
            }
            /* done factorizing 1 x 1 block */
        }
    }

    /* solve L*X=B, with B containing 2 right hand sides.
     * L is an n*n lower triangular matrix with ones on the diagonal.
     * L is stored by rows and its leading dimension is rowSkip.
     * B is an n*2 matrix that contains the right hand sides.
     * B is stored by columns and its leading dimension is also rowSkip.
     * B is overwritten with X.
     * this processes blocks of 2*2.
     * if this is in the factorizer source file, n must be a multiple of 2.
     */
    static void solveL1Stripe_2(final double[] L, double[] B, int bPos, int rowCount, int rowSkip) {
        dIASSERT(rowCount != 0);
        dIASSERT(rowCount % 2 == 0);

        /* compute all 2 x 2 blocks of X */
        int blockStartRow = 0;
        for (boolean exitLoop = false, subsequentPass = false; !exitLoop; subsequentPass = true, exitLoop = (blockStartRow += 2) == rowCount) {
            int ptrLElement = 0;// dReal *ptrLElement;
            int ptrBElement = 0;//            dReal *ptrBElement;

            /* declare variables - Z matrix */
            double Z11, Z12, Z21, Z22;

            /* compute all 2 x 2 block of X, from rows i..i+2-1 */
            if (subsequentPass) {
                // ptrLElement = L + blockStartRow * rowSkip;
                ptrLElement = blockStartRow * rowSkip;
                ptrBElement = bPos;//B;

                /* set Z matrix to 0 */
                Z11 = 0;
                Z12 = 0;
                Z21 = 0;
                Z22 = 0;

                /* the inner loop that computes outer products and adds them to Z */
                // The iteration starts with even number and decreases it by 2. So, it must end in zero
                for (int columnCounter = blockStartRow; ; ) {
                    /* declare p and q vectors, etc */
                    double p1, q1, p2, q2;

                    /* compute outer product and add it to the Z matrix */
                    p1 = L[ptrLElement + 0];
                    q1 = B[ptrBElement + 0];
                    Z11 += p1 * q1;
                    q2 = B[ptrBElement + rowSkip];
                    Z12 += p1 * q2;
                    p2 = L[ptrLElement + rowSkip];
                    Z21 += p2 * q1;
                    Z22 += p2 * q2;

                    /* compute outer product and add it to the Z matrix */
                    p1 = L[ptrLElement + 1];
                    q1 = B[ptrBElement + 1];
                    Z11 += p1 * q1;
                    q2 = B[ptrBElement + 1 + rowSkip];
                    Z12 += p1 * q2;
                    p2 = L[ptrLElement + 1 + rowSkip];
                    Z21 += p2 * q1;
                    Z22 += p2 * q2;

                    if (columnCounter > 6) {
                        columnCounter -= 6;

                        /* advance pointers */
                        ptrLElement += 6;
                        ptrBElement += 6;

                        /* compute outer product and add it to the Z matrix */
                        p1 = L[ptrLElement + -4];
                        q1 = B[ptrBElement + -4];
                        Z11 += p1 * q1;
                        q2 = B[ptrBElement + -4 + rowSkip];
                        Z12 += p1 * q2;
                        p2 = L[ptrLElement + -4 + rowSkip];
                        Z21 += p2 * q1;
                        Z22 += p2 * q2;

                        /* compute outer product and add it to the Z matrix */
                        p1 = L[ptrLElement + -3];
                        q1 = B[ptrBElement + -3];
                        Z11 += p1 * q1;
                        q2 = B[ptrBElement + -3 + rowSkip];
                        Z12 += p1 * q2;
                        p2 = L[ptrLElement + -3 + rowSkip];
                        Z21 += p2 * q1;
                        Z22 += p2 * q2;

                        /* compute outer product and add it to the Z matrix */
                        p1 = L[ptrLElement + -2];
                        q1 = B[ptrBElement + -2];
                        Z11 += p1 * q1;
                        q2 = B[ptrBElement + -2 + rowSkip];
                        Z12 += p1 * q2;
                        p2 = L[ptrLElement + -2 + rowSkip];
                        Z21 += p2 * q1;
                        Z22 += p2 * q2;

                        /* compute outer product and add it to the Z matrix */
                        p1 = L[ptrLElement + -1];
                        q1 = B[ptrBElement + -1];
                        Z11 += p1 * q1;
                        q2 = B[ptrBElement + -1 + rowSkip];
                        Z12 += p1 * q2;
                        p2 = L[ptrLElement + -1 + rowSkip];
                        Z21 += p2 * q1;
                        Z22 += p2 * q2;
                    } else {
                        /* advance pointers */
                        ptrLElement += 2;
                        ptrBElement += 2;

                        if ((columnCounter -= 2) == 0) {
                            break;
                        }
                    }
                    /* end of inner loop */
                }
            } else {
                ptrLElement = 0;//L/* + blockStartRow * rowSkip*/;
                dIASSERT(blockStartRow == 0);
                ptrBElement = bPos;//B;

                /* set Z matrix to 0 */
                Z11 = 0;
                Z12 = 0;
                Z21 = 0;
                Z22 = 0;
            }

            /* finish computing the X(i) block */

            double Y11 = B[ptrBElement + 0] - Z11;
            double Y12 = B[ptrBElement + rowSkip] - Z12;

            double p2 = L[ptrLElement + rowSkip];

            B[ptrBElement + 0] = Y11;
            B[ptrBElement + rowSkip] = Y12;

            double Y21 = B[ptrBElement + 1] - Z21 - p2 * Y11;
            double Y22 = B[ptrBElement + 1 + rowSkip] - Z22 - p2 * Y12;

            B[ptrBElement + 1] = Y21;
            B[ptrBElement + 1 + rowSkip] = Y22;
            /* end of outer loop */
        }
    }

    //template<unsigned int d_stride>
    private static void scaleAndFactorizeL1Stripe_2(double[] ARow, int aPos, double[] d, int factorizationRow, int rowSkip, int d_stride) {
        dIASSERT(factorizationRow != 0);
        dIASSERT(factorizationRow % 2 == 0);

        int ptrAElement = aPos;//ARow;
        int ptrDElement = 0;//d;

        /* scale the elements in a 2 x i block at A(i,0), and also */
        /* compute Z = the outer product matrix that we'll need. */
        double Z11 = 0, Z21 = 0, Z22 = 0;

        for (int columnCounter = factorizationRow; ; ) {
            double p1, q1, p2, q2, dd;

            p1 = ARow[ptrAElement + 0];
            p2 = ARow[ptrAElement + rowSkip];
            dd = d[ptrDElement + 0 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ARow[ptrAElement + 0] = q1;
            ARow[ptrAElement + rowSkip] = q2;
            Z11 += p1 * q1;
            Z21 += p2 * q1;
            Z22 += p2 * q2;

            p1 = ARow[ptrAElement + 1];
            p2 = ARow[ptrAElement + 1 + rowSkip];
            dd = d[ptrDElement + 1 * d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ARow[ptrAElement + 1] = q1;
            ARow[ptrAElement + 1 + rowSkip] = q2;
            Z11 += p1 * q1;
            Z21 += p2 * q1;
            Z22 += p2 * q2;

            if (columnCounter > 6) {
                columnCounter -= 6;

                ptrAElement += 6;
                ptrDElement += 6 * d_stride;

                p1 = ARow[ptrAElement + -4];
                p2 = ARow[ptrAElement + -4 + rowSkip];
                dd = d[ptrDElement + -4 * d_stride];
                q1 = p1 * dd;
                q2 = p2 * dd;
                ARow[ptrAElement + -4] = q1;
                ARow[ptrAElement + -4 + rowSkip] = q2;
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z22 += p2 * q2;

                p1 = ARow[ptrAElement + -3];
                p2 = ARow[ptrAElement + -3 + rowSkip];
                dd = d[ptrDElement + -3 * d_stride];
                q1 = p1 * dd;
                q2 = p2 * dd;
                ARow[ptrAElement + -3] = q1;
                ARow[ptrAElement + -3 + rowSkip] = q2;
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z22 += p2 * q2;

                p1 = ARow[ptrAElement + -2];
                p2 = ARow[ptrAElement + -2 + rowSkip];
                dd = d[ptrDElement + -2 * d_stride];
                q1 = p1 * dd;
                q2 = p2 * dd;
                ARow[ptrAElement + -2] = q1;
                ARow[ptrAElement + -2 + rowSkip] = q2;
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z22 += p2 * q2;

                p1 = ARow[ptrAElement + -1];
                p2 = ARow[ptrAElement + -1 + rowSkip];
                dd = d[ptrDElement + -1 * d_stride];
                q1 = p1 * dd;
                q2 = p2 * dd;
                ARow[ptrAElement + -1] = q1;
                ARow[ptrAElement + -1 + rowSkip] = q2;
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z22 += p2 * q2;
            } else {
                ptrAElement += 2;
                ptrDElement += 2 * d_stride;

                if ((columnCounter -= 2) == 0) {
                    break;
                }
            }
        }

        /* solve for diagonal 2 x 2 block at A(i,i) */
        double Y11 = ARow[ptrAElement + 0] - Z11;
        double Y21 = ARow[ptrAElement + rowSkip] - Z21;
        double Y22 = ARow[ptrAElement + 1 + rowSkip] - Z22;

        /* factorize 2 x 2 block Y, ptrDElement */
        /* factorize row 1 */
        double dd = dRecip(Y11);

        d[ptrDElement + 0 * d_stride] = dd;
        //dIASSERT(ptrDElement == d + (int)factorizationRow * d_stride);
        dIASSERT(ptrDElement == factorizationRow * d_stride);

        /* factorize row 2 */
        double q2 = Y21 * dd;
        ARow[ptrAElement + rowSkip] = q2;

        double sum = Y21 * q2;
        d[ptrDElement + 1 * d_stride] = dRecip(Y22 - sum);
    }

    //template<unsigned int d_stride>
    private static void scaleAndFactorizeL1FirstRowStripe_2(double[] ARow, int aPos, double[] d, int rowSkip, int d_stride) {
        int ptrAElement = aPos; //ARow;
        double[] ptrDElement = d;

        /* solve for diagonal 2 x 2 block at A(0,0) */
        double Y11 = ARow[ptrAElement + 0]/* - Z11*/;
        double Y21 = ARow[ptrAElement + rowSkip]/* - Z21*/;
        double Y22 = ARow[ptrAElement + 1 + rowSkip]/* - Z22*/;

        /* factorize 2 x 2 block Y, ptrDElement */
        /* factorize row 1 */
        double dd = dRecip(Y11);

        ptrDElement[0 * d_stride] = dd;
        dIASSERT(ptrDElement == d/* + (size_t)factorizationRow * d_stride*/);

        /* factorize row 2 */
        double q2 = Y21 * dd;
        ARow[ptrAElement + rowSkip] = q2;

        double sum = Y21 * q2;
        ptrDElement[1 * d_stride] = dRecip(Y22 - sum);
    }


    /* solve L*X=B, with B containing 1 right hand sides.
     * L is an n*n lower triangular matrix with ones on the diagonal.
     * L is stored by rows and its leading dimension is lskip.
     * B is an n*1 matrix that contains the right hand sides.
     * B is stored by columns and its leading dimension is also lskip.
     * B is overwritten with X.
     * this processes blocks of 2*2.
     * if this is in the factorizer source file, n must be a multiple of 2.
     */
    static void solveStripeL1_1(final double[] L, double[] B, int bPos, int rowCount, int rowSkip) {
        dIASSERT(rowCount != 0);
        dIASSERT(rowCount % 2 == 0);

        /* compute all 2 x 1 blocks of X */
        int blockStartRow = 0;
        for (boolean exitLoop = false, subsequentPass = false; !exitLoop; subsequentPass = true, exitLoop = (blockStartRow += 2) == rowCount) {
            int ptrLElement;//final double[] ptrLElement;
            int ptrBElement; //double[] ptrBElement;

            /* declare variables - Z matrix */
            double Z11, Z21;

            if (subsequentPass) {
                //ptrLElement = L + (int)blockStartRow * rowSkip;
                ptrLElement = blockStartRow * rowSkip;
                ptrBElement = bPos;//B;

                /* set the Z matrix to 0 */
                Z11 = 0;
                Z21 = 0;

                /* compute all 2 x 1 block of X, from rows i..i+2-1 */

                /* the inner loop that computes outer products and adds them to Z */
                // The iteration starts with even number and decreases it by 2. So, it must end in zero
                for (int columnCounter = blockStartRow; ; ) {
                    /* declare p and q vectors, etc */
                    double p1, q1, p2;

                    /* compute outer product and add it to the Z matrix */
                    p1 = L[ptrLElement + 0];
                    q1 = B[ptrBElement + 0];
                    Z11 += p1 * q1;
                    p2 = L[ptrLElement + rowSkip];
                    Z21 += p2 * q1;

                    /* compute outer product and add it to the Z matrix */
                    p1 = L[ptrLElement + 1];
                    q1 = B[ptrBElement + 1];
                    Z11 += p1 * q1;
                    p2 = L[ptrLElement + 1 + rowSkip];
                    Z21 += p2 * q1;

                    if (columnCounter > 6) {
                        columnCounter -= 6;

                        /* advance pointers */
                        ptrLElement += 6;
                        ptrBElement += 6;

                        /* compute outer product and add it to the Z matrix */
                        p1 = L[ptrLElement + -4];
                        q1 = B[ptrBElement + -4];
                        Z11 += p1 * q1;
                        p2 = L[ptrLElement + -4 + rowSkip];
                        Z21 += p2 * q1;

                        /* compute outer product and add it to the Z matrix */
                        p1 = L[ptrLElement + -3];
                        q1 = B[ptrBElement + -3];
                        Z11 += p1 * q1;
                        p2 = L[ptrLElement + -3 + rowSkip];
                        Z21 += p2 * q1;

                        /* compute outer product and add it to the Z matrix */
                        p1 = L[ptrLElement + -2];
                        q1 = B[ptrBElement + -2];
                        Z11 += p1 * q1;
                        p2 = L[ptrLElement + -2 + rowSkip];
                        Z21 += p2 * q1;

                        /* compute outer product and add it to the Z matrix */
                        p1 = L[ptrLElement + -1];
                        q1 = B[ptrBElement + -1];
                        Z11 += p1 * q1;
                        p2 = L[ptrLElement + -1 + rowSkip];
                        Z21 += p2 * q1;
                    } else {
                        /* advance pointers */
                        ptrLElement += 2;
                        ptrBElement += 2;

                        if ((columnCounter -= 2) == 0) {
                            break;
                        }
                    }
                    /* end of inner loop */
                }
            } else {
                ptrLElement = 0;//L/* + (size_t)blockStartRow * rowSkip*/; dIASSERT(blockStartRow == 0);
                ptrBElement = bPos;//B;

                /* set the Z matrix to 0 */
                Z11 = 0;
                Z21 = 0;
            }

            /* finish computing the X(i) block */
            double p2 = L[ptrLElement + rowSkip];

            double Y11 = B[ptrBElement + 0] - Z11;
            double Y21 = B[ptrBElement + 1] - Z21 - p2 * Y11;

            B[ptrBElement + 0] = Y11;
            B[ptrBElement + 1] = Y21;
            /* end of outer loop */
        }
    }

    //template<unsigned int d_stride>
    private static void scaleAndFactorizeL1Stripe_1(double[] ARow, int APos, double[] d, int factorizationRow, int d_stride) {
        int ptrAElement = APos;//ARow;
        int ptrDElement = 0;//d;

        /* scale the elements in a 1 x i block at A(i,0), and also */
        /* compute Z = the outer product matrix that we'll need. */
        double Z11 = 0, Z22 = 0;

        for (int columnCounter = factorizationRow; ; ) {
            double p1, p2, q1, q2, dd1, dd2;

            p1 = ARow[ptrAElement + 0];
            p2 = ARow[ptrAElement + 1];
            dd1 = d[ptrDElement + 0 * d_stride];
            dd2 = d[ptrDElement + 1 * d_stride];
            q1 = p1 * dd1;
            q2 = p2 * dd2;
            ARow[ptrAElement + 0] = q1;
            ARow[ptrAElement + 1] = q2;
            Z11 += p1 * q1;
            Z22 += p2 * q2;

            if (columnCounter > 6) {
                columnCounter -= 6;

                ptrAElement += 6;
                ptrDElement += 6 * d_stride;

                p1 = ARow[ptrAElement + -4];
                p2 = ARow[ptrAElement + -3];
                dd1 = d[ptrDElement + -4 * d_stride];
                dd2 = d[ptrDElement + -3 * d_stride];
                q1 = p1 * dd1;
                q2 = p2 * dd2;
                ARow[ptrAElement + -4] = q1;
                ARow[ptrAElement + -3] = q2;
                Z11 += p1 * q1;
                Z22 += p2 * q2;

                p1 = ARow[ptrAElement + -2];
                p2 = ARow[ptrAElement + -1];
                dd1 = d[ptrDElement + -2 * d_stride];
                dd2 = d[ptrDElement + -1 * d_stride];
                q1 = p1 * dd1;
                q2 = p2 * dd2;
                ARow[ptrAElement + -2] = q1;
                ARow[ptrAElement + -1] = q2;
                Z11 += p1 * q1;
                Z22 += p2 * q2;
            } else {
                ptrAElement += 2;
                ptrDElement += 2 * d_stride;

                if ((columnCounter -= 2) == 0) {
                    break;
                }
            }
        }

        double Y11 = ARow[ptrAElement + 0] - (Z11 + Z22);

        /* solve for diagonal 1 x 1 block at A(i,i) */
        //dIASSERT(ptrDElement == d + (int)factorizationRow * d_stride);
        dIASSERT(ptrDElement == factorizationRow * d_stride);
        /* factorize 1 x 1 block Y, ptrDElement */
        /* factorize row 1 */
        d[ptrDElement + 0 * d_stride] = dRecip(Y11);
    }

    //template<unsigned int d_stride>
    private static void scaleAndFactorizeL1FirstRowStripe_1(double[] ARow, int APos, double[] d, int d_stride) {
        int ptrAElement = APos; //ARow;
        double[] ptrDElement = d;

        double Y11 = ARow[ptrAElement + 0];

        /* solve for diagonal 1 x 1 block at A(0,0) */
        /* factorize 1 x 1 block Y, ptrDElement */
        /* factorize row 1 */
        ptrDElement[0 * d_stride] = dRecip(Y11);
    }


//#endif // #ifndef _ODE_FASTLDLT_IMPL_H_

}
