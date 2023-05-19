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


import static org.ode4j.ode.internal.Common.dAASSERT;

class FastVecScale {

    //    #ifndef _ODE_FASTVECSCALE_IMPL_H_
    //    #define _ODE_FASTVECSCALE_IMPL_H_



    // matrix_impl.h

    //	template<unsigned int a_stride, unsigned int d_stride>
    //	void scaleLargeVector (dReal *aStart, const dReal *dStart, unsigned elementCount)
    @SuppressWarnings("fallthrough")
    public static void scaleLargeVector(double[] A, final int aStart, final double[] D, int dStart, int elementCount, int a_stride, int d_stride) {
        dAASSERT(aStart >= 0 && dStart >= 0 && elementCount >= 0);

        final int step = 4;
        //		dReal *ptrA = aStart;
        //    const dReal *ptrD = dStart;
        //    const dReal *const dStepsEnd = dStart + (size_t)(elementCount & ~(step - 1)) * d_stride;
        int ptrA = aStart;
        int ptrD = dStart;
        final int dStepsEnd = dStart + (elementCount & ~(step - 1)) * d_stride;

        for (; ptrD != dStepsEnd; ptrA += step * a_stride, ptrD += step * d_stride) {
            double a0 = A[ptrA + 0], a1 = A[ptrA + 1 * a_stride], a2 = A[ptrA + 2 * a_stride], a3 = A[ptrA + 3 * a_stride];
            double d0 = D[ptrD + 0], d1 = D[ptrD + 1 * d_stride], d2 = D[ptrD + 2 * d_stride], d3 = D[ptrD + 3 * d_stride];
            a0 *= d0;
            a1 *= d1;
            a2 *= d2;
            a3 *= d3;
            A[ptrA + 0] = a0;
            A[ptrA + 1 * a_stride] = a1;
            A[ptrA + 2 * a_stride] = a2;
            A[ptrA + 3 * a_stride] = a3;
            //dSASSERT(step == 4);
        }

        switch (elementCount & (step - 1)) {
            case 3: {
                double a2 = A[ptrA + 2 * a_stride];
                double d2 = D[ptrD + 2 * d_stride];
                A[ptrA + 2 * a_stride] = a2 * d2;
                // break; -- proceed to case 2
            }

            case 2: {
                double a1 = A[ptrA + 1 * a_stride];
                double d1 = D[ptrD + 1 * d_stride];
                A[ptrA + 1 * a_stride] = a1 * d1;
                // break; -- proceed to case 1
            }

            case 1: {
                double a0 = A[ptrA + 0];
                double d0 = D[ptrD + 0];
                A[ptrA + 0] = a0 * d0;
                break;
            }
        }
    }

    private FastVecScale() {}
}
