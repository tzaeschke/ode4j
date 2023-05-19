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
import static org.ode4j.ode.internal.FastLSolve.solveL1Straight;
import static org.ode4j.ode.internal.FastLTSolve.solveL1Transposed;
import static org.ode4j.ode.internal.FastVecScale.scaleLargeVector;

class FastLDLTSolve {


    // #ifndef _ODE_MATRIX_IMPL_H_
    // #define _ODE_MATRIX_IMPL_H_


    //	template<unsigned int d_stride, unsigned int b_stride>
    //	void solveEquationSystemWithLDLT (const dReal *L, const dReal *d, dReal *b, unsigned rowCount, unsigned rowSkip)
    public static void solveEquationSystemWithLDLT(final double[] L, double[] dArray, int dPos, double[] bArray, int bPos, int rowCount, int rowSkip, int d_stride, int b_stride) {
        dAASSERT(L != null);
        dAASSERT(dArray != null);
        dAASSERT(bArray != null);
        dAASSERT(rowCount > 0);
        dAASSERT(rowSkip >= rowCount);

        solveL1Straight(L, bArray, bPos, rowCount, rowSkip, b_stride);
        scaleLargeVector(bArray, bPos, dArray, dPos, rowCount, b_stride, d_stride);
        solveL1Transposed(L, bArray, bPos, rowCount, rowSkip, b_stride);
    }

    private FastLDLTSolve() {}
}
