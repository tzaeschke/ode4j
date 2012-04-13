/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2012 Tilmann Zäschke      *
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

import org.ode4j.ode.internal.DxWorldProcessMemArena.DxStateSave;

public final class DxWorldProcessMemArena {

    public static class DxStateSave {
        
    }
    
    public final double[] AllocateArrayDReal(int size) {
        return new double[size];
    }

    public final int[] AllocateArrayInt(int size) {
       return new int[size];
    }

    public final void dummy() {
        // TODO Auto-generated method stub
        
    }

    public final void ShrinkArrayDJointWithInfo1(DxQuickStep.DJointWithInfo1[] jointiinfos,
            int _nj, int njXXX) {
        // TODO Auto-generated method stub
        
    }

    public final DxStateSave saveState() {
        // TODO Auto-generated method stub
        return null;
    }

    public final void restoreState(DxStateSave state) {
        // TODO Auto-generated method stub
        
    }

    public final double[][] AllocateArrayDRealDReal(int n) {
        return new double[n][];
    }

    public final boolean[] AllocateArrayBool(int n) {
        return new boolean[n];
    }

    public DxStateSave BEGIN_STATE_SAVE() {
        return saveState();
    }

    public void END_STATE_SAVE(DxStateSave saveInner) {
        restoreState(saveInner);
    }

    public Object[][] PeekBufferRemainder() {
        // TODO Auto-generated method stub
        return null;
    }

    public static DxWorldProcessMemArena allocateTemporary(int memreq,
            Object object, Object object2) {
        return new DxWorldProcessMemArena();
    }

    public static void freeTemporary(DxWorldProcessMemArena arena) {
        // TODO Auto-generated method stub
        
    }

}
