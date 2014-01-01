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
package org.ode4j.ode.internal.processmem;


/**
 *
 */
public class DxWorldProcessMemoryManager {

    DxWorldProcessMemoryManager(DxUtil.alloc_block_fn_t fnAlloc, 
            DxUtil.shrink_block_fn_t fnShrink, DxUtil.free_block_fn_t fnFree) {
        m_fnAlloc = fnAlloc;
        m_fnShrink = fnShrink;
        m_fnFree = fnFree;
        //        Assign(fnAlloc, fnShrink, fnFree);
    }

    //    void Assign(DxUtil.alloc_block_fn_t fnAlloc, 
    //            DxUtil.shrink_block_fn_t fnShrink, DxUtil.free_block_fn_t fnFree)
    //    {
    //        m_fnAlloc = fnAlloc;
    //        m_fnShrink = fnShrink;
    //        m_fnFree = fnFree;
    //    }

    final DxUtil.alloc_block_fn_t m_fnAlloc;
    final DxUtil.shrink_block_fn_t m_fnShrink;
    final DxUtil.free_block_fn_t m_fnFree;

    /**
     * Destructor.
     */
    public void DESTRUCTOR() {
        // Nothing to do
    }
}
