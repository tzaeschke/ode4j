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

import org.ode4j.ode.internal.Common;
import org.ode4j.ode.internal.DxWorld;

public class DxStepWorkingMemory {
    //public:
    public DxStepWorkingMemory() {
        //: 
        m_uiRefCount = 1;//(1), 
        m_ppcProcessingContext = null; 
        m_priReserveInfo = null;//(NULL), 
        m_pmmMemoryManager = null;//(NULL) {}
    }

    //private:
    //private /*friend*/ struct dBase; // To avoid GCC warning regarding private destructor
    private void DESTRUCTOR()//~dxStepWorkingMemory() // Use Release() instead
    {
        //          delete m_ppcProcessingContext;
        //          delete m_priReserveInfo;
        //          delete m_pmmMemoryManager;
    }

    //  public:
    public void Addref()
    {
        Common.dIASSERT(~m_uiRefCount != 0);
        ++m_uiRefCount;
    }

    public void Release()
    {
        Common.dIASSERT(m_uiRefCount != 0);
        if (--m_uiRefCount == 0)
        {
            //delete
            this.DESTRUCTOR();
        }
    }

    //public:
    public void CleanupMemory()
    {
        //delete 
        m_ppcProcessingContext.DESTRUCTOR();
        m_ppcProcessingContext = null;
    }

    public void CleanupWorldReferences(DxWorld world)
    {
        if (m_ppcProcessingContext != null)
        {
            m_ppcProcessingContext.CleanupWorldReferences(world);
        }
    }

    //public: 
    public DxWorldProcessContext SureGetWorldProcessingContext() { 
        //return DxWorld.AllocateOnDemand(m_ppcProcessingContext);
        if (m_ppcProcessingContext == null) {
            m_ppcProcessingContext = new DxWorldProcessContext();
        }
        return m_ppcProcessingContext;
    }
    public DxWorldProcessContext GetWorldProcessingContext() { 
        return m_ppcProcessingContext; 
    }

    public DxWorldProcessMemoryReserveInfo GetMemoryReserveInfo() { 
        return m_priReserveInfo; 
    }
    public DxWorldProcessMemoryReserveInfo SureGetMemoryReserveInfo() { 
        return m_priReserveInfo!=null ? m_priReserveInfo : DxUtil.g_WorldProcessDefaultReserveInfo; 
    }
    public void SetMemoryReserveInfo(double fReserveFactor, int uiReserveMinimum)
    {
        if (m_priReserveInfo!=null) { 
            //TZ use only constructor to allow fields to be final for concurrency tests.
            //m_priReserveInfo.Assign(fReserveFactor, uiReserveMinimum); 
            m_priReserveInfo = new DxWorldProcessMemoryReserveInfo(fReserveFactor, uiReserveMinimum); 
        }
        else { 
            m_priReserveInfo = new DxWorldProcessMemoryReserveInfo(fReserveFactor, uiReserveMinimum); 
        }
    }
    public void ResetMemoryReserveInfoToDefault()
    {
        if (m_priReserveInfo!=null) { 
            //delete 
            m_priReserveInfo.DESTRUCTOR(); 
            m_priReserveInfo = null; 
        }
    }

    public DxWorldProcessMemoryManager GetMemoryManager() { 
        return m_pmmMemoryManager; 
    }
    public DxWorldProcessMemoryManager SureGetMemoryManager() { 
        return m_pmmMemoryManager!=null ? m_pmmMemoryManager : DxUtil.g_WorldProcessMallocMemoryManager; 
    }
    public void SetMemoryManager(
            DxUtil.alloc_block_fn_t fnAlloc, 
            DxUtil.shrink_block_fn_t fnShrink, 
            DxUtil.free_block_fn_t fnFree) 
    {
        if (m_pmmMemoryManager!=null) { 
            //m_pmmMemoryManager.Assign(fnAlloc, fnShrink, fnFree);
            //TZ for concurrency tests and MemManager final fields
            m_pmmMemoryManager = new DxWorldProcessMemoryManager(fnAlloc, fnShrink, fnFree); 
        }
        else { 
            m_pmmMemoryManager = new DxWorldProcessMemoryManager(fnAlloc, fnShrink, fnFree); 
        }
    }
    public void ResetMemoryManagerToDefault()
    {
        if (m_pmmMemoryManager!=null) { 
            //delete 
            m_pmmMemoryManager.DESTRUCTOR(); 
            m_pmmMemoryManager = null; 
        }
    }

    //private:
        private int m_uiRefCount;
    private DxWorldProcessContext m_ppcProcessingContext;
    private DxWorldProcessMemoryReserveInfo m_priReserveInfo;
    private DxWorldProcessMemoryManager m_pmmMemoryManager;

}
