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

import java.util.concurrent.atomic.AtomicReference;

import org.ode4j.ode.internal.Common;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.processmem.DxWorldProcessIslandsInfo.dmemestimate_fn_t;
import org.ode4j.ode.threading.Atomics;
import org.ode4j.ode.threading.MutexGroup;
import org.ode4j.ode.threading.MutexGroup.dxProcessContextMutex;
import org.ode4j.ode.threading.MutexGroupImpl;


public class DxWorldProcessContext {

    //public:
    //public DxWorldProcessContext();
    //public DESTRUCTOR();

    //void CleanupWorldReferences(dxWorld *pswWorldInstance);

    //public DxWorldProcessMemArena *ObtainStepperMemArena();
    //public void ReturnStepperMemArena(dxWorldProcessMemArena *pmaArenaInstance);

    //    public DxWorldProcessMemArena ReallocateIslandsMemArena(int nMemoryRequirement, 
    //            final DxWorldProcessMemoryManager pmmMemortManager, float fReserveFactor, int uiReserveMinimum);
    //    public DxWorldProcessMemArena ReallocateStepperMemArena(int nMemoryRequirement, 
    //            final DxWorldProcessMemoryManager pmmMemortManager, float fReserveFactor, int uiReserveMinimum);
    //public boolean ReallocateStepperMemArenas(dxWorld *world, unsigned nIslandThreadsCount, size_t nMemoryRequirement, 
    //        const dxWorldProcessMemoryManager *pmmMemortManager, float fReserveFactor, unsigned uiReserveMinimum);

    //private static void FreeArenasList(dxWorldProcessMemArena *pmaExistingArenas);
    
    //private:
    private void SetIslandsMemArena(DxWorldProcessMemArena pmaInstance) { 
        m_pmaIslandsArena = pmaInstance; 
    }
    DxWorldProcessMemArena GetIslandsMemArena() { return m_pmaIslandsArena; }

    //private:
    void SetStepperArenasList(DxWorldProcessMemArena pmaInstance) { m_pmaStepperArenas.set(pmaInstance); }
    DxWorldProcessMemArena GetStepperArenasList() { return m_pmaStepperArenas.get(); }

    //DxWorldProcessMemArena GetStepperArenasHead();
    //boolean TryExtractingStepperArenasHead(dxWorldProcessMemArena *pmaHeadInstance);
    //boolean TryInsertingStepperArenasHead(dxWorldProcessMemArena *pmaArenaInstance, dxWorldProcessMemArena *pmaExistingHead);

    //public:
    //void LockForAddLimotSerialization();
    //void UnlockForAddLimotSerialization();
    //void LockForStepbodySerialization();
    //void UnlockForStepbodySerialization();

    //private:

    //private:
    //private DxWorldProcessMemArena m_pmaIslandsArena;
    //private DxWorldProcessMemArena m_pmaStepperArena;
    DxWorldProcessMemArena  m_pmaIslandsArena;
    //volatile DxWorldProcessMemArena  m_pmaStepperArenas;
    final AtomicReference<DxWorldProcessMemArena>  m_pmaStepperArenas = 
    		new AtomicReference<DxWorldProcessMemArena>();
    DxWorld                 m_pswObjectsAllocWorld;
    MutexGroup           m_pmgStepperMutexGroup;



    //****************************************************************************
    // dxWorldProcessContext

    DxWorldProcessContext() { //:
        m_pmaIslandsArena = null;//(NULL),
        //m_pmaStepperArenas = null;//(NULL)
        m_pswObjectsAllocWorld = null;//(NULL),
        m_pmgStepperMutexGroup = null;//(NULL),
       // Do nothing
    }

    //dxWorldProcessContext::~dxWorldProcessContext()
    public void DESTRUCTOR()
    {
    	Common.dIASSERT((m_pswObjectsAllocWorld != null) == (m_pmgStepperMutexGroup != null));

        // TODO TZ: Ignored for migration to 0.16.2
        //        if (m_pswObjectsAllocWorld != null)
        //        {
        //            throw new UnsupportedOperationException("This has not been ported yet from ODE");
        //            // TODO m_pswObjectsAllocWorld->FreeMutexGroup(m_pmgStepperMutexGroup);
        //            // m_pswObjectsAllocWorld->FreeThreadedCallWait(m_pcwIslandsSteppingWait); -- The stock call wait can not be freed
        //        }

        DxWorldProcessMemArena pmaStepperArenas = m_pmaStepperArenas.get();
        if (pmaStepperArenas != null)
        {
            FreeArenasList(pmaStepperArenas);
        }

        if (m_pmaIslandsArena != null)
        {
            DxWorldProcessMemArena.FreeMemArena(m_pmaIslandsArena);
        }
    }

    void CleanupWorldReferences(DxWorld pswWorldInstance)
    {
        Common.dIASSERT((m_pswObjectsAllocWorld != null) == (m_pmgStepperMutexGroup != null));

        if (m_pswObjectsAllocWorld == pswWorldInstance)
        {
            m_pswObjectsAllocWorld = null;
            m_pmgStepperMutexGroup = null;
        }
    }

    boolean EnsureStepperSyncObjectsAreAllocated(DxWorld pswWorldInstance)
    {
    	Common.dIASSERT((m_pswObjectsAllocWorld != null) == (m_pmgStepperMutexGroup != null));

        boolean bResult = false;

        MutexGroup pmbStepperMutexGroup = null;

        do
        {
            if (m_pswObjectsAllocWorld == null)
            {
                pmbStepperMutexGroup = new MutexGroupImpl();
                m_pswObjectsAllocWorld = pswWorldInstance;
                m_pmgStepperMutexGroup = pmbStepperMutexGroup;
            }

            bResult = true;
        }
        while (false);

        return bResult;
    }


    DxWorldProcessMemArena ObtainStepperMemArena()
    {
        DxWorldProcessMemArena pmaArenaInstance = null;

        while (true)
        {
            DxWorldProcessMemArena pmaRawArenasHead = GetStepperArenasHead();
            if (pmaRawArenasHead == null)
            {
                break;
            }

            // Extraction must be locked so that other thread does not "steal" head arena,
            // use it and then reinsert back with a different "next"
            m_pmgStepperMutexGroup.lock(dxProcessContextMutex.dxPCM_STEPPER_ARENA_OBTAIN);

            DxWorldProcessMemArena pmaArenasHead = GetStepperArenasHead(); // Arenas head must be re-extracted after mutex has been locked
            boolean bExchangeResult = pmaArenasHead != null && TryExtractingStepperArenasHead(pmaArenasHead);

            m_pmgStepperMutexGroup.unlock(dxProcessContextMutex.dxPCM_STEPPER_ARENA_OBTAIN);

            if (bExchangeResult)
            {
                pmaArenasHead.ResetState();
                pmaArenaInstance = pmaArenasHead;
                break;
            }
        }

        return pmaArenaInstance;
    }

    void ReturnStepperMemArena(DxWorldProcessMemArena pmaArenaInstance)
    {
        while (true)
        {
            DxWorldProcessMemArena pmaArenasHead = GetStepperArenasHead();
            pmaArenaInstance.SetNextMemArena(pmaArenasHead);

            if (TryInsertingStepperArenasHead(pmaArenaInstance, pmaArenasHead))
            {
                break;
            }
        }
    }

    DxWorldProcessMemArena ReallocateIslandsMemArena(int nMemoryRequirement, 
            final DxWorldProcessMemoryManager pmmMemortManager, 
            double fReserveFactor, int uiReserveMinimum)
    {
        DxWorldProcessMemArena pmaExistingArena = GetIslandsMemArena();
        DxWorldProcessMemArena pmaNewMemArena = DxWorldProcessMemArena.ReallocateMemArena(
                pmaExistingArena, nMemoryRequirement, pmmMemortManager, fReserveFactor, uiReserveMinimum);
        SetIslandsMemArena(pmaNewMemArena);

        pmaNewMemArena.ResetState();

        return pmaNewMemArena;
    }

    boolean ReallocateStepperMemArenas(
    		DxWorld world, int nIslandThreadsCount, int nMemoryRequirement, 
    		final DxWorldProcessMemoryManager pmmMemortManager, double fReserveFactor, int uiReserveMinimum)
    {
    	DxWorldProcessMemArena pmaRebuiltArenasHead = null, pmaRebuiltArenasTail = null;
    	DxWorldProcessMemArena pmaExistingArenas = GetStepperArenasList();
    	int nArenasToProcess = nIslandThreadsCount;

    	//???(void)world; // unused

    	// NOTE!
    	// The list is reallocated in a way to assure the largest arenas are at end 
    	// and if number of threads decreases they will be freed first of all.

    	while (true)
    	{
    		if (nArenasToProcess == 0)
    		{
    			FreeArenasList(pmaExistingArenas);
    			break;
    		}

    		DxWorldProcessMemArena pmaOldMemArena = pmaExistingArenas;

    		if (pmaExistingArenas != null)
    		{
    			pmaExistingArenas = pmaExistingArenas.GetNextMemArena();
    		}
    		else
    		{
    			// If existing arenas ended, terminate and erase tail so that new arenas 
    			// would be appended to list head.
    			if (pmaRebuiltArenasTail != null)
    			{
    				pmaRebuiltArenasTail.SetNextMemArena(null);
    				pmaRebuiltArenasTail = null;
    			}
    		}

    		DxWorldProcessMemArena pmaNewMemArena = DxWorldProcessMemArena.ReallocateMemArena(
    				pmaOldMemArena, nMemoryRequirement, pmmMemortManager, fReserveFactor, 
    				uiReserveMinimum);

    		if (pmaNewMemArena != null)
    		{
    			// Append reallocated arenas to list tail while old arenas still exist...
    			if (pmaRebuiltArenasTail != null)
    			{
    				pmaRebuiltArenasTail.SetNextMemArena(pmaNewMemArena);
    				pmaRebuiltArenasTail = pmaNewMemArena;
    			}
    			else if (pmaRebuiltArenasHead == null)
    			{
    				pmaRebuiltArenasHead = pmaNewMemArena;
    				pmaRebuiltArenasTail = pmaNewMemArena;
    			}
    			// ...and append them to list head if those are additional arenas created
    			else
    			{
    				pmaNewMemArena.SetNextMemArena(pmaRebuiltArenasHead);
    				pmaRebuiltArenasHead = pmaNewMemArena;
    			}

    			--nArenasToProcess;
    		}
    		else if (pmaOldMemArena == null)
    		{
    			break;
    		}
    	}

    	if (pmaRebuiltArenasTail != null)
    	{
    		pmaRebuiltArenasTail.SetNextMemArena(null);
    	}

    	SetStepperArenasList(pmaRebuiltArenasHead);

    	boolean bResult = nArenasToProcess == 0;
    	return bResult;
    }

    void FreeArenasList(DxWorldProcessMemArena pmaExistingArenas)
    {
//    	while (pmaExistingArenas != null)
//    	{
//    		DxWorldProcessMemArena pmaCurrentMemArena = pmaExistingArenas;
//    		pmaExistingArenas = pmaExistingArenas.GetNextMemArena();
//
//    		DxWorldProcessMemArena.FreeMemArena(pmaCurrentMemArena);
//    	}
    	//TZ TODO 
    }

    DxWorldProcessMemArena GetStepperArenasHead()
    {
    	return m_pmaStepperArenas.get();
    }

    boolean TryExtractingStepperArenasHead(DxWorldProcessMemArena pmaHeadInstance)
    {
    	DxWorldProcessMemArena pmaNextInstance = pmaHeadInstance.GetNextMemArena();
    	return Atomics.ThrsafeCompareExchangePointer(m_pmaStepperArenas, pmaHeadInstance, pmaNextInstance);
    }

    boolean TryInsertingStepperArenasHead(DxWorldProcessMemArena pmaArenaInstance, 
    		DxWorldProcessMemArena pmaExistingHead)
    {
    	return Atomics.ThrsafeCompareExchangePointer(m_pmaStepperArenas, pmaExistingHead, pmaArenaInstance);
    }


    public void LockForAddLimotSerialization()
    {
    	m_pmgStepperMutexGroup.lock(dxProcessContextMutex.dxPCM_STEPPER_ADDLIMOT_SERIALIZE);
    }

    public void UnlockForAddLimotSerialization()
    {
    	m_pmgStepperMutexGroup.unlock(dxProcessContextMutex.dxPCM_STEPPER_ADDLIMOT_SERIALIZE);
    }


    public void LockForStepbodySerialization()
    {
    	m_pmgStepperMutexGroup.lock(dxProcessContextMutex.dxPCM_STEPPER_STEPBODY_SERIALIZE);
    }

    public void UnlockForStepbodySerialization()
    {
    	m_pmgStepperMutexGroup.unlock(dxProcessContextMutex.dxPCM_STEPPER_STEPBODY_SERIALIZE);
    }


    public static boolean dxReallocateWorldProcessContext (DxWorld world, DxWorldProcessIslandsInfo islandsInfo, 
    		double stepSize, dmemestimate_fn_t stepperEstimate)
    {
    	boolean result = false;

    	do
    	{
    		//DxStepWorkingMemory wmem = AllocateOnDemand(world.wmem);
    		DxStepWorkingMemory wmem = world.wmem != null ? world.wmem : new DxStepWorkingMemory();
    		world.wmem = wmem;
    		if (wmem == null)
    		{
    			break;
    		}

    		DxWorldProcessContext context = wmem.SureGetWorldProcessingContext();
    		if (context == null)
    		{
    			break;
    		}

    		if (!context.EnsureStepperSyncObjectsAreAllocated(world))
    		{
    			break;
    		}

    		final DxWorldProcessMemoryReserveInfo reserveInfo = wmem.SureGetMemoryReserveInfo();
    		final DxWorldProcessMemoryManager memmgr = wmem.SureGetMemoryManager();

    		int islandsReq = world.EstimateIslandProcessingMemoryRequirements();
    		Common.dIASSERT(islandsReq == DxUtil.dEFFICIENT_SIZE(islandsReq));

    		DxWorldProcessMemArena islandsArena = 
    				context.ReallocateIslandsMemArena(islandsReq, memmgr, 1.0f, 
    						reserveInfo.m_uiReserveMinimum);
    		if (islandsArena == null)
    		{
    			break;
    		}
    		Common.dIASSERT(islandsArena.IsStructureValid());

    		int stepperReq = 
    				DxWorldProcessIslandsInfo.BuildIslandsAndEstimateStepperMemoryRequirements(
    						islandsInfo, islandsArena, world, stepSize, stepperEstimate);
    		Common.dIASSERT(stepperReq == DxUtil.dEFFICIENT_SIZE(stepperReq));

    		//TODO this is just wrong!!!! (TZ)
    		int stepperReqWithCallContext = stepperReq + DxUtil.dEFFICIENT_SIZE(1);//DxSingleIslandCallContext.class);

            // TODO TZ: This has not been migrfated from ODE 0.16.2
            // throw new UnsupportedOperationException("This has not been ported yet from ODE");
            int islandThreadsCount = world.GetThreadingIslandsMaxThreadsCount(null);
            if (!context.ReallocateStepperMemArenas(world, islandThreadsCount, stepperReqWithCallContext,
                    memmgr, reserveInfo.m_fReserveFactor, reserveInfo.m_uiReserveMinimum))
            {
                break;
            }

            result = true;
    	}
    	while (false);

    	return result;
    }
}
