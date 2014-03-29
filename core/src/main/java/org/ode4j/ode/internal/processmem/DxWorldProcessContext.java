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
import org.ode4j.ode.internal.processmem.DxWorldProcessIslandsInfo.dmemestimate_fn_t;
import org.ode4j.ode.threading.DThreadingFunctionsInfo.DCallWait;
import org.ode4j.ode.threading.DThreadingFunctionsInfo.DMutexGroup;


public class DxWorldProcessContext {

    //public:
    //public DxWorldProcessContext();
    //public DESTRUCTOR();

    //void CleanupWorldReferences(dxWorld *pswWorldInstance);

    //public boolean EnsureStepperSyncObjectsAreAllocated(dxWorld *pswWorldInstance);
    public DCallWait GetIslandsSteppingWait() { return m_pcwIslandsSteppingWait; }

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
    void SetStepperArenasList(DxWorldProcessMemArena pmaInstance) { m_pmaStepperArenas = pmaInstance; }
    DxWorldProcessMemArena GetStepperArenasList() { return m_pmaStepperArenas; }

    //DxWorldProcessMemArena GetStepperArenasHead();
    //boolean TryExtractingStepperArenasHead(dxWorldProcessMemArena *pmaHeadInstance);
    //boolean TryInsertingStepperArenasHead(dxWorldProcessMemArena *pmaArenaInstance, dxWorldProcessMemArena *pmaExistingHead);

    //public:
    //void LockForAddLimotSerialization();
    //void UnlockForAddLimotSerialization();
    //void LockForStepbodySerialization();
    //void UnlockForStepbodySerialization();

    //private:
    private enum dxProcessContextMutex
    {
        dxPCM_STEPPER_ARENA_OBTAIN,
        dxPCM_STEPPER_ADDLIMOT_SERIALIZE,
        dxPCM_STEPPER_STEPBODY_SERIALIZE,

        //dxPCM__MAX
    };

    //static const char *const m_aszContextMutexNames[dxPCM__MAX];
    private static final String[] m_aszContextMutexNames = //new String[dxPCM__MAX];
	{
	    "Stepper Arena Obtain Lock" , // dxPCM_STEPPER_ARENA_OBTAIN,
	    "Joint addLimot Serialize Lock" , // dxPCM_STEPPER_ADDLIMOT_SERIALIZE
	    "Stepper StepBody Serialize Lock" , // dxPCM_STEPPER_STEPBODY_SERIALIZE,
	};
   
    
    //private:
    //private DxWorldProcessMemArena m_pmaIslandsArena;
    //private DxWorldProcessMemArena m_pmaStepperArena;
    DxWorldProcessMemArena  m_pmaIslandsArena;
    volatile DxWorldProcessMemArena  m_pmaStepperArenas;
    DxWorld                 m_pswObjectsAllocWorld;
    DMutexGroup           m_pmgStepperMutexGroup;
    DCallWait             m_pcwIslandsSteppingWait;



    //****************************************************************************
    // dxWorldProcessContext

    DxWorldProcessContext() { //:
        m_pmaIslandsArena = null;//(NULL),
        m_pmaStepperArenas = null;//(NULL)
        m_pswObjectsAllocWorld = null;//(NULL),
        m_pmgStepperMutexGroup = null;//(NULL),
        m_pcwIslandsSteppingWait = null;//(NULL)
       // Do nothing
    }

    //dxWorldProcessContext::~dxWorldProcessContext()
    public void DESTRUCTOR()
    {
    	Common.dIASSERT((m_pswObjectsAllocWorld != null) == (m_pmgStepperMutexGroup != null));
    	Common.dIASSERT((m_pswObjectsAllocWorld != null) == (m_pcwIslandsSteppingWait != null));

        if (m_pswObjectsAllocWorld != null)
        {
            m_pswObjectsAllocWorld.FreeMutexGroup(m_pmgStepperMutexGroup);
            m_pswObjectsAllocWorld.FreeThreadedCallWait(m_pcwIslandsSteppingWait);
        }

        DxWorldProcessMemArena pmaStepperArenas = m_pmaStepperArenas;
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
        Common.dIASSERT((m_pswObjectsAllocWorld != null) == (m_pcwIslandsSteppingWait != null));

        if (m_pswObjectsAllocWorld == pswWorldInstance)
        {
            m_pswObjectsAllocWorld.FreeMutexGroup(m_pmgStepperMutexGroup);
            m_pswObjectsAllocWorld.FreeThreadedCallWait(m_pcwIslandsSteppingWait);

            m_pswObjectsAllocWorld = null;
            m_pmgStepperMutexGroup = null;
            m_pcwIslandsSteppingWait = null;
        }
    }

    boolean EnsureStepperSyncObjectsAreAllocated(DxWorld pswWorldInstance)
    {
    	Common.dIASSERT((m_pswObjectsAllocWorld != null) == (m_pmgStepperMutexGroup != null));
    	Common.dIASSERT((m_pswObjectsAllocWorld != null) == (m_pcwIslandsSteppingWait != null));

        boolean bResult = false;

        DMutexGroup pmbStepperMutexGroup = null;
        boolean bStepperMutexGroupAllocated = false;

        do
        {
            if (m_pswObjectsAllocWorld == null)
            {
                pmbStepperMutexGroup = pswWorldInstance.AllocMutexGroup(dxPCM__MAX, m_aszContextMutexNames);
                if (pmbStepperMutexGroup == null)
                {
                    break;
                }

                bStepperMutexGroupAllocated = true;

                DCallWait pcwIslandsSteppingWait = pswWorldInstance.AllocThreadedCallWait();
                if (pcwIslandsSteppingWait == null)
                {
                    break;
                }

                m_pswObjectsAllocWorld = pswWorldInstance;
                m_pmgStepperMutexGroup = pmbStepperMutexGroup;
                m_pcwIslandsSteppingWait = pcwIslandsSteppingWait;
            }

            bResult = true;
        }
        while (false);

        if (!bResult)
        {
            if (bStepperMutexGroupAllocated)
            {
                pswWorldInstance.FreeMutexGroup(pmbStepperMutexGroup);
            }
        }

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
            DxMutexGroupLockHelper lhLockHelper(m_pswObjectsAllocWorld, m_pmgStepperMutexGroup, dxPCM_STEPPER_ARENA_OBTAIN);

            DxWorldProcessMemArena pmaArenasHead = GetStepperArenasHead(); // Arenas head must be re-extracted after mutex has been locked
            boolean bExchangeResult = pmaArenasHead != null && TryExtractingStepperArenasHead(pmaArenasHead);

            lhLockHelper.UnlockMutex();

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
    		final DxWorldProcessMemoryManager pmmMemortManager, float fReserveFactor, int uiReserveMinimum)
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
    	while (pmaExistingArenas != null)
    	{
    		DxWorldProcessMemArena pmaCurrentMemArena = pmaExistingArenas;
    		pmaExistingArenas = pmaExistingArenas.GetNextMemArena();

    		DxWorldProcessMemArena.FreeMemArena(pmaCurrentMemArena);
    	}
    }

    DxWorldProcessMemArena GetStepperArenasHead()
    {
    	return m_pmaStepperArenas;
    }

    boolean TryExtractingStepperArenasHead(DxWorldProcessMemArena pmaHeadInstance)
    {
    	DxWorldProcessMemArena pmaNextInstance = pmaHeadInstance.GetNextMemArena();
    	return ThrsafeCompareExchangePointer((volatile atomicptr *)&m_pmaStepperArenas, (atomicptr)pmaHeadInstance, (atomicptr)pmaNextInstance);
    }

    boolean TryInsertingStepperArenasHead(DxWorldProcessMemArena pmaArenaInstance, 
    		DxWorldProcessMemArena pmaExistingHead)
    {
    	return ThrsafeCompareExchangePointer((volatile atomicptr *)&m_pmaStepperArenas, (atomicptr)pmaExistingHead, (atomicptr)pmaArenaInstance);
    }


    void LockForAddLimotSerialization()
    {
    	m_pswObjectsAllocWorld.LockMutexGroupMutex(m_pmgStepperMutexGroup, 
    			dxProcessContextMutex.dxPCM_STEPPER_ADDLIMOT_SERIALIZE);
    }

    void UnlockForAddLimotSerialization()
    {
    	m_pswObjectsAllocWorld.UnlockMutexGroupMutex(m_pmgStepperMutexGroup, 
    			dxProcessContextMutex.dxPCM_STEPPER_ADDLIMOT_SERIALIZE);
    }


    public void LockForStepbodySerialization()
    {
    	m_pswObjectsAllocWorld.LockMutexGroupMutex(m_pmgStepperMutexGroup, 
    			dxProcessContextMutex.dxPCM_STEPPER_STEPBODY_SERIALIZE);
    }

    public void UnlockForStepbodySerialization()
    {
    	m_pswObjectsAllocWorld.UnlockMutexGroupMutex(m_pmgStepperMutexGroup, 
    			dxProcessContextMutex.dxPCM_STEPPER_STEPBODY_SERIALIZE);
    }


    //  bool dxReallocateWorldProcessContext (dxWorld *world, dxWorldProcessIslandsInfo &islandsinfo, 
    //  dReal stepsize, dmemestimate_fn_t stepperestimate);
    //void dxCleanupWorldProcessContext (dxWorld *world);
    //TODO remove! ODE 0.12
    private static boolean dxReallocateWorldProcessContext (DxWorld world, 
            DxWorldProcessIslandsInfo islandsinfo, 
            double stepsize, dmemestimate_fn_t stepperestimate)
    {
        //TZ DxStepWorkingMemory wmem = DxWorld.AllocateOnDemand(world.wmem);
        if (world.wmem == null) {
            world.wmem = new DxStepWorkingMemory();
        }
        DxStepWorkingMemory wmem = world.wmem;

        if (wmem == null) return false;

        DxWorldProcessContext context = wmem.SureGetWorldProcessingContext();
        if (context == null) return false;
        Common.dIASSERT (context.IsStructureValid());

        final DxWorldProcessMemoryReserveInfo reserveinfo = wmem.SureGetMemoryReserveInfo();
        final DxWorldProcessMemoryManager memmgr = wmem.SureGetMemoryManager();

        int islandsreq = world.EstimateIslandsProcessingMemoryRequirements();
        Common.dIASSERT(islandsreq == DxUtil.dEFFICIENT_SIZE(islandsreq));

        DxWorldProcessMemArena stepperarena = null;
        DxWorldProcessMemArena islandsarena = context.ReallocateIslandsMemArena(
                islandsreq, memmgr, 1.0f, reserveinfo.m_uiReserveMinimum);

        if (islandsarena != null)
        {
            int stepperreq = 
                DxWorldProcessIslandsInfo.BuildIslandsAndEstimateStepperMemoryRequirements(
                        islandsinfo, islandsarena, world, stepsize, stepperestimate);
            Common.dIASSERT(stepperreq == DxUtil.dEFFICIENT_SIZE(stepperreq));

            stepperarena = context.ReallocateStepperMemArena(stepperreq, 
                    memmgr, reserveinfo.m_fReserveFactor, reserveinfo.m_uiReserveMinimum);
        }

        return stepperarena != null;
    }

    public static void dxCleanupWorldProcessContext (DxWorld world)
    {
        DxStepWorkingMemory wmem = world.wmem;
        if (wmem != null)
        {
            DxWorldProcessContext context = wmem.GetWorldProcessingContext();
            if (context != null)
            {
                context.CleanupContext();
                Common.dIASSERT(context.IsStructureValid());
            }
        }
    }


}
