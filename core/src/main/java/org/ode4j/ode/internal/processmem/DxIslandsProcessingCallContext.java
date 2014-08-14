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

import java.util.concurrent.atomic.AtomicInteger;

import org.ode4j.ode.internal.Common;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dstepper_fn_t;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;
import org.ode4j.ode.threading.ThreadingUtils;
import org.ode4j.ode.threading.Threading_H.CallContext;
import org.ode4j.ode.threading.Threading_H.DCallReleasee;
import org.ode4j.ode.threading.Threading_H.dThreadedCallFunction;

public class DxIslandsProcessingCallContext implements CallContext {

	public DxIslandsProcessingCallContext(DxWorld world, DxWorldProcessIslandsInfo islandsInfo, 
			double stepSize, dstepper_fn_t stepper) {
		m_world = world;
		m_islandsInfo = islandsInfo;
		m_stepSize = stepSize;
		m_stepper = stepper;
		m_groupReleasee = null;
		//m_islandToProcessStorage = 0;
		m_stepperAllowedThreads = 0;
	}

	public void AssignGroupReleasee(DCallReleasee groupReleasee) { m_groupReleasee = groupReleasee; }
	public void SetStepperAllowedThreads(int allowedThreadsLimit) { m_stepperAllowedThreads = allowedThreadsLimit; }

//    static int ThreadedProcessGroup_Callback(void callContext, 
//    		dcallindex_t callInstanceIndex, DCallReleasee callThisReleasee);
//    boolean ThreadedProcessGroup();
//
//    static int ThreadedProcessJobStart_Callback(void callContext, 
//    		dcallindex_t callInstanceIndex, DCallReleasee callThisReleasee);
//    void ThreadedProcessJobStart();
//
//    static int ThreadedProcessIslandSearch_Callback(void callContext, 
//    		dcallindex_t callInstanceIndex, DCallReleasee callThisReleasee);
//    void ThreadedProcessIslandSearch(DxSingleIslandCallContext stepperCallContext);
//
//    static int ThreadedProcessIslandStepper_Callback(void callContext, 
//    		dcallindex_t callInstanceIndex, DCallReleasee callThisReleasee);
//    void ThreadedProcessIslandStepper(DxSingleIslandCallContext stepperCallContext);
//
//    int ObtainNextIslandToBeProcessed(int islandsCount);

    final DxWorld                   m_world;
    final DxWorldProcessIslandsInfo m_islandsInfo;
    final double                    m_stepSize;
    final dstepper_fn_t             m_stepper;
    DCallReleasee                 m_groupReleasee;
    //volatile int                  m_islandToProcessStorage;
    final AtomicInteger             m_islandToProcessStorage = new AtomicInteger();
    int                        m_stepperAllowedThreads;

    
    public static dThreadedCallFunction ThreadedProcessGroup_Callback = new dThreadedCallFunction() {
    	@Override
    	public boolean run(CallContext callContext, 
    			int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
    	{
    		//(void)callInstanceIndex; // unused
    		//(void)callThisReleasee; // unused
    		return ((DxIslandsProcessingCallContext)callContext).ThreadedProcessGroup();
    	}
    };

    private boolean ThreadedProcessGroup()
    {
        // Do nothing - it's just a wrapper call
        return true;
    }

    public static dThreadedCallFunction ThreadedProcessJobStart_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext callContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			((DxIslandsProcessingCallContext)callContext).ThreadedProcessJobStart();
			return true;
		}
    };

    private void ThreadedProcessJobStart()
    {
        DxWorldProcessContext context = m_world.UnsafeGetWorldProcessingContext(); 

        DxWorldProcessMemArena stepperArena = context.ObtainStepperMemArena();
        Common.dIASSERT(stepperArena != null && stepperArena.IsStructureValid());

        final DxWorldProcessIslandsInfo islandsInfo = m_islandsInfo;
        DxBody[] islandBodiesStart = islandsInfo.GetBodiesArray();
        DxJoint[] islandJointsStart = islandsInfo.GetJointsArray();
        
        DxSingleIslandCallContext stepperCallContext = null;//TZ (DxSingleIslandCallContext)stepperArena.AllocateBlock(sizeof(dxSingleIslandCallContext));
        // Save area state after context allocation to be restored for the stepper
        BlockPointer arenaState = stepperArena.SaveState();
        //new(stepperCallContext) DxSingleIslandCallContext(this, stepperArena, arenaState, islandBodiesStart, islandJointsStart);
        //stepperCallContext = new DxSingleIslandCallContext(this, stepperArena, arenaState, islandBodiesStart, islandJointsStart);
        stepperCallContext = new DxSingleIslandCallContext(this, stepperArena, arenaState, 
        		islandBodiesStart, 
    			islandJointsStart);
        
        // Summary fault flag may be omitted as any failures will automatically propagate to dependent releasee (i.e. to m_groupReleasee)
        m_world.threading().PostThreadedCallForUnawareReleasee(null, null, 0, m_groupReleasee, null, 
            DxIslandsProcessingCallContext.ThreadedProcessIslandSearch_Callback, stepperCallContext, 0, "World Islands Stepping Selection");
    }

    public static dThreadedCallFunction ThreadedProcessIslandSearch_Callback = 
    		new dThreadedCallFunction() {
    	@Override
    	public boolean run(CallContext callContext, 
    			int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
    	{
    		//(void)callInstanceIndex; // unused
    		//(void)callThisReleasee; // unused
    		DxSingleIslandCallContext stepperCallContext = (DxSingleIslandCallContext)(callContext);
    		stepperCallContext.m_islandsProcessingContext.ThreadedProcessIslandSearch(stepperCallContext);
    		return true;
    	}
    };

    private void ThreadedProcessIslandSearch(DxSingleIslandCallContext stepperCallContext)
    {
        boolean finalizeJob = false;

        final DxWorldProcessIslandsInfo islandsInfo = m_islandsInfo;
        int[] islandSizes = islandsInfo.GetIslandSizes();

        final int islandsCount = islandsInfo.GetIslandsCount();
        int islandToProcess = ObtainNextIslandToBeProcessed(islandsCount);

        if (islandToProcess != islandsCount) {
            // First time, the counts are zeros and on next passes, adding counts will skip island that has just been processed by stepper
            DxBody[] islandBodiesStartA = stepperCallContext.GetSelectedIslandBodiesA();
            int islandBodiesStartP = stepperCallContext.GetSelectedIslandBodiesEndP();
            DxJoint[] islandJointsStartA = stepperCallContext.GetSelectedIslandJointsA();
            int islandJointsStartP = stepperCallContext.GetSelectedIslandJointsEndP();
            int islandIndex = stepperCallContext.m_islandIndex;

            for (; ; ++islandIndex) {
                int bcount = islandSizes[islandIndex * DxWorldProcessIslandsInfo.dxISE__MAX + DxWorldProcessIslandsInfo.dxISE_BODIES_COUNT];
                int jcount = islandSizes[islandIndex * DxWorldProcessIslandsInfo.dxISE__MAX + DxWorldProcessIslandsInfo.dxISE_JOINTS_COUNT];

                if (islandIndex == islandToProcess) {
                    // Store selected island details
                    stepperCallContext.AssignIslandSelection(
                    		islandBodiesStartA, islandBodiesStartP, 
                    		islandJointsStartA, islandJointsStartP, bcount, jcount);

                    // Store next island index to continue search from
                    ++islandIndex;
                    stepperCallContext.AssignIslandSearchProgress(islandIndex);

                    // Restore saved stepper memory arena position
                    stepperCallContext.RestoreSavedMemArenaStateForStepper();

                    Ref<DCallReleasee> nextSearchReleasee = new Ref<DCallReleasee>();

                    // Summary fault flag may be omitted as any failures will automatically propagate to dependent releasee (i.e. to m_groupReleasee)
                    m_world.threading().PostThreadedCallForUnawareReleasee(null, nextSearchReleasee, 1, m_groupReleasee, null, 
                        DxIslandsProcessingCallContext.ThreadedProcessIslandSearch_Callback, stepperCallContext, 0, "World Islands Stepping Selection");

                    stepperCallContext.AssignStepperCallFinalReleasee(nextSearchReleasee.get());

                    m_world.threading().PostThreadedCall(null, null, 0, nextSearchReleasee.get(), null, 
                        DxIslandsProcessingCallContext.ThreadedProcessIslandStepper_Callback, stepperCallContext, 0, "Island Stepping Job Start");

                    break;
                }

                islandBodiesStartP += bcount;
                islandJointsStartP += jcount;
            }
        }
        else {
            finalizeJob = true;
        }

        if (finalizeJob) {
            DxWorldProcessMemArena stepperArena = stepperCallContext.m_stepperArena;
            //stepperCallContext.dxSingleIslandCallContext::~dxSingleIslandCallContext();
            stepperCallContext.DESTRUCTOR();//dxSingleIslandCallContext.DESTRUCTOR();

            DxWorldProcessContext context = m_world.UnsafeGetWorldProcessingContext(); 
            context.ReturnStepperMemArena(stepperArena);
        }
    }

    public static dThreadedCallFunction ThreadedProcessIslandStepper_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext callContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			DxSingleIslandCallContext stepperCallContext = (DxSingleIslandCallContext)(callContext);
			stepperCallContext.m_islandsProcessingContext.ThreadedProcessIslandStepper(stepperCallContext);
			return true;
		}
    };

    private void ThreadedProcessIslandStepper(DxSingleIslandCallContext stepperCallContext)
    {
        m_stepper.run(stepperCallContext.m_stepperCallContext);
    }

    int ObtainNextIslandToBeProcessed(int islandsCount)
    {
        return ThreadingUtils.ThrsafeIncrementSizeUpToLimit(m_islandToProcessStorage, islandsCount);
    }


}
