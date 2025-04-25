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
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dstepper_fn_t;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;
import org.ode4j.ode.threading.Atomics;
import org.ode4j.ode.threading.task.Task;
import org.ode4j.ode.threading.task.TaskGroup;

public class DxIslandsProcessingCallContext {

	public DxIslandsProcessingCallContext(DxWorld world, DxWorldProcessIslandsInfo islandsInfo, 
			double stepSize, dstepper_fn_t stepper) {
		m_world = world;
		m_islandsInfo = islandsInfo;
		m_stepSize = stepSize;
		m_stepper = stepper;
		//m_islandToProcessStorage = 0;
		m_stepperAllowedThreads = 0;
        m_lcpAllowedThreads = 0;
	}

	public void SetStepperAllowedThreads(int stepperAllowedThreadCount, int lcpAllowedThreadCount) {
        m_stepperAllowedThreads = stepperAllowedThreadCount;
        m_lcpAllowedThreads = lcpAllowedThreadCount;
    }

    final DxWorld                   m_world;
    final DxWorldProcessIslandsInfo m_islandsInfo;
    final double                    m_stepSize;
    final dstepper_fn_t             m_stepper;
    //volatile int                  m_islandToProcessStorage;
    final AtomicInteger             m_islandToProcessStorage = new AtomicInteger();
    int                             m_stepperAllowedThreads;
    int                             m_lcpAllowedThreads;
    
    public void ThreadedProcessJobStart(final TaskGroup parent)
    {
        DxWorldProcessContext context = m_world.UnsafeGetWorldProcessingContext(); 

        DxWorldProcessMemArena stepperArena = context.ObtainStepperMemArena();
        Common.dIASSERT(stepperArena != null && stepperArena.IsStructureValid());

        final DxWorldProcessIslandsInfo islandsInfo = m_islandsInfo;
        DxBody[] islandBodiesStart = islandsInfo.GetBodiesArray();
        DxJoint[] islandJointsStart = islandsInfo.GetJointsArray();
        
        // Save area state after context allocation to be restored for the stepper
        BlockPointer arenaState = stepperArena.SaveState();
        //new(stepperCallContext) DxSingleIslandCallContext(this, stepperArena, arenaState, islandBodiesStart, islandJointsStart);
        //stepperCallContext = new DxSingleIslandCallContext(this, stepperArena, arenaState, islandBodiesStart, islandJointsStart);
        final DxSingleIslandCallContext stepperCallContext = new DxSingleIslandCallContext(this, stepperArena, arenaState, 
        		islandBodiesStart, islandJointsStart);
        
        Task task = parent.subtask("World Islands Stepping Selection", new Runnable() {
			@Override
			public void run() {
				stepperCallContext.m_islandsProcessingContext.ThreadedProcessIslandSearch(stepperCallContext, parent);
			}
		});
        task.submit();
    }

    private void ThreadedProcessIslandSearch(final DxSingleIslandCallContext stepperCallContext, final TaskGroup parent)
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
                    TaskGroup searchTask = parent.subgroup("World Islands Stepping Selection", new Runnable() {
						@Override
						public void run() {
							stepperCallContext.m_islandsProcessingContext.ThreadedProcessIslandSearch(stepperCallContext, parent);
						}
                    });
                    stepperCallContext.m_stepperCallContext.AssignStepperTaskGroup(searchTask);
                    Task stepperTask = searchTask.subtask("Island Stepping Job Start", new Runnable() {
						@Override
						public void run() {
							stepperCallContext.m_islandsProcessingContext.ThreadedProcessIslandStepper(stepperCallContext);
						}
					});
                    stepperTask.submit();
                    searchTask.submit();
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
            DxWorldProcessContext context = m_world.UnsafeGetWorldProcessingContext(); 
            context.ReturnStepperMemArena(stepperArena);
        }
    }

    private void ThreadedProcessIslandStepper(DxSingleIslandCallContext stepperCallContext)
    {
        m_stepper.run(stepperCallContext.m_stepperCallContext);
    }

    int ObtainNextIslandToBeProcessed(int islandsCount)
    {
        return Atomics.ThrsafeIncrementSizeUpToLimit(m_islandToProcessStorage, islandsCount);
    }


}
