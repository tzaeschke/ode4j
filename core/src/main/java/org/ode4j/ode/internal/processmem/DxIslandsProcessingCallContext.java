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

import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dstepper_fn_t;
import org.ode4j.ode.threading.Threading_H.DCallReleasee;

public class DxIslandsProcessingCallContext {

	public DxIslandsProcessingCallContext(DxWorld world, DxWorldProcessIslandsInfo islandsInfo, 
			double stepSize, dstepper_fn_t stepper) {
		m_world = world;
		m_islandsInfo = islandsInfo;
		m_stepSize = stepSize;
		m_stepper = stepper;
		m_groupReleasee = null;
		m_islandToProcessStorage = 0;
		m_stepperAllowedThreads = 0;
	}

	public void AssignGroupReleasee(DCallReleasee groupReleasee) { m_groupReleasee = groupReleasee; }
	public void SetStepperAllowedThreads(int allowedThreadsLimit) { m_stepperAllowedThreads = allowedThreadsLimit; }

    static int ThreadedProcessGroup_Callback(void callContext, 
    		dcallindex_t callInstanceIndex, DCallReleasee callThisReleasee);
    boolean ThreadedProcessGroup();

    static int ThreadedProcessJobStart_Callback(void callContext, 
    		dcallindex_t callInstanceIndex, DCallReleasee callThisReleasee);
    void ThreadedProcessJobStart();

    static int ThreadedProcessIslandSearch_Callback(void callContext, 
    		dcallindex_t callInstanceIndex, DCallReleasee callThisReleasee);
    void ThreadedProcessIslandSearch(DxSingleIslandCallContext stepperCallContext);

    static int ThreadedProcessIslandStepper_Callback(void callContext, 
    		dcallindex_t callInstanceIndex, DCallReleasee callThisReleasee);
    void ThreadedProcessIslandStepper(DxSingleIslandCallContext stepperCallContext);

    int ObtainNextIslandToBeProcessed(int islandsCount);

    final DxWorld                   m_world;
    final DxWorldProcessIslandsInfo m_islandsInfo;
    final double                    m_stepSize;
    final dstepper_fn_t             m_stepper;
    DCallReleasee                 m_groupReleasee;
    volatile int                  m_islandToProcessStorage;
    int                        m_stepperAllowedThreads;

}
