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

import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.threading.Threading_H.CallContext;
import org.ode4j.ode.threading.Threading_H.DCallReleasee;

public class DxSingleIslandCallContext implements CallContext {
	DxSingleIslandCallContext(DxIslandsProcessingCallContext islandsProcessingContext, 
			DxWorldProcessMemArena stepperArena, DxUtil.BlockPointer arenaInitialState, 
			DxBody[] islandBodiesStart, 
			DxJoint[] islandJointsStart) {
		m_islandsProcessingContext = islandsProcessingContext;
		m_islandIndex = 0; 
		m_stepperArena = stepperArena;
		m_arenaInitialState = arenaInitialState; 
		m_stepperCallContext = new DxStepperProcessingCallContext(islandsProcessingContext.m_world, 
				islandsProcessingContext.m_stepSize, 
				islandsProcessingContext.m_stepperAllowedThreads, 
				stepperArena, 
				islandBodiesStart, islandJointsStart);
	}

	//TZ
	public void DESTRUCTOR() {
		//nothing
	}
	
	void AssignIslandSearchProgress(int islandIndex)
	{
		m_islandIndex = islandIndex; 
	}

	void AssignIslandSelection(DxBody[] islandBodiesStartA, int islandBodiesStartP, 
			DxJoint[] islandJointsStartA, int islandJointsStartP, 
			int islandBodiesCount, int islandJointsCount)
	{
		m_stepperCallContext.AssignIslandSelection(islandBodiesStartA, islandBodiesStartP, 
				islandJointsStartA, islandJointsStartP, 
				islandBodiesCount, islandJointsCount);
	}

	DxBody[] GetSelectedIslandBodiesA() { 
		return m_stepperCallContext.m_islandBodiesStartA(); }
	int GetSelectedIslandBodies() { 
		return m_stepperCallContext.m_islandBodiesStartOfs(); }
	int GetSelectedIslandBodiesEndP() { 
		return m_stepperCallContext.GetSelectedIslandBodiesEnd(); }
	DxJoint[] GetSelectedIslandJointsA() { 
		return m_stepperCallContext.m_islandJointsStartA(); }
	int GetSelectedIslandJointsP() { 
		return m_stepperCallContext.m_islandJointsStartOfs(); }
	int GetSelectedIslandJointsEndP() { 
		return m_stepperCallContext.GetSelectedIslandJointsEnd(); }

	void RestoreSavedMemArenaStateForStepper()
	{
		m_stepperArena.RestoreState(m_arenaInitialState);
	}

	void AssignStepperCallFinalReleasee(DCallReleasee finalReleasee)
	{
		m_stepperCallContext.AssignStepperCallFinalReleasee(finalReleasee);
	}

	DxIslandsProcessingCallContext  m_islandsProcessingContext;
	int                          	m_islandIndex;
	DxWorldProcessMemArena          m_stepperArena;
	DxUtil.BlockPointer            	m_arenaInitialState;
	DxStepperProcessingCallContext  m_stepperCallContext;


}
