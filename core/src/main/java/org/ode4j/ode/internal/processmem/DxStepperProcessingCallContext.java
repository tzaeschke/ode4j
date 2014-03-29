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
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.threading.DThreadingFunctionsInfo.DCallReleasee;

public class DxStepperProcessingCallContext {

	//typedef void (*dstepper_fn_t) (const dxStepperProcessingCallContext *callContext);
	public interface dstepper_fn_t {
		public void run(DxStepperProcessingCallContext callContext);
	}
	//typedef unsigned (*dmaxcallcountestimate_fn_t) (unsigned activeThreadCount, unsigned allowedThreadCount);
	public interface dmaxcallcountestimate_fn_t {
		public int run(int activeThreadCount, int allowedThreadCount);
	}

	
	
	DxStepperProcessingCallContext(DxWorld world, double stepSize, int stepperAllowedThreads, 
			DxWorldProcessMemArena stepperArena, DxBody[] islandBodiesStart, DxJoint[] islandJointsStart) {
		m_world = world;
		m_stepSize = stepSize;
		m_stepperArena = stepperArena;
		m_finalReleasee = null; 
		m_islandBodiesStart = islandBodiesStart;
		m_islandJointsStart = islandJointsStart;
		m_islandBodiesCount = 0;
		m_islandJointsCount = 0;
		m_stepperAllowedThreads = stepperAllowedThreads;
	}

	void AssignIslandSelection(DxBody[] islandBodiesStart, DxJoint[] islandJointsStart, 
			int islandBodiesCount, int islandJointsCount)
	{
		m_islandBodiesStart = islandBodiesStart;
		m_islandJointsStart = islandJointsStart;
		m_islandBodiesCount = islandBodiesCount;
		m_islandJointsCount = islandJointsCount;
	}

	DxBody[] GetSelectedIslandBodiesEnd() { return m_islandBodiesStart + m_islandBodiesCount; }
	DxJoint[] GetSelectedIslandJointsEnd() { return m_islandJointsStart + m_islandJointsCount; }

	void AssignStepperCallFinalReleasee(DCallReleasee finalReleasee)
	{
		m_finalReleasee = finalReleasee;
	}

	DxWorld            m_world;
	double             m_stepSize;
	DxWorldProcessMemArena  m_stepperArena;
	DCallReleasee         m_finalReleasee;
	DxBody[]           m_islandBodiesStart;
	DxJoint[]          m_islandJointsStart;
	int                m_islandBodiesCount;
	int                m_islandJointsCount;
	int                m_stepperAllowedThreads;

}
