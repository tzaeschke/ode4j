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
import org.ode4j.ode.threading.Threading_H.DCallReleasee;

public final class DxStepperProcessingCallContext {

	//typedef void (*dstepper_fn_t) (const dxStepperProcessingCallContext *callContext);
	public interface dstepper_fn_t {
		public void run(DxStepperProcessingCallContext callContext);
	}
	//typedef unsigned (*dmaxcallcountestimate_fn_t) (unsigned activeThreadCount, unsigned allowedThreadCount);
	public interface dmaxcallcountestimate_fn_t {
		public int run(int activeThreadCount, int allowedThreadCount);
	}

	
	
	DxStepperProcessingCallContext(DxWorld world, double stepSize, int stepperAllowedThreads, 
			DxWorldProcessMemArena stepperArena, 
			DxBody[] islandBodiesStart,
			DxJoint[] islandJointsStart) {
		m_world = world;
		m_stepSize = stepSize;
		m_stepperArena = stepperArena;
		m_finalReleasee = null; 
		m_islandBodiesStartA = islandBodiesStart;
		m_islandBodiesStartOfs = 0;
		m_islandJointsStartA = islandJointsStart;
		m_islandJointsStartOfs = 0;
		m_islandBodiesCount = 0;
		m_islandJointsCount = 0;
		m_stepperAllowedThreads = stepperAllowedThreads;
	}

	void AssignIslandSelection(DxBody[] islandBodiesStartA, int islandBodiesStartOfs,
			DxJoint[] islandJointsStartA, int islandJointsStartOfs,
			int islandBodiesCount, int islandJointsCount)
	{
		m_islandBodiesStartA = islandBodiesStartA;
		m_islandBodiesStartOfs = islandBodiesStartOfs;
		m_islandJointsStartA = islandJointsStartA;
		m_islandJointsStartOfs = islandJointsStartOfs;
		m_islandBodiesCount = islandBodiesCount;
		m_islandJointsCount = islandJointsCount;
	}

	int/*DxBody[]*/ GetSelectedIslandBodiesEnd() { return m_islandBodiesStartOfs + m_islandBodiesCount; }
	int/*DxJoint[]*/ GetSelectedIslandJointsEnd() { return m_islandJointsStartOfs + m_islandJointsCount; }

	void AssignStepperCallFinalReleasee(DCallReleasee finalReleasee)
	{
		m_finalReleasee = finalReleasee;
	}

	private DxWorld            m_world;
	private double             m_stepSize;
	private DxWorldProcessMemArena  m_stepperArena;
	private DCallReleasee         m_finalReleasee;
	private DxBody[]           m_islandBodiesStartA;
	private int         	   m_islandBodiesStartOfs;
	private DxJoint[]          m_islandJointsStartA;
	private int  		       m_islandJointsStartOfs;
	private int                m_islandBodiesCount;
	private int                m_islandJointsCount;
	private int                m_stepperAllowedThreads;
	
	public DxWorldProcessMemArena m_stepperArena() {
		return m_stepperArena;
	}

	public DxWorld m_world() {
		return m_world;
	}

	public int m_islandBodiesCount() {
		return m_islandBodiesCount;
	}

	public int m_islandJointsCount() {
		return m_islandJointsCount;
	}

	public int m_stepperAllowedThreads() {
		return m_stepperAllowedThreads;
	}

	public DCallReleasee m_finalReleasee() {
		return m_finalReleasee;
	}

	public DxBody[] m_islandBodiesStartA() {
		return m_islandBodiesStartA;
	}

	public int m_islandBodiesStartOfs() {
		return m_islandBodiesStartOfs;
	}

	public DxJoint[] m_islandJointsStartA() {
		return m_islandJointsStartA;
	}

	public int m_islandJointsStartOfs() {
		return m_islandJointsStartOfs;
	}

	public double m_stepSize() {
		return m_stepSize;
	}

}
