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
package org.ode4j.ode.threading;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class MutexGroupImpl implements MutexGroup
{
	final Lock[] m_Mutex_array;

	public MutexGroupImpl() {
		m_Mutex_array = new Lock[dxProcessContextMutex.values().length];
		for (int i = 0; i < m_Mutex_array.length; ++i) {
			m_Mutex_array[i] = new ReentrantLock();
		}
	}

	public void lock(dxProcessContextMutex mutex_index) { 
		m_Mutex_array[mutex_index.ordinal()].lock(); 
	}

	public boolean tryLock(dxProcessContextMutex mutex_index) { 
		return m_Mutex_array[mutex_index.ordinal()].tryLock(); 
	}

	public void unlock(dxProcessContextMutex mutex_index) { 
		m_Mutex_array[mutex_index.ordinal()].unlock(); 
	}

}