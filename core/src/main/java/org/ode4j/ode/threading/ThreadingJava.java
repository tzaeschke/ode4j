/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * Threading implementation templates file.                              *
 * Copyright (C) 2011-2012 Oleh Derevenko. All rights reserved.          *
 * e-mail: odar@eleks.com (change all "a" to "e")                        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.threading;

import static org.ode4j.ode.internal.Common.*;

import java.util.List;
import java.util.concurrent.Semaphore;

import org.ode4j.ode.threading.ThreadingTemplates.tThreadLull;
import org.ode4j.ode.threading.ThreadingTemplates.tThreadMutex;
import org.ode4j.ode.threading.ThreadingTemplates.tThreadWakeup;
import org.ode4j.ode.threading.Threading_H.DThreadedWaitTime;

/*
 *  Self-wakeup implementation for built-in threading support provider.
 *  Fake mutex implementation for built-in threading support provider.
 *
 *  The classes have been moved into a separate header as they are to be used 
 *  in both WIN and POSIX implementations.
 */
public class ThreadingJava {

	/************************************************************************/
	/* dxSelfWakeup class definition                                        */
	/************************************************************************/

	static class dxSelfWakeup implements tThreadWakeup
	{
	//public:
	    public dxSelfWakeup() {
	        m_wakeup_state = false;
	        m_state_is_permanent = false;
	    }

	    @Override
		public boolean InitializeObject() { return true; }

	//public:
	    @Override
		public void ResetWakeup() { m_wakeup_state = false; m_state_is_permanent = false; }
	    public void WakeupAThread() { dIASSERT(!m_state_is_permanent); m_wakeup_state = true; } // Wakeup should not be used after permanent signal
	    @Override
		public void WakeupAllThreads() { m_wakeup_state = true; m_state_is_permanent = true; }

	    //public boolean WaitWakeup(const dThreadedWaitTime *timeout_time_ptr);

	//private:
	    private boolean          m_wakeup_state;
	    private boolean          m_state_is_permanent;
//	};


	@Override
	public
	boolean WaitWakeup(final DThreadedWaitTime timeout_time_ptr)
	{
	    //(void)timeout_time_ptr; // unused
	    boolean wait_result = m_wakeup_state;

	    if (m_wakeup_state)
	    {
	        m_wakeup_state = m_state_is_permanent;
	    }
	    else
	    {
	        dICHECK(false); // Self-wakeup should only be used in cases when waiting is called after object is signaled
	    }

	    return wait_result;
	}
	}

	/************************************************************************/
	/* Java mutex class implementation                                      */
	/************************************************************************/

	static class dxFakeMutex implements tThreadMutex {
		private final Semaphore s = new Semaphore(1);
	
		public dxFakeMutex() {}

		@Override
	    public boolean InitializeObject() { return true; }

		@Override
	    public void LockMutex() { 
			try {
				s.acquire();
			} catch (InterruptedException e) {
				throw new RuntimeException(e);
			} 
		}
		
		@Override
	    public boolean TryLockMutex() {
			return s.tryAcquire(); 
		}
		@Override
	    public void UnlockMutex() { 
			s.release(); 
		}

		@Override
		public void DESTRUCTOR() {
			//
		}
	};


	/************************************************************************/
	/* Fake lull class implementation                                      */
	/************************************************************************/

	static class dxFakeLull implements tThreadLull {
		
		List<Thread> threads;
	//public:
		public dxFakeLull() {}

		@Override
		public boolean InitializeObject() { 
			return true; 
		}

	//public:
		@Override
		public void RegisterToLull() { 
			/* Do nothing */ 
		}
		
		@Override
		public void WaitForLullAlarm() { 
			dICHECK(false); 
		} // Fake lull can't be waited
		
		@Override
		public void UnregisterFromLull() { 
			/* Do nothing */ 
		}

		@Override
		public void SignalLullAlarmIfAnyRegistrants() { 
			/* Do nothing */ 
		}
	};


//	#endif // #ifndef _ODE_THREADING_FAKE_SYNC_H_

}
