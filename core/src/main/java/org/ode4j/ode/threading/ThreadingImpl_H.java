/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * Threading Windows implementation file.                                *
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

import org.ode4j.ode.threading.ThreadingFake.dxFakeLull;
import org.ode4j.ode.threading.ThreadingFake.dxFakeMutex;
import org.ode4j.ode.threading.ThreadingTemplates.dxtemplateJobListContainer;
import org.ode4j.ode.threading.ThreadingTemplates.dxtemplateJobListSelfHandlertemplate;
import org.ode4j.ode.threading.ThreadingTemplates.dxtemplateThreadingImplementation;
import org.ode4j.ode.threading.ThreadingTemplates.tJobListContainer;
import org.ode4j.ode.threading.ThreadingTemplates.tThreadLull;
import org.ode4j.ode.threading.ThreadingTemplates.tThreadMutex;
import org.ode4j.ode.threading.ThreadingTemplates.tThreadWakeup;

public class ThreadingImpl_H {

	/************************************************************************/
	/* Self-threaded job list definition                                    */
	/************************************************************************/

	//typedef dxtemplateJobListContainer<dxFakeLull, dxFakeMutex, dxFakeAtomicsProvider> dxSelfThreadedJobListContainer;
	//typedef dxtemplateJobListSelfHandler<dxSelfWakeup, dxSelfThreadedJobListContainer> dxSelfThreadedJobListHandler;
	//typedef dxtemplateThreadingImplementation<dxSelfThreadedJobListContainer, dxSelfThreadedJobListHandler> dxSelfThreadedThreading;
	public static class dxSelfThreadedJobListContainer extends dxtemplateJobListContainer{};//<dxFakeLull, dxFakeMutex> {};//, dxFakeAtomicsProvider> {};
	public static class dxSelfThreadedJobListHandler extends dxtemplateJobListSelfHandlertemplate{};//<dxSelfWakeup, dxSelfThreadedJobListContainer> {};
	public static class dxSelfThreadedThreading extends dxtemplateThreadingImplementation{};//<dxSelfThreadedJobListContainer, dxSelfThreadedJobListHandler> {};

	//TZ
	public static ThreadFactory THREAD_FACTORY = new ThreadFactory() {
		@Override
		public tThreadMutex createThreadMutex() {
			return new dxFakeMutex();
		}
		@Override
		public tThreadLull createThreadLull() {
			return new dxFakeLull();
		}
		@Override
		public tJobListContainer createJobListContainer() {
			return new dxSelfThreadedJobListContainer();
		}
		@Override
		public tThreadWakeup createThreadWakeup() {
			return new ThreadingFake.dxSelfWakeup();
		}
	};
	public static interface ThreadFactory {
		public tThreadMutex createThreadMutex();
		public tThreadLull createThreadLull();
		//public tThreadMutex createThreadMutex();
		public tJobListContainer createJobListContainer();
		public tThreadWakeup createThreadWakeup();
	}

//	#if dBUILTIN_THREADING_IMPL_ENABLED

	/************************************************************************/
	/* Multi-threaded job list definition                                   */
	/************************************************************************/

//	typedef dxtemplateJobListContainer<dxtemplateThreadedLull<dxEventWakeup, dxOUAtomicsProvider, false>, dxCriticalSectionMutex, dxOUAtomicsProvider> dxMultiThreadedJobListContainer;
//	typedef dxtemplateJobListThreadedHandler<dxEventWakeup, dxMultiThreadedJobListContainer> dxMultiThreadedJobListHandler;
//	typedef dxtemplateThreadingImplementation<dxMultiThreadedJobListContainer, dxMultiThreadedJobListHandler> dxMultiThreadedThreading;
//	typedef dxtemplateJobListContainer<dxtemplateThreadedLull<dxEventWakeup, dxOUAtomicsProvider, false>, dxCriticalSectionMutex, dxOUAtomicsProvider> dxMultiThreadedJobListContainer;
//	typedef dxtemplateJobListThreadedHandler<dxEventWakeup, dxMultiThreadedJobListContainer> dxMultiThreadedJobListHandler;
//	typedef dxtemplateThreadingImplementation<dxMultiThreadedJobListContainer, dxMultiThreadedJobListHandler> dxMultiThreadedThreading;

//	#endif // #if dBUILTIN_THREADING_IMPL_ENABLED

	
}
