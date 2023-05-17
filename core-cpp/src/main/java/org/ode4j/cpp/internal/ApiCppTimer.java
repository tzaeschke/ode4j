package org.ode4j.cpp.internal;

import org.ode4j.ode.DStopwatch;
import org.ode4j.ode.internal.Timer;
import org.ode4j.ode.internal.cpp4j.FILE;

/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
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

/** stop watch objects */

public abstract class ApiCppTimer {

//ODE_API 
void dStopwatchReset (DStopwatch sw) {
	Timer.dStopwatchStart(sw);
}
//ODE_API 
 void dStopwatchStart (DStopwatch sw) {
	 Timer.dStopwatchStart(sw);
 }
//ODE_API 
 void dStopwatchStop  (DStopwatch sw) {
	 Timer.dStopwatchStop(sw);
 }
 /** returns total time in secs */
//ODE_API 
 double dStopwatchTime (DStopwatch sw) {
	 return Timer.dStopwatchTime(sw);
 }


/** code timers */
/** 
 * pass a static string here 
 * @param description description
 */
//ODE_API 
public static void dTimerStart (final String description) {
	Timer.dTimerStart(description);
}
 /** 
  * pass a static string here. 
 * @param description description
 */
//ODE_API 
public static void dTimerNow (final String description) {
	Timer.dTimerNow(description);
}

//ODE_API 
public static void dTimerEnd() {
	 Timer.dTimerEnd();
 }

/** 
 * print out a timer report. if `average' is nonzero, print out the average
 * time for each slot (this is only meaningful if the same start-now-end
 * calls are being made repeatedly.
 * @param fout file out
 * @param average average
 */
//ODE_API 
public static void dTimerReport (FILE fout, int average) {
	Timer.dTimerReport(fout, average);
}


/* resolution */

/** returns the timer ticks per second implied by the timing hardware or API.
 * the actual timer resolution may not be this great.
 */
//ODE_API 
 double dTimerTicksPerSecond() {
	 return Timer.dTimerTicksPerSecond();
 }

/** returns an estimate of the actual timer resolution, in seconds. this may
 * be greater than 1/ticks_per_second.
 */
//ODE_API 
 double dTimerResolution() {
	 return Timer.dTimerResolution();
 }

 protected ApiCppTimer() {}
}
