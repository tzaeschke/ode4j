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
package org.ode4j.ode.internal;


import static org.ode4j.ode.internal.cpp4j.Cstdio.*;

import org.ode4j.ode.DStopwatch;
import org.ode4j.ode.internal.cpp4j.FILE;

/**
 * TODO
 * ----
 * 
 * - gettimeofday() and the pentium time stamp counter return the real time,
 *   not the process time. fix this somehow!
 *
 */
public class Timer {


	//******************** JAVA implementation by TZ, based on time-of-day implementation

	//	private static void getClockCount (long[] cc)
	//	{
	//		long ms = System.nanoTime();//TimeMillis();
	//		//  cc[1] = ms.lo / 1000000;
	//		//  cc[0] = ms.lo - ( cc[1] * 1000000 );
	//	}
	private static long getClockCount ()
	{
		return System.nanoTime();//TimeMillis();
	}



	private static void serialize()
	{
	}


	//	private static double loadClockCount (long[] a)
	//	{
	//		return a[1]*1.0e6 + a[0];
	//	}
	private static double loadClockCount (long a)
	{
		return a;
	}


	public static double dTimerResolution()
	{
		//		long cc1[] = new long[2], cc2[] = new long[2];
		//		getClockCount (cc1);
		//		do {
		//			getClockCount (cc2);
		//		}
		//		while (cc1[0]==cc2[0] && cc1[1]==cc2[1]);
		//		do {
		//			getClockCount (cc1);
		//		}
		//		while (cc1[0]==cc2[0] && cc1[1]==cc2[1]);
		//		double t1 = loadClockCount (cc1);
		//		double t2 = loadClockCount (cc2);
		long cc2;
		long cc1 = System.nanoTime();
		do {
			cc2 = System.nanoTime(); 
		} while (cc1 == cc2);
		do {
			cc1 = System.nanoTime(); 
		} while (cc1 == cc2);

		return (cc1-cc2) / dTimerTicksPerSecond();
	}


	public static double dTimerTicksPerSecond()
	{
		return 1000000000;
	}


	// ***************** End of TZ JAVA implementation



	//****************************************************************************
	// stop watches

	public static void dStopwatchReset (DStopwatch s)
	{
		s.reset();
		//		s.time = 0;
		//		s.cc = 0;
		////		s.cc[0] = 0;
		////		s.cc[1] = 0;
	}


	public static void dStopwatchStart (DStopwatch s)
	{
		s.start();
		//		serialize();
		//		s.cc = getClockCount ();
	}


	public static void dStopwatchStop  (DStopwatch s)
	{
		//		//long[] cc =new long[2];
		//		serialize();
		//		long cc = getClockCount ();
		//		double t1 = loadClockCount (s.cc);
		//		double t2 = loadClockCount (cc);
		//		s.time += t2-t1;
		s.stop();
	}


	public static double dStopwatchTime (DStopwatch s)
	{
		//		return s.time / dTimerTicksPerSecond();
		return s.getTime();
	}

	//****************************************************************************
	// code timers

	// maximum number of events to record
	//#define MAXNUM 100
	private static final int MAXNUM = 100;

	private static int num = 0;		// number of entries used in event array
	private static class Event {
		//long[] cc = new long[2];		// clock counts
		long cc;
		double total_t = 0;		// total clocks used in this slot.
		double total_p = 0;		// total percentage points used in this slot.
		int count = 0;			// number of times this slot has been updated.
		String description;		// pointer to static string
	}
	private static final Event[] event = new Event[MAXNUM];
	static {
		for (int i = 0; i < event.length; i++)
			event[i] = new Event();
	}


	// make sure all slot totals and counts reset to 0 at start

	//	static void initSlots()
	//	{
	//		static int initialized=0;
	//		if (!initialized) {
	//			for (int i=0; i<MAXNUM; i++) {
	//				event[i].count = 0;
	//				event[i].total_t = 0;
	//				event[i].total_p = 0;
	//			}
	//			initialized = 1;
	//		}
	//}  TODO ??


	public static void dTimerStart (final String description)
	{
		//			initSlots();
		event[0].description = description;//const_cast<char*> (description);
		num = 1;
		serialize();
		event[0].cc = getClockCount ();
	}


	public static void dTimerNow (final String description)
	{
		if (num < MAXNUM) {
			// do not serialize
			event[num].cc = getClockCount ();
			event[num].description = description;//const_cast<char*> (description);
			num++;
		}
	}


	public static void dTimerEnd()
	{
		if (num < MAXNUM) {
			serialize();
			event[num].cc = getClockCount ();
			event[num].description = "TOTAL";
			num++;
		}
	}

	//****************************************************************************
	// print report

	//static void fprintDoubleWithPrefix (FILE *f, double a, const char *fmt)
	private static void fprintDoubleWithPrefix (FILE f, double a, final String fmt)
	{
		if (a >= 0.999999) {
			fprintf (f,fmt,a);
			return;
		}
		a *= 1000.0;
		if (a >= 0.999999) {
			fprintf (f,fmt,a);
			fprintf (f,"m");
			return;
		}
		a *= 1000.0;
		if (a >= 0.999999) {
			fprintf (f,fmt,a);
			fprintf (f,"u");
			return;
		}
		a *= 1000.0;
		fprintf (f,fmt,a);
		fprintf (f,"n");
	}


	public static void dTimerReport (FILE fout, int average)
	{
		int i;
		int maxl;//size_t maxl;
		double ccunit = 1.0/dTimerTicksPerSecond();
		fprintf (fout,"\nTimer Report (");
		fprintDoubleWithPrefix (fout,ccunit,"%.2f ");
		fprintf (fout,"s resolution)\n------------\n");
		if (num < 1) return;

		// get maximum description length
		maxl = 0;
		for (i=0; i<num; i++) {
			//size_t l = strlen (event[i].description);
			int l = event[i].description.length();
			if (l > maxl) maxl = l;
		}

		// calculate total time
		double tt1 = loadClockCount (event[0].cc);
		double tt2 = loadClockCount (event[num-1].cc);
		double total = tt2 - tt1;
		if (total <= 0) total = 1;

		// compute time difference for all slots except the last one. update totals
		double []times = new double[num];//(double*) ALLOCA (num * sizeof(double));
		for (i=0; i < (num-1); i++) {
			double t1 = loadClockCount (event[i].cc);
			double t2 = loadClockCount (event[i+1].cc);
			times[i] = t2 - t1;
			event[i].count++;
			event[i].total_t += times[i];
			event[i].total_p += times[i]/total * 100.0;
		}

		// print report (with optional averages)
		for (i=0; i<num; i++) {
			double t,p;
			if (i < (num-1)) {
				t = times[i];
				p = t/total * 100.0;
			}
			else {
				t = total;
				p = 100.0;
			}
			//fprintf (fout,"%-*s %7.2fms %6.2f%%",(int)maxl,event[i].description, //TODO -*
			fprintf (fout,"%s %7.2fms %6.2f%%",event[i].description,  //TODO (int)maxl
					t*ccunit * 1000.0, p);
			if (average!=0 && i < (num-1)) {
				fprintf (fout,"  (avg %7.2fms %6.2f%%)",
						(event[i].total_t / event[i].count)*ccunit * 1000.0,
						event[i].total_p / event[i].count);
			}
			fprintf (fout,"\n");
		}
		fprintf (fout,"\n");
	}

	private Timer() {}
}
