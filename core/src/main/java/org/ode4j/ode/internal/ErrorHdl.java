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

import org.ode4j.ode.internal.ErrorHandler.dMessageFunction;


/**
 *
 * Error handlers.
 */
public class ErrorHdl {

	/**
	 * @author Tilmann Zaeschke
	 */
	public static class ErrorJump extends RuntimeException {
		private static final long serialVersionUID = 1L;

	}


	private static dMessageFunction error_function = null;
	private static dMessageFunction debug_function = null;
	private static dMessageFunction message_function = null;


	//extern "C" 
	/**
	 * set a new error, debug or warning handler. if fn is 0, the default handlers
	 * are used.
	 */
	public static void dSetErrorHandler (dMessageFunction fn)
	{
		error_function = fn;
	}


	//extern "C" 
	/**
	 * set a new error, debug or warning handler. if fn is 0, the default handlers
	 * are used.
	 */
	public static void dSetDebugHandler (dMessageFunction fn)
	{
		debug_function = fn;
	}


	//extern "C" 
	/**
	 * set a new error, debug or warning handler. if fn is 0, the default handlers
	 * are used.
	 */
	public static void dSetMessageHandler (dMessageFunction fn)
	{
		message_function = fn;
	}


	//extern "C" 
	/**
	 *  return the current error, debug or warning handler. if the return value is
	 * 0, the default handlers are in place.
	 */
	public static dMessageFunction dGetErrorHandler()
	{
		return error_function;
	}


	//extern "C" 
	/**
	 *  return the current error, debug or warning handler. if the return value is
	 * 0, the default handlers are in place.
	 */
	public static dMessageFunction dGetDebugHandler()
	{
		return debug_function;
	}


	//extern "C" 
	/**
	 *  return the current error, debug or warning handler. if the return value is
	 * 0, the default handlers are in place.
	 */
	public static dMessageFunction dGetMessageHandler()
	{
		return message_function;
	}


	//static void printMessage (int num, const char *msg1, const char *msg2,
	//		  va_list ap)
	private static void printMessage (int num, String msg1, String msg2,
			Object ... ap)
	{
		//	System.out.println();
		//	if (num != 0) {
		//		System.err.println(msg1 + " " + num + ": ");
		//	} else {
		//		System.err.println(msg1 + ": ");
		//	}
		//	String m = "";
		//	for (Object o: ap) {
		//		m += ap.toString();
		//	}
		//	System.out.println(msg2 + );
		System.err.flush();
		System.out.flush();
		if (num!=0) System.err.printf("\n%s %d: ",msg1,num);
		else System.err.printf("\n%s: ",msg1);
		System.err.printf(msg2,ap);
		System.err.println();
		System.err.flush();
		//	fflush (stderr);
		//	fflush (stdout);
		//	if (num!=0) fprintf (stderr,"\n%s %d: ",msg1,num);
		//	else fprintf (stderr,"\n%s: ",msg1);
		//	vfprintf (stderr,msg2,ap);
		//	fprintf (stderr,"\n");
		//	fflush (stderr);
	}

	//****************************************************************************
	// unix

	//#ifndef WIN32

	//extern "C" 
	/**
	 * generate a fatal error, debug trap or a message. 
	 */
	public static void dError (int num, String msg, Object ... ap)
	{
		//  va_list ap;
		//  va_start (ap,msg);
		if (error_function!=null) error_function.call (num,msg,ap);
		else printMessage (num,"ODE Error",msg,ap);
		//TZ:
		printMessage (num,"ODE Error",msg,ap);
		throw new RuntimeException("#"+num + ": " + msg);
		//System.exit (1);
	}


	//extern "C" 
	/**
	 * generate a fatal error, debug trap or a message. 
	 */
	public static void dDebug (int num, String msg, Object ... ap)
	{
		//  va_list ap;
		//  va_start (ap,msg);
		if (debug_function!=null) debug_function.call (num,msg,ap);
		else printMessage (num,"ODE INTERNAL ERROR",msg,ap);
		// *((char *)0) = 0;   ... commit SEGVicide
		//abort();
		//TZ:
		printMessage (num,"ODE INTERNAL ERROR",msg,ap);
		msg += " -> ";
		for (Object o:ap) msg += o.toString() + " , ";
		throw new RuntimeException("#"+num + ": " + msg);
		//  System.exit(2);
	}


	//extern "C" 
	/**
	 * generate a fatal error, debug trap or a message. 
	 */
	public static void dMessage (int num, String msg, Object ... ap)
	{
		//  va_list ap;
		//  va_start (ap,msg);
		if (message_function!=null) message_function.call (num,msg,ap);
		else printMessage (num,"ODE Message",msg,ap);
	}

	//#endif
}
//****************************************************************************
// windows

//#ifdef WIN32
//
//// isn't cygwin annoying!
//#ifdef CYGWIN
//#define _snprintf snprintf
//#define _vsnprintf vsnprintf
//#endif
//
//
//#include "windows.h"
//
//
//extern "C" void dError (int num, const char *msg, ...)
//{
//  va_list ap;
//  va_start (ap,msg);
//  if (error_function) error_function (num,msg,ap);
//  else {
//    char s[1000],title[100];
//    _snprintf (title,sizeof(title),"ODE Error %d",num);
//    _vsnprintf (s,sizeof(s),msg,ap);
//    s[sizeof(s)-1] = 0;
//    MessageBox(0,s,title,MB_OK | MB_ICONWARNING);
//  }
//  exit (1);
//}
//
//
//extern "C" void dDebug (int num, const char *msg, ...)
//{
//  va_list ap;
//  va_start (ap,msg);
//  if (debug_function) debug_function (num,msg,ap);
//  else {
//    char s[1000],title[100];
//    _snprintf (title,sizeof(title),"ODE INTERNAL ERROR %d",num);
//    _vsnprintf (s,sizeof(s),msg,ap);
//    s[sizeof(s)-1] = 0;
//    MessageBox(0,s,title,MB_OK | MB_ICONSTOP);
//  }
//  abort();
//}
//
//
//extern "C" void dMessage (int num, const char *msg, ...)
//{
//  va_list ap;
//  va_start (ap,msg);
//  if (message_function) message_function (num,msg,ap);
//  else printMessage (num,"ODE Message",msg,ap);
//}
//
//}
////#endif
