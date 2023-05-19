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


/** 
 * (TZ) This is for internal use only. Please try to avoid using this in
 * user applications.
 * This comes from the `reuse' library. copy any changes back to the source. 
 */
public abstract class ErrorHandler {


	/** all user defined error functions have this type. error and debug functions
	 * should not return.
	 */
	//public void dMessageFunction (int errnum, String msg, va_list ap);
	public interface dMessageFunction {
		void call(int errnum, String msg, Object ... ap);
	}

	/** set a new error, debug or warning handler. if fn is 0, the default handlers
	 * are used.
	 * @param fn fn
	 */
	////ODE_API
	public static void dSetErrorHandler (dMessageFunction fn) {
		ErrorHdl.dSetErrorHandler(fn);
	}
	////ODE_API
	public static void dSetDebugHandler (dMessageFunction fn) {
		ErrorHdl.dSetDebugHandler(fn);
	}
	////ODE_API
	public static void dSetMessageHandler (dMessageFunction fn) {
		ErrorHdl.dSetMessageHandler(fn);
	}

	/**
	 * @return the current error, debug or warning handler. if the return value is
	 * 0, the default handlers are in place.
	 */
	//ODE_API
	public static dMessageFunction dGetErrorHandler() {
		return ErrorHdl.dGetErrorHandler();
	}
	/** 
	 * @return the current error, debug or warning handler. if the return value is
	 * 0, the default handlers are in place.
	 */
	public static  dMessageFunction dGetDebugHandler() {
		return ErrorHdl.dGetDebugHandler();
	}
	/** 
	 * @return the current error, debug or warning handler. if the return value is
	 * 0, the default handlers are in place.
	 */
	public static  dMessageFunction dGetMessageHandler() {
		return ErrorHdl.dGetMessageHandler();
	}

	/** 
	 * generate a fatal error, debug trap or a message. 
	 * @param num error number
	 * @param msg message
	 * @param ap objects 
	 */
	//ODE_API
	public static void dError (int num, final String msg, Object ... ap) {
		ErrorHdl.dError(num, msg, ap);
	}
	/** 
	 * generate a fatal error, debug trap or a message. 
	 * @param num error number
	 * @param msg message
	 * @param ap objects 
	 */
	public static void dDebug (int num, final String msg, Object ... ap) {
		ErrorHdl.dDebug(num, msg, ap);
	}
	/** 
	 * generate a fatal error, debug trap or a message. 
	 * @param num error number
	 * @param msg message
	 * @param ap objects 
	 */
	public static void dMessage (int num, final String msg, Object ... ap) {
		ErrorHdl.dMessage(num, msg, ap);
	}

	private ErrorHandler() {}
}

