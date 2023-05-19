/** ***********************************************************************
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
 ************************************************************************ */
package org.ode4j.cpp;

import org.ode4j.cpp.internal.ApiCppWorld;

/**
 * This class represents the static API of the C/C++ version of ODE.
 * It is very close to the actual documentation.
 * <p>
 * There are two ways of using this class:<br>
 * a) Using <tt>import static org.ode4j.cpp.ODE_API.*</tt> gives direct 
 *    access to all methods, for example one can simply call 
 *    <tt>dWorld wId = dWorldCreate();</tt> .<br>
 * 
 * b) Using <tt>extends ODE_API</tt> gives direct access to all methods, 
 *    for example one can simply call <tt>dWorld wId = dWorldCreate();</tt> .<br>
 * 
 * c) Using <tt>private static final Ode ODE = Ode.getInstance();</tt> in a class
 *    allows extending other classes instead. The above example then looks like:
 *    <tt> dWorld wId = ODE.dWorldCreate();</tt> .<br>
 *    
 * <p>   
 * These approaches are not quit object oriented, but they are close to the
 * existing documentation and they simplify porting code from C/C++ to Java.
 * <p>
 * For new applications it is recommended to use the object oriented API and avoid
 * this class altogether.   
 * 
 * @author Tilmann Zaeschke, based on ODE for C/C++
 */

@Deprecated // To be removed in 0.6.0
public class OdeCpp extends ApiCppWorld {
	
	protected OdeCpp() {
		//protected
		super();
	}
}