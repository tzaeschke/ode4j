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
package org.ode4j.ode;



/**
 * From the Wiki:
 * <p> 
 * The fixed joint maintains a fixed relative position and orientation between
 * two bodies, or between a body and the static environment. Using this joint
 * is almost never a good idea in practice, except when debugging. 
 * If you need two bodies to be glued together it is better to represent that 
 * as a single body. 
 */
public interface DFixedJoint extends DJoint {

	void set();


	/**
	 * Call this on the fixed joint after it has been attached to
	 * remember the current desired relative offset and desired relative
	 * rotation between the bodies.
	 */
	void setFixed();

	
	/**
	 * Sets joint parameter.
	 */
	@Override
	void setParam (PARAM_N parameter, double value);
	
	
	/**
	 * Get joint parameter.
	 */
	@Override
	double getParam (PARAM_N type);

}
