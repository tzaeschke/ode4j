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

import org.ode4j.math.DVector3;


/**
 * Describe the contact point between two geoms.
 *
 * If two bodies touch, or if a body touches a static feature in its 
 * environment, the contact is represented by one or more "contact 
 * points", described by dContactGeom.
 *
 * The convention is that if body 1 is moved along the normal vector by 
 * a distance depth (or equivalently if body 2 is moved the same distance 
 * in the opposite direction) then the contact depth will be reduced to 
 * zero. This means that the normal vector points "in" to body 1.
 */
public class DContactGeom {
	
	/** contact position */
	public final DVector3 pos = new DVector3();          
    /** normal vector */
	public final DVector3 normal = new DVector3();
    /** penetration depth */
	public double depth;
	/** the colliding geoms */
	public DGeom g1;        
	public DGeom g2;
	/** (to be documented) */
	public int side1;       
	public int side2;
}
