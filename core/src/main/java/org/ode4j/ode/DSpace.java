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
 * collision space.
 */
public interface DSpace extends DGeom {

	void setCleanup (boolean mode);
	boolean getCleanup();

	/**
	 * Turn all dirty geoms into clean geoms by computing their AABBs and any
	 * other space data structures that are required. this should clear the
	 * GEOM_DIRTY and GEOM_AABB_BAD flags of all geoms.
	 */
	void cleanGeoms();

	void add (DGeom x);
	void remove (DGeom x);
	boolean query (DGeom x);

	int getNumGeoms();
	/**
	 *  
	 * @param i geom id
	 * @return geometry object
	 * @deprecated Use getGeoms() instead 
	 */
	@Deprecated
	DGeom getGeom (int i);
	Iterable<DGeom> getGeoms();

	/** 
	 * This is equivalent to OdeHelper.spaceCollide(...) 
	 * @param data data 
	 * @param callback callback
	 */
	void collide (Object data, DNearCallback callback);
	
	
	/**
	 * Sets manual cleanup flag for a space.
	 * <p>
	 * Manual cleanup flag marks a space as eligible for manual thread data cleanup.
	 * This function should be called for every space object right after creation in 
	 * case if ODE has been initialized with <tt>dInitFlagManualThreadCleanup</tt> flag.
	 * <p>
	 * Failure to set manual cleanup flag for a space may lead to some resources 
	 * remaining leaked until the program exit.
	 *
	 * @param mode 1 for manual cleanup mode and 0 for default cleanup mode
	 * @see #setManualCleanup(int)
	 * @see OdeHelper#initODE2(int)
	 */
	void setManualCleanup(int mode);

	
	/**
	 * Get manual cleanup flag of a space.
	 * <p>
	 * Manual cleanup flag marks a space space as eligible for manual thread data cleanup.
	 * See <tt>setManualCleanup</tt> for more details.
	 * 
	 * @return 1 for manual cleanup mode and 0 for default cleanup mode of the space
	 * @see #setManualCleanup(int)
	 * @see OdeHelper#initODE2(int)
	 */
	int getManualCleanup();

	/**
	 * Sets sublevel value for a space.
	 * <p>
	 * Sublevel affects how the space is handled in spaceCollide2 when it is collided
	 * with another space. If sublevels of both spaces match, the function iterates 
	 * geometries of both spaces and collides them with each other. If sublevel of one
	 * space is greater than the sublevel of another one, only the geometries of the 
	 * space with greater sublevel are iterated, another space is passed into 
	 * collision callback as a geometry itself. By default all the spaces are assigned
	 * zero sublevel.
	 *
	 * <p>NOTE:
	 * The space sublevel <b> IS NOT </b> automatically updated when one space is inserted
	 * into another or removed from one. It is a client's responsibility to update sublevel
	 * value if necessary.
	 *
	 * @param sublevel the sublevel value to be assigned
	 * @see #getSublevel()
	 * @see OdeHelper#spaceCollide2(DGeom, DGeom, Object, org.ode4j.ode.DGeom.DNearCallback)
	 */
	void setSublevel (int sublevel);

	/**
	 * Gets sublevel value of a space.
	 * <p>
	 * Sublevel affects how the space is handled in spaceCollide2 when it is collided
	 * with another space. See <tt>setSublevel</tt> for more details.
	 *
	 * @return the sublevel value of the space
	 * @see #setSublevel(int)
	 * @see OdeHelper#spaceCollide2(DGeom, DGeom, Object, org.ode4j.ode.DGeom.DNearCallback)
	 */
	int getSublevel ();
}
