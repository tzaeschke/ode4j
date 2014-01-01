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

public interface DHeightfield extends DGeom {
	/**
	 * Callback prototype.
	 * <p>
	 * Used by the callback heightfield data type to sample a height for a
	 * given cell position.
	 */
	//typedef double dHeightfieldGetHeight( void* p_user_data, int x, int z ) {
	public static interface DHeightfieldGetHeight {
		/**
		 * Callback prototype.
		 * <p>
		 * Used by the callback heightfield data type to sample a height for a
		 * given cell position.
		 *
		 * @param pUserData User data specified when creating the dHeightfieldData
		 * @param x The index of a sample in the local x axis. It is a value
		 * in the range zero to ( nWidthSamples - 1 ).
		 * @param z The index of a sample in the local z axis. It is a value
		 * in the range zero to ( nDepthSamples - 1 ).
		 *
		 * @return The sample height which is then scaled and offset using the
		 * values specified when the heightfield data was created.
		 */
		public double call( Object pUserData, int x, int z );
	}
	
	/**
	 * Assigns a dHeightfieldData to a heightfield geom.
	 * <p>
	 * Associates the given dHeightfieldData with a heightfield geom.
	 * This is done without affecting the GEOM_PLACEABLE flag.
	 *
	 * @param d A dHeightfieldData created by dGeomHeightfieldDataCreate
	 */
	void setHeightfieldData( DHeightfieldData d );


	/**
	 * Gets the dHeightfieldData bound to a heightfield geom.
	 * <p>
	 * Returns the dHeightfieldData associated with a heightfield geom.
	 *
	 * @return The dHeightfieldData which may be NULL if none was assigned.
	 */
	DHeightfieldData getHeightfieldData();

}
