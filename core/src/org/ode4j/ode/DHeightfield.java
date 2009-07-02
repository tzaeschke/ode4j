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
package org.ode4j.ode;

public interface DHeightfield extends DGeom {
	/**
	 * @brief Callback prototype
	 *
	 * Used by the callback heightfield data type to sample a height for a
	 * given cell position.
	 *
	 * @ingroup collide
	 */
	//typedef double dHeightfieldGetHeight( void* p_user_data, int x, int z ) {
	public static interface DHeightfieldGetHeight {
		/**
		 * @brief Callback prototype
		 *
		 * Used by the callback heightfield data type to sample a height for a
		 * given cell position.
		 *
		 * @param pUserDdata User data specified when creating the dHeightfieldData
		 * @param x The index of a sample in the local x axis. It is a value
		 * in the range zero to ( nWidthSamples - 1 ).
		 * @param z The index of a sample in the local z axis. It is a value
		 * in the range zero to ( nDepthSamples - 1 ).
		 *
		 * @return The sample height which is then scaled and offset using the
		 * values specified when the heightfield data was created.
		 *
		 * @ingroup collide
		 */
		public double call( Object[] pUserData, int x, int z );
	}
}
