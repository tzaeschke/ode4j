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

import org.ode4j.ode.DHeightfield.DHeightfieldGetHeight;

public interface DHeightfieldData {
	
	/**
	 * @brief Configures a dHeightfieldData to use a callback to
	 * retrieve height data.
	 *
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is computed by
	 * the user and it should use the given callback when determining
	 * the height of a given element of it's shape.
	 *
	 * @param d A new dHeightfieldData created by dGeomHeightfieldDataCreate
	 *
	 * @param width Specifies the total 'width' of the heightfield along
	 * the geom's local x axis.
	 * @param depth Specifies the total 'depth' of the heightfield along
	 * the geom's local z axis.
	 *
	 * @param widthSamples Specifies the number of vertices to sample
	 * along the width of the heightfield. Each vertex has a corresponding
	 * height value which forms the overall shape.
	 * Naturally this value must be at least two or more.
	 * @param depthSamples Specifies the number of vertices to sample
	 * along the depth of the heightfield.
	 *
	 * @param scale A uniform scale applied to all raw height data.
	 * @param offset An offset applied to the scaled height data.
	 *
	 * @param thickness A value subtracted from the lowest height
	 * value which in effect adds an additional cuboid to the base of the
	 * heightfield. This is used to prevent geoms from looping under the
	 * desired terrain and not registering as a collision. Note that the
	 * thickness is not affected by the scale or offset parameters.
	 *
	 * @param bWrap If non-zero the heightfield will infinitely tile in both
	 * directions along the local x and z axes. If zero the heightfield is
	 * bounded from zero to width in the local x axis, and zero to depth in
	 * the local z axis.
	 *
	 * @ingroup collide
	 */
	void buildCallback(
				Object[] pUserData, DHeightfieldGetHeight pCallback,
				double width, double depth, int widthSamples, int depthSamples,
				double scale, double offset, double thickness, boolean bWrap );

	/**
	 * @brief Destroys a dHeightfieldData.
	 *
	 * Deallocates a given dHeightfieldData and all managed resources.
	 *
	 * @param d A dHeightfieldData created by dGeomHeightfieldDataCreate
	 * @ingroup collide
	 */
	void destroy();

	/**
	 * @brief Manually set the minimum and maximum height bounds.
	 *
	 * This call allows you to set explicit min / max values after initial
	 * creation typically for callback heightfields which default to +/- infinity,
	 * or those whose data has changed. This must be set prior to binding with a
	 * geom, as the the AABB is not recomputed after it's first generation.
	 *
	 * @remarks The minimum and maximum values are used to compute the AABB
	 * for the heightfield which is used for early rejection of collisions.
	 * A close fit will yield a more efficient collision check.
	 *
	 * @param d A dHeightfieldData created by dGeomHeightfieldDataCreate
	 * @param min_height The new minimum height value. Scale, offset and thickness is then applied.
	 * @param max_height The new maximum height value. Scale and offset is then applied.
	 * @ingroup collide
	 */
	void setBounds( DHeightfieldData d, double minHeight, double maxHeight );
}
