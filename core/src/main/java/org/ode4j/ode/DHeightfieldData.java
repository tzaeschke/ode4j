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

import org.ode4j.ode.DHeightfield.DHeightfieldGetHeight;

public interface DHeightfieldData {
	
	/**
	 * Configures a dHeightfieldData to use a callback to
	 * retrieve height data.
	 * <p>
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is computed by
	 * the user and it should use the given callback when determining
	 * the height of a given element of it's shape.
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
	 */
	void buildCallback(
				Object pUserData, DHeightfieldGetHeight pCallback,
				double width, double depth, int widthSamples, int depthSamples,
				double scale, double offset, double thickness, boolean bWrap );

	/**
	 * Destroys a dHeightfieldData.
	 * <p>
	 * Deallocates a given dHeightfieldData and all managed resources.
	 */
	void destroy();

	/**
	 * Manually set the minimum and maximum height bounds.
	 * <p>
	 * This call allows you to set explicit min / max values after initial
	 * creation typically for callback heightfields which default to +/- infinity,
	 * or those whose data has changed. This must be set prior to binding with a
	 * geom, as the the AABB is not recomputed after it's first generation.
	 * 
	 * <p> The minimum and maximum values are used to compute the AABB
	 * for the heightfield which is used for early rejection of collisions.
	 * A close fit will yield a more efficient collision check.
	 *
	 * @param minHeight The new minimum height value. Scale, offset and thickness is then applied.
	 * @param maxHeight The new maximum height value. Scale and offset is then applied.
	 */
	void setBounds( double minHeight, double maxHeight );

	/**
	 * Configures a dHeightfieldData to use height data in byte format.
	 * <p>
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is stored as a rectangular
	 * array of bytes (8 bit unsigned) representing the height at each sample point.
	 * <p>
	 * See build(double[], ...) for example code.
	 *
	 * @param pHeightData A pointer to the height data.
	 * @param bCopyHeightData When non-zero the height data is copied to an
	 * internal store. When zero the height data is accessed by reference and
	 * so must persist throughout the lifetime of the heightfield.
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
	 */
	void build(byte[] pHeightData,
			boolean bCopyHeightData, double width, double depth,
			int widthSamples, int depthSamples, double scale, double offset,
			double thickness, boolean bWrap);
	
	/**
	 * Configures a dHeightfieldData to use height data in short format.
	 * <p>
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is stored as a rectangular
	 * array of shorts (16 bit signed) representing the height at each sample point.
	 * <p>
	 * See build(double[], ...) for example code.
	 *
	 * @param pHeightData A pointer to the height data.
	 * @param bCopyHeightData When non-zero the height data is copied to an
	 * internal store. When zero the height data is accessed by reference and
	 * so must persist throughout the lifetime of the heightfield.
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
	 */
	//ODE_API 
	// void dGeomHeightfieldDataBuildShort( dHeightfieldData d,
	//				final short* pHeightData, int bCopyHeightData,
	//				double width, double depth, int widthSamples, int depthSamples,
	//				double scale, double offset, double thickness, int bWrap ) {
	void build( final short[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap );

	/**
	 * Configures a dHeightfieldData to use height data in
	 * single precision floating point format.
	 * <p>
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is stored as a rectangular
	 * array of single precision floats representing the height at each
	 * sample point.
	 * <p>
	 * See build(double[], ...) for example code.
	 *
	 * @param pHeightData A pointer to the height data.
	 * @param bCopyHeightData When non-zero the height data is copied to an
	 * internal store. When zero the height data is accessed by reference and
	 * so must persist throughout the lifetime of the heightfield.
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
	 */
	//ODE_API 
	// void dGeomHeightfieldDataBuildSingle( dHeightfieldData d,
	//				final float* pHeightData, int bCopyHeightData,
	//				double width, double depth, int widthSamples, int depthSamples,
	//				double scale, double offset, double thickness, int bWrap ) {
	void build( final float[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap );

	/**
	 * Configures a dHeightfieldData to use height data in
	 * double precision floating point format.
	 * <p>
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is stored as a rectangular
	 * array of double precision floats representing the height at each
	 * sample point.
	 * <p>
	 * <pre>
	 * DHeightfieldData heightData = OdeHelper.createHeightfieldData();
	 * double[] data = new double[MAX_X*MAX_Z];
	 * for (int x = 0; x < MAX_X; x++) {
	 *    for (int z = 0; z < MAX_Z; z++) {
	 *        data[x+MAX_X*z] = heightfieldFunction(x, z);
	 *    }
	 * }
	 * heightData.build(data, false, X_WIDTH, Z_DEPTH, MAX_X, MAX_Z, 1.0, 0.0, 0.0, false );
	 * </pre>
	 * <p>
	 * See DemoHeightfield for a full example.
	 *
	 * @param pHeightData A pointer to the height data.
	 * @param bCopyHeightData When non-zero the height data is copied to an
	 * internal store. When zero the height data is accessed by reference and
	 * so must persist throughout the lifetime of the heightfield.
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
	 */
	//ODE_API 
	// void dGeomHeightfieldDataBuildDouble( dHeightfieldData d,
	//				final double* pHeightData, int bCopyHeightData,
	//				double width, double depth, int widthSamples, int depthSamples,
	//				double scale, double offset, double thickness, int bWrap ) {
	void build( final double[] pHeightData, final boolean bCopyHeightData,
			final double width, final double depth, 
			final int widthSamples, final int depthSamples,
			final double scale, final double offset, 
			final double thickness, final boolean bWrap );

}
