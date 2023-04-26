/*************************************************************************
 *                                                                       *
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
 *       the file ODE4J-LICENSE-BSD.TXT.                                 *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and ODE4J-LICENSE-BSD.TXT for more details.               *
 *                                                                       *
 *************************************************************************/
package org.ode4j.drawstuff.internal;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;

public interface DrawStuffApi {

	public abstract void dsSimulationLoop(String[] args, int window_width,
			int window_height, dsFunctions fn);

	public abstract void dsStop();

	public abstract void dsSetTexture(DS_TEXTURE_NUMBER texture_number);

	public abstract void dsSetColor(float red, float green, float blue);

	public abstract void dsDrawBox(final float[] pos, final float[] R,
			final float[] sides);

	public abstract void dsDrawSphere(final float[] pos, final float[] R,
			float radius);

	public abstract void dsDrawCylinder(final float[] pos, final float[] R,
			float length, float radius);

	public abstract void dsDrawCapsule(final float[] pos, final float[] R,
			float length, float radius);

	public abstract void dsDrawLine(final float[] pos1, final float[] pos2);

	public abstract void dsDrawBox(final DVector3C pos, final DMatrix3C R,
			final DVector3C sides);

	public abstract void dsDrawConvex(final DVector3C pos, final DMatrix3C R,
			double[] _planes, int _planecount, double[] _points,
			int _pointcount, int[] _polygons);

	public abstract void dsDrawSphere(final DVector3C pos, final DMatrix3C R,
			float radius);

	public abstract void dsDrawCylinder(final DVector3C pos, final DMatrix3C R,
			float length, float radius);

	public abstract void dsDrawCapsule(final DVector3C pos, final DMatrix3C R,
			float length, float radius);

	public abstract void dsDrawLine(final DVector3C pos1, final DVector3C pos2);

	public abstract void dsSetViewpoint(float[] xyz, float[] hpr);

	public abstract void dsGetViewpoint(float[] xyz, float[] hpr);

	public abstract double dsElapsedTime();

	public abstract void dsSetColorAlpha(float red, float green, float blue,
			float alpha);

	public abstract void dsSetDrawMode(int mode);

	public abstract void dsDrawTriangle(DVector3C pos, DMatrix3C rot,
			float[] v, int i, int j, int k, boolean solid);

	public abstract void dsDrawTriangle(DVector3C pos, DMatrix3C r,
			DVector3C v0, DVector3C v1, DVector3C v2, boolean solid);

	public void dsDrawTriangle (final DVector3C pos, final DMatrix3C R,
								final float[] v0, final float[] v1, final float[] v2, boolean solid);

	public void dsDrawTriangles(final float[] pos, final float[] R,
								final float[][] v, boolean solid);

	public void dsDrawTriangles(final DVector3C pos, final DMatrix3C R,
								final DVector3C[] v, boolean solid);

	public void dsSetSphereQuality(int n);
}