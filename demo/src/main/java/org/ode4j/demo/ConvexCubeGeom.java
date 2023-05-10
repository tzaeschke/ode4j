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
package org.ode4j.demo;

class ConvexCubeGeom {
	static final double[] planes = // planes for a cube
			{1.0f, 0.0f, 0.0f, 0.25f,
					0.0f, 1.0f, 0.0f, 0.25f,
					0.0f, 0.0f, 1.0f, 0.25f,
					0.0f, 0.0f, -1.0f, 0.25f,
					0.0f, -1.0f, 0.0f, 0.25f,
					-1.0f, 0.0f, 0.0f, 0.25f};
	static final int planecount = 6;
	static final double points[] = // points for a cube
	{ 0.25f, 0.25f, 0.25f, // point 0
			-0.25f, 0.25f, 0.25f, // point 1

			0.25f, -0.25f, 0.25f, // point 2
			-0.25f, -0.25f, 0.25f, // point 3

			0.25f, 0.25f, -0.25f, // point 4
			-0.25f, 0.25f, -0.25f, // point 5

			0.25f, -0.25f, -0.25f, // point 6
			-0.25f, -0.25f, -0.25f,// point 7
	};
	static final int pointcount = 8;
	static final int polygons[] = // Polygons for a cube (6 squares)
	{ 4, 0, 2, 6, 4, // positive X
			4, 1, 0, 4, 5, // positive Y
			4, 0, 1, 3, 2, // positive Z
			4, 3, 1, 5, 7, // negative X
			4, 2, 3, 7, 6, // negative Y
			4, 5, 4, 6, 7, // negative Z
	};
}