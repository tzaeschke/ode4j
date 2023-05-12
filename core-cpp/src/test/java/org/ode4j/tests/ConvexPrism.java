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
package org.ode4j.tests;


public class ConvexPrism {

    public static int prism_pointcount = 8;

    public static int prism_planecount = 6;
    public static double[] prism_points = { // [24]= {
        10.0, 1.0, -1.0,
                10.0, -1.0, -1.0,
                -10.0, -1.0, -1.0,
                -10.0, 1.0, -1.0,
                10.0, 1.0, 1.0,
                10.0, -1.0, 1.0,
                -10.0, -1.0, 1.0,
                -10.0, 1.0, 1.0
    };

    public static int[] prism_polygons = {
            4, 0, 1, 2, 3,
            4, 4, 7, 6, 5,
            4, 0, 4, 5, 1,
            4, 1, 5, 6, 2,
            4, 2, 6, 7, 3,
            4, 4, 0, 3, 7,
    };
    public static double[] prism_planes = {
            0.0, 0.0, -1.0, 1.0,
            0.0, 0.0, 1.0, 1.0,
            1.0, 0.0, 0.0, 10.0,
            0.0, -1.0, 0.0, 1.0,
            -1.0, 0.0, -0.0, 10.0,
            0.0, 1.0, 0.0, 1.0,
    };
}