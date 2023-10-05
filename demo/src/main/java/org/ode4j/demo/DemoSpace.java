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

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.*;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DGeom.DNearCallback;

/**
 * testing procedure:
 * - create a bunch of random boxes
 * - test for intersections directly, put results in n^2 array
 * - get space to report collisions:
 *   -- all correct collisions reported
 *   -- no pair reported more than once
 *   -- no incorrect collisions reported
 */
class DemoSpace extends dsFunctions {

	// some constants

	private static final int NUM = 20;			// number of boxes to test


	// collision objects and globals

	private static DSpace space;
	private static DGeom[] geom = new DGeom[NUM];
	private static DAABB[] bounds = new DAABB[NUM];
	private static boolean[][] good_matrix = new boolean[NUM][NUM];	// correct collision matrix
	private static boolean[][] test_matrix = new boolean[NUM][NUM];	// testing collision matrix
	private static int[] hits = new int[NUM];		// number of collisions a box has
	private static long seed=37;


	private void init_test()
	{
		int i;
		final double scale = 0.5;

		// set random boxes
		dRandSetSeed (seed);
		for (i=0; i < NUM; i++) {
			if (bounds[i] == null) bounds[i] = new DAABB(); 
			bounds[i].setMin0( dRandReal()*2-1 );
			bounds[i].setMax0( bounds[i].getMin0() + dRandReal()*scale );
			bounds[i].setMin1( dRandReal()*2-1 );
			bounds[i].setMax1( bounds[i].getMin1() + dRandReal()*scale );
			bounds[i].setMin2( dRandReal()*2 );
			bounds[i].setMax2( bounds[i].getMin2() + dRandReal()*scale );

			if (geom[i]!=null) geom[i].destroy();
			geom[i] = OdeHelper.createBox (space,
					bounds[i].len0(),
					bounds[i].len1(),
					bounds[i].len2());
			geom[i].setPosition (bounds[i].getCenter());
			geom[i].setData (i);//(void*)(size_t)(i));
		}

		// compute all intersections and put the results in "good_matrix"
		for (i=0; i < NUM; i++) {
			for (int j=0; j < NUM; j++) good_matrix[i][j] = false;
		}
		for (i=0; i < NUM; i++) hits[i] = 0;

		for (i=0; i < NUM; i++) {
			for (int j=i+1; j < NUM; j++) {
				if (bounds[i].isDisjoint(bounds[j])) continue;
				good_matrix[i][j] = true;
				good_matrix[j][i] = true;
				hits[i]++;
				hits[j]++;
			}
		}
	}

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};

	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i,j;
		i = (Integer) o1.getData();
		j = (Integer) o2.getData();
		if (i==j)
			System.out.println ("collision ("+i+","+j+") is between the same object");
		if (!good_matrix[i][j] || !good_matrix[j][i])
			System.out.println ("collision ("+i+","+j+") is incorrect");
		if (test_matrix[i][j] || test_matrix[j][i])
			System.out.println ("collision ("+i+","+j+") reported more than once");
		test_matrix[i][j] = true;
		test_matrix[j][i] = true;
	}


	private static float[] xyz = {2.1640f,-1.3079f,1.7600f};
	private static float[] hpr = {125.5000f,-17.0000f,0.0000f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		dsSetViewpoint (xyz,hpr);
	}


	@Override
	public void command (char cmd)
	{
		if (cmd == ' ') {
			seed++;
			init_test();
		}
	}


	// simulation loop

	private void simLoop (boolean pause)
	{
		int i,j;

		for (i=0; i < NUM; i++) {
			for (j=0; j < NUM; j++) test_matrix[i][j] = false;
		}
		OdeHelper.spaceCollide (space,0,nearCallback);
		for (i=0; i < NUM; i++) {
			for (j=i+1; j < NUM; j++) {
				if (good_matrix[i][j] && !test_matrix[i][j]) {
					System.out.println ("failed to report collision ("+i+","+j+") (seed="+seed+")");
				}
			}
		}

		seed++;
		init_test();

		for (i=0; i<NUM; i++) {
			DMatrix3 R = new DMatrix3();
			R.setIdentity();
			DVector3 pos = bounds[i].getCenter();
			DVector3 side = bounds[i].getLengths();
			if (hits[i] > 0) dsSetColor (1,0,0);
			else dsSetColor (1,1,0);
			dsDrawBox (pos,R,side);
		}
	}


	public static void main(String[] args) {
		new DemoSpace().demo(args);
	}
	
	private void demo(String[] args) {

		OdeHelper.initODE2(0);

		// test the simple space:
		// space = dSimpleSpaceCreate();

		// test the hash space:
		// space = dHashSpaceCreate (0);
		// dHashSpaceSetLevels (space,-10,10);

		// test the quadtree space
		DVector3 Center = new DVector3();
		DVector3 Extents = new DVector3(10, 0, 10);
		space = OdeHelper.createQuadTreeSpace(null, Center, Extents, 7);
		//space = OdeHelper.createSapSpace(space, AXES.XYZ);// 36);
		//space = OdeHelper.createSimpleSpace(null);
		//space = OdeHelper.createHashSpace(null);

		for (int i=0; i < NUM; i++) geom[i] = null;
		init_test();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		space.destroy ();
		OdeHelper.closeODE();
	}


	@Override
	public void step(boolean pause) {
		simLoop(pause);
	}


	@Override
	public void stop() {
		// Nothing
	}
}
