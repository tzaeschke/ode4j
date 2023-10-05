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
package org.ode4j.democpp;

import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetData;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetData;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dQuadTreeSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DMisc.dRandSetSeed;
import static org.ode4j.ode.DRotation.dRSetIdentity;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DSpace;

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
	private static DGeom[] geom=new DGeom[NUM];
	private static double[][] bounds=new double[NUM][6];
	private static boolean[][] good_matrix=new boolean[NUM][NUM];	// correct collision matrix
	private static boolean[][] test_matrix=new boolean[NUM][NUM];	// testing collision matrix
	private static int[] hits = new int[NUM];		// number of collisions a box has
	private static long seed=37;


	private void init_test()
	{
		int i,j;
		final double scale = 0.5;

		// set random boxes
		dRandSetSeed (seed);
		for (i=0; i < NUM; i++) {
			bounds[i][0] = dRandReal()*2-1;
			bounds[i][1] = bounds[i][0] + dRandReal()*scale;
			bounds[i][2] = dRandReal()*2-1;
			bounds[i][3] = bounds[i][2] + dRandReal()*scale;
			bounds[i][4] = dRandReal()*2;
			bounds[i][5] = bounds[i][4] + dRandReal()*scale;

			if (geom[i]!=null) dGeomDestroy (geom[i]);
			geom[i] = dCreateBox (space,
					bounds[i][1] - bounds[i][0],
					bounds[i][3] - bounds[i][2],
					bounds[i][5] - bounds[i][4]);
			dGeomSetPosition (geom[i],
					(bounds[i][0] + bounds[i][1])*0.5,
					(bounds[i][2] + bounds[i][3])*0.5,
					(bounds[i][4] + bounds[i][5])*0.5);
			dGeomSetData (geom[i],i);//(void*)(size_t)(i));
		}

		// compute all intersections and put the results in "good_matrix"
		for (i=0; i < NUM; i++) {
			for (j=0; j < NUM; j++) good_matrix[i][j] = false;
		}
		for (i=0; i < NUM; i++) hits[i] = 0;

		for (i=0; i < NUM; i++) {
			for (j=i+1; j < NUM; j++) {
//				dReal *bounds1 = &bounds[i][0];
//				dReal *bounds2 = &bounds[j][0];
//				if (bounds1[0] > bounds2[1] ||
//						bounds1[1] < bounds2[0] ||
//						bounds1[2] > bounds2[3] ||
//						bounds1[3] < bounds2[2] ||
//						bounds1[4] > bounds2[5] ||
//						bounds1[5] < bounds2[4]) continue;
				if (bounds[i][0] > bounds[j][1] ||
						bounds[i][1] < bounds[j][0] ||
						bounds[i][2] > bounds[j][3] ||
						bounds[i][3] < bounds[j][2] ||
						bounds[i][4] > bounds[j][5] ||
						bounds[i][5] < bounds[j][4]) continue;

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
		i = (Integer) dGeomGetData (o1);
		j = (Integer) dGeomGetData (o2);
		if (i==j)
			printf ("collision (%d,%d) is between the same object\n",i,j);
		if (!good_matrix[i][j] || !good_matrix[j][i])
			printf ("collision (%d,%d) is incorrect\n",i,j);
		if (test_matrix[i][j] || test_matrix[j][i])
			printf ("collision (%d,%d) reported more than once\n",i,j);
		test_matrix[i][j] = true;
		test_matrix[j][i] = true;
	}


	private static float[] xyz = {2.1640f,-1.3079f,1.7600f};
	private static float[] hpr = {125.5000f,-17.0000f,0.0000f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

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
		dSpaceCollide (space,0,nearCallback);
		for (i=0; i < NUM; i++) {
			for (j=i+1; j < NUM; j++) {
				if (good_matrix[i][j] && !test_matrix[i][j]) {
					printf ("failed to report collision (%d,%d) (seed=%d)\n",i,j,seed);
				}
			}
		}

		seed++;
		init_test();

		for (i=0; i<NUM; i++) {
			DVector3 pos = new DVector3(), side = new DVector3();
			DMatrix3 R = new DMatrix3();
			dRSetIdentity (R);
			for (j=0; j<3; j++) pos.set(j, (bounds[i][j*2+1] + bounds[i][j*2]) * 0.5 );
			for (j=0; j<3; j++) side.set(j, bounds[i][j*2+1] - bounds[i][j*2] );
			if (hits[i] > 0) dsSetColor (1,0,0);
			else dsSetColor (1,1,0);
			dsDrawBox (pos,R,side);
		}
	}


	public static void main(String[] args) {
		new DemoSpace().demo(args);
	}
	
	private void demo(String[] args) {
		int i;

		// setup pointers to drawstuff callback functions
		//dsFunctions fn = this;
		//fn.version = DS_VERSION;
		//  fn.start = &start;
		//  fn.step = &simLoop;
		//  fn.command = &command;
		//  fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		dInitODE2(0);

		// test the simple space:
		// space = dSimpleSpaceCreate();

		// test the hash space:
		// space = dHashSpaceCreate (0);
		// dHashSpaceSetLevels (space,-10,10);

		// test the quadtree space
		DVector3 Center = new DVector3();//{0, 0, 0, 0};
		DVector3 Extents = new DVector3(10, 0, 10);//{10, 0, 10, 0};
		space = dQuadTreeSpaceCreate(null, Center, Extents, 7);
		//space = dSweepAndPruneSpaceCreate(space, 36);
		//space = dSimpleSpaceCreate(null);
		//space = dHashSpaceCreate(null);

		for (i=0; i < NUM; i++) geom[i] = null;
		init_test();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dSpaceDestroy (space);
		dCloseODE();
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
