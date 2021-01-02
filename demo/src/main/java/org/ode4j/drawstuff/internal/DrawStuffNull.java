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

import static org.ode4j.ode.internal.cpp4j.Cstdio.fprintf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.stderr;

import org.ode4j.drawstuff.DrawStuff;
import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.OdeHelper;

/**
 * Empty implementation of DrawStuff. Does not draw and is therefore suitable
 * for measuring computation performance of demos.
 *
 * @author Tilmann Zaeschke
 */
public class DrawStuffNull implements DrawStuffApi {

	private static volatile boolean _run = true;
	
	@Override
	public void dsDrawBox(float[] pos, float[] R, float[] sides) {
		// Nothing
	}

	@Override
	public void dsDrawCapsule(float[] pos, float[] R, float length, float radius) {
		// Nothing
	}

	@Override
	public void dsDrawCylinder(float[] pos, float[] R, float length,
			float radius) {
		// Nothing
	}

	@Override
	public void dsDrawLine(float[] pos1, float[] pos2) {
		// Nothing
	}

	@Override
	public void dsDrawSphere(float[] pos, float[] R, float radius) {
		// Nothing
	}

	@Override
	public void dsSetColor(float red, float green, float blue) {
		// Nothing
	}

	@Override
	public void dsSetTexture(DS_TEXTURE_NUMBER texture_number) {
		// Nothing
	}

	@Override
	public void dsSimulationLoop(String[] args, int window_width,
			int window_height, dsFunctions fn) {
		// look for flags that apply to us
		boolean initial_pause = false;
		for (int i=1; i<args.length; i++) {
//			if (args[i].equals("-notex")) use_textures = false;
//			if (args[i].equals("-noshadow")) use_shadows = false;
//			if (args[i].equals("-noshadows")) use_shadows = false;
			if (args[i].equals("-pause")) initial_pause = true;
		}

//		initMotionModel();
		dsPlatformSimLoop (window_width,window_height,fn,initial_pause);

	}

	private static boolean firsttime=true;
	void dsPlatformSimLoop (int window_width, int window_height, dsFunctions fn,
			boolean initial_pause)
	{
//		dsStartGraphics (window_width,window_height,fn);

		//TZ static bool firsttime=true;
		if (firsttime)
		{
			System.err.println();
			System.err.print("Using ode4j version: " + OdeHelper.getVersion());
			System.err.println("  [" + OdeHelper.getConfiguration() + "]");
			System.err.println();
			fprintf
			(
					stderr,
					"\n" +
					"Simulation test environment v%d.%02d\n" +
					"   Ctrl-P : pause / unpause (or say `-pause' on command line).\n" +
					"   Ctrl-O : single step when paused.\n" +
					"   Ctrl-T : toggle textures (or say `-notex' on command line).\n" +
					"   Ctrl-S : toggle shadows (or say `-noshadow' on command line).\n" +
					"   Ctrl-V : print current viewpoint coordinates (x,y,z,h,p,r).\n" +
					"   Ctrl-W : write frames to ppm files: frame/frameNNN.ppm\n" +
					"   Ctrl-X : exit.\n" +
					"\n" +
					"Change the camera position by clicking + dragging in the window.\n" +
					"   Left button - pan and tilt.\n" +
					"   Right button - forward and sideways.\n" +
					"   Left + Right button (or middle button) - sideways and up.\n" +
					"\n",DrawStuff.DS_VERSION >> 8,DrawStuff.DS_VERSION & 0xff
			);
			firsttime = false;
		}

		//if (fn.start) 
		fn.start();

		long startTime = System.currentTimeMillis() + 5000;
		long fps = 0;
//		int loops = 0;
		_run = true;
		while (_run) {
			//  while (run) {
			// read in and process all pending events for the main window
			//    XEvent event;
			//    while (run && XPending (display)) {
			//      XNextEvent (display,event);
			//      handleEvent (event,fn);
			//    }
//			handleKeyboard(fn);
//			handleMouse();
//			try {
//				if (System.in.available() > 0) {
//					break;
//				}
//			} catch (IOException e) {
//				throw new RuntimeException(e);
//			}

//			GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);
//
//			dsDrawFrame (width,height,fn,pause && !singlestep);
			fn.step(false);
//			singlestep = false;


			if (startTime > System.currentTimeMillis()) {
				fps++;
			} else {
				long timeUsed = 5000 + (startTime - System.currentTimeMillis());
				startTime = System.currentTimeMillis() + 5000;
				System.out.println(fps + " frames in " + (timeUsed / 1000f) + " seconds = "
						+ (fps / (timeUsed / 1000f)));
				fps = 0;
//				loops++;
//				if (loops >=5) break;
			}
			//    glFlush();
			//    glXSwapBuffers (display,win);
			//    XSync (display,0);

		}

		//if (fn.stop) 
		fn.stop();
	}
	
	@Override
	public double dsElapsedTime() {
		// Nothing
		return 0;
	}

	@Override
	public void dsGetViewpoint(float[] xyz, float[] hpr) {
		// Nothing
	}

	@Override
	public void dsSetViewpoint(float[] xyz, float[] hpr) {
		// Nothing
	}

	@Override
	public void dsStop() {
		_run = false;
	}

	@Override
	public void dsDrawBox(DVector3C pos, DMatrix3C R, DVector3C sides) {
		// Nothing
	}

	@Override
	public void dsDrawCapsule(DVector3C pos, DMatrix3C R, float length,
			float radius) {
		// Nothing
	}

	@Override
	public void dsDrawConvex(DVector3C pos, DMatrix3C R, double[] _planes,
			int _planecount, double[] _points, int _pointcount, int[] _polygons) {
		// Nothing
	}

	@Override
	public void dsDrawCylinder(DVector3C pos, DMatrix3C R, float length,
			float radius) {
		// Nothing
	}

	@Override
	public void dsDrawLine(DVector3C pos1, DVector3C pos2) {
		// Nothing
	}

	@Override
	public void dsDrawSphere(DVector3C pos, DMatrix3C R, float radius) {
		// Nothing
	}

	@Override
	public void dsSetColorAlpha(float red, float green, float blue, float alpha) {
		// Nothing
	}

	@Override
	public void dsSetDrawMode(int mode) {
		// Nothing
	}

	@Override
	public void dsDrawTriangle(DVector3C pos, DMatrix3C rot, float[] v, int i,
			int j, int k, boolean solid) {
		// Nothing
	}

	@Override
	public void dsDrawTriangle(DVector3C pos, DMatrix3C r, DVector3C v0,
			DVector3C v1, DVector3C v2, boolean solid) {
		// Nothing
	}

	@Override
	public void dsDrawTriangle(DVector3C pos, DMatrix3C R, float[] v0,
			float[] v1, float[] v2, boolean solid) {
		// Nothing
	}

	@Override
	public void dsDrawTriangles(final float[] pos, final float[] R,
								final float[][] v, boolean solid) {
		// Nothing
	}

	@Override
	public void dsDrawTriangles(final DVector3C pos, final DMatrix3C R,
								final DVector3C[] v, boolean solid) {
		// Nothing
	}

	@Override
    public void dsSetSphereQuality(int n) {
        // Nothing
    }
}
