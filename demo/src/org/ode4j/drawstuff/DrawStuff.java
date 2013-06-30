/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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
package org.ode4j.drawstuff;


import static org.cpp4j.Cstdio.*;

import org.ode4j.drawstuff.internal.DrawStuffApi;
import org.ode4j.drawstuff.internal.DrawStuffGL;
import org.ode4j.drawstuff.internal.DrawStuffNull;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;

/** 
 * @defgroup drawstuff DrawStuff
 * 
 * DrawStuff is a library for rendering simple 3D objects in a virtual
 * environment, for the purposes of demonstrating the features of ODE.
 * It is provided for demonstration purposes and is not intended for
 * production use.
 * 
 * @section Notes
 * In the virtual world, the z axis is "up" and z=0 is the floor. <br>
 *  <br>
 * The user is able to click+drag in the main window to move the camera: <br>
 * - left button - pan and tilt. <br>
 * - right button - forward and sideways. <br>
 * - left + right button (or middle button) - sideways and up. <br>
 */
public class DrawStuff {

	private static DrawStuffApi DS;
	
	private static DrawStuffApi get() {
		if (DS == null) {
			//default;
			DS = new DrawStuffGL();
			//DS = new DrawStuffNull();
		}
		return DS;
	}
	
	/**
	 * Use lwjgl for output.
	 */
	public static void setOutputGL() {
		DS = new DrawStuffGL();
	}
	
	/**
	 * Use no output. This can be useful for benchmarking.
	 */
	public static void setOutputNull() {
		DS = new DrawStuffNull();
	}
	
	//texturepath.h
	// Sourceforge tests require the textures in the drawstuff folder
	public static String DRAWSTUFF_TEXTURE_PATH = "../../drawstuff/textures";

	//version.h
	/* high byte is major version, low byte is minor version */
	public static int DS_VERSION = 0x0002;

	/* texture numbers */
	  public static enum DS_TEXTURE_NUMBER
	  {
	    DS_NONE, // = 0,       /* uses the current color instead of a texture */
	    DS_WOOD,
	    DS_CHECKERED,
	    DS_GROUND,
	    DS_SKY;
	  }

	/* draw modes */

//	#define DS_POLYFILL  0
	  public static final int DS_POLYFILL = 0;
	//#define DS_WIREFRAME 1
	  public static final int DS_WIREFRAME = 1;

	/**
	 * @struct dsFunctions
	 * @brief Set of functions to be used as callbacks by the simulation loop.
	 * @ingroup drawstuff
	 */
//	typedef struct dsFunctions {
//		  int version;			/* put DS_VERSION here */
//		  /* version 1 data */
//		  void (*start)();		/* called before sim loop starts */
//		  void (*step) (int pause);	/* called before every frame */
//		  void (*command) (int cmd);	/* called if a command key is pressed */
//		  void (*stop)();		/* called after sim loop exits */
//		  /* version 2 data */
//		  const char *path_to_textures;	/* if nonzero, path to texture files */
//		} dsFunctions;
	  public abstract static class dsFunctions { 
		  public int version = DS_VERSION;			/* put DS_VERSION here */
		  /* version 1 data */
		  public abstract void start();		/* called before sim loop starts */
		  public abstract void step (boolean pause);	/* called before every frame */
		  public abstract void command (char cmd);	/* called if a command key is pressed */
		  public abstract void stop();		/* called after sim loop exits */
		  /* version 2 data */
		  /* if nonzero, path to texture files */
		  public String path_to_textures = DRAWSTUFF_TEXTURE_PATH;	
		  public void setVersion(int ds_version) {
			  version = ds_version;
		  }
		  public void setPathToTextures(String drawstuff_texture_path) {
			  path_to_textures = drawstuff_texture_path;
		  }
		  public String getPathToTextures() {
			  return path_to_textures;
		  }
		  public int getVersion() {
			  return version;
		  }
	  }
//	} dsFunctions;


	/**
	 * @brief Does the complete simulation.
	 * @ingroup drawstuff
	 * This function starts running the simulation, and only exits when the simulation is done.
	 * Function pointers should be provided for the callbacks.
	 * @param argv supports flags like '-notex' '-noshadow' '-pause'
	 * @param fn Callback functions.
	 */
	//DS_API 
//		public static void dsSimulationLoop (int argc, char **argv,
//			       int window_width, int window_height,
//			       struct dsFunctions *fn);
	  public static void dsSimulationLoop (String[] args,
			  int window_width, int window_height,
			  dsFunctions fn) {
		  get().dsSimulationLoop(args, window_width, window_height, fn);
	  }

	/**
	 * @brief exit with error message.
	 * @ingroup drawstuff
	 * This function displays an error message then exit.
	 * @param msg format strin, like printf, without the newline character.
	 */
	//DS_API 
	public static void dsError (final String ... msgs) { //sconst char *msg, ...);
		System.err.print("Error");
		for (String s: msgs) {
			System.out.print(" " + s);
		}
		throw new RuntimeException();
	}
 
	/**
	 * @brief exit with error message and core dump.
	 * @ingroup drawstuff
	 * this functions tries to dump core or start the debugger.
	 * @param msg format strin, like printf, without the newline character.
	 */
	//DS_API 
	public static void dsDebug (final String ... msgs) { // (const char *msg, ...);
		System.err.print("INTERNAL ERROR");
		for (String s: msgs) {
			System.out.print(" " + s);
		}
		throw new RuntimeException();
	}

	/**
	 * @brief print log message
	 * @ingroup drawstuff
	 * @param msg format string, like printf, without the \n.
	 */
	//DS_API 
	public static void dsPrint  (String msg, Object ... objs) { //(const char *msg, ...);
		printf(msg, objs);
	}

	/**
	 * @brief Sets the viewpoint
	 * @ingroup drawstuff
	 * @param xyz camera position.
	 * @param hpr contains heading, pitch and roll numbers in degrees. heading=0
	 * points along the x axis, pitch=0 is looking towards the horizon, and
	 * roll 0 is "unrotated".
	 */
	//DS_API 
//	public static  void dsSetViewpoint (float xyz[3], float hpr[3]);
	public static void dsSetViewpoint (float xyz[], float hpr[]) {
		get().dsSetViewpoint(xyz, hpr);
	}


	/**
	 * @brief Gets the viewpoint
	 * @ingroup drawstuff
	 * @param xyz position
	 * @param hpr heading,pitch,roll.
	 */
	//DS_API 
//	public static  void dsGetViewpoint (float xyz[3], float hpr[3]);
	public static void dsGetViewpoint (float[] xyz, float[] hpr) {
		get().dsGetViewpoint(xyz, hpr);
	}

	/**
	 * @brief Stop the simulation loop.
	 * @ingroup drawstuff
	 * Calling this from within dsSimulationLoop()
	 * will cause it to exit and return to the caller. it is the same as if the
	 * user used the exit command. using this outside the loop will have no
	 * effect.
	 */
	//DS_API 
	public static void dsStop() {
		get().dsStop();
	}

	/**
	 * @brief Get the elapsed time (on wall-clock)
	 * @ingroup drawstuff
	 * It returns the nr of seconds since the last call to this function.
	 */
	//DS_API 
	public static double dsElapsedTime() {
		return get().dsElapsedTime();
	}

	/**
	 * @brief Toggle the rendering of textures.
	 * @ingroup drawstuff
	 * It changes the way objects are drawn. these changes will apply to all further
	 * dsDrawXXX() functions. 
	 * @param texture_number The texture number must be a DS_xxx texture constant.
	 * The current texture is colored according to the current color.
	 * At the start of each frame, the texture is reset to none and the color is
	 * reset to white.
	 */
	//DS_API 
	public static void dsSetTexture (DS_TEXTURE_NUMBER texture_number) {	
		get().dsSetTexture(texture_number);
	}


	/**
	 * @brief Set the color with which geometry is drawn.
	 * @ingroup drawstuff
	 * @param red Red component from 0 to 1
	 * @param green Green component from 0 to 1
	 * @param blue Blue component from 0 to 1
	 */
	//DS_API 
	public static  void dsSetColor (float red, float green, float blue) {
		get().dsSetColor(red, green, blue);
	}

	/**
	 * @brief Set the color and transparency with which geometry is drawn.
	 * @ingroup drawstuff
	 * @param alpha Note that alpha transparency is a misnomer: it is alpha opacity.
	 * 1.0 means fully opaque, and 0.0 means fully transparent.
	 */
	//DS_API 
	public static  void dsSetColorAlpha (float red, float green, float blue, float alpha) {
		get().dsSetColorAlpha(red, green, blue, alpha);
	}

	/**
	 * @brief Draw a box.
	 * @ingroup drawstuff
	 * @param pos is the x,y,z of the center of the object.
	 * @param R is a 3x3 rotation matrix for the object, stored by row like this:
	 *        [ R11 R12 R13 0 ]
	 *        [ R21 R22 R23 0 ]
	 *        [ R31 R32 R33 0 ]
	 * @param sides is an array of x,y,z side lengths.
	 */
	//DS_API 
//	public static  void dsDrawBox (final float pos[3], final float R[12], final float sides[3]) {
	public static  void dsDrawBox (final float[] pos, final float[] R, final float sides[]) {
		get().dsDrawBox(pos, R, sides);
	}
	public static  void dsDrawBox (final DVector3C pos, final DMatrix3C R, final DVector3C sides) {
		get().dsDrawBox(pos, R, sides);
	}

	/**
	 * @brief Draw a sphere.
	 * @ingroup drawstuff
	 * @param pos Position of center.
	 * @param R orientation.
	 * @param radius
	 */
	//DS_API 
	//public static  void dsDrawSphere (final float pos[3], final float R[12], float radius) {
	public static  void dsDrawSphere (final float[] pos, final float[] R, 
			float radius) {
		get().dsDrawSphere(pos, R, radius);
	}
	public static  void dsDrawSphere (final DVector3C pos, final DMatrix3C R, 
			double radius) {
		get().dsDrawSphere(pos, R, (float) radius);
	}

	/**
	 * @brief Draw a triangle.
	 * @ingroup drawstuff
	 * @param pos Position of center
	 * @param R orientation
	 * @param v0 first vertex
	 * @param v1 second
	 * @param v2 third vertex
	 * @param solid set to 0 for wireframe
	 */
	//DS_API 
//	public static  void dsDrawTriangle (final float pos[3], final float R[12],
//		     final float *v0, final float *v1, final float *v2, int solid) {
	public static  void dsDrawTriangle (final float[] pos, final float[] R,
		     final float[] v0, final float[] v1, final float[] v2, int solid) {
		throw new UnsupportedOperationException();
	}

	public static  void dsDrawTriangle (final DVector3C pos, final DMatrix3C R,
		     final DVector3C v0, final DVector3C v1, final DVector3C v2, boolean solid) {
		get().dsDrawTriangle(pos, R, v0, v1, v2, solid);
	}

	public static void dsDrawTriangle(DVector3C pos, DMatrix3C rot,
			float[] v, int i, int j, int k, boolean solid) {
		get().dsDrawTriangle(pos, rot, v, i, j, k, solid);
	}

	/**
	 * @brief Draw a z-aligned cylinder
	 * @ingroup drawstuff
	 */
	//DS_API 
//	public static  void dsDrawCylinder (final float pos[3], final float R[12],
//		     float length, float radius) {
	public static  void dsDrawCylinder (final float[] pos, final float[] R,
		     float length, float radius) {
	get().dsDrawCylinder(pos, R, length, radius);
}
	public static  void dsDrawCylinder (final DVector3C pos, final DMatrix3C R,
		     double length, double radius) {
	get().dsDrawCylinder(pos, R, (float)length, (float)radius);
}

	
	/**
	 * @brief Draw a z-aligned capsule
	 * @ingroup drawstuff
	 */
	//DS_API 
//	public static  void dsDrawCapsule (final float pos[3], final float R[12],
//		    float length, float radius) {
	public static  void dsDrawCapsule (final float[] pos, final float[] R,
		    float length, float radius) {
	get().dsDrawCapsule(pos, R, length, radius);
}
	public static  void dsDrawCapsule (final DVector3C pos, final DMatrix3C R,
		    double length, double radius) {
	get().dsDrawCapsule(pos, R, (float)length, (float)radius);
}

	
	/**
	 * @brief Draw a line.
	 * @ingroup drawstuff
	 */
	//DS_API 
//	public static  void dsDrawLine (final float pos1[3], final float pos2[3]) {
	public static  void dsDrawLine (final float[] pos1, final float[] pos2) {
		get().dsDrawLine(pos1, pos2);
	}
	public static  void dsDrawLine (final DVector3C pos1, final DVector3C pos2) {
		get().dsDrawLine(pos1, pos2);
	}


	/**
	 * @brief Draw a convex shape.
	 * @ingroup drawstuff
	 */
	//DS_API 
//	public static  void dsDrawConvex(final float pos[3], final float R[12],
//			  float *_planes,
//			  unsigned int _planecount,
//			  float *_points,
//			  unsigned int _pointcount,
//			  unsigned int *_polygons) {
	public static  void dsDrawConvex(final float[] pos, final float[] R,
			  float[] _planes,
			  int _planecount,
			  float[] _points,
			  int _pointcount,
			  int[] _polygons) {
		throw new UnsupportedOperationException();
	}
	public static  void dsDrawConvex(final DVector3C pos, final DMatrix3C R,
			  double[] _planes,
			  int _planecount,
			  double[] _points,
			  int _pointcount,
			  int[] _polygons) {
		get().dsDrawConvex(pos, R, _planes, _planecount, _points, _pointcount, _polygons);
	}


	//DS_API 
//	public static  void dsDrawSphereD (final double pos[3], final double R[12],
//		    final float radius) {
	public static void dsDrawSphere (final DVector3C pos, final DMatrix3C R,
			    final float radius) {
		get().dsDrawSphere(pos, R, radius);
	}

	//DS_API 
//	public static  void dsDrawTriangleD (final double pos[3], final double R[12],
//		      final double *v0, final double *v1, final double *v2, int solid) {
	public static void dsDrawTriangleD (final double[] pos, final double[] R,
			      final double[] v0, final double[] v1, final double[] v2, int solid) {
		throw new UnsupportedOperationException();
	}

	//DS_API 
//	public static void dsDrawLineD (final double pos1[3], final double pos2[3]) {
	public static void dsDrawLineD (final double[] pos1, final double[] pos2) {
		throw new UnsupportedOperationException();
	}

	//DS_API 
//	public static  void dsDrawConvexD(final double pos[3], final double R[12],
//			  double *_planes,
//			  unsigned int _planecount,
//			  double *_points,
//			  unsigned int _pointcount,
//			  unsigned int *_polygons) {
	public static  void dsDrawConvexD(final double[] pos, final double[] R,
			  double[] _planes,
			  int _planecount,
			  double[] _points,
			  int _pointcount,
			  int[] _polygons) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the quality with which curved objects are rendered.
	 * @ingroup drawstuff
	 * Higher numbers are higher quality, but slower to draw. 
	 * This must be set before the first objects are drawn to be effective.
	 * Default sphere quality is 1, default capsule quality is 3.
	 */
	//DS_API 
	public static  void dsSetSphereQuality (int n) {		/* default = 1 */
		throw new UnsupportedOperationException();
	}

	//DS_API 
	public static  void dsSetCapsuleQuality (int n) {		/* default = 3 */
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set Drawmode 0=Polygon Fill,1=Wireframe).
	 * Use the DS_POLYFILL and DS_WIREFRAME macros.
	 * @ingroup drawstuff
	 */
	//DS_API 
	public static  void dsSetDrawMode(int mode) {
		get().dsSetDrawMode(mode);
	}

	
	// Backwards compatible API
//	#define dsDrawCappedCylinder dsDrawCapsule
//	#define dsDrawCappedCylinderD dsDrawCapsuleD
//	#define dsSetCappedCylinderQuality dsSetCapsuleQuality
	/** @deprecated */
	public static void dsDrawCappedCylinder(final float pos[], final float[] R,
		    float length, float radius) {
		dsDrawCapsule(pos, R, length, radius);
	}
	/** @deprecated */
	public static void dsDrawCappedCylinder(final DVector3C pos, 
			final DMatrix3C R,
		     float length, float radius) {
		dsDrawCapsule(pos, R, length, radius);
	}
	/** @deprecated */
	public static void dsSetCappedCylinderQuality(int n) { 
		dsSetCapsuleQuality(n);
	}
}
