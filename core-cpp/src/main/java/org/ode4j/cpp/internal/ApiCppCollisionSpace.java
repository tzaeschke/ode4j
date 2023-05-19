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
package org.ode4j.cpp.internal;

import org.ode4j.math.DVector3C;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHashSpace;
import org.ode4j.ode.DQuadTreeSpace;
import org.ode4j.ode.DSimpleSpace;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DSapSpace.AXES;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

public abstract class ApiCppCollisionSpace extends ApiCppCollisionTrimesh {

	//ODE_API 
	public static DSimpleSpace dSimpleSpaceCreate (DSpace space) {
		return OdeHelper.createSimpleSpace(space);
	}
	//ODE_API 
	public static DHashSpace dHashSpaceCreate (DSpace space) {
		return OdeHelper.createHashSpace(space);
	}
	//ODE_API 
	public static  DQuadTreeSpace dQuadTreeSpaceCreate (DSpace space, 
			DVector3C Center, DVector3C Extents, int Depth) {
		return OdeHelper.createQuadTreeSpace(space, 
				Center, Extents, Depth);
	}


	// SAP
	// Order XZY or ZXY usually works best, if your Y is up.
	//#define dSAP_AXES_XYZ  ((0)|(1<<2)|(2<<4))
	//#define dSAP_AXES_XZY  ((0)|(2<<2)|(1<<4))
	//#define dSAP_AXES_YXZ  ((1)|(0<<2)|(2<<4))
	//#define dSAP_AXES_YZX  ((1)|(2<<2)|(0<<4))
	//#define dSAP_AXES_ZXY  ((2)|(0<<2)|(1<<4))
	//#define dSAP_AXES_ZYX  ((2)|(1<<2)|(0<<4))
	public static final int dSAP_AXES_XYZ = ((0)|(1<<2)|(2<<4));
	public static final int dSAP_AXES_XZY = ((0)|(2<<2)|(1<<4));
	public static final int dSAP_AXES_YXZ = ((1)|(0<<2)|(2<<4));
	public static final int dSAP_AXES_YZX = ((1)|(2<<2)|(0<<4));
	public static final int dSAP_AXES_ZXY = ((2)|(0<<2)|(1<<4));
	public static final int dSAP_AXES_ZYX = ((2)|(1<<2)|(0<<4));

	//ODE_API 
	public static DSpace dSweepAndPruneSpaceCreate( DSpace space, AXES axisorder ) {
		return OdeHelper.createSapSpace(space, axisorder);
	}


	//ODE_API 
	public static void dSpaceDestroy (DSpace s) {
		s.destroy();
	}

	//ODE_API 
	public static void dHashSpaceSetLevels (DHashSpace space, int minlevel, int maxlevel) {
		space.setLevels(minlevel, maxlevel);
	}
	//ODE_API 
	// void dHashSpaceGetLevels (dSpace space, int *minlevel, int *maxlevel) {
	public static void dHashSpaceGetLevels (DHashSpace space, RefInt minlevel, RefInt maxlevel) {
		minlevel.set( space.getLevelMin() );
		maxlevel.set( space.getLevelMax() );
	}

	//ODE_API 
	public static void dSpaceSetCleanup (DSpace space, boolean mode) {
		space.setCleanup(mode);
	}
	//ODE_API 
	public static boolean dSpaceGetCleanup (DSpace space) {
		return space.getCleanup();
	}

	/**
	 * Sets sublevel value for a space.
	 *
	 * Sublevel affects how the space is handled in dSpaceCollide2 when it is collided
	 * with another space. If sublevels of both spaces match, the function iterates 
	 * geometries of both spaces and collides them with each other. If sublevel of one
	 * space is greater than the sublevel of another one, only the geometries of the 
	 * space with greater sublevel are iterated, another space is passed into 
	 * collision callback as a geometry itself. By default all the spaces are assigned
	 * zero sublevel.
	 *
	 * <p>NOTE:
	 * The space sublevel <b> IS NOT </b> automatically updated when one space is inserted
	 * into another or removed from one. It is a client's responsibility to update sublevel
	 * value if necessary.
	 *
	 * @param space the space to modify
	 * @param sublevel the sublevel value to be assigned
	 * @see #dSpaceGetSublevel(DSpace)
	 * @see ApiCppCollision#dSpaceCollide2(DGeom, DGeom, Object, org.ode4j.ode.DGeom.DNearCallback)
	 */
	//ODE_API 
	public static void dSpaceSetSublevel (DSpace space, int sublevel) {
		space.setSublevel(sublevel);
	}

	/**
	 * Gets sublevel value of a space.
	 *
	 * Sublevel affects how the space is handled in dSpaceCollide2 when it is collided
	 * with another space. See {@code dSpaceSetSublevel} for more details.
	 *
	 * @param space the space to query
	 * @return the sublevel value of the space
	 * @see #dSpaceSetSublevel(DSpace, int)
	 * @see ApiCppCollision#dSpaceCollide2(DGeom, DGeom, Object, org.ode4j.ode.DGeom.DNearCallback)
	 */
	//ODE_API 
	public static int dSpaceGetSublevel (DSpace space) {
		return space.getSublevel();
	}


	/**
	 * Sets manual cleanup flag for a space.
	 *
	 * Manual cleanup flag marks a space as eligible for manual thread data cleanup.
	 * This function should be called for every space object right after creation in 
	 * case if ODE has been initialized with {@code dInitFlagManualThreadCleanup} flag.
	 * 
	 * Failure to set manual cleanup flag for a space may lead to some resources 
	 * remaining leaked until the program exit.
	 *
	 * @param space the space to modify
	 * @param mode 1 for manual cleanup mode and 0 for default cleanup mode
	 * @see #dSpaceGetManualCleanup(DSpace)
	 * @see ApiCppOdeInit#dInitODE2(int)
	 */
	//ODE_API 
	public static void dSpaceSetManualCleanup (DSpace space, int mode) {
		space.setManualCleanup(mode);
	}

	/**
	 * Get manual cleanup flag of a space.
	 *
	 * Manual cleanup flag marks a space space as eligible for manual thread data cleanup.
	 * See {@code dSpaceSetManualCleanup} for more details.
	 * 
	 * @param space the space to query
	 * @return 1 for manual cleanup mode and 0 for default cleanup mode of the space
	 * @see #dSpaceSetManualCleanup(DSpace, int)
	 * @see ApiCppOdeInit#dInitODE2(int)
	 */
	//ODE_API 
	public static int dSpaceGetManualCleanup (DSpace space) {
		return space.getManualCleanup();
	}

	//ODE_API 
	public static void dSpaceAdd (DSpace s, DGeom g) {
		s.add(g);
	}
	//ODE_API 
	public static void dSpaceRemove (DSpace s, DGeom g) {
		s.remove(g);
	}
	//ODE_API 
	public static boolean dSpaceQuery (DSpace s, DGeom g) {
		return s.query(g);
	}
	//ODE_API 
	public static void dSpaceClean (DSpace s) {
		s.cleanGeoms();
	}
	//ODE_API 
	public static int dSpaceGetNumGeoms (DSpace s) {
		return s.getNumGeoms();
	}
	//ODE_API
	/** 
	 * @param s space 
	 * @param i i
	 * @return geom
	 * @deprecated Use dSpaceGetGeoms() instead */
	@Deprecated
	public static DGeom dSpaceGetGeom (DSpace s, int i) {
		return s.getGeom(i);
	}
	
	public static Iterable<? extends DGeom> dSpaceGetGeoms (DSpace s) {
		return s.getGeoms();
	}

	/**
	 * Given a space, this returns its class.
	 * <p>
	 * The ODE classes are:
	 *  <li> dSimpleSpaceClass </li>
	 *  <li> dHashSpaceClass </li>
	 *  <li> dSweepAndPruneSpaceClass </li>
	 *  <li> dQuadTreeSpaceClass </li>
	 *  <li> dFirstUserClass </li>
	 *  <li> dLastUserClass </li>
	 * <p>
	 * The class id not defined by the user should be between
	 * dFirstSpaceClass and dLastSpaceClass.
	 * <p>
	 * User-defined class will return their own number.
	 *
	 * @param space the space to query
	 * @return The space class ID.
	 */
	//ODE_API 
	int dSpaceGetClass(DSpace space) {
		throw new UnsupportedOperationException();
	}

	protected ApiCppCollisionSpace() {}
}
