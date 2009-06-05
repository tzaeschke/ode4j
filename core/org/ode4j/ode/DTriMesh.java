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

/**
 * TriMesh code by Erwin de Vries.
 *
 * Trimesh data.
 * This is where the actual vertexdata (pointers), and BV tree is stored.
 * Vertices should be single precision!
 * This should be more sophisticated, so that the user can easyly implement
 * another collision library, but this is a lot of work, and also costs some
 * performance because some data has to be copied.
 */
public interface DTriMesh extends DGeom {
	
	/**
	 * Ray callback.
	 * Allows the user to say if a ray collides with a triangle on barycentric
	 * coords. The user can for example sample a texture with alpha transparency
	 * to determine if a collision should occur.
	 */
	//typedef int dTriRayCallback(dGeom TriMesh, dGeom Ray, int TriangleIndex, double u, double v);
	interface dTriRayCallback {
		int call(DGeom TriMesh, DGeom Ray, int TriangleIndex, double u, double v);
	}
	
	
	/**
	 * Per object callback. Allows the user to get the list of triangles in 1
	 * shot. Maybe we should remove this one.
	 */
	//typedef void dTriArrayCallback(dGeom TriMesh, dGeom RefObject, final int* TriIndices, int TriCount);
	interface dTriArrayCallback {
		void call(DGeom TriMesh, DGeom RefObject, final int[] TriIndices, int TriCount);
	}
	
	
	/**
	 * Per triangle callback. Allows the user to say if he wants a collision with
	 * a particular triangle.
	 */
	//typedef int dTriCallback(dGeom TriMesh, dGeom RefObject, int TriangleIndex);
	interface dTriCallback {
		int call(DGeom TriMesh, DGeom RefObject, int TriangleIndex);
	}
}
