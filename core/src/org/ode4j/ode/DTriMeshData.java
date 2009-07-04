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
public interface DTriMeshData {

	/**
	 * Build a TriMesh data object with single precision vertex data.
	 */
	void buildSingle(
			final double[] Vertices, int VertexStride, int VertexCount, 
			final int[] Indices, int IndexCount, int TriStride);
	/**
	 * Build a TriMesh data object with single precision vertex data.
	 */
	void buildSingle(
			final float[] Vertices, int VertexStride, int VertexCount, 
			final int[] Indices, int IndexCount, int TriStride);

	void destroy();
}
