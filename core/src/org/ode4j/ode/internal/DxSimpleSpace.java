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
package org.ode4j.ode.internal;

import org.ode4j.ode.DSimpleSpace;

import static org.ode4j.ode.internal.Common.*;


/** 
 * ***************************************************************************
 * simple space - reports all n^2 object intersections
 */
public class DxSimpleSpace extends DxSpace implements DSimpleSpace {

	//	  dxSimpleSpace (dSpace _space);
	//	  void cleanGeoms();
	//	  void collide (void *data, dNearCallback *callback);
	//	  void collide2 (void *data, dxGeom *geom, dNearCallback *callback);


	public static DxSimpleSpace dSimpleSpaceCreate (DxSpace space)
	{
		return new DxSimpleSpace (space);
	}


	DxSimpleSpace (DxSpace space)// : dxSpace (_space)
	{
		super((DxSpace) space);
		type = dSimpleSpaceClass;
	}


	void cleanGeoms()
	{
		// compute the AABBs of all dirty geoms, and clear the dirty flags
		lock_count++;
		//for (dxGeom g=_first; g!=null && (g.gflags & GEOM_DIRTY) != 0; g=g.getNext()) {
		for (DxGeom g: _geoms) {
			if ((g._gflags & GEOM_DIRTY) == 0) break;
			if (g instanceof DxSpace) {
				((DxSpace)g).cleanGeoms();
			}
			g.recomputeAABB();
			g._gflags &= (~(GEOM_DIRTY|GEOM_AABB_BAD));
		}
		lock_count--;
	}

	@Override
	public void collide (Object data, DNearCallback callback)
	{
		dAASSERT (callback);

		lock_count++;
		cleanGeoms();

		// intersect all bounding boxes
		//for (dxGeom g1=_first; g1!=null; g1=g1.getNext()) {
		//for (dxGeom g1: _geoms) {
		for (int i = 0; i < _geoms.size(); i++) {
			DxGeom g1 = _geoms.get(i);
			if (GEOM_ENABLED(g1)){
				//for (dxGeom g2=g1.getNext(); g2!=null; g2=g2.getNext()) {
				for (int j = i+1; j< _geoms.size(); j++ ) {
					DxGeom g2 = _geoms.get(j);
					if (GEOM_ENABLED(g2)){
						collideAABBs (g1,g2,data,callback);
					}
				}
			}
		}
		lock_count--;
	}


	@Override
	void collide2 (Object data, DxGeom geom,
			DNearCallback callback)
	{
		dAASSERT (geom, callback);

		lock_count++;
		cleanGeoms();
		geom.recomputeAABB();

		// intersect bounding boxes
		//for (dxGeom g=_first; g!=null; g=g.getNext()) {
		for (DxGeom g: _geoms) {
			if (GEOM_ENABLED(g)){
				collideAABBs (g,geom,data,callback);
			}
		}
		lock_count--;
	}
}
