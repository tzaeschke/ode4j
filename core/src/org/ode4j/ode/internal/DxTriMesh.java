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
package org.ode4j.ode.internal;

import org.ode4j.ode.DAABB;
import org.ode4j.ode.OdeConfig;
import org.ode4j.ode.DTriMesh;

public abstract class DxTriMesh extends DxGeom implements DTriMesh {

	//TODO #if !dTRIMESH_ENABLED

	//#include "collision_util.h"
	//#include "collision_trimesh_internal.h"

	//TZ from "collision_trimesh_internal.h":
	// Callbacks
	dTriCallback Callback;
	dTriArrayCallback ArrayCallback;
	dTriRayCallback RayCallback;

	// Data types
	DxTriMeshData _Data;

	boolean doSphereTC;
	boolean doBoxTC;
	boolean doCapsuleTC;

	abstract void ClearTCCache();

	abstract boolean AABBTest(DAABB aabb);
	@Override
	abstract void computeAABB();


	//dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){ type = dTriMeshClass; }
	public DxTriMesh(DxSpace space, DxTriMeshData data) //: dxGeom(Space, 1)
	{ 
		super(space, true);
		type = dTriMeshClass;
		_Data = data;
	}
	//dxTriMesh::~dxTriMesh(){}

	public static DxTriMesh dCreateTriMesh(DxSpace space, 
			DxTriMeshData Data,
			dTriCallback Callback,
			dTriArrayCallback ArrayCallback,
			dTriRayCallback RayCallback)
	{
		DxTriMesh Geom;
		switch (OdeConfig.dTRIMESH_TYPE) {
		case DISABLED: Geom = new DxTriMeshDisabled(space, Data); break;
		default: throw new IllegalArgumentException(OdeConfig.dTRIMESH_TYPE.name());
		}
		Geom.Callback = Callback;
		Geom.ArrayCallback = ArrayCallback;
		Geom.RayCallback = RayCallback;

		return Geom;
	}
}

