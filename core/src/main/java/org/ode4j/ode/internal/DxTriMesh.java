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
package org.ode4j.ode.internal;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.OdeConfig;

public abstract class DxTriMesh extends DxGeom implements DTriMesh {

	//TZ from "collision_trimesh_internal.h":
	// Callbacks
	DTriCallback Callback;
	DTriArrayCallback ArrayCallback;
	DTriRayCallback RayCallback;
	DTriTriMergeCallback TriMergeCallback;
	
	// Data types
	//DxTriMeshData _Data;

	boolean doSphereTC;
	boolean doBoxTC;
	boolean doCapsuleTC;

	abstract void ClearTCCache();

	@Override
	abstract void computeAABB();


	//dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){ type = dTriMeshClass; }
	public DxTriMesh(DxSpace space) //: dxGeom(Space, 1)
	{ 
		super(space, true);
		type = dTriMeshClass;
//		_Data = data;
	}
	//dxTriMesh::~dxTriMesh(){}

	public static DxTriMesh dCreateTriMesh(DxSpace space, 
			DxTriMeshData Data,
			DTriCallback Callback,
			DTriArrayCallback ArrayCallback,
			DTriRayCallback RayCallback)
	{
		DxTriMesh Geom;
		switch (OdeConfig.dTRIMESH_TYPE) {
		case DISABLED: Geom = new DxTriMeshDisabled(space, Data); break;
		case GIMPACT: Geom = new DxGimpact(space, (DxGimpactData) Data); break;
		default: throw new IllegalArgumentException(OdeConfig.dTRIMESH_TYPE.name());
		}
		Geom.Callback = Callback;
		Geom.ArrayCallback = ArrayCallback;
		Geom.RayCallback = RayCallback;

		return Geom;
	}

	abstract public int FetchTriangleCount();

	abstract public void FetchTransformedTriangle(int i, DVector3[] v);
}

