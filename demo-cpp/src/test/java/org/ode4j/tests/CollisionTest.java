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
package org.ode4j.tests;

import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateHeightfield;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateRay;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateSphere;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomHeightfieldDataBuildByte;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomHeightfieldDataCreate;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomHeightfieldDataDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomHeightfieldDataSetBounds;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomRaySet;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetRotation;
import static org.ode4j.cpp.internal.ApiCppCollisionTrimesh.dCreateTriMesh;
import static org.ode4j.cpp.internal.ApiCppCollisionTrimesh.dGeomTriMeshDataBuildSingle;
import static org.ode4j.cpp.internal.ApiCppCollisionTrimesh.dGeomTriMeshDataCreate;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_ARRAY_EQUAL;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHeightfieldData;
import org.ode4j.ode.DRay;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.tests.UnitTestPlusPlus.TestSuperClass;

/**
 * Tests from collision.cpp.
 *
 * @author Tilmann Zaeschke
 */
public class CollisionTest extends TestSuperClass {

	/**
	 * This tests some extreme cases, where a sphere barely touches some triangles
	 * with zero depth.
	 */
	//TEST(
	@Test public void test_collision_trimesh_sphere_exact()
	{
		//	    if (dTRIMESH_GIMPACT) {//#ifdef dTRIMESH_GIMPACT
		//	    /*
		//	     * Although GIMPACT is algorithmically able to handle this extreme case,
		//	     * the numerical approximation used for the square root produces inexact results.
		//	     */
		//	    return;
		//	    }//#endif

		final int VertexCount = 4;
		final int IndexCount = 2*3;
		// this is a square on the XY plane
		/*
	           3    2
	           +----+
	           |   /|
	           |  / |
	           | /  |
	           |/   |
	           +----+
	           0    1
		 */
		float[] vertices = {//[VertexCount * 3] = {
				-1,-1,0,
				1,-1,0,
				1,1,0,
				-1,1,0
		};
		int[] indices = {//[IndexCount] = {
				0,1,2,
				0,2,3
		};

		DTriMeshData data = dGeomTriMeshDataCreate();
		dGeomTriMeshDataBuildSingle(data,
				vertices,
				3,// * sizeof(float),
				VertexCount,
				indices,
				IndexCount,
				3);// * sizeof(dTriIndex));
		DGeom trimesh = dCreateTriMesh(null, data, null, null, null);
		final double radius = 4;
		DGeom sphere = dCreateSphere(null, radius);
		//dContactGeom cg[4];
		DContactGeomBuffer cg = new DContactGeomBuffer(4);
		int nc;
		DVector3 trinormal = new DVector3( 0, 0, -1 );

		// Test case: sphere touches the diagonal edge
		dGeomSetPosition(sphere, 0,0,radius);
		nc = dCollide(trimesh, sphere, 4, cg);//&cg[0], sizeof cg[0]);
		//TODO TZ In ODE C/C++, this fails for GIMPACT but works for OPCODE.
		//In ode4j there is only GIMPACT, therefore it fails.
		CHECK_EQUAL(2, nc);
		for (int i=0; i<nc; ++i) {
			CHECK_EQUAL(0, cg.get(i).depth);
			CHECK_ARRAY_EQUAL(trinormal, cg.get(i).normal, 3);
		}
		//TODO remove tz
//		CHECK_EQUAL(1, nc);
//		//CHECK_EQUAL(0, cg.get(0).depth);
//		CHECK_CLOSE(0, cg.get(0).depth, 0.00000000001);  //TZ is not ==0!

		// now translate both geoms
		dGeomSetPosition(trimesh, 10,30,40);
		dGeomSetPosition(sphere, 10,30,40+radius);
		// check extreme case, again
		nc = dCollide(trimesh, sphere, 4, cg);//&cg[0], sizeof cg[0]);
		CHECK_EQUAL(2, nc);
		for (int i=0; i<nc; ++i) {
			CHECK_EQUAL(0, cg.get(i).depth);
			CHECK_ARRAY_EQUAL(trinormal, cg.get(i).normal, 3);
		}
		//TODO remove tz
//		CHECK_EQUAL(1, nc);
//		//CHECK_EQUAL(0, cg.get(0).depth);
//		CHECK_CLOSE(0, cg.get(0).depth, 0.00000000001);  //TZ is not ==0!

		// and now, let's rotate the trimesh, 90 degrees on X
		DMatrix3 rot = new DMatrix3( 1, 0, 0, //0,
				0, 0, -1, //0,
				0, 1, 0);//, 0 };
		dGeomSetPosition(trimesh, 10,30,40);
		dGeomSetRotation(trimesh, rot);

		dGeomSetPosition(sphere, 10,30-radius,40);
		// check extreme case, again
		nc = dCollide(trimesh, sphere, 4, cg);//&cg[0], sizeof cg[0]);
		CHECK_EQUAL(2, nc);
		DVector3 rtrinormal = new DVector3( 0, 1, 0 );
		for (int i=0; i<nc; ++i) {
			CHECK_EQUAL(0, cg.get(i).depth);
			CHECK_ARRAY_EQUAL(rtrinormal, cg.get(i).normal, 3);
		}
		//TODO remove tz
//		CHECK_EQUAL(1, nc);
//		//CHECK_EQUAL(0, cg.get(0).depth);
//		CHECK_CLOSE(0, cg.get(0).depth, 0.00000000001);  //TZ is not ==0!
	}



	/**
	 * This test demonstrated a bug in the AABB handling of the
	 * heightfield.
	 */
	//TEST(
	@Test public void test_collision_heightfield_ray_fail()
	{
		// Create quick heightfield with dummy data
		DHeightfieldData heightfieldData = dGeomHeightfieldDataCreate();
		//unsigned char dataBuffer[16+1] = "1234567890123456";
		byte[] dataBuffer = {1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6};
		dGeomHeightfieldDataBuildByte(heightfieldData, dataBuffer, false, 4, 4, 4, 4, 1, 0, 0, false);
		dGeomHeightfieldDataSetBounds(heightfieldData, '0', '9');
		DGeom height = dCreateHeightfield(null, heightfieldData, true);

		// Create ray outside bounds
		DRay ray = dCreateRay(null, 20);
		dGeomRaySet(ray, 5, 10, 1, 0, -1, 0);
		//DContact contactBuf[10];
		DContactBuffer contactBuf = new DContactBuffer(10);

		// Make sure it does not crash!
		dCollide(ray, height, 10, contactBuf.getGeomBuffer());//&(contactBuf[0].geom), sizeof(dContact));

		dGeomDestroy(height);
		dGeomDestroy(ray);
		dGeomHeightfieldDataDestroy(heightfieldData);
	}


}
