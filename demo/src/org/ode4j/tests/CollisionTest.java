package org.ode4j.tests;

import org.junit.Test;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHeightfieldData;
import org.ode4j.ode.DRay;
import org.ode4j.ode.OdeHelper;

import static org.ode4j.cpp.OdeCpp.*;

public class CollisionTest {

    /**
     * This tests some extreme cases, where a sphere barely touches some triangles
     * with zero depth.
     */
	//TEST(
	@Test public void test_collision_trimesh_sphere_exact()
	{
		//TODO TZ disabled, because there is no TRIMESH!!
		System.err.println("WARNING: Test disabled, because there is no TRIMESH: " +
				"CollisionTest.test_collision_trimesh_sphere_exact()");
//	    if (dTRIMESH_GIMPACT) {//#ifdef dTRIMESH_GIMPACT
//	    /*
//	     * Although GIMPACT is algorithmically able to handle this extreme case,
//	     * the numerical approximation used for the square root produces inexact results.
//	     */
//	    return;
//	    }//#endif
//
//	    OdeHelper.initODE();
//
//	    try {
//	        final int VertexCount = 4;
//	        final int IndexCount = 2*3;
//	        // this is a square on the XY plane
//	        float[] vertices = {//[VertexCount * 3] = {
//	            -1,-1,0,
//	            1,-1,0,
//	            1,1,0,
//	            -1,1,0
//	        };
//	        int[] indices = {//[IndexCount] = {
//	            0,1,2,
//	            0,2,3
//	        };
//	        
//	        DTriMeshData data = dGeomTriMeshDataCreate();
//	        dGeomTriMeshDataBuildSingle(data,
//	                                    vertices,
//	                                    3,// * sizeof(float),
//	                                    VertexCount,
//	                                    indices,
//	                                    IndexCount,
//	                                    3);// * sizeof(dTriIndex));
//	        DGeom trimesh = dCreateTriMesh(null, data, null, null, null);
//	        final double radius = 4;
//	        DGeom sphere = dCreateSphere(null, radius);
//	        dGeomSetPosition(sphere, 0,0,radius);
//	        //dContactGeom cg[4];
//	        DContactGeomBuffer cg = new DContactGeomBuffer(4);
//	        int nc;
//
//	        // check extreme case
//	        nc = dCollide(trimesh, sphere, 4, cg);//&cg[0], sizeof cg[0]);
//	        CHECK_EQUAL(1, nc);
//	        CHECK_EQUAL(0, cg.get(0).depth);
//	        
//	        // now translate both geoms
//	        dGeomSetPosition(trimesh, 10,30,40);
//	        dGeomSetPosition(sphere, 10,30,40+radius);
//	        // check extreme case, again
//	        nc = dCollide(trimesh, sphere, 4, cg);//&cg[0], sizeof cg[0]);
//	        CHECK_EQUAL(1, nc);
//	        CHECK_EQUAL(0, cg.get(0).depth);
//	        
//	        // and now, let's rotate the trimesh, 90 degrees on X
//	        DMatrix3 rot = new DMatrix3( 1, 0, 0, //0,
//	                         0, 0, -1, //0,
//	                         0, 1, 0);//, 0 };
//	        dGeomSetPosition(trimesh, 10,30,40);
//	        dGeomSetRotation(trimesh, rot);
//	        
//	        dGeomSetPosition(sphere, 10,30-radius,40);
//	        // check extreme case, again
//	        nc = dCollide(trimesh, sphere, 4, cg);//&cg[0], sizeof cg[0]);
//	        CHECK_EQUAL(1, nc);
//	        CHECK_EQUAL(0, cg.get(0).depth);
//	    } finally {
//	    	OdeHelper.closeODE();
//	    }
	}



    /**
     * This test demonstrated a bug in the AABB handling of the
     * heightfield.
     */
	//TEST(
	@Test public void test_collision_heightfield_ray_fail()
	{
	    OdeHelper.initODE();
	    try {
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

	        // Crash!
	        dCollide(ray, height, 10, contactBuf.getGeomBuffer());//&(contactBuf[0].geom), sizeof(dContact));

	        dGeomDestroy(height);
	        dGeomDestroy(ray);
	        dGeomHeightfieldDataDestroy(heightfieldData);
	    } finally {
	    	OdeHelper.closeODE();
	    }
	}


}
