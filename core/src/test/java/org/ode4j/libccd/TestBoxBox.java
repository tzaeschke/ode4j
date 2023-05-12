/**
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2009-2014 Tilmann Zaeschke<ode4j@gmx.de>  
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */
package org.ode4j.libccd;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;

import static org.junit.Assert.*;
import static org.ode4j.ode.internal.cpp4j.Cstdio.*;
import static org.ode4j.ode.internal.libccd.CCD.*;
import static org.ode4j.ode.internal.libccd.CCDQuat.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;
import static org.ode4j.libccd.CCDTestCommon.*;
import static org.ode4j.libccd.CCDTestSupport.*;

public class TestBoxBox {

	@Before
	public void boxboxSetUp()
	{
	}

	@After
	public void boxboxTearDown()
	{
	}

	@Test
	public void boxboxAlignedX()
	{
	    int i;
	    ccd_t ccd = new ccd_t();
	    ccd_box_t box1 = CCD_BOX();
	    ccd_box_t box2 = CCD_BOX();
	    boolean res;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;
	    //ccd.max_iterations = 20;

	    box1.x = 1;
	    box1.y = 2;
	    box1.z = 1;
	    box2.x = 2;
	    box2.y = 1;
	    box2.z = 2;

	    ccdVec3Set(box1.pos, -5., 0., 0.);
	    ccdVec3Set(box2.pos, 0., 0., 0.);
	    ccdQuatSet(box1.quat, 0., 0., 0., 1.);
	    ccdQuatSet(box2.quat, 0., 0., 0., 1.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(box1, box2, ccd);
	        if (i < 35 || i > 65){
	            assertFalse(res);
	        }else if (i != 35 && i != 65){
	            assertTrue(res);
	        }

	        box1.pos.add(0, 0.1);
	    }


	    box1.x = 0.1;
	    box1.y = 0.2;
	    box1.z = 0.1;
	    box2.x = 0.2;
	    box2.y = 0.1;
	    box2.z = 0.2;

	    ccdVec3Set(box1.pos, -0.5, 0., 0.);
	    ccdVec3Set(box2.pos, 0., 0., 0.);
	    ccdQuatSet(box1.quat, 0., 0., 0., 1.);
	    ccdQuatSet(box2.quat, 0., 0., 0., 1.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(box1, box2, ccd);

	        if (i < 35 || i > 65){
	            assertFalse(res);
	        }else if (i != 35 && i != 65){
	            assertTrue(res);
	        }

	        box1.pos.add(0, 0.01);
	    }


	    box1.x = 1;
	    box1.y = 2;
	    box1.z = 1;
	    box2.x = 2;
	    box2.y = 1;
	    box2.z = 2;

	    ccdVec3Set(box1.pos, -5., -0.1, 0.);
	    ccdVec3Set(box2.pos, 0., 0., 0.);
	    ccdQuatSet(box1.quat, 0., 0., 0., 1.);
	    ccdQuatSet(box2.quat, 0., 0., 0., 1.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(box1, box2, ccd);

	        if (i < 35 || i > 65){
	            assertFalse(res);
	        }else if (i != 35 && i != 65){
	            assertTrue(res);
	        }

	        box1.pos.add(0, 0.1);
	    }
	}

	@Test
	public void boxboxAlignedY()
	{
	    int i;
	    ccd_t ccd = new ccd_t();
	    ccd_box_t box1 = CCD_BOX();
	    ccd_box_t box2 = CCD_BOX();
	    boolean res;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    box1.x = 1;
	    box1.y = 2;
	    box1.z = 1;
	    box2.x = 2;
	    box2.y = 1;
	    box2.z = 2;

	    ccdVec3Set(box1.pos, 0., -5., 0.);
	    ccdVec3Set(box2.pos, 0., 0., 0.);
	    ccdQuatSet(box1.quat, 0., 0., 0., 1.);
	    ccdQuatSet(box2.quat, 0., 0., 0., 1.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(box1, box2, ccd);

	        if (i < 35 || i > 65){
	            assertFalse(res);
	        }else if (i != 35 && i != 65){
	            assertTrue(res);
	        }

	        box1.pos.add(1, 0.1);
	    }
	}

	@Test
	public void boxboxAlignedZ()
	{
	    int i;
	    ccd_t ccd = new ccd_t();
	    ccd_box_t box1 = CCD_BOX();
	    ccd_box_t box2 = CCD_BOX();
	    boolean res;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    box1.x = 1;
	    box1.y = 2;
	    box1.z = 1;
	    box2.x = 2;
	    box2.y = 1;
	    box2.z = 2;

	    ccdVec3Set(box1.pos, 0., 0., -5.);
	    ccdVec3Set(box2.pos, 0., 0., 0.);
	    ccdQuatSet(box1.quat, 0., 0., 0., 1.);
	    ccdQuatSet(box2.quat, 0., 0., 0., 1.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(box1, box2, ccd);

	        if (i < 35 || i > 65){
	            assertFalse(res);
	        }else if (i != 35 && i != 65){
	            assertTrue(res);
	        }

	        box1.pos.add(2, 0.1);
	    }
	}


	@Test
	public void boxboxRot()
	{
	    int i;
	    ccd_t ccd = new ccd_t();
	    ccd_box_t box1 = CCD_BOX();
	    ccd_box_t box2 = CCD_BOX();
	    boolean res;
	    ccd_vec3_t axis = new ccd_vec3_t();
	    double angle;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    box1.x = 1;
	    box1.y = 2;
	    box1.z = 1;
	    box2.x = 2;
	    box2.y = 1;
	    box2.z = 2;

	    ccdVec3Set(box1.pos, -5., 0.5, 0.);
	    ccdVec3Set(box2.pos, 0., 0., 0.);
	    ccdQuatSet(box2.quat, 0., 0., 0., 1.);
	    ccdVec3Set(axis, 0., 1., 0.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);

	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(box1, box2, ccd);

	        if (i < 33 || i > 67){
	            assertFalse(res);
	        }else if (i != 33 && i != 67){
	            assertTrue(res);
	        }

	        box1.pos.add(0, 0.1);
	    }

	    box1.x = 1;
	    box1.y = 1;
	    box1.z = 1;
	    box2.x = 1;
	    box2.y = 1;
	    box2.z = 1;

	    ccdVec3Set(box1.pos, -1.01, 0., 0.);
	    ccdVec3Set(box2.pos, 0., 0., 0.);
	    ccdQuatSet(box1.quat, 0., 0., 0., 1.);
	    ccdQuatSet(box2.quat, 0., 0., 0., 1.);

	    ccdVec3Set(axis, 0., 1., 0.);
	    angle = 0.;
	    for (i = 0; i < 30; i++){
	        res = ccdGJKIntersect(box1, box2, ccd);

	        if (i != 0 && i != 10 && i != 20){
	            assertTrue(res);
	        }else{
	            assertFalse(res);
	        }

	        angle += M_PI / 20.;
	        ccdQuatSetAngleAxis(box1.quat, angle, axis);
	    }

	}



	private static void pConf(ccd_box_t box1, ccd_box_t box2, final ccd_vec3_t v)
	{
	    fprintf(stdout, "# box1.pos: [%f %f %f]\n",
	            ccdVec3X(box1.pos), ccdVec3Y(box1.pos), ccdVec3Z(box1.pos));
	    fprintf(stdout, "# box1.quat: [%f %f %f %f]\n",
	            box1.quat.get0(), box1.quat.get1(), box1.quat.get2(), box1.quat.get3());
	    fprintf(stdout, "# box2.pos: [%f %f %f]\n",
	            ccdVec3X(box2.pos), ccdVec3Y(box2.pos), ccdVec3Z(box2.pos));
	    fprintf(stdout, "# box2.quat: [%f %f %f %f]\n",
	            box2.quat.get0(), box2.quat.get1(), box2.quat.get2(), box2.quat.get3());
	    fprintf(stdout, "# sep: [%f %f %f]\n",
	            ccdVec3X(v), ccdVec3Y(v), ccdVec3Z(v));
	    fprintf(stdout, "#\n");
	}

	@Test
	public void boxboxSeparate()
	{
	    ccd_t ccd = new ccd_t();
	    ccd_box_t box1 = CCD_BOX();
	    ccd_box_t box2 = CCD_BOX();
	    int res;
	    ccd_vec3_t sep = new ccd_vec3_t(), expsep = new ccd_vec3_t();
	    ccd_vec3_t expsep2 = new ccd_vec3_t(), axis = new ccd_vec3_t();

	    // fprintf(stderr, "\n\n\n---- boxboxSeparate ----\n\n\n");

	    box1.x = box1.y = box1.z = 1.;
	    box2.x = 0.5;
	    box2.y = 1.;
	    box2.z = 1.5;


	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    ccdVec3Set(box1.pos, -0.5, 0.5, 0.2);
	    boolean resB = ccdGJKIntersect(box1, box2, ccd);
	    assertTrue(resB);

	    res = ccdGJKSeparate(box1, box2, ccd, sep);
	    assertTrue(res == 0);
	    ccdVec3Set(expsep, 0.25, 0., 0.);
	    assertTrue(ccdVec3Eq(sep, expsep));

	    ccdVec3Scale(sep, -1.);
	    ccdVec3Add(box1.pos, sep);
	    res = ccdGJKSeparate(box1, box2, ccd, sep);
	    assertTrue(res == 0);
	    ccdVec3Set(expsep, 0., 0., 0.);
	    assertTrue(ccdVec3Eq(sep, expsep));


	    ccdVec3Set(box1.pos, -0.3, 0.5, 1.);
	    res = ccdGJKSeparate(box1, box2, ccd, sep);
	    assertTrue(res == 0);
	    ccdVec3Set(expsep, 0., 0., -0.25);
	    assertTrue(ccdVec3Eq(sep, expsep));



	    box1.x = box1.y = box1.z = 1.;
	    box2.x = box2.y = box2.z = 1.;
	    ccdVec3Set(axis, 0., 0., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, 0., 0., 0.);

	    res = ccdGJKSeparate(box1, box2, ccd, sep);
	    assertTrue(res == 0);
	    ccdVec3Set(expsep, 0., 0., 1.);
	    ccdVec3Set(expsep2, 0., 0., -1.);
	    assertTrue(ccdVec3Eq(sep, expsep) || ccdVec3Eq(sep, expsep2));



	    box1.x = box1.y = box1.z = 1.;
	    ccdVec3Set(axis, 0., 0., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, -0.5, 0., 0.);

	    res = ccdGJKSeparate(box1, box2, ccd, sep);
	    assertTrue(res == 0);
	    pConf(box1, box2, sep);



	    box1.x = box1.y = box1.z = 1.;
	    ccdVec3Set(axis, 0., 1., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, -0.5, 0.1, 0.4);

	    res = ccdGJKSeparate(box1, box2, ccd, sep);
	    assertTrue(res == 0);
	    pConf(box1, box2, sep);
	}


//	private void TOSVT(ccd_box_t box1, ccd_box_t box2, double depth, 
//			ccd_vec3_t dir, ccd_vec3_t pos) {
//	    svtObjPen(box1, box2, stdout, "Pen 1", depth, dir, pos);
//	    ccdVec3Scale(dir, depth);
//	    ccdVec3Add(box2.pos, dir);
//	    svtObjPen(box1, box2, stdout, "Pen 1", depth, dir, pos);
//	}

	@Test
	public void boxboxPenetration()
	{
	    ccd_t ccd = new ccd_t();
	    ccd_box_t box1 = CCD_BOX();
	    ccd_box_t box2 = CCD_BOX();
	    int res;
	    ccd_vec3_t axis = new ccd_vec3_t();
	    ccd_quat_t rot = new ccd_quat_t();
	    RefDouble depth = new RefDouble();
	    ccd_vec3_t dir = new ccd_vec3_t(), pos = new ccd_vec3_t();

	    // fprintf(stderr, "\n\n\n---- boxboxPenetration ----\n\n\n");

	    box1.x = box1.y = box1.z = 1.;
	    box2.x = 0.5;
	    box2.y = 1.;
	    box2.z = 1.5;


	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    ccdVec3Set(box2.pos, 0.1, 0., 0.);
	    res = ccdGJKPenetration(box1, box2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 1");
	    //TOSVT();


	    ccdVec3Set(box1.pos, -0.3, 0.5, 1.);
	    res = ccdGJKPenetration(box1, box2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 2");
	    //TOSVT(); <<<


	    box1.x = box1.y = box1.z = 1.;
	    box2.x = box2.y = box2.z = 1.;
	    ccdVec3Set(axis, 0., 0., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, 0.1, 0., 0.1);

	    res = ccdGJKPenetration(box1, box2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 3");
	    //TOSVT();


	    box1.x = box1.y = box1.z = 1.;
	    box2.x = box2.y = box2.z = 1.;
	    ccdVec3Set(axis, 0., 0., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, -0.5, 0., 0.);

	    res = ccdGJKPenetration(box1, box2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 4");
	    //TOSVT();


	    box1.x = box1.y = box1.z = 1.;
	    box2.x = box2.y = box2.z = 1.;
	    ccdVec3Set(axis, 0., 0., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, -0.5, 0.5, 0.);

	    res = ccdGJKPenetration(box1, box2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 5");
	    //TOSVT();


	    box1.x = box1.y = box1.z = 1.;
	    box2.x = box2.y = box2.z = 1.;
	    ccdVec3Set(box2.pos, 0.1, 0., 0.);

	    box1.x = box1.y = box1.z = 1.;
	    ccdVec3Set(axis, 0., 1., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, -0.5, 0.1, 0.4);

	    res = ccdGJKPenetration(box1, box2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 6");
	    //TOSVT();


	    box1.x = box1.y = box1.z = 1.;
	    ccdVec3Set(axis, 0., 1., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(axis, 1., 1., 1.);
	    ccdQuatSetAngleAxis(rot, M_PI / 4., axis);
	    ccdQuatMul(box1.quat, rot);
	    ccdVec3Set(box1.pos, -0.5, 0.1, 0.4);

	    res = ccdGJKPenetration(box1, box2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 7");
	    //TOSVT(); <<<


	    box1.x = box1.y = box1.z = 1.;
	    box2.x = 0.2; box2.y = 0.5; box2.z = 1.;
	    box2.x = box2.y = box2.z = 1.;

	    ccdVec3Set(axis, 0., 0., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(axis, 1., 0., 0.);
	    ccdQuatSetAngleAxis(rot, M_PI / 4., axis);
	    ccdQuatMul(box1.quat, rot);
	    ccdVec3Set(box1.pos, -1.3, 0., 0.);

	    ccdVec3Set(box2.pos, 0., 0., 0.);

	    res = ccdGJKPenetration(box1, box2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 8");
	    //TOSVT();
	}

}
