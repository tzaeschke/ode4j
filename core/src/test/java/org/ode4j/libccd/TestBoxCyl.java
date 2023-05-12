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

import org.junit.Test;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

import static org.junit.Assert.*;
import static org.ode4j.ode.internal.cpp4j.Cstdio.*;
import static org.ode4j.ode.internal.libccd.CCD.*;
import static org.ode4j.ode.internal.libccd.CCDQuat.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;
import static org.ode4j.libccd.CCDTestCommon.*;
import static org.ode4j.libccd.CCDTestSupport.*;

public class TestBoxCyl {

//	private static void TOSVT(ccd_box_t box, ccd_cyl_t cyl, double depth, 
//			ccd_vec3_t dir, ccd_vec3_t pos) {
//	    svtObjPen(box, cyl, stdout, "Pen 1", depth, dir, pos);
//	    ccdVec3Scale(dir, depth);
//	    ccdVec3Add(cyl.pos, dir);
//	    svtObjPen(box, cyl, stdout, "Pen 1", depth, dir, pos);
//	}

	@Test
	public void boxcylIntersect()
	{
	    ccd_t ccd = new ccd_t();
	    ccd_box_t box = CCD_BOX();
	    ccd_cyl_t cyl = CCD_CYL();
	    boolean res;
	    ccd_vec3_t axis = new ccd_vec3_t();

	    box.x = 0.5;
	    box.y = 1.;
	    box.z = 1.5;
	    cyl.radius = 0.4;
	    cyl.height = 0.7;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    ccdVec3Set(cyl.pos, 0.1, 0., 0.);
	    res = ccdGJKIntersect(box, cyl, ccd);
	    assertTrue(res);

	    ccdVec3Set(cyl.pos, .6, 0., 0.);
	    res = ccdGJKIntersect(box, cyl, ccd);
	    assertTrue(res);

	    ccdVec3Set(cyl.pos, .6, 0.6, 0.);
	    res = ccdGJKIntersect(box, cyl, ccd);
	    assertTrue(res);

	    ccdVec3Set(cyl.pos, .6, 0.6, 0.5);
	    res = ccdGJKIntersect(box, cyl, ccd);
	    assertTrue(res);

	    ccdVec3Set(axis, 0., 1., 0.);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 3., axis);
	    ccdVec3Set(cyl.pos, .6, 0.6, 0.5);
	    res = ccdGJKIntersect(box, cyl, ccd);
	    assertTrue(res);

	    ccdVec3Set(axis, 0.67, 1.1, 0.12);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 4., axis);
	    ccdVec3Set(cyl.pos, .6, 0., 0.5);
	    res = ccdGJKIntersect(box, cyl, ccd);
	    assertTrue(res);

	    ccdVec3Set(axis, -0.1, 2.2, -1.);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 5., axis);
	    ccdVec3Set(cyl.pos, .6, 0., 0.5);
	    ccdVec3Set(axis, 1., 1., 0.);
	    ccdQuatSetAngleAxis(box.quat, -M_PI / 4., axis);
	    ccdVec3Set(box.pos, .6, 0., 0.5);
	    res = ccdGJKIntersect(box, cyl, ccd);
	    assertTrue(res);

	    ccdVec3Set(axis, -0.1, 2.2, -1.);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 5., axis);
	    ccdVec3Set(cyl.pos, .6, 0., 0.5);
	    ccdVec3Set(axis, 1., 1., 0.);
	    ccdQuatSetAngleAxis(box.quat, -M_PI / 4., axis);
	    ccdVec3Set(box.pos, .9, 0.8, 0.5);
	    res = ccdGJKIntersect(box, cyl, ccd);
	    assertTrue(res);
	}


	@Test
	public void boxcylPenEPA()
	{
	    ccd_t ccd = new ccd_t();
	    ccd_box_t box = CCD_BOX();
	    ccd_cyl_t cyl = CCD_CYL();
	    int res;
	    ccd_vec3_t axis = new ccd_vec3_t();
	    RefDouble depth = new RefDouble();
	    ccd_vec3_t dir = new ccd_vec3_t(), pos = new ccd_vec3_t();

	    box.x = 0.5;
	    box.y = 1.;
	    box.z = 1.5;
	    cyl.radius = 0.4;
	    cyl.height = 0.7;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    ccdVec3Set(cyl.pos, 0.1, 0., 0.);
	    res = ccdGJKPenetration(box, cyl, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 1");
	    //TOSVT();

	    ccdVec3Set(cyl.pos, .6, 0., 0.);
	    res = ccdGJKPenetration(box, cyl, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 2");
	    //TOSVT(); <<<

	    ccdVec3Set(cyl.pos, .6, 0.6, 0.);
	    res = ccdGJKPenetration(box, cyl, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 3");
	    //TOSVT();

	    ccdVec3Set(cyl.pos, .6, 0.6, 0.5);
	    res = ccdGJKPenetration(box, cyl, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 4");
	    //TOSVT();

	    ccdVec3Set(axis, 0., 1., 0.);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 3., axis);
	    ccdVec3Set(cyl.pos, .6, 0.6, 0.5);
	    res = ccdGJKPenetration(box, cyl, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 5");
	    //TOSVT();

	    ccdVec3Set(axis, 0.67, 1.1, 0.12);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 4., axis);
	    ccdVec3Set(cyl.pos, .6, 0., 0.5);
	    res = ccdGJKPenetration(box, cyl, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 6");
	    //TOSVT();

	    ccdVec3Set(axis, -0.1, 2.2, -1.);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 5., axis);
	    ccdVec3Set(cyl.pos, .6, 0., 0.5);
	    ccdVec3Set(axis, 1., 1., 0.);
	    ccdQuatSetAngleAxis(box.quat, -M_PI / 4., axis);
	    ccdVec3Set(box.pos, .6, 0., 0.5);
	    res = ccdGJKPenetration(box, cyl, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 7");
	    //TOSVT();

	    ccdVec3Set(axis, -0.1, 2.2, -1.);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 5., axis);
	    ccdVec3Set(cyl.pos, .6, 0., 0.5);
	    ccdVec3Set(axis, 1., 1., 0.);
	    ccdQuatSetAngleAxis(box.quat, -M_PI / 4., axis);
	    ccdVec3Set(box.pos, .9, 0.8, 0.5);
	    res = ccdGJKPenetration(box, cyl, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 8");
	    //TOSVT();
	}


}
