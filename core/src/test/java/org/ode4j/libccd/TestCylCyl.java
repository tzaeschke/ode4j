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

public class TestCylCyl {
	
	@Before
	public void cylcylSetUp()
	{
	}

	@After
	public void cylcylTearDown()
	{
	}


	@Test
	public void cylcylAlignedX()
	{
	    ccd_t ccd = new ccd_t();
	    ccd_cyl_t c1 = CCD_CYL();
	    ccd_cyl_t c2 = CCD_CYL();
	    int i;
	    boolean res;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    c1.radius = 0.35;
	    c1.height = 0.5;
	    c2.radius = 0.5;
	    c2.height = 1.;

	    ccdVec3Set(c1.pos, -5., 0., 0.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(c1, c2, ccd);

	        if (i < 42 || i > 58){
	            assertFalse(res);
	        }else{
	            assertTrue(res);
	        }

	        c1.pos.add(0, 0.1);
	    }
	}

	@Test
	public void cylcylAlignedY()
	{
	    ccd_t ccd = new ccd_t();
	    ccd_cyl_t c1 = CCD_CYL();
	    ccd_cyl_t c2 = CCD_CYL();
	    int i;
	    boolean res;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    c1.radius = 0.35;
	    c1.height = 0.5;
	    c2.radius = 0.5;
	    c2.height = 1.;

	    ccdVec3Set(c1.pos, 0., -5., 0.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(c1, c2, ccd);

	        if (i < 42 || i > 58){
	            assertFalse(res);
	        }else{
	            assertTrue(res);
	        }

	        c1.pos.add(1, 0.1);
	    }
	}

	@Test
	public void cylcylAlignedZ()
	{
	    ccd_t ccd = new ccd_t();
	    ccd_cyl_t c1 = CCD_CYL();
	    ccd_cyl_t c2 = CCD_CYL();
	    int i;
	    boolean res;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    c1.radius = 0.35;
	    c1.height = 0.5;
	    c2.radius = 0.5;
	    c2.height = 1.;

	    ccdVec3Set(c1.pos, 0., 0., -5.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(c1, c2, ccd);

	        if (i < 43 || i > 57){
	            assertFalse(res);
	        }else{
	            assertTrue(res);
	        }

	        c1.pos.add(2, 0.1);
	    }
	}

//	#define TOSVT() \
//	    svtObjPen(cyl1, cyl2, stdout, "Pen 1", depth, dir, pos); \
//	    ccdVec3Scale(dir, depth); \
//	    ccdVec3Add(cyl2.pos, dir); \
//	    svtObjPen(cyl1, cyl2, stdout, "Pen 1", depth, dir, pos)

	@Test
	public void cylcylPenetrationEPA()
	{
	    ccd_t ccd = new ccd_t();
	    ccd_cyl_t cyl1 = CCD_CYL();
	    ccd_cyl_t cyl2 = CCD_CYL();
	    int res;
	    ccd_vec3_t axis = new ccd_vec3_t();
	    RefDouble depth = new RefDouble();
	    ccd_vec3_t dir = new ccd_vec3_t(), pos = new ccd_vec3_t();

	    // fprintf(stderr, "\n\n\n---- cylcylPenetration ----\n\n\n");

	    cyl1.radius = 0.35;
	    cyl1.height = 0.5;
	    cyl2.radius = 0.5;
	    cyl2.height = 1.;

	    CCD_INIT(ccd);
	    ccd.support1 = ccdSupport;
	    ccd.support2 = ccdSupport;

	    ccdVec3Set(cyl2.pos, 0., 0., 0.3);
	    res = ccdGJKPenetration(cyl1, cyl2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 1");
	    //TOSVT();

	    ccdVec3Set(cyl1.pos, 0.3, 0.1, 0.1);
	    res = ccdGJKPenetration(cyl1, cyl2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 2");
	    //TOSVT(); <<<

	    ccdVec3Set(axis, 0., 1., 1.);
	    ccdQuatSetAngleAxis(cyl2.quat, M_PI / 4., axis);
	    ccdVec3Set(cyl2.pos, 0., 0., 0.);
	    res = ccdGJKPenetration(cyl1, cyl2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 3");
	    //TOSVT();

	    ccdVec3Set(axis, 0., 1., 1.);
	    ccdQuatSetAngleAxis(cyl2.quat, M_PI / 4., axis);
	    ccdVec3Set(cyl2.pos, -0.2, 0.7, 0.2);
	    res = ccdGJKPenetration(cyl1, cyl2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 4");
	    //TOSVT();

	    ccdVec3Set(axis, 0.567, 1.2, 1.);
	    ccdQuatSetAngleAxis(cyl2.quat, M_PI / 4., axis);
	    ccdVec3Set(cyl2.pos, 0.6, -0.7, 0.2);
	    res = ccdGJKPenetration(cyl1, cyl2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 5");
	    //TOSVT();

	    ccdVec3Set(axis, -4.567, 1.2, 0.);
	    ccdQuatSetAngleAxis(cyl2.quat, M_PI / 3., axis);
	    ccdVec3Set(cyl2.pos, 0.6, -0.7, 0.2);
	    res = ccdGJKPenetration(cyl1, cyl2, ccd, depth, dir, pos);
	    assertTrue(res == 0);
	    recPen(depth, dir, pos, stdout, "Pen 6");
	    //TOSVT();
	}

}
