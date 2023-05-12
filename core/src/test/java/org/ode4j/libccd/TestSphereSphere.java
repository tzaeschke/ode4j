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

import static org.junit.Assert.*;
import static org.ode4j.ode.internal.libccd.CCD.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;

public class TestSphereSphere {

	@Before
	public void spheresphereSetUp()
	{
	}

	@After
	public void spheresphereTearDown()
	{
	}

	@Test
	public void spheresphereAlignedX()
	{
	    ccd_t ccd = new ccd_t();
	    CCDTestSupport.ccd_sphere_t s1 = CCDTestSupport.CCD_SPHERE();
	    CCDTestSupport.ccd_sphere_t s2 = CCDTestSupport.CCD_SPHERE();
	    int i;
	    boolean res;

	    CCD_INIT(ccd);
	    ccd.support1 = CCDTestSupport.ccdSupport;
	    ccd.support2 = CCDTestSupport.ccdSupport;

	    s1.radius = 0.35;
	    s2.radius = .5;

	    ccdVec3Set(s1.pos, -5., 0., 0.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(s1, s2, ccd);

	        if (i < 42 || i > 58){
	            assertFalse(res);
	        }else{
	            assertTrue(res);
	        }

	        s1.pos.add(0, 0.1);
	    }
	}

	@Test
	public void spheresphereAlignedY()
	{
	    ccd_t ccd = new ccd_t();
	    CCDTestSupport.ccd_sphere_t s1 = CCDTestSupport.CCD_SPHERE();
	    CCDTestSupport.ccd_sphere_t s2 = CCDTestSupport.CCD_SPHERE();
	    int i;
	    boolean res;

	    CCD_INIT(ccd);
	    ccd.support1 = CCDTestSupport.ccdSupport;
	    ccd.support2 = CCDTestSupport.ccdSupport;

	    s1.radius = 0.35;
	    s2.radius = .5;

	    ccdVec3Set(s1.pos, 0., -5., 0.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(s1, s2, ccd);

	        if (i < 42 || i > 58){
	            assertFalse(res);
	        }else{
	            assertTrue(res);
	        }

	        s1.pos.add(1, 0.1);
	    }
	}

	@Test
	public void spheresphereAlignedZ()
	{
	    ccd_t ccd = new ccd_t();
	    CCDTestSupport.ccd_sphere_t s1 = CCDTestSupport.CCD_SPHERE();
	    CCDTestSupport.ccd_sphere_t s2 = CCDTestSupport.CCD_SPHERE();
	    int i;
	    boolean res;

	    CCD_INIT(ccd);
	    ccd.support1 = CCDTestSupport.ccdSupport;
	    ccd.support2 = CCDTestSupport.ccdSupport;

	    s1.radius = 0.35;
	    s2.radius = .5;

	    ccdVec3Set(s1.pos, 0., 0., -5.);
	    for (i = 0; i < 100; i++){
	        res = ccdGJKIntersect(s1, s2, ccd);

	        if (i < 42 || i > 58){
	            assertFalse(res);
	        }else{
	            assertTrue(res);
	        }

	        s1.pos.add(2, 0.1);
	    }
	}

}
