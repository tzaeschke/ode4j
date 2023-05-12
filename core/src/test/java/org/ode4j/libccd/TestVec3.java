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

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;

public class TestVec3 {

	@BeforeClass
	public static void vec3SetUp()
	{
	}

	@AfterClass
	public static void vec3TearDown()
	{
	}


	@Test
	public void vec3PointSegmentDist()
	{
	    final ccd_vec3_t P = new ccd_vec3_t(), a = new ccd_vec3_t();
	    final ccd_vec3_t b = new ccd_vec3_t(), w = new ccd_vec3_t(), ew = new ccd_vec3_t();
	    double dist;

	    ccdVec3Set(a, 0., 0., 0.);
	    ccdVec3Set(b, 1., 0., 0.);

	    // extereme w == a
	    ccdVec3Set(P, -1., 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 1.));
	    assertTrue(ccdVec3Eq(w, a));

	    ccdVec3Set(P, -0.5, 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 0.5 * 0.5));
	    assertTrue(ccdVec3Eq(w, a));

	    ccdVec3Set(P, -0.1, 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, .1 * .1));
	    assertTrue(ccdVec3Eq(w, a));

	    ccdVec3Set(P, 0., 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, a));

	    ccdVec3Set(P, -1., 1., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 2.));
	    assertTrue(ccdVec3Eq(w, a));

	    ccdVec3Set(P, -0.5, 0.5, 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 0.5));
	    assertTrue(ccdVec3Eq(w, a));

	    ccdVec3Set(P, -0.1, -1., 2.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 5.01));
	    assertTrue(ccdVec3Eq(w, a));


	    // extereme w == b
	    ccdVec3Set(P, 2., 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 1.));
	    assertTrue(ccdVec3Eq(w, b));

	    ccdVec3Set(P, 1.5, 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 0.5 * 0.5));
	    assertTrue(ccdVec3Eq(w, b));

	    ccdVec3Set(P, 1.1, 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, .1 * .1));
	    assertTrue(ccdVec3Eq(w, b));

	    ccdVec3Set(P, 1., 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, b));

	    ccdVec3Set(P, 2., 1., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 2.));
	    assertTrue(ccdVec3Eq(w, b));

	    ccdVec3Set(P, 1.5, 0.5, 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 0.5));
	    assertTrue(ccdVec3Eq(w, b));

	    ccdVec3Set(P, 1.1, -1., 2.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 5.01));
	    assertTrue(ccdVec3Eq(w, b));

	    // inside segment
	    ccdVec3Set(P, .5, 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, P));

	    ccdVec3Set(P, .9, 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, P));

	    ccdVec3Set(P, .5, 1., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 1.));
	    ccdVec3Set(ew, 0.5, 0., 0.);
	    assertTrue(ccdVec3Eq(w, ew));

	    ccdVec3Set(P, .5, 1., 1.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 2.));
	    ccdVec3Set(ew, 0.5, 0., 0.);
	    assertTrue(ccdVec3Eq(w, ew));



	    ccdVec3Set(a, -.5, 2., 1.);
	    ccdVec3Set(b, 1., 1.5, 0.5);

	    // extereme w == a
	    ccdVec3Set(P, -10., 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 9.5 * 9.5 + 2. * 2. + 1.));
	    assertTrue(ccdVec3Eq(w, a));

	    ccdVec3Set(P, -10., 9.2, 3.4);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 9.5 * 9.5 + 7.2 * 7.2 + 2.4 * 2.4));
	    assertTrue(ccdVec3Eq(w, a));

	    // extereme w == b
	    ccdVec3Set(P, 10., 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 9. * 9. + 1.5 * 1.5 + 0.5 * 0.5));
	    assertTrue(ccdVec3Eq(w, b));

	    ccdVec3Set(P, 10., 9.2, 3.4);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 9. * 9. + 7.7 * 7.7 + 2.9 * 2.9));
	    assertTrue(ccdVec3Eq(w, b));

	    // inside ab
	    ccdVec3Set(a, -.1, 1., 1.);
	    ccdVec3Set(b, 1., 1., 1.);
	    ccdVec3Set(P, 0., 0., 0.);
	    dist = ccdVec3PointSegmentDist2(P, a, b, w);
	    assertTrue(ccdEq(dist, 2.));
	    ccdVec3Set(ew, 0., 1., 1.);
	    assertTrue(ccdVec3Eq(w, ew));
	}


	@Test
	public void vec3PointTriDist()
	{
	    final ccd_vec3_t P = new ccd_vec3_t(), a = new ccd_vec3_t(), b = new ccd_vec3_t();
	    final ccd_vec3_t c = new ccd_vec3_t(), w = new ccd_vec3_t(), P0 = new ccd_vec3_t();
	    double dist;

	    ccdVec3Set(a, -1., 0., 0.);
	    ccdVec3Set(b, 0., 1., 1.);
	    ccdVec3Set(c, -1., 0., 1.);

	    ccdVec3Set(P, -1., 0., 0.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, a));

	    ccdVec3Set(P, 0., 1., 1.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, b));

	    ccdVec3Set(P, -1., 0., 1.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, c));

	    ccdVec3Set(P, 0., 0., 0.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, null);
	    assertTrue(ccdEq(dist, 2./3.));


	    // region 4
	    ccdVec3Set(P, -2., 0., 0.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, ccdVec3Dist2(P, a)));
	    assertTrue(ccdVec3Eq(w, a));
	    ccdVec3Set(P, -2., 0.2, -1.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, ccdVec3Dist2(P, a)));
	    assertTrue(ccdVec3Eq(w, a));

	    // region 2
	    ccdVec3Set(P, -1.3, 0., 1.2);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, ccdVec3Dist2(P, c)));
	    assertTrue(ccdVec3Eq(w, c));
	    ccdVec3Set(P, -1.2, 0.2, 1.1);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, ccdVec3Dist2(P, c)));
	    assertTrue(ccdVec3Eq(w, c));

	    // region 6
	    ccdVec3Set(P, 0.3, 1., 1.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, ccdVec3Dist2(P, b)));
	    assertTrue(ccdVec3Eq(w, b));
	    ccdVec3Set(P, .1, 1., 1.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, ccdVec3Dist2(P, b)));
	    assertTrue(ccdVec3Eq(w, b));

	    // region 1
	    ccdVec3Set(P, 0., 1., 2.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 1.));
	    assertTrue(ccdVec3Eq(w, b));
	    ccdVec3Set(P, -1., 0., 2.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 1.));
	    assertTrue(ccdVec3Eq(w, c));
	    ccdVec3Set(P, -0.5, 0.5, 2.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 1.));
	    ccdVec3Set(P0, -0.5, 0.5, 1.);
	    assertTrue(ccdVec3Eq(w, P0));

	    // region 3
	    ccdVec3Set(P, -2., -1., 0.7);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 2.));
	    ccdVec3Set(P0, -1., 0., 0.7);
	    assertTrue(ccdVec3Eq(w, P0));

	    // region 5
	    ccdVec3Set(P, 0., 0., 0.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 2./3.));
	    ccdVec3Set(P0, -2./3., 1./3., 1./3.);
	    assertTrue(ccdVec3Eq(w, P0));

	    // region 0
	    ccdVec3Set(P, -0.5, 0.5, 0.5);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, P));
	    ccdVec3Set(P, -0.5, 0.5, 0.7);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, P));
	    ccdVec3Set(P, -0.5, 0.5, 0.9);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 0.));
	    assertTrue(ccdVec3Eq(w, P));

	    ccdVec3Set(P, 0., 0., 0.5);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 0.5));
	    ccdVec3Set(P0, -.5, .5, .5);
	    assertTrue(ccdVec3Eq(w, P0));

	    ccdVec3Set(a, -1., 0., 0.);
	    ccdVec3Set(b, 0., 1., -1.);
	    ccdVec3Set(c, 0., 1., 1.);
	    ccdVec3Set(P, 0., 0., 0.);
	    dist = ccdVec3PointTriDist2(P, a, b, c, w);
	    assertTrue(ccdEq(dist, 0.5));
	    ccdVec3Set(P0, -.5, .5, 0.);
	    assertTrue(ccdVec3Eq(w, P0));
	    //fprintf(stderr, "dist: %lf\n", dist);
	}

}
