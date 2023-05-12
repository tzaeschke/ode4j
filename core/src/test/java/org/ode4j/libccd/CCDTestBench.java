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

import static org.ode4j.ode.internal.libccd.CCD.*;
import static org.ode4j.ode.internal.libccd.CCDQuat.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;

import org.junit.Assert;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;

public class CCDTestBench {

	private static int bench_num = 1;
	private static int cycles = 10000;

	private static void runBench(final Object o1, final Object o2, final ccd_t ccd)
	{
	    RefDouble depth = new RefDouble();
	    ccd_vec3_t dir = new ccd_vec3_t(), pos = new ccd_vec3_t();
	    int res = 0;
	    int i;
	    //const struct timespec *timer;

	    long t1 = System.currentTimeMillis();//cuTimerStart();
	    for (i = 0; i < cycles; i++){
	        res = ccdGJKPenetration(o1, o2, ccd, depth, dir, pos);
	    }
	    long t2 = System.currentTimeMillis();//timer = cuTimerStop();
//	    fprintf(stdout, "%02d: %ld %ld\n", bench_num,
//	                    (long)timer.tv_sec, (long)timer.tv_nsec);
//	    fflush(stdout);
	    System.out.println(bench_num + ": " + (t2-t1));

	    //TZ to avoid inlining and warning
	    Assert.assertTrue(res > -10);

	    bench_num++;
	}

	private static void boxbox()
	{
	    //fprintf(stdout, "%s:\n", __func__);
		System.out.println("CCDBench.boxbox(): ");

	    ccd_t ccd = new ccd_t();
	    CCDTestSupport.ccd_box_t box1 = CCDTestSupport.CCD_BOX();
	    CCDTestSupport.ccd_box_t box2 = CCDTestSupport.CCD_BOX();
	    ccd_vec3_t axis = new ccd_vec3_t();
	    ccd_quat_t rot = new ccd_quat_t();

	    box1.x = box1.y = box1.z = 1.;
	    box2.x = 0.5;
	    box2.y = 1.;
	    box2.z = 1.5;

	    bench_num = 1;

	    CCD_INIT(ccd);
	    ccd.support1 = CCDTestSupport.ccdSupport;
	    ccd.support2 = CCDTestSupport.ccdSupport;

	    runBench(box1, box2, ccd);
	    runBench(box2, box1, ccd);

	    ccdVec3Set(box1.pos, -0.3, 0.5, 1.);
	    runBench(box1, box2, ccd);
	    runBench(box2, box1, ccd);

	    box1.x = box1.y = box1.z = 1.;
	    box2.x = box2.y = box2.z = 1.;
	    ccdVec3Set(axis, 0., 0., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, 0., 0., 0.);
	    runBench(box1, box2, ccd);
	    runBench(box2, box1, ccd);

	    box1.x = box1.y = box1.z = 1.;
	    box2.x = box2.y = box2.z = 1.;
	    ccdVec3Set(axis, 0., 0., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, -0.5, 0., 0.);
	    runBench(box1, box2, ccd);
	    runBench(box2, box1, ccd);

	    box1.x = box1.y = box1.z = 1.;
	    box2.x = box2.y = box2.z = 1.;
	    ccdVec3Set(axis, 0., 0., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, -0.5, 0.5, 0.);
	    runBench(box1, box2, ccd);
	    runBench(box2, box1, ccd);

	    box1.x = box1.y = box1.z = 1.;
	    ccdVec3Set(axis, 0., 1., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(box1.pos, -0.5, 0.1, 0.4);
	    runBench(box1, box2, ccd);
	    runBench(box2, box1, ccd);

	    box1.x = box1.y = box1.z = 1.;
	    ccdVec3Set(axis, 0., 1., 1.);
	    ccdQuatSetAngleAxis(box1.quat, M_PI / 4., axis);
	    ccdVec3Set(axis, 1., 1., 1.);
	    ccdQuatSetAngleAxis(rot, M_PI / 4., axis);
	    ccdQuatMul(box1.quat, rot);
	    ccdVec3Set(box1.pos, -0.5, 0.1, 0.4);
	    runBench(box1, box2, ccd);
	    runBench(box2, box1, ccd);


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
	    runBench(box1, box2, ccd);
	    runBench(box2, box1, ccd);


	    //fprintf(stdout, "\n----\n\n");
	    System.out.println("\n----\n");
	}

	private static void cylcyl()
	{
	    //fprintf(stdout, "%s:\n", __func__);
		System.out.println("CCDBench.cylcyl(): ");

	    ccd_t ccd = new ccd_t();
	    CCDTestSupport.ccd_cyl_t cyl1 = CCDTestSupport.CCD_CYL();
	    CCDTestSupport.ccd_cyl_t cyl2 = CCDTestSupport.CCD_CYL();
	    ccd_vec3_t axis = new ccd_vec3_t();

	    cyl1.radius = 0.35;
	    cyl1.height = 0.5;
	    cyl2.radius = 0.5;
	    cyl2.height = 1.;

	    CCD_INIT(ccd);
	    ccd.support1 = CCDTestSupport.ccdSupport;
	    ccd.support2 = CCDTestSupport.ccdSupport;

	    runBench(cyl1, cyl2, ccd);
	    runBench(cyl2, cyl1, ccd);

	    ccdVec3Set(cyl1.pos, 0.3, 0.1, 0.1);
	    runBench(cyl1, cyl2, ccd);
	    runBench(cyl2, cyl1, ccd);

	    ccdVec3Set(axis, 0., 1., 1.);
	    ccdQuatSetAngleAxis(cyl2.quat, M_PI / 4., axis);
	    ccdVec3Set(cyl2.pos, 0., 0., 0.);
	    runBench(cyl1, cyl2, ccd);
	    runBench(cyl2, cyl1, ccd);

	    ccdVec3Set(axis, 0., 1., 1.);
	    ccdQuatSetAngleAxis(cyl2.quat, M_PI / 4., axis);
	    ccdVec3Set(cyl2.pos, -0.2, 0.7, 0.2);
	    runBench(cyl1, cyl2, ccd);
	    runBench(cyl2, cyl1, ccd);

	    ccdVec3Set(axis, 0.567, 1.2, 1.);
	    ccdQuatSetAngleAxis(cyl2.quat, M_PI / 4., axis);
	    ccdVec3Set(cyl2.pos, 0.6, -0.7, 0.2);
	    runBench(cyl1, cyl2, ccd);
	    runBench(cyl2, cyl1, ccd);

	    ccdVec3Set(axis, -4.567, 1.2, 0.);
	    ccdQuatSetAngleAxis(cyl2.quat, M_PI / 3., axis);
	    ccdVec3Set(cyl2.pos, 0.6, -0.7, 0.2);
	    runBench(cyl1, cyl2, ccd);
	    runBench(cyl2, cyl1, ccd);

	    //fprintf(stdout, "\n----\n\n");
	    System.out.println("\n----\n");
	}

	private static void boxcyl()
	{
	    //fprintf(stdout, "%s:\n", __func__);
		System.out.println("CCDBench.boxcyl(): ");

	    ccd_t ccd = new ccd_t();
	    CCDTestSupport.ccd_box_t box = CCDTestSupport.CCD_BOX();
	    CCDTestSupport.ccd_cyl_t cyl = CCDTestSupport.CCD_CYL();
	    ccd_vec3_t axis = new ccd_vec3_t();

	    box.x = 0.5;
	    box.y = 1.;
	    box.z = 1.5;
	    cyl.radius = 0.4;
	    cyl.height = 0.7;

	    CCD_INIT(ccd);
	    ccd.support1 = CCDTestSupport.ccdSupport;
	    ccd.support2 = CCDTestSupport.ccdSupport;

	    runBench(box, cyl, ccd);
	    runBench(cyl, box, ccd);

	    ccdVec3Set(cyl.pos, .6, 0., 0.);
	    runBench(box, cyl, ccd);
	    runBench(cyl, box, ccd);

	    ccdVec3Set(cyl.pos, .6, 0.6, 0.);
	    runBench(box, cyl, ccd);
	    runBench(cyl, box, ccd);

	    ccdVec3Set(cyl.pos, .6, 0.6, 0.5);
	    runBench(box, cyl, ccd);
	    runBench(cyl, box, ccd);

	    ccdVec3Set(axis, 0., 1., 0.);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 3., axis);
	    ccdVec3Set(cyl.pos, .6, 0.6, 0.5);
	    runBench(box, cyl, ccd);
	    runBench(cyl, box, ccd);

	    ccdVec3Set(axis, 0.67, 1.1, 0.12);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 4., axis);
	    ccdVec3Set(cyl.pos, .6, 0., 0.5);
	    runBench(box, cyl, ccd);
	    runBench(cyl, box, ccd);

	    ccdVec3Set(axis, -0.1, 2.2, -1.);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 5., axis);
	    ccdVec3Set(cyl.pos, .6, 0., 0.5);
	    ccdVec3Set(axis, 1., 1., 0.);
	    ccdQuatSetAngleAxis(box.quat, -M_PI / 4., axis);
	    ccdVec3Set(box.pos, .6, 0., 0.5);
	    runBench(box, cyl, ccd);
	    runBench(cyl, box, ccd);

	    ccdVec3Set(axis, -0.1, 2.2, -1.);
	    ccdQuatSetAngleAxis(cyl.quat, M_PI / 5., axis);
	    ccdVec3Set(cyl.pos, .6, 0., 0.5);
	    ccdVec3Set(axis, 1., 1., 0.);
	    ccdQuatSetAngleAxis(box.quat, -M_PI / 4., axis);
	    ccdVec3Set(box.pos, .9, 0.8, 0.5);
	    runBench(box, cyl, ccd);
	    runBench(cyl, box, ccd);

	    //fprintf(stdout, "\n----\n\n");
	    System.out.println("\n----\n");
	}

	public static void main(String[] args) {
	    if (args.length > 1){
	        cycles = Integer.parseInt(args[0]);
	    }

//	    fprintf(stdout, "Cycles: %zu\n", cycles);
//	    fprintf(stdout, "\n");

	    boxbox();
	    cylcyl();
	    boxcyl();

	    //return 0;
	}

}
