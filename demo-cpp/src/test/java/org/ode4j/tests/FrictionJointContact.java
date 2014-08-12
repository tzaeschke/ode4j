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
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.tests;

import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

import java.util.Arrays;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJoint.Info1;
import org.ode4j.ode.internal.joints.DxJoint.Info2Descr;
import org.ode4j.ode.internal.joints.DxJointContact;
import org.ode4j.tests.UnitTestPlusPlus.TestSuperClass;

//234567890123456789012345678901234567890123456789012345678901234567890123456789
//        1         2         3         4         5         6         7

////////////////////////////////////////////////////////////////////////////////
// This file create unit test for some of the functions found in:
// ode/src/joint.cpp
//
//
////////////////////////////////////////////////////////////////////////////////


/**
 * Tests for contact friction
 */

//SUITE(JointContact)
public class FrictionJointContact extends TestSuperClass
{
//    private static class ContactSetup
//    {
	private DWorld world;
	private DBody body1;
	private DBody body2;
	private DContactJoint joint;

        //ContactSetup()
	@Before
        public void FrictionJointContact_init()
        {
            world = OdeHelper.createWorld();
            body1 = OdeHelper.createBody(world);
            body2 = OdeHelper.createBody(world);

            body1.setPosition(-1, 0, 0);
            body2.setPosition( 1, 0, 0);
        }

        //~ContactSetup()
	@After
        public void DESTRUCTOR()
        {
            body1.destroy();
            body2.destroy();
            world.destroy();
        }
//    };
    
        //TZ copied from below
        private void ZERO_ALL(double[] dummy_J, double[] dummy_c, double[] dummy_cfm, double[] dummy_lo, double[] dummy_hi, int[] dummy_findex) {               
//            memset(dummy_J, 0, sizeof dummy_J);                 
//            memset(dummy_c, 0, sizeof dummy_c);                 
//            memset(dummy_cfm, 0, sizeof dummy_cfm);             
//            memset(dummy_lo, 0, sizeof dummy_lo);               
//            memset(dummy_hi, 0, sizeof dummy_hi);               
//            std::fill(dummy_findex, dummy_findex+3, -1);
       		Arrays.fill(dummy_J, 0);
    		Arrays.fill(dummy_c, 0);
    		Arrays.fill(dummy_cfm, 0);
    		Arrays.fill(dummy_lo, 0);
    		Arrays.fill(dummy_hi, 0);
    		Arrays.fill(dummy_findex, 0, 3, -1);
        }                                                       
    
        
    //TEST_FIXTURE(ContactSetup, test_ZeroMu)
//    static class Fixture_ContactSetup_test_ZeroMu
    @Test public void Fixture_ContactSetup_test_ZeroMu()
    {
    	
        DxJoint.Info1 info1 = new Info1();
        DxJoint.Info2Descr info2 = new Info2Descr();
        //double[][] dummy_J = new double[3][12];// = new double = {{0}};
        double[] dummy_J = new double[3*12];// = new double = {{0}};
        double[] dummy_c = new double[3];
        double[] dummy_cfm = new double[3];
        double[] dummy_lo = new double[3];
        double[] dummy_hi = new double[3];
        int[] dummy_findex = new int[3];

        double info2_fps = 100;
        double info2_erp = 0;
//        info2.J1l = dummy_J[0];
//        info2.J1a = dummy_J[0] + 3;
//        info2.J2l = dummy_J[0] + 6;
//        info2.J2a = dummy_J[0] + 9;
        info2.J1lp = 0;//dummy_J[0];
        info2.J1ap = 3;//dummy_J[0] + 3;
        info2.J2lp = 6;//dummy_J[0] + 6;
        info2.J2ap = 9;//dummy_J[0] + 9;
//        info2.rowskip = 12;
//        info2.c = dummy_c;
//        info2.cfm = dummy_cfm;
//        info2.lo = dummy_lo;
//        info2.hi = dummy_hi;
//        info2.findex = dummy_findex;
        info2.setRowskip(12);
        info2.setArrays(dummy_J, dummy_c, dummy_cfm, dummy_lo, dummy_hi, dummy_findex);

        //TZ moved to above
//#define ZERO_ALL do {                                           \
//            memset(dummy_J, 0, sizeof dummy_J);                 \
//            memset(dummy_c, 0, sizeof dummy_c);                 \
//            memset(dummy_cfm, 0, sizeof dummy_cfm);             \
//            memset(dummy_lo, 0, sizeof dummy_lo);               \
//            memset(dummy_hi, 0, sizeof dummy_hi);               \
//            std::fill(dummy_findex, dummy_findex+3, -1);;       \
//        }                                                       \
//        while (0)
        	
        DContactBuffer b = new DContactBuffer(1);
        DContact contact = b.get(0);
        contact.surface.mode = OdeConstants.dContactMu2 | 
        		OdeConstants.dContactFDir1 | 
        		OdeConstants.dContactApprox1;

        contact.geom.pos.set(0, 0, 0);
        //contact.geom.pos[1] = 0;
        //contact.geom.pos[2] = 0;

        // normal points into body1
        contact.geom.normal.set(-1, 0, 0);
        //contact.geom.normal[1] = 0;
        //contact.geom.normal[2] = 0;

        contact.geom.depth = 0;

        contact.geom.g1 = null;
        contact.geom.g2 = null;

        // we ask for fdir1 = +Y, so fdir2 = normal x fdir1 = -Z
        contact.fdir1.set(0, 1, 0);
        //contact.fdir1[1] = 1;
        //contact.fdir1[2] = 0;

        /*
         * First, test with mu = 0, mu2 = 1
         * Because there is no friction on the first direction (+Y) the body
         * is allowed to translate in the Y axis and rotate around the Z axis.
         *
         * That is, the only constraint will be for the second dir (-Z):
         * so J[1] = [  0  0 -1    0  1  0    0  0  1    0  1  0 ]
         */        
        contact.surface.mu = 0;
        contact.surface.mu2 = 1;
        joint = OdeHelper.createContactJoint(world, null, contact);
        joint.attach(body1, body2);
        ((DxJointContact)joint).getInfo1(info1);
        CHECK_EQUAL(2, info1.m);
        ZERO_ALL(dummy_J, dummy_c, dummy_cfm, dummy_lo, dummy_hi, dummy_findex);
        ((DxJointContact)joint).getInfo2(info2_fps, info2_erp, info2);
//        CHECK_CLOSE(0, dummy_J[1][0], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][1], 1e-6);
//        CHECK_CLOSE(-1, dummy_J[1][2], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][3], 1e-6);
//        CHECK_CLOSE(1, dummy_J[1][4], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][5], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][6], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][7], 1e-6);
//        CHECK_CLOSE(1, dummy_J[1][8], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][9], 1e-6);
//        CHECK_CLOSE(1, dummy_J[1][10], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][11], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+0], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+1], 1e-6);
        CHECK_CLOSE(-1, dummy_J[1*12+2], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+3], 1e-6);
        CHECK_CLOSE(1, dummy_J[1*12+4], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+5], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+6], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+7], 1e-6);
        CHECK_CLOSE(1, dummy_J[1*12+8], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+9], 1e-6);
        CHECK_CLOSE(1, dummy_J[1*12+10], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+11], 1e-6);
        CHECK_EQUAL(0, dummy_findex[1]); // because of dContactApprox1
        joint.destroy();


        /*
         * Now try with no frictino in the second direction. The Jacobian should look like:
         * J[1] = [  0  1  0    0  0  1    0 -1  0    0  0  1 ]
         */
        // try again, with zero mu2
        contact.surface.mu = 1;
        contact.surface.mu2 = 0;
        joint = OdeHelper.createContactJoint(world, null, contact);
        joint.attach(body1, body2);
        ((DxJointContact)joint).getInfo1(info1);
        CHECK_EQUAL(2, info1.m);
        ZERO_ALL(dummy_J, dummy_c, dummy_cfm, dummy_lo, dummy_hi, dummy_findex);
        ((DxJointContact)joint).getInfo2(info2_fps, info2_erp, info2);
//        CHECK_CLOSE(0, dummy_J[1][0], 1e-6);
//        CHECK_CLOSE(1, dummy_J[1][1], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][2], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][3], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][4], 1e-6);
//        CHECK_CLOSE(1, dummy_J[1][5], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][6], 1e-6);
//        CHECK_CLOSE(-1, dummy_J[1][7], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][8], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][9], 1e-6);
//        CHECK_CLOSE(0, dummy_J[1][10], 1e-6);
//        CHECK_CLOSE(1, dummy_J[1][11], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+0], 1e-6);
        CHECK_CLOSE(1, dummy_J[1*12+1], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+2], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+3], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+4], 1e-6);
        CHECK_CLOSE(1, dummy_J[1*12+5], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+6], 1e-6);
        CHECK_CLOSE(-1, dummy_J[1*12+7], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+8], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+9], 1e-6);
        CHECK_CLOSE(0, dummy_J[1*12+10], 1e-6);
        CHECK_CLOSE(1, dummy_J[1*12+11], 1e-6);
        CHECK_EQUAL(0, dummy_findex[1]);  // because of dContactApprox1
        joint.destroy();
    }

}
