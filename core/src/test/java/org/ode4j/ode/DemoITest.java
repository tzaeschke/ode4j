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
package org.ode4j.ode;

import org.junit.Test;
import org.ode4j.math.*;

import static org.junit.Assert.assertTrue;
import static org.ode4j.ode.OdeMath.*;


/**
 * test that the rotational physics is correct.
 * <p>
 * an "anchor body" has a number of other randomly positioned bodies
 * ("particles") attached to it by ball-and-socket joints, giving it some
 * random effective inertia tensor. the effective inertia matrix is calculated,
 * and then this inertia is assigned to another "test" body. a random torque is
 * applied to both bodies and the difference in angular velocity and orientation
 * is observed after a number of iterations.
 * <p>
 * typical errors for each test cycle are about 1e-5 ... 1e-4.
 */
public class DemoITest {

    // some constants

    private static final int NUM = 10;            // number of particles
    private static final float SIDE = 0.1f;        // visual size of the particles


    // dynamics objects an globals

    private static DWorld world = null;
    private static DBody anchor_body;
    private static final DBody[] particle = new DBody[NUM];
    private static DBody test_body;
    private static final DBallJoint[] particle_joint = new DBallJoint[NUM];
    private static final DVector3 torque = new DVector3();
    private static int iteration;


    // compute the mass parameters of a particle set. q = particle positions,
    // pm = particle masses
    private static void computeMassParams(DMass m, DVector3[] q, DVectorN pm) {
        double pmi, q0, q1, q2;
        m.setZero();
        DVector3 C = m.getC().copy();
        DMatrix3 I = m.getI().copy();
        for (int i = 0; i < NUM; i++) {
            pmi = pm.get(i);
            m.setMass(m.getMass() + pmi);// += pmi;
            C.eqSum(C, q[i], pmi);
            q0 = q[i].get0();
            q1 = q[i].get1();
            q2 = q[i].get2();
            I.add(0, 0, pmi * (q1 * q1 + q2 * q2));
            I.add(1, 1, pmi * (q0 * q0 + q2 * q2));
            I.add(2, 2, pmi * (q0 * q0 + q1 * q1));
            I.sub(0, 1, pmi * (q0 * q1));
            I.sub(0, 2, pmi * (q0 * q2));
            I.sub(1, 2, pmi * (q1 * q2));
        }
        //for (j=0; j<3; j++) m.c.v[j] /= m.mass;
        C.scale(1. / m.getMass());
        I.set(1, 0, I.get(0, 1));
        I.set(2, 0, I.get(0, 2));
        I.set(2, 1, I.get(1, 2));
        m.setC(C);
        m.setI(I);
    }

    private static void reset_test() {
        int i;
        DMass m = OdeHelper.createMass(), anchor_m = OdeHelper.createMass();
        //float q[NUM][3], pm[NUM];	// particle positions and masses
        DVector3[] q = DVector3.newArray(NUM);
        DVectorN pm = new DVectorN(NUM);
        DVector3 pos1 = new DVector3(1, 0, 1);    // point of reference (POR)
        DVector3 pos2 = new DVector3(-1, 0, 1);    // point of reference (POR)

        // make random particle positions (relative to POR) and masses
        for (i = 0; i < NUM; i++) {
            pm.set(i, dRandReal() + 0.1);
            q[i].set0(dRandReal() - 0.5);
            q[i].set1(dRandReal() - 0.5);
            q[i].set2(dRandReal() - 0.5);
        }

        // adjust particle positions so centor of mass = POR
        computeMassParams(m, q, pm);
        for (i = 0; i < NUM; i++) {
            q[i].sub(m.getC());
        }

        if (world != null) world.destroy();
        world = OdeHelper.createWorld();

        anchor_body = OdeHelper.createBody(world);
        anchor_body.setPosition(pos1);
        anchor_m.setBox(1, SIDE, SIDE, SIDE);
        anchor_m.adjust(0.1);
        anchor_body.setMass(anchor_m);

        for (i = 0; i < NUM; i++) {
            particle[i] = OdeHelper.createBody(world);
            particle[i].setPosition(
                    pos1.get0() + q[i].get0(),
                    pos1.get1() + q[i].get1(),
                    pos1.get2() + q[i].get2());
            m.setBox(1, SIDE, SIDE, SIDE);
            m.adjust(pm.get(i));
            particle[i].setMass(m);
        }

        for (i = 0; i < NUM; i++) {
            particle_joint[i] = OdeHelper.createBallJoint(world, null);
            particle_joint[i].attach(anchor_body, particle[i]);
            DVector3C p = particle[i].getPosition();
            particle_joint[i].setAnchor(p);
        }

        // make test_body with the same mass and inertia of the anchor_body plus
        // all the particles
        test_body = OdeHelper.createBody(world);
        test_body.setPosition(pos2);
        computeMassParams(m, q, pm);
        m.setMass(m.getMass() + anchor_m.getMass()); //+= anchor_m._mass;
        //for (i=0; i<12; i++) m.I.v[i] = m.I.v[i] + anchor_m.I.v[i];
        m.setI(m.getI().copy().add(anchor_m.getI()));
        test_body.setMass(m);

        // rotate the test and anchor bodies by a random amount
        DQuaternion qrot = new DQuaternion();
        for (i = 0; i < 4; i++) qrot.set(i, dRandReal() - 0.5);
        dNormalize4(qrot);
        anchor_body.setQuaternion(qrot);
        test_body.setQuaternion(qrot);
        DMatrix3 R = new DMatrix3();
        dRfromQ(R, qrot);
        for (i = 0; i < NUM; i++) {
            DVector3 v = new DVector3();
//			dMultiply0 (v,R,q[i][0],3,3,1);
//			dMultiply0 (v.v,R.v,q.v,i*3,3,3,1);
            dMultiply0(v, R, q[i]);
            particle[i].setPosition(v.add(pos1));
        }

        // set random torque
        for (i = 0; i < 3; i++) torque.set(i, (dRandReal() - 0.5) * 0.1);

        iteration = 0;
    }


    // simulation loop

    private void simLoop() {
        anchor_body.addTorque(torque);
        test_body.addTorque(torque);
        world.step(0.03);

        iteration++;
        if (iteration >= 100) {
            // measure the difference between the anchor and test bodies
            DVector3C w1 = anchor_body.getAngularVel();
            DVector3C w2 = test_body.getAngularVel();
            DQuaternionC q1 = anchor_body.getQuaternion();
            DQuaternionC q2 = test_body.getQuaternion();
            double maxdiff = dMaxDifference(w1, w2);
            double eps = 1e-3;
            assertTrue("w-error = " + maxdiff + "  (" + w1 + ") and (" + w2 + ")", maxdiff < eps);
            maxdiff = dMaxDifference(q1, q2, 1, 4);
            assertTrue("q-error = " + maxdiff, maxdiff < 1e-3);
            reset_test();
        }
    }

    @Test
    public void demo() {
        OdeHelper.initODE2(0);
        dRandSetSeed(0); // System.currentTimeMillis());
        reset_test();

        // run simulation
        for (int i = 0; i < 1000; ++i) {
            simLoop();
        }

        world.destroy();
        OdeHelper.closeODE();
    }
}