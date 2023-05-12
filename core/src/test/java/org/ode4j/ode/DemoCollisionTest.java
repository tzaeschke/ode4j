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

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.internal.DxBox;
import org.ode4j.ode.internal.DxCollisionUtil;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import static org.junit.Assert.*;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.ErrorHandler.dDebug;

/**
 * collision tests. if this program is run without any arguments it will
 * perform all the tests multiple times, with different random data for each
 * test. if this program is given a test number it will run that test
 * graphically/interactively, in which case the space bar can be used to
 * change the random test conditions.
 */
public class DemoCollisionTest {

    private static final boolean PRINT = false;

    //****************************************************************************
    // test infrastructure, including constants and macros

    private static final int TEST_REPS1 = 1000;        // run each test this many times (first batch)
    private static final int TEST_REPS2 = 10000;    // run each test this many times (second batch)
    private static final float tol = 1e-8f;        // tolerance used for numerical checks

    private boolean runtest(String name) {
        try {
            Method m = DemoCollisionTest.class.getDeclaredMethod(name);
            return (Boolean) m.invoke(this);
        } catch (SecurityException | NoSuchMethodException | IllegalArgumentException | IllegalAccessException |
                 InvocationTargetException e) {
            if (e.getCause() instanceof RuntimeException) {
                throw (RuntimeException) e.getCause();
            }
            if (e.getCause() instanceof AssertionError) {
                throw (AssertionError) e.getCause();
            }
            throw new RuntimeException(e.getCause());
        }
    }

    private boolean testFAILED() {
        throw new RuntimeException();
    }

    private boolean retPASSED() {
        return true;
    }

    //****************************************************************************
    // globals

    /* int dBoxBox (const dVector3 p1, const dMatrix3 R1,
         const dVector3 side1, const dVector3 p2,
         const dMatrix3 R2, const dVector3 side2,
         dVector3 normal, dReal *depth, int *code,
         int maxc, dContactGeom *contact, int skip); */

    //void dLineClosestApproach (final dVector3 pa, final dVector3 ua,
    //			   final dVector3 pb, final dVector3 ub,
    //			   double *alpha, double *beta);

    //****************************************************************************
    // point depth tests

    private boolean test_sphere_point_depth() {
        int j;
        DVector3 p = new DVector3(), q = new DVector3();
        DMatrix3 R = new DMatrix3();
        double r, d;

        //dSimpleSpace space(0);
        DSimpleSpace space = OdeHelper.createSimpleSpace(null);
        DSphere sphere = OdeHelper.createSphere(null, 1);
        space.add(sphere);

        // ********** make a random sphere of radius r at position p

        r = dRandReal() + 0.1;
        sphere.setRadius(r);
        dMakeRandomVector(p, 1.0);
        sphere.setPosition(p);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        sphere.setRotation(R);

        // ********** test center point has depth r

        if (dFabs(sphere.getPointDepth(p) - r) > tol) if (testFAILED()) return false;

        // ********** test point on surface has depth 0

        for (j = 0; j < 3; j++) q.set(j, dRandReal() - 0.5);
        q.normalize();
        q.eqSum(p, q, r);
        if (dFabs(sphere.getPointDepth(q)) > tol) if (testFAILED()) return false;

        // ********** test point at random depth

        d = (dRandReal() * 2 - 1) * r;
        for (j = 0; j < 3; j++) q.set(j, dRandReal() - 0.5);
        q.normalize();
        q.eqSum(p, q, (r - d));
        if (dFabs(sphere.getPointDepth(q) - d) > tol) if (testFAILED()) return false;

        return retPASSED();
    }


    private boolean test_box_point_depth() {
        int i, j;
        DVector3 s = new DVector3(), p = new DVector3(), q = new DVector3(), q2 = new DVector3();    // s = box sides
        DMatrix3 R = new DMatrix3();
        double ss, d;        // ss = smallest side

        DSimpleSpace space = OdeHelper.createSimpleSpace(null);
        DBox box = OdeHelper.createBox(null, 1, 1, 1);
        space.add(box);

        // ********** make a random box

        for (j = 0; j < 3; j++) s.set(j, dRandReal() + 0.1);
        box.setLengths(s);
        dMakeRandomVector(p, 1.0);
        box.setPosition(p);
        double angle = dRandReal();
        double az = dRandReal();
        double ay = dRandReal();
        double ax = dRandReal();
        dRFromAxisAndAngle(R, ax * 2 - 1, ay * 2 - 1, az * 2 - 1, angle * 10 - 5);
//        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
//                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        box.setRotation(R);

        // ********** test center point has depth of smallest side

        ss = 1e9;
        for (j = 0; j < 3; j++) if (s.get(j) < ss) ss = s.get(j);
        if (dFabs(box.getPointDepth(p) - 0.5 * ss) > tol)
            if (testFAILED()) return false;

        // ********** test point on surface has depth 0

        for (j = 0; j < 3; j++) q.set(j, (dRandReal() - 0.5) * s.get(j));
        i = dRandInt(3);
        if (dRandReal() > 0.5) q.set(i, 0.5 * s.get(i));
        else q.set(i, -0.5 * s.get(i));
        dMultiply0(q2, box.getRotation(), q);
        //for (j=0; j<3; j++) q2[j] += p[j];
        q2.add(p);
        if (dFabs(box.getPointDepth(q2)) > tol) if (testFAILED()) return false;

        // ********** test points outside box have -ve depth

        for (j = 0; j < 3; j++) {
            q.set(j, 0.5 * s.get(j) + dRandReal() + 0.01);
            if (dRandReal() > 0.5) q.scale(i, -1);
        }
        dMultiply0(q2, box.getRotation(), q);
        q2.add(p);
        if (box.getPointDepth(q2) >= 0) if (testFAILED()) return false;

        // ********** test points inside box have +ve depth

        for (j = 0; j < 3; j++) q.set(j, s.get(j) * 0.99 * (dRandReal() - 0.5));
        dMultiply0(q2, box.getRotation(), q);
        q2.add(p);
        if (box.getPointDepth(q2) <= 0) if (testFAILED()) return false;

        // ********** test random depth of point aligned along axis (up to ss deep)

        i = dRandInt(3);
        q.setZero();
        d = (dRandReal() * (ss * 0.5 + 1) - 1);
        q.set(i, s.get(i) * 0.5 - d);
        if (dRandReal() > 0.5) q.scale(i, -1);
        dMultiply0(q2, box.getRotation(), q);
        q2.add(p);
        if (dFabs(box.getPointDepth(q2) - d) >= tol) if (testFAILED()) return false;

        return retPASSED();
    }


    private boolean test_ccylinder_point_depth() {
        int j;
        DVector3 p = new DVector3(), a = new DVector3();
        DMatrix3 R = new DMatrix3();
        double r, l, beta, x, y, d;

        DSimpleSpace space = OdeHelper.createSimpleSpace(null);

        DCapsule ccyl = OdeHelper.createCapsule(null, 1, 1);
        space.add(ccyl);

        // ********** make a random ccyl

        r = dRandReal() * 0.5 + 0.01;
        l = dRandReal() * 1 + 0.01;
        ccyl.setParams(r, l);
        dMakeRandomVector(p, 1.0);
        ccyl.setPosition(p);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        ccyl.setRotation(R);

        // ********** test point on axis has depth of 'radius'

        beta = dRandReal() - 0.5;
        a.eqSum(p, R.viewCol(2), l * beta);
        if (dFabs(ccyl.getPointDepth(a) - r) >= tol)
            if (testFAILED()) return false;

        // ********** test point on surface (excluding caps) has depth 0

        beta = dRandReal() * 2 * M_PI;
        x = r * Math.sin(beta);
        y = r * Math.cos(beta);
        beta = dRandReal() - 0.5;
        for (j = 0; j < 3; j++) a.set(j, p.get(j) + x * R.get(j, 0) + y * R.get(j, 1) + l * beta * R.get(j, 2));
        if (dFabs(ccyl.getPointDepth(a)) >= tol) if (testFAILED()) return false;

        // ********** test point on surface of caps has depth 0

        for (j = 0; j < 3; j++) a.set(j, dRandReal() - 0.5);
        a.normalize();
        if (dCalcVectorDot3_14(a, R, 2) > 0) {
            for (j = 0; j < 3; j++) a.set(j, p.get(j) + a.get(j) * r + l * 0.5 * R.get(j, 2));
        } else {
            for (j = 0; j < 3; j++) a.set(j, p.get(j) + a.get(j) * r - l * 0.5 * R.get(j, 2));
        }
        if (dFabs(ccyl.getPointDepth(a)) >= tol) if (testFAILED()) return false;

        // ********** test point inside ccyl has positive depth

        for (j = 0; j < 3; j++) a.set(j, dRandReal() - 0.5);
        a.normalize();
        beta = dRandReal() - 0.5;
        for (j = 0; j < 3; j++) a.set(j, p.get(j) + a.get(j) * r * 0.99 + l * beta * R.get(j, 2));
        if (ccyl.getPointDepth(a) < 0) if (testFAILED()) return false;

        // ********** test point depth (1)

        d = (dRandReal() * 2 - 1) * r;
        beta = dRandReal() * 2 * M_PI;
        x = (r - d) * Math.sin(beta);
        y = (r - d) * Math.cos(beta);
        beta = dRandReal() - 0.5;
        for (j = 0; j < 3; j++) a.set(j, p.get(j) + x * R.get(j, 0) + y * R.get(j, 1) + l * beta * R.get(j, 2));
        if (dFabs(ccyl.getPointDepth(a) - d) >= tol)
            if (testFAILED()) return false;

        // ********** test point depth (2)

        d = (dRandReal() * 2 - 1) * r;
        for (j = 0; j < 3; j++) a.set(j, dRandReal() - 0.5);
        a.normalize();
        if (dCalcVectorDot3_14(a, R, 2) > 0) {
            for (j = 0; j < 3; j++) a.set(j, p.get(j) + a.get(j) * (r - d) + l * 0.5 * R.get(j, 2));
        } else {
            for (j = 0; j < 3; j++) a.set(j, p.get(j) + a.get(j) * (r - d) - l * 0.5 * R.get(j, 2));
        }
        if (dFabs(ccyl.getPointDepth(a) - d) >= tol)
            if (testFAILED()) return false;

        return retPASSED();
    }


    private boolean test_plane_point_depth() {
        int j;
        DVector3 n = new DVector3(), p = new DVector3(), q = new DVector3(),
                a = new DVector3(), b = new DVector3();    // n = plane normal
        double d;

        DSimpleSpace space = OdeHelper.createSimpleSpace(null);

        DPlane plane = OdeHelper.createPlane(null, 0, 0, 1, 0);
        space.add(plane);

        // ********** make a random plane

        for (j = 0; j < 3; j++) n.set(j, dRandReal() - 0.5);
        n.normalize();
        d = dRandReal() - 0.5;
        plane.setParams(n, d);
        dPlaneSpace(n, p, q);

        // ********** test point on plane has depth 0

        a.set0(dRandReal() - 0.5);
        a.set1(dRandReal() - 0.5);
        a.set2(0);
        for (j = 0; j < 3; j++) b.set(j, a.get0() * p.get(j) + a.get1() * q.get(j) + (a.get2() + d) * n.get(j));
        if (dFabs(plane.getPointDepth(b)) >= tol) if (testFAILED()) return false;

        // ********** test arbitrary depth point

        a.set0(dRandReal() - 0.5);
        a.set1(dRandReal() - 0.5);
        a.set2(dRandReal() - 0.5);
        for (j = 0; j < 3; j++) b.set(j, a.get0() * p.get(j) + a.get1() * q.get(j) + (a.get2() + d) * n.get(j));
        if (dFabs(plane.getPointDepth(b) + a.get2()) >= tol)
            if (testFAILED()) return false;

        // ********** test depth-1 point

        a.set0(dRandReal() - 0.5);
        a.set1(dRandReal() - 0.5);
        a.set2(-1);
        for (j = 0; j < 3; j++) b.set(j, a.get0() * p.get(j) + a.get1() * q.get(j) + (a.get2() + d) * n.get(j));
        if (dFabs(plane.getPointDepth(b) - 1) >= tol) if (testFAILED()) return false;

        return retPASSED();
    }

    //****************************************************************************
    // ray tests

    private boolean test_ray_and_sphere() {
        DContactGeomBuffer contacts = new DContactGeomBuffer(1);
        DVector3 p = new DVector3(), q = new DVector3(), q2 = new DVector3(),
                n = new DVector3(), v1 = new DVector3();
        DMatrix3 R = new DMatrix3();
        double r, k;

        DSimpleSpace space = OdeHelper.createSimpleSpace(null);

        DRay ray = OdeHelper.createRay(null, 0);
        DSphere sphere = OdeHelper.createSphere(null, 1);
        space.add(ray);
        space.add(sphere);

        // ********** make a random sphere of radius r at position p

        r = dRandReal() + 0.1;
        sphere.setRadius(r);
        dMakeRandomVector(p, 1.0);
        sphere.setPosition(p);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        sphere.setRotation(R);

        // ********** test zero length ray just inside sphere

        ray.setLength(0);
        dMakeRandomVector(q, 1.0);
        q.normalize();
        q.eqSum(p, q, 0.11 * r);
        ray.setPosition(q);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        ray.setRotation(R);
        if (OdeHelper.collide(ray, sphere, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test zero length ray just outside that sphere

        ray.setLength(0);
        dMakeRandomVector(q, 1.0);
        q.normalize();
        q.eqSum(p, q, 1.01 * r);
        ray.setPosition(q);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        ray.setRotation(R);
        if (OdeHelper.collide(ray, sphere, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test finite length ray totally contained inside the sphere

        dMakeRandomVector(q, 1.0);
        q.normalize();
        k = dRandReal();
        q.eqSum(p, q, k * r * 0.99);
        dMakeRandomVector(q2, 1.0);
        q2.normalize();
        k = dRandReal();
        q2.eqSum(p, q2, k * r * 0.99);
        n.eqDiff(q2, q);
        n.normalize();
        ray.set(q, n);
        ray.setLength(q.distance(q2));
        if (OdeHelper.collide(ray, sphere, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test finite length ray totally outside the sphere

        dMakeRandomVector(q, 1.0);
        q.normalize();
        do {
            dMakeRandomVector(n, 1.0);
            n.normalize();
        }
        while (n.dot(q) < 0);    // make sure normal goes away from sphere
        q.eqSum(p, q, 1.01 * r);
        ray.set(q, n);
        ray.setLength(100);
        if (OdeHelper.collide(ray, sphere, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test ray from outside to just above surface

        dMakeRandomVector(q, 1.0);
        q.normalize();
        n.set(q).scale(-1);
        q2.eqSum(p, q, 2 * r);
        ray.set(q2, n);
        ray.setLength(0.99 * r);
        if (OdeHelper.collide(ray, sphere, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test ray from outside to just below surface

        ray.setLength(1.01 * r);
        if (OdeHelper.collide(ray, sphere, 1, contacts) != 1) if (testFAILED()) return false;
        q2.eqSum(p, q, r);
        if (contacts.get(0).pos.distance(q2) > tol) if (testFAILED()) return false;

        // ********** test contact point distance for random rays

        dMakeRandomVector(q, 1.0);
        q.normalize();
        k = dRandReal() + 0.5;
        q.eqSum(p, q, k * r);
        dMakeRandomVector(n, 1.0);
        n.normalize();
        ray.set(q, n);
        ray.setLength(100);
        if (OdeHelper.collide(ray, sphere, 1, contacts) != 0) {
            DContactGeom contact = contacts.get(0);
            k = contacts.get(0).pos.distance(sphere.getPosition());
            if (dFabs(k - r) > tol) if (testFAILED()) return false;
            // also check normal signs
            if (n.dot(contact.normal) > 0) if (testFAILED()) return false;
            // also check depth of contact point
            if (dFabs(sphere.getPointDepth(contact.pos)) > tol)
                if (testFAILED()) return false;
        }

        // ********** test tangential grazing - miss

        dMakeRandomVector(q, 1.0);
        q.normalize();
        dPlaneSpace(q, n, v1);
        q.eqSum(p, q, 1.01 * r);
        q.sub(n);
        ray.set(q, n);
        ray.setLength(2);
        if (OdeHelper.collide(ray, sphere, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test tangential grazing - hit

        dMakeRandomVector(q, 1.0);
        q.normalize();
        dPlaneSpace(q, n, v1);
        q.eqSum(p, q, 0.99 * r);
        q.sub(n);
        ray.set(q, n);
        ray.setLength(2);
        if (OdeHelper.collide(ray, sphere, 1, contacts) != 1) if (testFAILED()) return false;

        return retPASSED();
    }


    private boolean test_ray_and_box() {
        int i, j;
        //dContactGeom contact;
        DContactGeomBuffer contacts = new DContactGeomBuffer(1);
        DVector3 s = new DVector3(), p = new DVector3(), q = new DVector3(), n = new DVector3(),
                q2 = new DVector3(), q3 = new DVector3(), q4 = new DVector3();        // s = box sides
        DMatrix3 R = new DMatrix3();
        double k;

        DSimpleSpace space = OdeHelper.createSimpleSpace(null);

        DRay ray = OdeHelper.createRay(null, 0);
        DBox box = OdeHelper.createBox(null, 1, 1, 1);
        space.add(ray);
        space.add(box);

        // ********** make a random box

        for (j = 0; j < 3; j++) s.set(j, dRandReal() + 0.1);
        box.setLengths(s);
        dMakeRandomVector(p, 1.0);
        box.setPosition(p);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        box.setRotation(R);

        // ********** test zero length ray just inside box

        ray.setLength(0);
        for (j = 0; j < 3; j++) q.set(j, (dRandReal() - 0.5) * s.get(j));
        i = dRandInt(3);
        if (dRandReal() > 0.5) q.set(i, 0.99 * 0.5 * s.get(i));
        else q.set(i, -0.99 * 0.5 * s.get(i));
        dMultiply0(q2, box.getRotation(), q);
        q2.add(p);
        ray.setPosition(q2);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        ray.setRotation(R);
        if (OdeHelper.collide(ray, box, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test zero length ray just outside box

        ray.setLength(0);
        for (j = 0; j < 3; j++) q.set(j, (dRandReal() - 0.5) * s.get(j));
        i = dRandInt(3);
        if (dRandReal() > 0.5) q.set(i, 1.01 * 0.5 * s.get(i));
        else q.set(i, -1.01 * 0.5 * s.get(i));
        dMultiply0(q2, box.getRotation(), q);
        q2.add(p);
        ray.setPosition(q2);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        ray.setRotation(R);
        if (OdeHelper.collide(ray, box, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test finite length ray totally contained inside the box

        for (j = 0; j < 3; j++) q.set(j, (dRandReal() - 0.5) * 0.99 * s.get(j));
        dMultiply0(q2, box.getRotation(), q);
        q2.add(p);
        for (j = 0; j < 3; j++) q3.set(j, (dRandReal() - 0.5) * 0.99 * s.get(j));
        dMultiply0(q4, box.getRotation(), q3);
        q4.add(p);
        n.eqDiff(q4, q2);
        n.normalize();
        ray.set(q2, n);
        ray.setLength(q2.distance(q4));
        if (OdeHelper.collide(ray, box, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test finite length ray totally outside the box

        for (j = 0; j < 3; j++) q.set(j, (dRandReal() - 0.5) * s.get(j));
        i = dRandInt(3);
        if (dRandReal() > 0.5) q.set(i, 1.01 * 0.5 * s.get(i));
        else q.set(i, -1.01 * 0.5 * s.get(i));
        dMultiply0(q2, box.getRotation(), q);
        q3.eqSum(q2, p);
        q2.normalize();
        ray.set(q3, q2);
        ray.setLength(10);
        if (OdeHelper.collide(ray, box, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test ray from outside to just above surface

        for (j = 0; j < 3; j++) q.set(j, (dRandReal() - 0.5) * s.get(j));
        i = dRandInt(3);
        if (dRandReal() > 0.5) q.set(i, 1.01 * 0.5 * s.get(i));
        else q.set(i, -1.01 * 0.5 * s.get(i));
        dMultiply0(q2, box.getRotation(), q);
        q3.eqSum(p, q2, 2);
        k = q2.length();
        q2.scale(-1);
        ray.set(q3, q2);
        ray.setLength(k * 0.99);
        if (OdeHelper.collide(ray, box, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test ray from outside to just below surface

        ray.setLength(k * 1.01);
        if (OdeHelper.collide(ray, box, 1, contacts) != 1) if (testFAILED()) return false;

        // ********** test contact point position for random rays

        for (j = 0; j < 3; j++) q.set(j, dRandReal() * s.get(j));
        dMultiply0(q2, box.getRotation(), q);
        q2.add(p);
        for (j = 0; j < 3; j++) q3.set(j, dRandReal() - 0.5);
        q3.normalize();
        ray.set(q2, q3);
        ray.setLength(10);
        if (OdeHelper.collide(ray, box, 1, contacts) != 0) {
            DContactGeom contact = contacts.get(0);
            // check depth of contact point
            if (dFabs(box.getPointDepth(contact.pos)) > tol)
                if (testFAILED()) return false;
            // check position of contact point
            contact.pos.sub(p);
            dMultiply1(q, box.getRotation(), contact.pos);
            if (dFabs(dFabs(q.get0()) - 0.5 * s.get0()) > tol &&
                    dFabs(dFabs(q.get1()) - 0.5 * s.get1()) > tol &&
                    dFabs(dFabs(q.get2()) - 0.5 * s.get2()) > tol) {
                if (testFAILED()) return false;
            }
            // also check normal signs
            if (q3.dot(contact.normal) > 0) if (testFAILED()) return false;
        }

        return retPASSED();
    }


    private boolean test_ray_and_ccylinder() {
        int j;
        DContactGeomBuffer contacts = new DContactGeomBuffer(1);
        DVector3 p = new DVector3(), a = new DVector3(), b = new DVector3(), n = new DVector3();
        DMatrix3 R = new DMatrix3();
        double r, l, k, x, y;

        DSimpleSpace space = OdeHelper.createSimpleSpace(null);

        DRay ray = OdeHelper.createRay(null, 0);
        DCapsule ccyl = OdeHelper.createCapsule(null, 1, 1);
        space.add(ray);
        space.add(ccyl);

        // ********** make a random capped cylinder

        r = dRandReal() * 0.5 + 0.01;
        l = dRandReal() * 1 + 0.01;
        ccyl.setParams(r, l);
        dMakeRandomVector(p, 1.0);
        ccyl.setPosition(p);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        ccyl.setRotation(R);

        // ********** test ray completely within ccyl

        for (j = 0; j < 3; j++) a.set(j, dRandReal() - 0.5);
        a.normalize();
        k = (dRandReal() - 0.5) * l;
        for (j = 0; j < 3; j++) a.set(j, p.get(j) + r * 0.99 * a.get(j) + k * 0.99 * R.get(j, 2));
        for (j = 0; j < 3; j++) b.set(j, dRandReal() - 0.5);
        b.normalize();
        k = (dRandReal() - 0.5) * l;
        for (j = 0; j < 3; j++) b.set(j, p.get(j) + r * 0.99 * b.get(j) + k * 0.99 * R.get(j, 2));
        ray.setLength(a.distance(b));
        b.sub(a);
        b.normalize();
        ray.set(a, b);
        if (OdeHelper.collide(ray, ccyl, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test ray outside ccyl that just misses (between caps)

        k = dRandReal() * 2 * M_PI;
        x = Math.sin(k);
        y = Math.cos(k);
        for (j = 0; j < 3; j++) a.set(j, x * R.get(j, 0) + y * R.get(j, 1));
        k = (dRandReal() - 0.5) * l;
        for (j = 0; j < 3; j++) b.set(j, -a.get(j) * r * 2 + k * R.get(j, 2) + p.get(j));
        ray.set(b, a);
        ray.setLength(r * 0.99);
        if (OdeHelper.collide(ray, ccyl, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test ray outside ccyl that just hits (between caps)

        ray.setLength(r * 1.01);
        if (OdeHelper.collide(ray, ccyl, 1, contacts) != 1) if (testFAILED()) return false;
        // check depth of contact point
        if (dFabs(ccyl.getPointDepth(contacts.get(0).pos)) > tol)
            if (testFAILED()) return false;

        // ********** test ray outside ccyl that just misses (caps)

        for (j = 0; j < 3; j++) a.set(j, dRandReal() - 0.5);
        a.normalize();
        if (dCalcVectorDot3_14(a, R, 2) < 0) {
            for (j = 0; j < 3; j++) b.set(j, p.get(j) - a.get(j) * 2 * r + l * 0.5 * R.get(j, 2));
        } else {
            for (j = 0; j < 3; j++) b.set(j, p.get(j) - a.get(j) * 2 * r - l * 0.5 * R.get(j, 2));
        }
        ray.set(b, a);
        ray.setLength(r * 0.99);
        if (OdeHelper.collide(ray, ccyl, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test ray outside ccyl that just hits (caps)

        ray.setLength(r * 1.01);
        if (OdeHelper.collide(ray, ccyl, 1, contacts) != 1) if (testFAILED()) return false;
        // check depth of contact point
        if (dFabs(ccyl.getPointDepth(contacts.get(0).pos)) > tol)
            if (testFAILED()) return false;

        // ********** test random rays

        for (j = 0; j < 3; j++) a.set(j, dRandReal() - 0.5);
        for (j = 0; j < 3; j++) n.set(j, dRandReal() - 0.5);
        n.normalize();
        ray.set(a, n);
        ray.setLength(10);

        if (OdeHelper.collide(ray, ccyl, 1, contacts) != 0) {
            // check depth of contact point
            if (dFabs(ccyl.getPointDepth(contacts.get(0).pos)) > tol)
                if (testFAILED()) return false;

            // check normal signs
            if (n.dot(contacts.get(0).normal) > 0) if (testFAILED()) return false;
        }

        return retPASSED();
    }


    /**
     * Test rays within the cylinder
     * -completely inside
     * -exiting through side
     * -exiting through cap
     * -exiting through corner
     * Test rays outside the cylinder
     */
    private boolean test_ray_and_cylinder() {
        DVector3 a = new DVector3(), b = new DVector3();

        DSimpleSpace space = OdeHelper.createSimpleSpace();
        DRay ray = OdeHelper.createRay(space, 4);

        // The first thing that happens is the ray is
        // rotated into cylinder coordinates.  We'll trust that's
        // done right.  The major axis is in the z-dir.


        // Random tests
      /*b[0]=4*dRandReal()-2;
      b[1]=4*dRandReal()-2;
      b[2]=4*dRandReal()-2;
      a[0]=2*dRandReal()-1;
      a[1]=2*dRandReal()-1;
      a[2]=2*dRandReal()-1;*/

        // Inside out
        b.set0(dRandReal() - 0.5);
        b.set1(dRandReal() - 0.5);
        b.set2(dRandReal() - 0.5);
        a.set0(2 * dRandReal() - 1);
        a.set1(2 * dRandReal() - 1);
        a.set2(2 * dRandReal() - 1);

        // Outside in
      /*b[0]=4*dRandReal()-2;
      b[1]=4*dRandReal()-2;
      b[2]=4*dRandReal()-2;
      a[0]=-b[0];
      a[1]=-b[1];
      a[2]=-b[2];*/


        ray.set(b, a);
        // This is just for visual inspection right now.
        //if (dCollide (ray,cyl,1,&contact,sizeof(dContactGeom)) != 1) FAILED();

        return retPASSED();
    }


    private boolean test_ray_and_plane() {
        int j;
        DContactGeomBuffer contacts = new DContactGeomBuffer(1);
        DVector3 n = new DVector3(), p = new DVector3(), q = new DVector3(), a = new DVector3(),
                b = new DVector3(), g = new DVector3(), h = new DVector3();        // n,d = plane parameters
        DMatrix3 R = new DMatrix3();
        double d;

        DSimpleSpace space = OdeHelper.createSimpleSpace(null);

        DRay ray = OdeHelper.createRay(null, 0);
        DPlane plane = OdeHelper.createPlane(null, 0, 0, 1, 0);
        space.add(ray);
        space.add(plane);

        // ********** make a random plane

        for (j = 0; j < 3; j++) n.set(j, dRandReal() - 0.5);
        n.normalize();
        d = dRandReal() - 0.5;
        plane.setParams(n, d);
        dPlaneSpace(n, p, q);

        // ********** test finite length ray below plane

        ray.setLength(0.09);
        a.set0(dRandReal() - 0.5);
        a.set1(dRandReal() - 0.5);
        a.set2(-dRandReal() * 0.5 - 0.1);
        for (j = 0; j < 3; j++) b.set(j, a.get0() * p.get(j) + a.get1() * q.get(j) + (a.get2() + d) * n.get(j));
        ray.setPosition(b);
        dRFromAxisAndAngle(R, dRandReal() * 2 - 1, dRandReal() * 2 - 1,
                dRandReal() * 2 - 1, dRandReal() * 10 - 5);
        ray.setRotation(R);
        if (OdeHelper.collide(ray, plane, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test finite length ray above plane

        a.set0(dRandReal() - 0.5);
        a.set1(dRandReal() - 0.5);
        a.set2(dRandReal() * 0.5 + 0.01);
        for (j = 0; j < 3; j++) b.set(j, a.get0() * p.get(j) + a.get1() * q.get(j) + (a.get2() + d) * n.get(j));
        g.set0(dRandReal() - 0.5);
        g.set1(dRandReal() - 0.5);
        g.set2(dRandReal() + 0.01);
        for (j = 0; j < 3; j++) h.set(j, g.get0() * p.get(j) + g.get1() * q.get(j) + g.get2() * n.get(j));
        h.normalize();
        ray.set(b, h);
        ray.setLength(10);
        if (OdeHelper.collide(ray, plane, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test finite length ray that intersects plane

        a.set0(dRandReal() - 0.5);
        a.set1(dRandReal() - 0.5);
        a.set2(dRandReal() - 0.5);
        for (j = 0; j < 3; j++) b.set(j, a.get0() * p.get(j) + a.get1() * q.get(j) + (a.get2() + d) * n.get(j));
        g.set0(dRandReal() - 0.5);
        g.set1(dRandReal() - 0.5);
        g.set2(dRandReal() - 0.5);
        for (j = 0; j < 3; j++) h.set(j, g.get0() * p.get(j) + g.get1() * q.get(j) + g.get2() * n.get(j));
        h.normalize();
        ray.set(b, h);
        ray.setLength(10);
        if (OdeHelper.collide(ray, plane, 1, contacts) != 0) {
            // test that contact is on plane surface
            if (dFabs(contacts.get(0).pos.dot(n) - d) > tol) if (testFAILED()) return false;
            // also check normal signs
            if (h.dot(contacts.get(0).normal) > 0) if (testFAILED()) return false;
            // also check contact point depth
            if (dFabs(plane.getPointDepth(contacts.get(0).pos)) > tol)
                if (testFAILED()) return false;
        }

        // ********** test ray that just misses

        b.set(n).scale(1 + d);
        h.set(n).scale(-1);
        ray.set(b, h);
        ray.setLength(0.99);
        if (OdeHelper.collide(ray, plane, 1, contacts) != 0) if (testFAILED()) return false;

        // ********** test ray that just hits

        ray.setLength(1.01);
        if (OdeHelper.collide(ray, plane, 1, contacts) != 1) if (testFAILED()) return false;

        // ********** test polarity with typical ground plane
        plane.setParams(0, 0, 1, 0);
        a.set(0.1, 0.1, 1);
        b.set(0, 0, -1);
        ray.set(a, b);
        ray.setLength(2);
        assertEquals(1, OdeHelper.collide(ray, plane, 1, contacts));
        assertFalse(Math.abs(contacts.get(0).depth - 1) > tol);
        a.set2(-1);
        b.set2(1);
        ray.set(a, b);
        assertEquals(1, OdeHelper.collide(ray, plane, 1, contacts));
        assertFalse(Math.abs(contacts.get(0).depth - 1) > tol);

        return retPASSED();
    }

    //****************************************************************************
    // a really inefficient, but hopefully correct implementation of
    // dBoxTouchesBox(), that does 144 edge-face tests.

    // return 1 if edge v1 -> v2 hits the rectangle described by p1,p2,p3

    private static boolean edgeIntersectsRect(DVector3 v1, DVector3 v2,
                                              DVector3 p1, DVector3 p2, DVector3 p3) {
        DVector3 u1 = new DVector3(), u2 = new DVector3(), n = new DVector3(), tmp = new DVector3();

        //for (k=0; k<3; k++) u1[k] = p3[k]-p1[k];
        u1.eqDiff(p3, p1);
        //for (k=0; k<3; k++) u2[k] = p2[k]-p1[k];
        u2.eqDiff(p2, p1);

        double d1 = dSqrt(u1.dot(u1));
        double d2 = dSqrt(u2.dot(u2));
        u1.normalize();
        u2.normalize();

        double error;
        // #ifdef dSINGLE
        // 	const dReal uEpsilon = 1e-5, pEpsilon = 1e-6, tmpEpsilon = 1.5e-4;
        // #else
        final double uEpsilon = 1e-6, pEpsilon = 1e-8, tmpEpsilon = 1e-6;
        // #endif

        error = dFabs(dCalcVectorDot3(u1, u2));
        if (error > uEpsilon) dDebug(0, "bad u1/u2");

        dCalcVectorCross3(n, u1, u2);

        // for (k=0; k < 3; k++) tmp[k] = v2[k] - v1[k];
        tmp.eqDiff(v2, v1);

        double d = -dCalcVectorDot3(n, p1);

        error = dFabs(dCalcVectorDot3(n, p1) + d);
        if (error > pEpsilon) dDebug(0, "bad n wrt p1");

        error = dFabs(dCalcVectorDot3(n, p2) + d);
        if (error > pEpsilon) dDebug(0, "bad n wrt p2");

        error = dFabs(dCalcVectorDot3(n, p3) + d);
        if (error > pEpsilon) dDebug(0, "bad n wrt p3");

        double alpha = -(d + dCalcVectorDot3(n, v1)) / dCalcVectorDot3(n, tmp);
        // for (k=0; k < 3; k++) tmp[k] = v1[k] + alpha * (v2[k] - v1[k]);
        tmp.eqDiff(v2, v1).scale(alpha).add(v1);

        error = dFabs(dCalcVectorDot3(n, tmp) + d);
        if (error > tmpEpsilon) dDebug(0, "bad tmp");

        if (alpha < 0) return false;
        if (alpha > 1) return false;

        //for (k=0; k<3; k++) tmp[k] -= p1[k];
        tmp.sub(p1);
        double a1 = u1.dot(tmp);
        double a2 = u2.dot(tmp);
        if (a1 < 0 || a2 < 0 || a1 > d1 || a2 > d2) return false;

        return true;
    }


    // return 1 if box 1 is completely inside box 2

    private static boolean box1inside2(final DVector3 p1, final DMatrix3 R1,
                                       final DVector3 side1, final DVector3 p2,
                                       final DMatrix3 R2, final DVector3 side2) {
        for (int i = -1; i <= 1; i += 2) {
            for (int j = -1; j <= 1; j += 2) {
                for (int k = -1; k <= 1; k += 2) {
                    DVector3 v = new DVector3(), vv = new DVector3();
                    v.set(side1).scale(i * 0.5, j * 0.5, k * 0.5);
                    dMultiply0_331(vv, R1, v);
                    vv.add(p1).sub(p2);
                    for (int axis = 0; axis < 3; axis++) {
                        double z = dCalcVectorDot3_14(vv, R2, axis);
                        if (z < (-side2.get(axis) * 0.5) || z > (side2.get(axis) * 0.5)) return false;
                    }
                }
            }
        }
        return true;
    }


    // test if any edge from box 1 hits a face from box 2

    private static boolean testBoxesTouch2(final DVector3 p1, final DMatrix3 R1,
                                           final DVector3 side1, final DVector3 p2,
                                           final DMatrix3 R2, final DVector3 side2) {
        int j, k, j1, j2;

        // for 6 faces from box 2
        for (int fd = 0; fd < 3; fd++) {        // direction for face

            for (int fo = 0; fo < 2; fo++) {    // offset of face
                // get four points on the face. first get 2 indexes that are not fd
                int k1 = 0, k2 = 0;
                if (fd == 0) {
                    k1 = 1;
                    k2 = 2;
                }
                if (fd == 1) {
                    k1 = 0;
                    k2 = 2;
                }
                if (fd == 2) {
                    k1 = 0;
                    k2 = 1;
                }
                DVector3[] fp = DVector3.newArray(4);
                DVector3 tmp = new DVector3();
                k = 0;
                for (j1 = -1; j1 <= 1; j1 += 2) {
                    for (j2 = -1; j2 <= 1; j2 += 2) {
                        fp[k].set(k1, j1);
                        fp[k].set(k2, j2);
                        fp[k].set(fd, fo * 2 - 1);
                        k++;
                    }
                }
                for (j = 0; j < 4; j++) {
                    fp[j].scale(side2).scale(0.5);
                    dMultiply0_331(tmp, R2, fp[j]);
                    fp[j].eqSum(tmp, p2);
                }

                // for 8 vertices
                double[] v1 = new double[3];
                for (v1[0] = -1; v1[0] <= 1; v1[0] += 2) {
                    for (v1[1] = -1; v1[1] <= 1; v1[1] += 2) {
                        for (v1[2] = -1; v1[2] <= 1; v1[2] += 2) {
                            // for all possible +ve leading edges from those vertices
                            for (int ei = 0; ei < 3; ei++) {
                                if (v1[ei] < 0) {
                                    // get vertex1 -> vertex2 = an edge from box 1
                                    DVector3 vv1 = new DVector3(), vv2 = new DVector3();
                                    vv1.set(v1[0], v1[1], v1[2]).scale(0.5).scale(side1);
                                    for (k = 0; k < 3; k++)
                                        vv2.set(k, (v1[k] + (k == ei ? 1 : 0) * 2) * 0.5 * side1.get(k));
                                    DVector3 vertex1 = new DVector3(), vertex2 = new DVector3();
                                    dMultiply0_331(vertex1, R1, vv1);
                                    dMultiply0_331(vertex2, R1, vv2);
                                    //for (k=0; k<3; k++) vertex1[k] += p1[k];
                                    vertex1.add(p1);
                                    //for (k=0; k<3; k++) vertex2[k] += p1[k];
                                    vertex2.add(p1);

                                    // see if vertex1 -> vertex2 interesects face
                                    if (edgeIntersectsRect(vertex1, vertex2, fp[0], fp[1], fp[2]))
                                        return true;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (box1inside2(p1, R1, side1, p2, R2, side2)) return true;
        if (box1inside2(p2, R2, side2, p1, R1, side1)) return true;

        return false;
    }

    //****************************************************************************
    // dBoxTouchesBox() test

    private boolean test_dBoxTouchesBox() {
        int k;
        boolean bt1, bt2;
        DVector3 p1 = new DVector3(), p2 = new DVector3(), side1 = new DVector3(), side2 = new DVector3();
        DMatrix3 R1 = new DMatrix3(), R2 = new DMatrix3();

        DSimpleSpace space = OdeHelper.createSimpleSpace(null);

        DBox box1 = OdeHelper.createBox(null, 1, 1, 1);
        space.add(box1);
        DBox box2 = OdeHelper.createBox(null, 1, 1, 1);
        space.add(box2);

        dMakeRandomVector(p1, 0.5);
        dMakeRandomVector(p2, 0.5);
        for (k = 0; k < 3; k++) side1.set(k, dRandReal() + 0.01);
        for (k = 0; k < 3; k++) side2.set(k, dRandReal() + 0.01);
        double r13 = dRandReal();
        double r12 = dRandReal();
        double r11 = dRandReal();
        double r10 = dRandReal();
        dRFromAxisAndAngle(R1, r10 * 2.0 - 1.0, r11 * 2.0 - 1.0,
                r12 * 2.0 - 1.0, r13 * 10.0 - 5.0);
        double r23 = dRandReal();
        double r22 = dRandReal();
        double r21 = dRandReal();
        double r20 = dRandReal();
        dRFromAxisAndAngle(R2, r20 * 2.0 - 1.0, r21 * 2.0 - 1.0,
                r22 * 2.0 - 1.0, r23 * 10.0 - 5.0);
//        dRFromAxisAndAngle(R1, dRandReal() * 2.0 - 1.0, dRandReal() * 2.0 - 1.0,
//                dRandReal() * 2.0 - 1.0, dRandReal() * 10.0 - 5.0);
//        dRFromAxisAndAngle(R2, dRandReal() * 2.0 - 1.0, dRandReal() * 2.0 - 1.0,
//                dRandReal() * 2.0 - 1.0, dRandReal() * 10.0 - 5.0);

        box1.setLengths(side1);
        box2.setLengths(side2);
        box1.setPosition(p1);
        box1.setRotation(R1);
        box2.setPosition(p2);
        box2.setRotation(R2);

        boolean t1 = testBoxesTouch2(p1, R1, side1, p2, R2, side2);
        boolean t2 = testBoxesTouch2(p2, R2, side2, p1, R1, side1);
        bt1 = t1 || t2;
        bt2 = DxCollisionUtil.dBoxTouchesBox(p1, R1, side1, p2, R2, side2);

        assertEquals(bt1, bt2);

        /*
    // some more debugging info if necessary
    if (bt1 && bt2) printf ("agree - boxes touch\n");
    if (!bt1 && !bt2) printf ("agree - boxes don't touch\n");
    if (bt1 && !bt2) printf ("disagree - boxes touch but dBoxTouchesBox "
                 "says no\n");
    if (!bt1 && bt2) printf ("disagree - boxes don't touch but dBoxTouchesBox "
                 "says yes\n");
         */

        return retPASSED();
    }

    //****************************************************************************
    // test box-box collision

    private boolean test_dBoxBox() {
        int k, bt;
        DVector3 p1 = new DVector3(), p2 = new DVector3(), side1 = new DVector3(), side2 = new DVector3(),
                normal = new DVector3(), normal2 = new DVector3();
        DMatrix3 R1 = new DMatrix3(), R2 = new DMatrix3();
        RefDouble depth = new RefDouble(), depth2 = new RefDouble();
        RefInt code = new RefInt();
        //dContactGeom contact[48];
        DContactGeomBuffer contacts = new DContactGeomBuffer(48);

        DSimpleSpace space = OdeHelper.createSimpleSpace(null);

        DBox box1 = OdeHelper.createBox(null, 1, 1, 1);
        space.add(box1);
        DBox box2 = OdeHelper.createBox(null, 1, 1, 1);
        space.add(box2);

        dMakeRandomVector(p1, 0.5);
        dMakeRandomVector(p2, 0.5);
        for (k = 0; k < 3; k++) side1.set(k, dRandReal() + 0.01);
        for (k = 0; k < 3; k++) side2.set(k, dRandReal() + 0.01);

//        dRFromAxisAndAngle(R1, dRandReal() * 2.0 - 1.0, dRandReal() * 2.0 - 1.0,
//                dRandReal() * 2.0 - 1.0, dRandReal() * 10.0 - 5.0);
//        dRFromAxisAndAngle(R2, dRandReal() * 2.0 - 1.0, dRandReal() * 2.0 - 1.0,
//                dRandReal() * 2.0 - 1.0, dRandReal() * 10.0 - 5.0);
        double a1 = dRandReal();
        double az1 = dRandReal();
        double ay1 = dRandReal();
        double ax1 = dRandReal();
        dRFromAxisAndAngle(R1, ax1 * 2.0 - 1.0, ay1 * 2.0 - 1.0, az1 * 2.0 - 1.0, a1 * 10.0 - 5.0);
        double a2 = dRandReal();
        double az2 = dRandReal();
        double ay2 = dRandReal();
        double ax2 = dRandReal();
        dRFromAxisAndAngle(R2, ax2 * 2.0 - 1.0, ay2 * 2.0 - 1.0, az2 * 2.0 - 1.0, a2 * 10.0 - 5.0);

        // dRSetIdentity (R1);	// we can also try this
        // dRSetIdentity (R2);

        box1.setLengths(side1);
        box2.setLengths(side2);
        box1.setPosition(p1);
        box1.setRotation(R1);
        box2.setPosition(p2);
        box2.setRotation(R2);

        code.set(0); //= 0;
        depth.set(0);// = 0;
        bt = DxBox.dBoxBox(p1, R1, side1, p2, R2, side2, normal, depth, code, 8, contacts, 1);
        //sizeof(dContactGeom));
        if (bt == 1) {
            p2.eqSum(p2, normal, 0.96 * depth.get());
            bt = DxBox.dBoxBox(p1, R1, side1, p2, R2, side2, normal2, depth2, code, 8, contacts, 1);
            //sizeof(dContactGeom));

            /*
    dGeomSetPosition (box2,p2[0],p2[1],p2[2]);
             */

            if (bt != 1) {
                if (testFAILED()) return false;
                box2.setPosition(p2);
            }

            p2.eqSum(p2, normal, 0.08 * depth.get());
            bt = DxBox.dBoxBox(p1, R1, side1, p2, R2, side2, normal2, depth2, code, 8, contacts, 1);
            //sizeof(dContactGeom));
            if (bt != 0) if (testFAILED()) return false;

            // dGeomSetPosition (box2,p2[0],p2[1],p2[2]);
        }

        // printf ("code=%2d  depth=%.4f  ",code,depth);

        return retPASSED();
    }


    //****************************************************************************
    // do all the tests

    private void do_test(int number, String function) {
        // do two test batches. the first test batch has far fewer reps and will
        // catch problems quickly. if all tests in the first batch passes, the
        // second batch is run.
        for (int batch = 0; batch < 2; batch++) {
            int reps = (batch == 0) ? TEST_REPS1 : TEST_REPS2;
            if (PRINT) {
                System.out.println(
                        "testing " + number + " " + function + " batch " + (batch + 1) + " (" + reps + " reps)...");
            }

            // run tests
            for (int j = 0; j < reps; j++) {
                runtest(function);
            }
        }
    }

    // TZ: This was added because the original ODE demo_collision reports 0.02% errors
    private void do_test_partial(int number, String function, double max_failure_ratio) {
        // do two test batches. the first test batch has far fewer reps and will
        // catch problems quickly. if all tests in the first batch passes, the
        // second batch is run.
        int failcount = 0;
        int total_reps = 0;
        String last_failed_line = "";
        for (int batch = 0; batch < 2; batch++) {
            int reps = (batch == 0) ? TEST_REPS1 : TEST_REPS2;
            total_reps += reps;
            if (PRINT) {
                System.out.println(
                        "testing " + number + " " + function + " batch " + (batch + 1) + " (" + reps + " reps)...");
            }

            // run tests
            for (int j = 0; j < reps; j++) {
                try {
                    if (!runtest(function)) {
                        failcount++;
                    }
                } catch (RuntimeException e) {
                    last_failed_line = e.getMessage();
                    failcount++;
                }
            }
        }

        // print results
        if (PRINT) {
            System.out.printf("%3d: %-30s: ", number, function);
        }
        if (failcount > 0) {
            double failure_ratio = (double) failcount / (double) total_reps;
            if (PRINT) {
                System.out.printf("FAILED (%.2f%%) at line %s\n", failure_ratio * 100.0, last_failed_line);
            }
            assertTrue(failure_ratio < max_failure_ratio);
        } else {
            if (PRINT) {
                System.out.println("ok");
            }
        }
    }

    //****************************************************************************


    @BeforeClass
    public static void beforeClass() {
        OdeHelper.initODE2(0);
    }

    @AfterClass
    public static void afterClass() {
        OdeHelper.closeODE();
    }

    @Test
    public void testSphere_point_depth() {
        do_test(1, "test_sphere_point_depth");
    }

    @Test
    public void testBox_point_depth() {
        do_test(2, "test_box_point_depth");
    }

    @Test
    public void testCcylinder_point_depth() {
        do_test(3, "test_ccylinder_point_depth");
    }

    @Test
    public void testPlane_point_depth() {
        do_test(4, "test_plane_point_depth");
    }

    @Test
    public void testRay_and_sphere() {
        do_test(10, "test_ray_and_sphere");
    }

    @Test
    public void testRay_and_box() {
        do_test(11, "test_ray_and_box");
    }

    @Test
    public void testRay_and_ccylinder() {
        do_test(12, "test_ray_and_ccylinder");
    }

    @Test
    public void testRay_and_plane() {
        do_test(13, "test_ray_and_plane");
    }

    @Test
    public void testRay_and_cylinder() {
        do_test(14, "test_ray_and_cylinder");
    }

    @Test
    public void testDBoxTouchesBox() {
        do_test(100, "test_dBoxTouchesBox");
    }

    @Test
    public void testDBoxBox() {
        do_test_partial(101, "test_dBoxBox", 0.0002);
    }
}