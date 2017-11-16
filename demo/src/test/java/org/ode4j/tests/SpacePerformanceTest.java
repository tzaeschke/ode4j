package org.ode4j.tests;

import java.util.Random;

import org.junit.Test;
import org.ode4j.math.DQuaternion;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DSapSpace.AXES;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.DxSAPSpace2;

/*
 * Results (in microseconds):
 * 
 * iterations   geoms        old          new       gain
 *     500000      10    1933846       1635831       15%
 *     100000     100    1668035       1334365       20%
 *      40000     200    1544012       1113094       25%
 *       5000    1000    2257900       1348609       40%
 *       1200    2000    1908258       1063400       40%
 *        250    5000    4848528       1039026       75%
 *        100   10000    2657288       1339070       50%
 */

public class SpacePerformanceTest {

    private final static int STATIC_CATEGORY = 1;
    private final static int DYNAMIC_CATEGORY = 2;

    int spaceCollisions = 0;

    @Test
    public void test() {
        OdeHelper.initODE2(0);
        // Warmup
        testSpaces(1000, 100, 100);
        System.out.println("---------------");
        testSpaces(500000, 0, 20);
        testSpaces(100000, 0, 100);
        testSpaces(40000, 200, 10);
        testSpaces(5000, 1000, 100);
        testSpaces(1200, 2000, 200);
        testSpaces(250, 5000, 250);
        testSpaces(100, 10000, 500);
        testSpaces(100, 15000, 100);
    }

    private void testSpaces(int iterations, int staticGeoms, int geoms) {
        System.out.println("-= Iterations: " + iterations + "  Static: " + staticGeoms + " Dynamic: " + geoms + " =-");
        DSpace space = OdeHelper.createSapSpace(AXES.XZY);
        testSpace(space, iterations, staticGeoms, geoms);
        space = DxSAPSpace2.dSweepAndPruneSpaceCreate(null, AXES.XZY.getCode(), 0);
        testSpace(space, iterations, staticGeoms, geoms);
        space = DxSAPSpace2.dSweepAndPruneSpaceCreate(null, AXES.XZY.getCode(), STATIC_CATEGORY);
        testSpace(space, iterations, staticGeoms, geoms);
    }

    private void testSpace(DSpace space, int iterations, int passiveGeomNum, int geomNum) {
        spaceCollisions = 0;
        System.out.println("==== " + space.getClass().getSimpleName());
        DGeom[] passiveGeoms = new DGeom[passiveGeomNum];
        Random r = new Random(123);
        for (int i = 0; i < passiveGeomNum; i++) {
            passiveGeoms[i] = OdeHelper.createBox(space, 3, 1, 0.25f);
            passiveGeoms[i].setPosition(r.nextInt(100), r.nextInt(100), r.nextInt(100));
            double angle = 0.5 * r.nextDouble() * Math.PI;
            passiveGeoms[i].setQuaternion(new DQuaternion(0, Math.sin(angle), 0, Math.cos(angle)));
            passiveGeoms[i].setCategoryBits(STATIC_CATEGORY);
            passiveGeoms[i].setCollideBits(0);
        }
        DGeom[] geoms = new DGeom[geomNum];
        for (int i = 0; i < geomNum; i++) {
            geoms[i] = OdeHelper.createSphere(space, 4);
            geoms[i].setPosition(10 + r.nextInt(80), 10 + r.nextInt(80), 10 + r.nextInt(80));
            geoms[i].setCategoryBits(DYNAMIC_CATEGORY);
            geoms[i].setCollideBits(~0);
        }
        long timer1 = 0;
        long timer2 = 0;
        for (int j = 0; j < iterations; j++) {
            long time1 = System.nanoTime();
            space.collide(null, new DNearCallback() {
                @Override
                public void call(Object data, DGeom o1, DGeom o2) {
                    spaceCollisions++;
                }
            });
            long time2 = System.nanoTime();
            for (int k = 0; k < geomNum; k++) {
                geoms[k].setPosition(10 + r.nextInt(80), 10 + r.nextInt(80), 10 + r.nextInt(80));
            }
            long time3 = System.nanoTime();
            timer1 += time2 - time1;
            timer2 += time3 - time2;
            for (int k = 0; k < 40; k++) {
                int i = r.nextInt(geoms.length);
                geoms[i].setPosition(r.nextInt(100), r.nextInt(100), r.nextInt(100));
            }
            for (int k = 0; k < 20; k++) {
                int i = r.nextInt(geoms.length);
                geoms[i].disable();
            }
            for (int k = 0; k < 20; k++) {
                int i = r.nextInt(geoms.length);
                geoms[i].enable();
            }
            for (int k = 0; k < 10; k++) {
                int i = r.nextInt(geoms.length);
                geoms[i].destroy();
                geoms[i] = OdeHelper.createSphere(space, 4);
                geoms[i].setCategoryBits(DYNAMIC_CATEGORY);
                geoms[i].setCollideBits(~0);
            }
        }
        System.out.println(spaceCollisions + "   " + (double) timer1 / 1000_000 / iterations + "   "
                + (double) timer2 / 1000_000 / iterations);
    }
}
