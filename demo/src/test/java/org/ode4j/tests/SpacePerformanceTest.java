package org.ode4j.tests;

import java.util.Random;

import org.junit.Test;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DSapSpace.AXES;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.DxPersistentSAPSpace;
import org.ode4j.ode.internal.DxSAPSpace;

/*
 * Results (in microseconds):
 * collide2
 * iterations   geoms        old          new       gain
 *     500000      10    7830177       5886252       20%
 *     100000     100    4349950       2012539       50%
 *      40000     200    4052088       1195051       70%
 *       5000    1000    3776681        398865       85%
 *       1200    2000    2650909        208568       90%
 *        250    5000    1613238        125016       90%
 *        100   10000    1399499        177928       85%
 */

public class SpacePerformanceTest {

    int spaceCollisions = 0;
    int geomCollisions = 0;

    @Test
    public void test() {
        OdeHelper.initODE2(0);
        // Warmup
        testSpaces(1000, 100);
        System.out.println("---------------");
        testSpaces(500000, 20);
        testSpaces(100000, 100);
        testSpaces(40000, 200);
        testSpaces(5000, 1000);
        testSpaces(1200, 2000);
        testSpaces(250, 5000);
        testSpaces(100, 10000);
    }

    private void testSpaces(int iterations, int geomnum) {
        System.out.println("-=" + iterations + "   " + geomnum + " =-");
        DxSAPSpace space = (DxSAPSpace) OdeHelper.createSapSpace(AXES.XZY);
        testSpace(space, iterations, geomnum);
        DxPersistentSAPSpace spaceTC = DxPersistentSAPSpace.dSweepAndPruneSpaceCreate(null, AXES.XZY.getCode());
        testSpace(spaceTC, iterations, geomnum);
    }
    
    private void testSpace(DSpace space, int iterations, int geomnum) {
        spaceCollisions = 0;
        geomCollisions = 0;
        System.out.println("==== " + space.getClass().getSimpleName());
        DGeom[] geoms = new DGeom[geomnum];
        Random r = new Random(123);
        for (int i = 0; i < geomnum; i++) {
            geoms[i] = OdeHelper.createBox(space, 1, 1, 1);
            geoms[i].setPosition(r.nextInt(100), r.nextInt(100), r.nextInt(100));
        }
        DGeom[] geoms2 = new DGeom[10];
        for (int i = 0; i < 10; i++) {
        	geoms2[i] = OdeHelper.createBox(space, 4, 4, 4);
            geoms2[i].setPosition(0.5 + r.nextInt(100), 0.5 + r.nextInt(100), 0.5 + r.nextInt(100));
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
            for (int k = 0; k < 100; k++) {
                int i = r.nextInt(geoms2.length);
                space.collide2(geoms2[i], null, new DNearCallback() {
                    @Override
                    public void call(Object data, DGeom o1, DGeom o2) {
                        geomCollisions++;
                    }
                });
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
                geoms[i] = OdeHelper.createBox(space, 1, 1, 1);
            }
        }
        System.out.println(spaceCollisions + "   " + geomCollisions + "   " + timer1 / 1000 + "  " + timer2 / 1000);
    }
}

