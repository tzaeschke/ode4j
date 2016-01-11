package org.ode4j.tests;

import java.util.Random;

import org.junit.Test;
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
        DSpace space = OdeHelper.createSapSpace(AXES.XZY); 
        testSpace(space, iterations, geomnum);
        space = DxSAPSpace2.dSweepAndPruneSpaceCreate(null, AXES.XZY.getCode());
        testSpace(space, iterations, geomnum);
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
            for (int k = 0; k < 10; k++) {
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

