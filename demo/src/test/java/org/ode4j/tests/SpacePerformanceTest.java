package org.ode4j.tests;

import java.util.Random;

import org.junit.Test;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DHashSpace;
import org.ode4j.ode.DSapSpace.AXES;
import org.ode4j.ode.DSimpleSpace;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.DxPersistentSAPSpace;

public class SpacePerformanceTest {

    int spaceCollisions = 0;
    int geomCollisions = 0;

    @Test
    public void test() {
        OdeHelper.initODE2(0);
        // Warmup
        DSpace space0 = OdeHelper.createSapSpace(AXES.XZY);
        testSpace(space0, 1000);
        space0 = DxPersistentSAPSpace.dSweepAndPruneSpaceCreate(null, AXES.XZY.getCode());
        testSpace(space0, 1000);
        space0 = OdeHelper.createHashSpace();
        testSpace(space0, 1000);
        System.out.println("---------------");
       // DSimpleSpace simple = OdeHelper.createSimpleSpace();
       // testSpace(simple);
        DSpace space = OdeHelper.createSapSpace(AXES.XZY);
        testSpace(space, 10000);
        DSpace space2 = DxPersistentSAPSpace.dSweepAndPruneSpaceCreate(null, AXES.XZY.getCode());
        testSpace(space2, 10000);
        DHashSpace hash = OdeHelper.createHashSpace();
        hash.setLevels(-1, 8);
        testSpace(hash, 10000);
    }

    private void testSpace(DSpace space, int num) {
        spaceCollisions = 0;
        geomCollisions = 0;
        System.out.println("==== " + space.getClass().getSimpleName());
        int geomnum = 1000;
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
        for (int j = 0; j < num; j++) {
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
