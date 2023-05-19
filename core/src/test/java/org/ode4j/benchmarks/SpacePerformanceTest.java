package org.ode4j.benchmarks;

import java.util.Random;

import org.junit.Test;
import org.ode4j.math.DQuaternion;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSapSpace.AXES;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeHelper;

public class SpacePerformanceTest {

    private final static int STATIC_CATEGORY = 1;
    private final static int DYNAMIC_CATEGORY = 2;
    private int spaceCollisions;
    private int geomCollisions;

    @Test
    public void test_performance_with_dynamic_world() {
        OdeHelper.initODE2(0);
        // Warmup
        test_performance_with_dynamic_world(1000, 100, 100);
        System.out.println("---------------");
        test_performance_with_dynamic_world(10000, 0, 20);
        test_performance_with_dynamic_world(4000, 0, 100);
        test_performance_with_dynamic_world(1000, 400, 10);
        test_performance_with_dynamic_world(500, 1000, 100);
        test_performance_with_dynamic_world(150, 2000, 200);
        test_performance_with_dynamic_world(80, 4000, 200);
        test_performance_with_dynamic_world(25, 10000, 300);
        test_performance_with_dynamic_world(16, 15000, 100);
    }

    private void test_performance_with_dynamic_world(int iterations, int staticGeoms, int geoms) {
        System.out.println("-= Iterations: " + iterations + "  Static: " + staticGeoms + " Dynamic: " + geoms + " =-");
        DSpace space = OdeHelper.createSapSpace(AXES.XZY);
        test_performance_with_dynamic_world(space, iterations, staticGeoms, geoms);
        space = OdeHelper.createSapSpace2(null, AXES.XZY, STATIC_CATEGORY);
        test_performance_with_dynamic_world(space, iterations, staticGeoms, geoms);
        space = OdeHelper.createBHVSpace(null, 16, false, 0.2, STATIC_CATEGORY);
        test_performance_with_dynamic_world(space, iterations, staticGeoms, geoms);
    }

    private void test_performance_with_dynamic_world(DSpace space, int iterations, int passiveGeomNum, int geomNum) {
        spaceCollisions = 0;
        geomCollisions = 0;
        System.out.printf("   %-16s ", space.getClass().getSimpleName());
        DGeom[] passiveGeoms = new DGeom[passiveGeomNum];
        Random r = new Random(123);
        for (int i = 0; i < passiveGeomNum; i++) {
            passiveGeoms[i] = OdeHelper.createBox(space, 0.25 + 6 * r.nextDouble(), 0.25 + 6 * r.nextDouble(),
                    0.25 + 6 * r.nextDouble());
            passiveGeoms[i].setPosition(250 * r.nextDouble(), 250 * r.nextDouble(), 250 * r.nextDouble());
            double angle = 0.5 * r.nextDouble() * Math.PI;
            passiveGeoms[i].setQuaternion(new DQuaternion(0, Math.sin(angle), 0, Math.cos(angle)));
            passiveGeoms[i].setCategoryBits(STATIC_CATEGORY);
            passiveGeoms[i].setCollideBits(0);
        }
        DGeom[] geoms = new DGeom[geomNum];
        for (int i = 0; i < geomNum; i++) {
            geoms[i] = createGeom(space, r);
        }
        long timer1 = 0;
        long timer2 = 0;
        for (int j = 0; j < iterations; j++) {
            long time1 = System.nanoTime();
            space.collide(null, (data, o1, o2) -> spaceCollisions++);
            long time2 = System.nanoTime();

            for (int k = 0; k < 50; k++) {
                int i = r.nextInt(geoms.length);
                space.collide2(geoms[i], null, (data, o1, o2) -> geomCollisions++);
            }

            long time3 = System.nanoTime();
            timer1 += time2 - time1;
            timer2 += time3 - time2;
            for (int i = 0; i < geomNum; i++) {
                double ms = 0.5;
                double mx = geoms[i].getPosition().get0() + ms * (r.nextDouble() - 0.5);
                double my = geoms[i].getPosition().get1() + ms * (r.nextDouble() - 0.5);
                double mz = geoms[i].getPosition().get2() + ms * (r.nextDouble() - 0.5);
                geoms[i].setPosition(mx, my, mz);
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
                geoms[i] = createGeom(space, r);
            }
        }
        System.out.println("Collisions: " + spaceCollisions + " Geom Collisions: " + geomCollisions + "  Collide Time: " + timer1 / 1000 / iterations + " Collide2 Time: "
                + timer2 / 1000 / iterations);
    }

    @Test
    public void test_performance_with_static_world() {
        OdeHelper.initODE2(0);
        // Warmup
        test_performance_with_static_world(1000, 100, 100);
        System.out.println("---------------");
        test_performance_with_static_world(100, 1000, 1);
        test_performance_with_static_world(100, 1000, 10);
        test_performance_with_static_world(100, 2500, 1);
        test_performance_with_static_world(100, 2500, 5);
        test_performance_with_static_world(100, 5000, 5);
        test_performance_with_static_world(100, 5000, 20);
        test_performance_with_static_world(100, 10000, 50);
    }

    private void test_performance_with_static_world(int iterations, int staticGeoms, int geoms) {
        System.out.println("-= Iterations: " + iterations + "  Static: " + staticGeoms + " Dynamic: " + geoms + " =-");
        DSpace space = OdeHelper.createSapSpace2(null, AXES.XZY, STATIC_CATEGORY);
        test_performance_with_static_world(space, iterations, staticGeoms, geoms);
        space = OdeHelper.createBHVSpace(null, 4, false, 0.2, STATIC_CATEGORY);
        test_performance_with_static_world(space, iterations, staticGeoms, geoms);
    }

    private void test_performance_with_static_world(DSpace space, int iterations, int passiveGeomNum, int geomNum) {
        spaceCollisions = 0;
        geomCollisions = 0;
        System.out.printf("   %-16s ", space.getClass().getSimpleName());
        DGeom[] passiveGeoms = new DGeom[passiveGeomNum];
        Random r = new Random(123);
        for (int i = 0; i < passiveGeomNum; i++) {
            passiveGeoms[i] = OdeHelper.createBox(space, 0.25 + 6 * r.nextDouble(), 0.25 + 6 * r.nextDouble(),
                    0.25 + 6 * r.nextDouble());
            passiveGeoms[i].setPosition(250 * r.nextDouble(), 250 * r.nextDouble(), 250 * r.nextDouble());
            double angle = 0.5 * r.nextDouble() * Math.PI;
            passiveGeoms[i].setQuaternion(new DQuaternion(0, Math.sin(angle), 0, Math.cos(angle)));
            passiveGeoms[i].setCategoryBits(STATIC_CATEGORY);
            passiveGeoms[i].setCollideBits(0);
        }
        DGeom[] geoms = new DGeom[geomNum];
        for (int i = 0; i < geomNum; i++) {
            geoms[i] = createGeom(space, r);
        }
        long timer1 = 0;
        long timer2 = 0;
        for (int j = 0; j < iterations; j++) {
            long time1 = System.nanoTime();
            space.collide(null, (data, o1, o2) -> spaceCollisions++);
            long time2 = System.nanoTime();

            for (int k = 0; k < 50; k++) {
                int i = r.nextInt(geoms.length);
                space.collide2(geoms[i], null, (data, o1, o2) -> geomCollisions++);
            }

            long time3 = System.nanoTime();
            timer1 += time2 - time1;
            timer2 += time3 - time2;
        }
        System.out.println("Collisions: " + spaceCollisions + " Geom Collisions: " + geomCollisions + "  Collide Time: " + timer1 / 1000 / iterations + " Collide2 Time: "
                + timer2 / 1000 / iterations);
    }

    private DSphere createGeom(DSpace space, Random r) {
        DSphere g = OdeHelper.createSphere(space, 4);
        g.setPosition(250 * r.nextDouble(), 250 * r.nextDouble(), 250 * r.nextDouble());
        g.setCategoryBits(DYNAMIC_CATEGORY);
        g.setCollideBits(~0);
        return g;
    }
}
