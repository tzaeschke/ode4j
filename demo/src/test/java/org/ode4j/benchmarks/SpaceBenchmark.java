package org.ode4j.benchmarks;

import org.ode4j.math.DQuaternion;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DSapSpace.AXES;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.DxBVHSpace;
import org.ode4j.ode.internal.DxSAPSpace2;
import org.openjdk.jmh.annotations.*;
import org.openjdk.jmh.runner.Runner;
import org.openjdk.jmh.runner.RunnerException;
import org.openjdk.jmh.runner.options.Options;
import org.openjdk.jmh.runner.options.OptionsBuilder;
import org.openjdk.jmh.runner.options.TimeValue;

import java.util.Random;
import java.util.concurrent.TimeUnit;

public class SpaceBenchmark {

    private final static int STATIC_CATEGORY = 1;
    private final static int DYNAMIC_CATEGORY = 2;
    private static volatile int spaceCollisions;
    private static volatile int geomCollisions;

    public static void main(String[] args) throws RunnerException {
        Options opt = new OptionsBuilder()
                //.include(SpaceBenchmark.class.getSimpleName())
                .timeUnit(TimeUnit.MILLISECONDS)
                //.warmupTime(TimeValue.seconds(2))
                .warmupIterations(0)
                .measurementTime(TimeValue.seconds(1))
                .measurementIterations(2)
                .forks(0)
                .threads(1)
                .mode(Mode.AverageTime)
                .shouldFailOnError(true)
                .shouldDoGC(true)
                //.verbosity(VerboseMode.SILENT)
                .build();

        new Runner(opt).run();
    }

    //@Test
//    public void test_performance_with_dynamic_world() {
//        OdeHelper.initODE2(0);
//        // Warmup
//        test_performance_with_dynamic_world(1000, 100, 100);
//        System.out.println("---------------");
//        test_performance_with_dynamic_world(10000, 0, 20);
//        test_performance_with_dynamic_world(4000, 0, 100);
//        test_performance_with_dynamic_world(1000, 400, 10);
//        test_performance_with_dynamic_world(500, 1000, 100);
//        test_performance_with_dynamic_world(150, 2000, 200);
//        test_performance_with_dynamic_world(80, 4000, 200);
//        test_performance_with_dynamic_world(25, 10000, 300);
//        test_performance_with_dynamic_world(16, 15000, 100);
//    }

    @Benchmark
//    @Fork(value = 1, warmups = 1)
//    @BenchmarkMode(Mode.AverageTime)
    public void benchmarkDynamicWorld(ExecutionPlan plan) {
        DSpace space = plan.space;
        Random r = plan.r;
        DGeom[] geoms = plan.geoms;

        spaceCollisions = 0;
        geomCollisions = 0;

        for (int ii = 0; ii < plan.iterations; ++ii) {
            space.collide(null, new DNearCallback() {
                @Override
                public void call(Object data, DGeom o1, DGeom o2) {
                    spaceCollisions++;
                }
            });
            long time2 = System.nanoTime();

            for (int k = 0; k < 50; k++) {
                int i = r.nextInt(geoms.length);
                space.collide2(geoms[i], null, new DNearCallback() {
                    @Override
                    public void call(Object data, DGeom o1, DGeom o2) {
                        geomCollisions++;
                    }
                });
            }

            for (int i = 0; i < plan.geomNum; i++) {
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
    }

    // @Test
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
        DSpace space = DxSAPSpace2.dSweepAndPruneSpaceCreate(null, AXES.XZY.getCode(), STATIC_CATEGORY);
        test_performance_with_static_world(space, iterations, staticGeoms, geoms);
        space = DxBVHSpace.bvhSpaceCreate(null, 4, false, 0.2, STATIC_CATEGORY);
        test_performance_with_static_world(space, iterations, staticGeoms, geoms);
    }

    private void test_performance_with_static_world(DSpace space, int iterations, int passiveGeomNum, int geomNum) {
        spaceCollisions = 0;
        geomCollisions = 0;
        System.out.println("==== " + space.getClass().getSimpleName());
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
            space.collide(null, new DNearCallback() {
                @Override
                public void call(Object data, DGeom o1, DGeom o2) {
                    spaceCollisions++;
                }
            });
            long time2 = System.nanoTime();

            for (int k = 0; k < 50; k++) {
                int i = r.nextInt(geoms.length);
                space.collide2(geoms[i], null, new DNearCallback() {
                    @Override
                    public void call(Object data, DGeom o1, DGeom o2) {
                        geomCollisions++;
                    }
                });
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

    enum DynamicPlan {
        WARMUP(1000, 100, 100),

        // Dynamic plans
        D_10000_0_20(10000, 0, 20),
        D_4000_0_100(4000, 0, 100),
        D_1000_400_10(1000, 400, 10),
        D_500_1000_100(500, 1000, 100),
        D_150_2000_200(150, 2000, 200),
        D_80_4000_200(80, 4000, 200),
        D_25_10000_300(25, 10000, 300),
        D_16_15000_100(16, 15000, 100);

        private int iterations;
        private int passiveGeomNum;
        private int geomNum;
        DynamicPlan(int it, int passive, int geoms) {
            iterations = it;
            passiveGeomNum = passive;
            geomNum = geoms;
        }
    }

    @State(Scope.Benchmark)
    public static class ExecutionPlan {
        @Param({"SAP", "SAP2", "BVH"})
        public String space_name;

        @Param({"WARMUP", "D_10000_0_20", "D_4000_0_100", "D_1000_400_10", "D_500_1000_100", "D_150_2000_200",
                "D_80_4000_200", "D_25_10000_300", "D_16_15000_100"})
        public String planName;
        public int iterations;
        public int geomNum;
        public DSpace space;
        Random r = new Random(123);
        DGeom[] passiveGeoms;
        DGeom[] geoms;

        @Setup(Level.Iteration)
        public void setUp() {
            switch (space_name) {
                case "SAP":
                    space = OdeHelper.createSapSpace(AXES.XZY);
                    break;
                case "SAP2":
                    space = DxSAPSpace2.dSweepAndPruneSpaceCreate(null, AXES.XZY.getCode(), STATIC_CATEGORY);
                    break;
                case "BVH":
                    space = DxBVHSpace.bvhSpaceCreate(null, 16, false, 0.2, STATIC_CATEGORY);
                    break;
                default:
                    throw new IllegalArgumentException(space_name);
            }
            DynamicPlan plan = DynamicPlan.valueOf(planName);
            geomNum = plan.geomNum;
            int passiveGeomNum = plan.passiveGeomNum;
            iterations = plan.iterations;

            passiveGeoms = new DGeom[passiveGeomNum];
            for (int i = 0; i < passiveGeomNum; i++) {
                passiveGeoms[i] = OdeHelper.createBox(space, 0.25 + 6 * r.nextDouble(), 0.25 + 6 * r.nextDouble(),
                        0.25 + 6 * r.nextDouble());
                passiveGeoms[i].setPosition(250 * r.nextDouble(), 250 * r.nextDouble(), 250 * r.nextDouble());
                double angle = 0.5 * r.nextDouble() * Math.PI;
                passiveGeoms[i].setQuaternion(new DQuaternion(0, Math.sin(angle), 0, Math.cos(angle)));
                passiveGeoms[i].setCategoryBits(STATIC_CATEGORY);
                passiveGeoms[i].setCollideBits(0);
            }
            geoms = new DGeom[geomNum];
            for (int i = 0; i < geomNum; i++) {
                geoms[i] = createGeom(space, r);
            }
        }

        private DSphere createGeom(DSpace space, Random r) {
            DSphere g = OdeHelper.createSphere(space, 4);
            g.setPosition(250 * r.nextDouble(), 250 * r.nextDouble(), 250 * r.nextDouble());
            g.setCategoryBits(DYNAMIC_CATEGORY);
            g.setCollideBits(~0);
            return g;
        }
    }

//    public static void main(String[] args) throws IOException {
//        org.openjdk.jmh.Main.main(args);
//    }
}
