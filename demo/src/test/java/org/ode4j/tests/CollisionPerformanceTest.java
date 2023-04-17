package org.ode4j.tests;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.tests.geoms.ConvexCubeGeom;
import org.ode4j.tests.geoms.IcosahedronGeom;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.OdeHelper.areConnectedExcluding;

public class CollisionPerformanceTest {

    private final static int WARMUP = 100 * 1000;
    private final static int BENCHMARK = 1000 * 1000;
    private static final double DENSITY = 5.0;        // density of all objects
    private static final int MAX_CONTACTS = 3;

    private DWorld world;
    private DSpace space;
    private DJointGroup contactgroup;
    private final Random r = new Random(0);

    private int cntCollisions = 0;
    private int cntContacts = 0;


    private static final float[] CUBE_POINTS = {
            0.25f, 0.25f, 0.25f, // point 0
            -0.25f, 0.25f, 0.25f, // point 1
            0.25f, -0.25f, 0.25f, // point 2
            -0.25f, -0.25f, 0.25f, // point 3
            0.25f, 0.25f, -0.25f, // point 4
            -0.25f, 0.25f, -0.25f, // point 5
            0.25f, -0.25f, -0.25f, // point 6
            -0.25f, -0.25f, -0.25f,// point 7
    };

    private static final int[] CUBE_INDICES = {
            0, 2, 6, // 0
            0, 6, 4, // 1
            1, 0, 4, // 2
            1, 4, 5, // 3
            0, 1, 3, // 4
            0, 3, 2, // 5
            3, 1, 5, // 6
            3, 5, 7, // 7
            2, 3, 7, // 8
            2, 7, 6, // 9
            5, 4, 6, // 10
            5, 6, 7  // 11
    };

    @Before
    public void beforeTest() {
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        space = createSpace();
        contactgroup = OdeHelper.createJointGroup();
        // world.setGravity (0,0,-0.5);
        world.setCFM(1e-5);
    }

    @After
    public void afterTest() {
        contactgroup.destroy();
        space.destroy();
        world.destroy();
        OdeHelper.closeODE();
    }

    @Test
    public void testBoxTrimesh() {
        collide(box(), trimesh(), WARMUP);
        collide(box(), trimesh(), BENCHMARK);
    }

    @Test
    public void testCapsuleTrimesh() {
        collide(capsule(), trimesh(), WARMUP);
        collide(capsule(), trimesh(), BENCHMARK);
    }

    @Test
    public void testConvexTrimesh() {
        collide(convex(), trimesh(), WARMUP);
        collide(convex(), trimesh(), BENCHMARK);
    }

    @Test
    public void testConvexIHTrimesh() {
        collide(convexIH(), trimesh(), WARMUP);
        collide(convexIH(), trimesh(), BENCHMARK);
    }

    @Test
    public void testCylinderTrimesh() {
        collide(cylinder(), trimesh(), WARMUP);
        collide(cylinder(), trimesh(), BENCHMARK);
    }

    @Test
    public void testSphereTrimesh() {
        collide(sphere(), trimesh(), WARMUP);
        collide(sphere(), trimesh(), BENCHMARK);
    }

    private DGeom box() {
        // new Body(), new Mass(), new Geom(), geom.setBody(), body.setMass().
        DVector3 sides = vector().scale(6).add(new DVector3(0.25, 0.25, 0.25));
        DGeom geom = OdeHelper.createBox(space, sides);

        DMass mass = OdeHelper.createMass();
        mass.setBox(DENSITY, sides);

        return assemble(geom, body(), mass);
    }

    private DGeom capsule() {
        // new Body(), new Mass(), new Geom(), geom.setBody(), body.setMass().
        double radius = r.nextDouble();
        double length = r.nextDouble();
        DGeom geom = OdeHelper.createCapsule(space, radius, length);

        DMass mass = OdeHelper.createMass();
        mass.setCapsule(DENSITY, r.nextInt(3) + 1, radius, length);

        return assemble(geom, body(), mass);
    }

    private DGeom convex() {
        DGeom geom = OdeHelper.createConvex(space, ConvexCubeGeom.planes,
                ConvexCubeGeom.planecount,
                ConvexCubeGeom.points,
                ConvexCubeGeom.pointcount,
                ConvexCubeGeom.polygons);

        DMass mass = OdeHelper.createMass();
        mass.setBox(DENSITY, 0.25, 0.25, 0.25);

        return assemble(geom, body(), mass);
    }

    private DGeom convexIH() {
        DGeom geom = OdeHelper.createConvex(space, IcosahedronGeom.planes,
                IcosahedronGeom.planecount,
                IcosahedronGeom.points,
                IcosahedronGeom.pointcount,
                IcosahedronGeom.polygons);

        DMass mass = OdeHelper.createMass();
        mass.setBox(DENSITY, 0.25, 0.25, 0.25);

        return assemble(geom, body(), mass);
    }

    private DGeom cylinder() {
        double radius = r.nextDouble();
        double length = r.nextDouble();
        DGeom geom = OdeHelper.createCylinder(space, radius, length);

        DMass mass = OdeHelper.createMass();
        mass.setCylinder(DENSITY, r.nextInt(3) + 1, radius, length);

        return assemble(geom, body(), mass);
    }

    private DGeom sphere() {
        // new Body(), new Mass(), new Geom(), geom.setBody(), body.setMass().
        double radius = r.nextDouble();
        DGeom geom = OdeHelper.createSphere(space, radius);

        DMass mass = OdeHelper.createMass();
        mass.setSphere(DENSITY, radius);

        return assemble(geom, body(), mass);
    }

    private DGeom trimesh() {
        DTriMeshData data = OdeHelper.createTriMeshData();
        // DxGimpactData data = new DxGimpactData(); // TODO make private
        data.build(CUBE_POINTS, CUBE_INDICES);
        data.preprocess();
        DTriMesh geom = OdeHelper.createTriMesh(space, data, null, null, null);

        DMass mass = OdeHelper.createMass();
        mass.setTrimesh(DENSITY, geom);

        return assemble(geom, body(), mass);
    }

    private DSpace createSpace() {
        return OdeHelper.createSimpleSpace();
        // return OdeHelper.createSapSpace(DSapSpace.AXES.XZY);
        // return OdeHelper.createBHVSpace(0);
    }

    private final DNearCallback nearCallback = new DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback(o1, o2);
        }
    };

    //private final DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);


    // this is called by dSpaceCollide when two objects in space are
    // potentially colliding.
    private void nearCallback(DGeom o1, DGeom o2) {
        // if (o1->body && o2->body) return;

        // exit without doing anything if the two bodies are connected by a joint
        DBody b1 = o1.getBody();
        DBody b2 = o2.getBody();
        if (b1 != null && b2 != null && areConnectedExcluding(b1, b2, DContactJoint.class)) return;

        DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);   // up to MAX_CONTACTS contacts per box-box
        for (int i = 0; i < MAX_CONTACTS; i++) {
            DContact contact = contacts.get(i);
            contact.surface.mode = dContactBounce | dContactSoftCFM;
            contact.surface.mu = dInfinity;
            contact.surface.mu2 = 0;
            contact.surface.bounce = 0.1;
            contact.surface.bounce_vel = 0.1;
            contact.surface.soft_cfm = 0.01;

            contact.geom.depth = 0;
            contact.geom.pos.setZero();
            contact.geom.g1 = null;
            contact.geom.g2 = null;

            contact.fdir1.setZero();
        }
        int numc = OdeHelper.collide(o1, o2, MAX_CONTACTS, contacts.getGeomBuffer());
        if (numc != 0) {
            cntCollisions++;
            cntContacts += numc;
            //double speed1 = o1.getBody().getLinearVel().length();
            //double speed2 = o2.getBody().getLinearVel().length();
            //System.out.println("contact: " + numc + "  speed: " + speed1 + "/" + speed2);
//            for (int i = 0; i < numc; i++) {
//                DContact contact = contacts.get(i);
//                DJoint c = OdeHelper.createContactJoint(world, contactgroup, contact);
//                c.attach(b1, b2);
//            }
        }
    }

    private void collide(DGeom geom1, DGeom geom2, int iterations) {
        reset(geom1, geom2);

        long timer = 0;
        int prevCount = 0;
        cntCollisions = 0;
        cntContacts = 0;
        int totalCount = 0;
        for (int j = 0; j < iterations; j++) {
            long time1 = System.nanoTime();
            space.collide(0, nearCallback);
            // world.step(0.05);
            world.quickStep(0.05); // TODO ?

            // remove all contact joints
            contactgroup.empty();  // TODO remove?!?!?
            long time2 = System.nanoTime();
            timer += (time2 - time1);

            if (prevCount > 0 && j < 2) {
                // If we have (almost) immediate collision then the geoms where to close and may have low speed.
                // (speed is calculated from distance).
                fail("j=" + j);
            }

            // abort once we have stopped getting contacts
            if (prevCount > 0 && prevCount == cntCollisions) {
                reset(geom1, geom2);
                totalCount += cntCollisions;
                prevCount = 0;
                cntCollisions = 0;
                cntContacts = 0;
                continue;
            }
            prevCount = cntCollisions;
            if (cntCollisions > 300) {
                fail();  // TODO why is this so high for Box? -> See also slow falling box in DemoTrimesh..?
            }
            if (cntContacts > MAX_CONTACTS * cntCollisions) {
                fail();
            }
        }
        String msg = geom1.getClass().getSimpleName() + "-" + geom2.getClass().getSimpleName();
        System.out.println("Benchmark: " + msg + " contact/iterations: " + totalCount + "/" + iterations
                + " time: " + timer / iterations + " ns/step");
        geom1.destroy();
        geom2.destroy();
    }

    private DBody body() {
        DBody body = OdeHelper.createBody(world);

        body.setPosition(vector().scale(25));

        DMatrix3 R = new DMatrix3();
        DVector3 a = vector().scale(2).sub(1, 1, 1);
        dRFromAxisAndAngle(R, a, r.nextDouble() * 10.0 - 5.0);
        body.setRotation(R);

        return body;
    }

    private void reset(DGeom geom1, DGeom geom2) {
        geom1.setPosition(vector());
        geom2.setPosition(vector());

        geom1.getBody().setAngularVel(0, 0, 0);
        geom2.getBody().setAngularVel(0, 0, 0);

        geom2.getBody().setLinearVel(0, 0, 0);


        // Assure minimum distance
        DVector3 pos1 = new DVector3(geom1.getPosition());
        DVector3C pos2 = geom2.getPosition();
        while (pos1.distance(pos2) < 3) {
            pos1.add(1, 1, 1);
        }
        geom1.setPosition(pos1);

        // set collision course
        DVector3 dir = pos2.reSub(pos1);
        dir.scale(0.3);
        geom1.getBody().setLinearVel(dir);
        assertEquals(geom1.getPosition(), geom1.getBody().getPosition());
        assertEquals(geom2.getPosition(), geom2.getBody().getPosition());
    }

    private DGeom assemble(DGeom geom, DBody body, DMass mass) {
        geom.setBody(body);
        body.setMass(mass);
        return geom;
    }

    private DVector3 vector() {
        return new DVector3(r.nextDouble(), r.nextDouble(), r.nextDouble());
    }
}
