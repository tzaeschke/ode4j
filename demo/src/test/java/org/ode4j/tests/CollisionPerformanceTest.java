package org.ode4j.tests;

import org.junit.After;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DSapSpace.AXES;
import org.ode4j.ode.internal.DxBVHSpace;
import org.ode4j.ode.internal.DxGimpactData;
import org.ode4j.ode.internal.DxMass;
import org.ode4j.ode.internal.DxSAPSpace2;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.OdeHelper.areConnectedExcluding;

public class CollisionPerformanceTest {

    private final static int WARMUP = 10;
    private final static int BENCHMARK = 100;

    private DWorld world;
    private DSpace space;
    private static DJointGroup contactgroup;
    private static Random r;
    private static final double DENSITY = (5.0);        // density of all objects

    private static final float CUBE_POINTS[] = {
            0.25f, 0.25f, 0.25f, // point 0
            -0.25f, 0.25f, 0.25f, // point 1
            0.25f, -0.25f, 0.25f, // point 2
            -0.25f, -0.25f, 0.25f, // point 3
            0.25f, 0.25f, -0.25f, // point 4
            -0.25f, 0.25f, -0.25f, // point 5
            0.25f, -0.25f, -0.25f, // point 6
            -0.25f, -0.25f, -0.25f,// point 7
    };

    private static final int CUBE_INDICES[] = {
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

    @BeforeClass
    public static void beforeClass() {
        // TODO warmup
        r = createRandom();
    }

    @Before
    public void beforeTest() {
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        space = createSpace();
        contactgroup = OdeHelper.createJointGroup();
        // world.setGravity (0,0,-0.5); // TODO ?
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
        for (int i = 0; i < WARMUP; ++i) {
            collide(space, box(), trimesh());
        }
        for (int i = 0; i < BENCHMARK; ++i) {
            collide(space, box(), trimesh());
        }
    }

    @Test
    public void testCylinderTrimesh() {
        for (int i = 0; i < WARMUP; ++i) {
            collide(space, cylinder(), trimesh());
        }
        for (int i = 0; i < BENCHMARK; ++i) {
            collide(space, cylinder(), trimesh());
        }
    }

    @Test
    public void testSphereTrimesh() {
        for (int i = 0; i < WARMUP; ++i) {
            collide(space, sphere(), trimesh());
        }
        for (int i = 0; i < BENCHMARK; ++i) {
            collide(space, sphere(), trimesh());
        }
    }

    private DGeom box() {
        // new Body(), new Mass(), new Geom(), geom.setBody(), body.setMass().
        DVector3 sides = vector().scale(6).add(new DVector3(0.25, 0.25, 0.25));
        DGeom geom = OdeHelper.createBox(space, sides);

        DMass mass = OdeHelper.createMass();
        mass.setBox(DENSITY, sides);

        return assemble(geom, body(), mass);
    }

    private DGeom cylinder() {
        // new Body(), new Mass(), new Geom(), geom.setBody(), body.setMass().
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
        DxGimpactData data = new DxGimpactData();
        data.build(CUBE_POINTS, CUBE_INDICES);
        data.preprocess();
        DTriMesh geom = OdeHelper.createTriMesh(space, data, null, null, null);

        DMass mass = OdeHelper.createMass();
        mass.setTrimesh(DENSITY, geom);

        return assemble(geom, body(), mass);
    }

    private DSpace createSpace() {
        return OdeHelper.createSimpleSpace();
        // return OdeHelper.createSapSpace(AXES.XZY);
        // return DxSAPSpace2.dSweepAndPruneSpaceCreate(null, AXES.XZY.getCode(), STATIC_CATEGORY);
        //return DxBVHSpace.bvhSpaceCreate(null, 16, false, 0.2, STATIC_CATEGORY);
    }

    private static Random createRandom() {
        return new Random(0);
    }

    private DNearCallback nearCallback = new DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback(data, o1, o2);
        }
    };

    // this is called by dSpaceCollide when two objects in space are
    // potentially colliding.

    int cnt1 = 0;
    int cnt2 = 0;

    private void nearCallback(Object data, DGeom o1, DGeom o2) {
        int MAX_CONTACTS = 5;
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
        }
        int numc = OdeHelper.collide(o1, o2, MAX_CONTACTS, contacts.getGeomBuffer());
        if (numc != 0) {
            cnt1++;
            cnt2 += numc;
            //double speed1 = o1.getBody().getLinearVel().length();
            //double speed2 = o2.getBody().getLinearVel().length();
            //System.out.println("contact: " + numc + "  speed: " + speed1 + "/" + speed2);
            DMatrix3 RI = new DMatrix3();
            RI.setIdentity();
            for (int i = 0; i < numc; i++) {
                DContact contact = contacts.get(i);
                DJoint c = OdeHelper.createContactJoint(world, contactgroup, contact);
                c.attach(b1, b2);
            }
        }
    }

    private void collide(DSpace space, DGeom geom1, DGeom geom2) {
        setCollisionCourse(geom1, geom2);

        int iterations = 1000;
        long timer = 0;
        int prevCount = 0;
        cnt1 = 0;
        cnt2 = 0;
        for (int j = 0; j < iterations; j++) {
            long time1 = System.nanoTime();
            space.collide(0, nearCallback);
            // world.step(0.05);
            world.quickStep(0.05); // TODO ?

            // remove all contact joints
            contactgroup.empty();  // TODO remove?!?!?
            long time2 = System.nanoTime();
            timer += (time2 - time1);

            if (prevCount > 0) {
                if (j < 2) {
                    fail("j=" + j);
                }
                //double dist = geom2.getPosition().distance(geom1.getPosition());
                //System.out.println("j=" + j + "  dist = " + dist + "   " + geom1.getPosition() + " " + geom2.getPosition());
            }

            if (prevCount > 0 && prevCount == cnt1) {
                break;
            }
            if (cnt1 > 10) {
                break;
            }
            prevCount = cnt1;
            if (j > 1000) {
                fail();
            }
        }
        System.out.println("Time step: " + cnt1 + "/" + cnt2 + " time=" + timer / 1000 / 1000 + "ms");
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

    private void setCollisionCourse(DGeom geom1, DGeom geom2) {
        // Assure minimum distance
        DVector3 pos1 = new DVector3(geom1.getPosition());
        DVector3C pos2 = geom2.getPosition();
        while (pos1.distance(pos2) < 2) {
            geom1.setPosition(pos1.add(1, 1, 1));
        }

        DVector3 dir = pos2.reSub(pos1);
        // TODO dir.scale(0.1);
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
