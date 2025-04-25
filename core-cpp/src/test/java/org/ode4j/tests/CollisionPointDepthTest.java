package org.ode4j.tests;

import org.junit.Test;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DSphere;

import static org.ode4j.cpp.internal.ApiCppCollision.*;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.*;

public class CollisionPointDepthTest {

    //    #ifdef dSINGLE
    //        #define PROXIMITY_TOLERANCE     REAL(1e-7)
    //    #else
    //        #define PROXIMITY_TOLERANCE     REAL(1e-12)
    private static final double PROXIMITY_TOLERANCE = 1e-12;
    //    #endif


    //TEST(test_collision_sphere_point_depth)
    @Test
    public void test_collision_sphere_point_depth() {
        // Test case: sphere at the origin.
        final double radius = 1;
        DSphere sphere = dCreateSphere(null, radius);

        dGeomSetPosition(sphere, 0, 0, 0);

        // depth at center should equal radius
        CHECK_EQUAL(radius, dGeomSpherePointDepth(sphere, 0, 0, 0));

        // half-radius depth
        CHECK_EQUAL((0.5) * radius, dGeomSpherePointDepth(sphere, 0.5, 0, 0));
        CHECK_EQUAL((0.5) * radius, dGeomSpherePointDepth(sphere, 0, 0.5, 0));
        CHECK_EQUAL((0.5) * radius, dGeomSpherePointDepth(sphere, 0, 0, 0.5));
        CHECK_EQUAL((0.5) * radius, dGeomSpherePointDepth(sphere, -0.5, 0, 0));
        CHECK_EQUAL((0.5) * radius, dGeomSpherePointDepth(sphere, 0, -0.5, 0));
        CHECK_EQUAL((0.5) * radius, dGeomSpherePointDepth(sphere, 0, 0, -0.5));
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, 0.3, 0.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, 0, 0.3, 0.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, 0.4, 0, 0.3), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, -0.3, 0.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, 0, -0.3, 0.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, 0.4, 0, -0.3), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, 0.3, -0.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, 0, 0.3, -0.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, -0.4, 0, 0.3), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, -0.3, -0.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, 0, -0.3, -0.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.5) * radius, dGeomSpherePointDepth(sphere, -0.4, 0, -0.3), PROXIMITY_TOLERANCE);

        // 0.1 radius depth
        CHECK_CLOSE((0.1) * radius, dGeomSpherePointDepth(sphere, 0.9, 0, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * radius, dGeomSpherePointDepth(sphere, 0, 0.9, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * radius, dGeomSpherePointDepth(sphere, 0, 0, 0.9), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * radius, dGeomSpherePointDepth(sphere, -0.9, 0, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * radius, dGeomSpherePointDepth(sphere, 0, -0.9, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * radius, dGeomSpherePointDepth(sphere, 0, 0, -0.9), PROXIMITY_TOLERANCE);

        // on surface (zero depth)
        CHECK_EQUAL(0.0, dGeomSpherePointDepth(sphere, 1.0, 0, 0));
        CHECK_EQUAL(0.0, dGeomSpherePointDepth(sphere, 0, 1.0, 0));
        CHECK_EQUAL(0.0, dGeomSpherePointDepth(sphere, 0, 0, 1.0));
        CHECK_EQUAL(0.0, dGeomSpherePointDepth(sphere, -1.0, 0, 0));
        CHECK_EQUAL(0.0, dGeomSpherePointDepth(sphere, 0, -1.0, 0));
        CHECK_EQUAL(0.0, dGeomSpherePointDepth(sphere, 0, 0, -1.0));
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, 0.6, 0.8, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, 0, 0.6, 0.8), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, 0.8, 0, 0.6), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, -0.6, 0.8, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, 0, -0.6, 0.8), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, 0.8, 0, -0.6), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, 0.6, -0.8, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, 0, 0.6, -0.8), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, -0.8, 0, 0.6), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, -0.6, -0.8, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, 0, -0.6, -0.8), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(0.0, dGeomSpherePointDepth(sphere, -0.8, 0, -0.6), PROXIMITY_TOLERANCE);

        // 0.1 radius from surface (negative depth)
        CHECK_CLOSE(-(0.1) * radius, dGeomSpherePointDepth(sphere, 1.1, 0, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * radius, dGeomSpherePointDepth(sphere, 0, 1.1, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * radius, dGeomSpherePointDepth(sphere, 0, 0, 1.1), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * radius, dGeomSpherePointDepth(sphere, -1.1, 0, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * radius, dGeomSpherePointDepth(sphere, 0, -1.1, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * radius, dGeomSpherePointDepth(sphere, 0, 0, -1.1), PROXIMITY_TOLERANCE);

        // half-radius from surface (negative depth)
        CHECK_EQUAL(-(0.5) * radius, dGeomSpherePointDepth(sphere, 1.5, 0, 0));
        CHECK_EQUAL(-(0.5) * radius, dGeomSpherePointDepth(sphere, 0, 1.5, 0));
        CHECK_EQUAL(-(0.5) * radius, dGeomSpherePointDepth(sphere, 0, 0, 1.5));
        CHECK_EQUAL(-(0.5) * radius, dGeomSpherePointDepth(sphere, -1.5, 0, 0));
        CHECK_EQUAL(-(0.5) * radius, dGeomSpherePointDepth(sphere, 0, -1.5, 0));
        CHECK_EQUAL(-(0.5) * radius, dGeomSpherePointDepth(sphere, 0, 0, -1.5));
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, 0.9, 1.2, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, 0, 0.9, 1.2), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, 1.2, 0, 0.9), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, -0.9, 1.2, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, 0, -0.9, 1.2), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, 1.2, 0, -0.9), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, 0.9, -1.2, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, 0, 0.9, -1.2), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, -1.2, 0, 0.9), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, -0.9, -1.2, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, 0, -0.9, -1.2), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5), dGeomSpherePointDepth(sphere, -1.2, 0, -0.9), PROXIMITY_TOLERANCE);
    }

    // TEST(test_collision_box_point_depth)
    @Test
    public void test_collision_box_point_depth() {
        // Test case: cube at the origin.

        final double length = 1;
        DBox cube = dCreateBox(null, 2 * length, 2 * length, 2 * length);

        dGeomSetPosition(cube, 0, 0, 0);

        // depth at center should equal half length
        CHECK_EQUAL(length, dGeomBoxPointDepth(cube, 0, 0, 0));

        // half-length depth
        CHECK_EQUAL((0.5) * length, dGeomBoxPointDepth(cube, 0.5, 0, 0));
        CHECK_EQUAL((0.5) * length, dGeomBoxPointDepth(cube, 0, 0.5, 0));
        CHECK_EQUAL((0.5) * length, dGeomBoxPointDepth(cube, 0, 0, 0.5));
        CHECK_EQUAL((0.5) * length, dGeomBoxPointDepth(cube, -0.5, 0, 0));
        CHECK_EQUAL((0.5) * length, dGeomBoxPointDepth(cube, 0, -0.5, 0));
        CHECK_EQUAL((0.5) * length, dGeomBoxPointDepth(cube, 0, 0, -0.5));

        // closest point 0.6 * length
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, 0.3, 0.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, 0, 0.3, 0.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, 0.4, 0, 0.3), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, -0.3, 0.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, 0, -0.3, 0.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, 0.4, 0, -0.3), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, 0.3, -0.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, 0, 0.3, -0.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, -0.4, 0, 0.3), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, -0.3, -0.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, 0, -0.3, -0.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.6) * length, dGeomBoxPointDepth(cube, -0.4, 0, -0.3), PROXIMITY_TOLERANCE);

        // 0.1 length depth
        CHECK_CLOSE((0.1) * length, dGeomBoxPointDepth(cube, 0.9, 0, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * length, dGeomBoxPointDepth(cube, 0, 0.9, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * length, dGeomBoxPointDepth(cube, 0, 0, 0.9), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * length, dGeomBoxPointDepth(cube, -0.9, 0, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * length, dGeomBoxPointDepth(cube, 0, -0.9, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE((0.1) * length, dGeomBoxPointDepth(cube, 0, 0, -0.9), PROXIMITY_TOLERANCE);

        // on surface (zero depth)
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 1.0, 0, 0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0, 1.0, 0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0, 0, 1.0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, -1.0, 0, 0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0, -1.0, 0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0, 0, -1.0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0.3, 1.0, 0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0, 0.3, 1.0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 1.0, 0, 0.3));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, -0.3, 1.0, 0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0, -0.3, 1.0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 1.0, 0, -0.3));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0.3, -1.0, 0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0, 0.3, -1.0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, -1.0, 0, 0.3));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, -0.3, -1.0, 0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, 0, -0.3, -1.0));
        CHECK_EQUAL(0.0, dGeomBoxPointDepth(cube, -1.0, 0, -0.3));

        // 0.1 length from surface (negative depth)
        CHECK_CLOSE(-(0.1) * length, dGeomBoxPointDepth(cube, 1.1, 0, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * length, dGeomBoxPointDepth(cube, 0, 1.1, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * length, dGeomBoxPointDepth(cube, 0, 0, 1.1), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * length, dGeomBoxPointDepth(cube, -1.1, 0, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * length, dGeomBoxPointDepth(cube, 0, -1.1, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.1) * length, dGeomBoxPointDepth(cube, 0, 0, -1.1), PROXIMITY_TOLERANCE);

        // half-length from surface face (negative depth)
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 1.5, 0, 0));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0, 1.5, 0));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0, 0, 1.5));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, -1.5, 0, 0));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0, -1.5, 0));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0, 0, -1.5));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0.3, 1.5, 0));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0, 0.3, 1.5));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 1.5, 0, 0.3));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, -0.3, 1.5, 0));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0, -0.3, 1.5));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 1.5, 0, -0.3));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0.3, -1.5, 0));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0, 0.3, -1.5));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, -1.5, 0, 0.3));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, -0.3, -1.5, 0));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, 0, -0.3, -1.5));
        CHECK_EQUAL(-(0.5) * length, dGeomBoxPointDepth(cube, -1.5, 0, -0.3));
        // half-length from surface edge (negative depth)
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, 1.3, 1.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, 0, 1.3, 1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, 1.4, 0, 1.3), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, -1.3, 1.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, 0, -1.3, 1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, 1.4, 0, -1.3), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, 1.3, -1.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, 0, 1.3, -1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, -1.4, 0, 1.3), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, -1.3, -1.4, 0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, 0, -1.3, -1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.5) * length, dGeomBoxPointDepth(cube, -1.4, 0, -1.3), PROXIMITY_TOLERANCE);
        // 0.6 length from corner (negative depth)
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, 1.2, 1.4, 1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, 1.4, 1.2, 1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, 1.4, 1.4, 1.2), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, -1.2, 1.4, 1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, 1.4, -1.2, 1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, 1.4, 1.4, -1.2), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, 1.2, -1.4, 1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, 1.4, 1.2, -1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, -1.4, 1.4, 1.2), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, -1.2, -1.4, 1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, 1.4, -1.2, -1.4), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(-(0.6) * length, dGeomBoxPointDepth(cube, -1.4, 1.4, -1.2), PROXIMITY_TOLERANCE);
    }
}
