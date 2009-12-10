package org.ode4j.tests.bugs;


import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;

import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.net.URL;
import java.security.CodeSource;
import java.security.ProtectionDomain;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;
import java.util.regex.Pattern;

import org.ode4j.ode.DBallJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.DxHashSpace;
import org.ode4j.ode.internal.Misc;
import org.ode4j.ode.internal.OdeInit;
import org.ode4j.ode.internal.Timer;

import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;

/**
 * Test harness to check for multi-thread tolerance in IA numeric and it's
 * sub-packages (SPR 3998).
 * <p>
 * If this test harness does not throw any errors, then it is at least possible 
 * to write multi-threaded code using IA numeric. 
 * However, it does not indicate thread-safety of any tested classes.
 * The command-line version of this test prints out a list of 
 * state-full (=mutable) classes. Instances of classes from this list should
 * not be shared between Threads. All other classes (not printed) are
 * state-less (=immutable) and therefore thread-safe.
 * <p>
 * As a rule of thumb, any object that can be created with 'new' (like Int3d 
 * and MatrixMultiply) should be used in one Thread only. All others (like
 * ArcSin.PROCEDURE) can be used in parallel in multiple Threads.
 * To clarify this, for example Int3d can be process in parallel, as long as the
 * two Threads work on different Int3d arrays.
 *
 * @author Tilmann Zaeschke
 */
public class BugsTest extends TestCase {

    /**
     * @param name
     */
    public BugsTest(String name) {
        super(name);
    }


    /**
     */
    public void testBodyDampening() {
    	int NUM = 10;			/* number of boxes */
    	double SIDE = (0.2);		/* side length of a box */
    	double MASS = (1.0);		/* mass of a box */
    	double RADIUS = (0.1732f);	/* sphere radius */


    	/* dynamics and collision objects */

    	DBody[] body = new DBody[NUM];
    	DBallJoint[] joint = new DBallJoint[NUM-1];
    	DSphere[] sphere=new DSphere[NUM];

    	int i;
		double k;
		DMass m;

		/* create world */
//		OdeHelper.initODE2(0);
		DWorld world = OdeHelper.createWorld();
		DSpace space = OdeHelper.createHashSpace(null);
		DJointGroup contactgroup = OdeHelper.createJointGroup ();
		world.setGravity (0, 0, -0.5);
		OdeHelper.createPlane (space,0,0,1,0);

		//TZ
		m = OdeHelper.createMass();
		for (i=0; i<NUM; i++) {
			body[i] = OdeHelper.createBody(world);
			k = i*SIDE;
			body[i].setPosition(k,k,k+0.4);
			m.setBox(1,SIDE,SIDE,SIDE);
			m.adjust (MASS);
			body[i].setMass (m);
			sphere[i] = OdeHelper.createSphere (space,RADIUS);
			sphere[i].setBody(body[i]);
			body[i].setAngularDamping(0.3);
			body[i].setAngularDampingThreshold(0.9);
			body[i].getAngularDamping();
			body[i].getAngularDampingThreshold();
			body[i].setLinearDamping(0.3);
			body[i].setLinearDampingThreshold(0.9);
			body[i].getLinearDamping();
			body[i].getLinearDampingThreshold();
			body[i].setDampingDefaults();
		}
		for (i=0; i<(NUM-1); i++) {
			joint[i] = OdeHelper.createBallJoint(world,null);
			joint[i].attach(body[i],body[i+1]);
			k = (i+0.5)*SIDE;
			joint[i].setAnchor(k,k,k+0.4);
		}

		/* run simulation */
//		dsSimulationLoop (args,352,288,this);

		contactgroup.destroy();
		space.destroy();
		world.destroy();
//		OdeHelper.closeODE();
    }

    
    /**
     * @return A new test suite.
     */
    public static Test suite() {
        return new TestSuite(BugsTest.class);
      }
}
