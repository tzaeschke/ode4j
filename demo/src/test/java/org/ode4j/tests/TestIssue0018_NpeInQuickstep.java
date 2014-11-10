package org.ode4j.tests;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawLine;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetColorAlpha;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;

import org.junit.Test;
import org.ode4j.drawstuff.DrawStuff;
import org.ode4j.drawstuff.internal.DrawStuffNull;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DDoubleHingeJoint;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

/**
 *
 * @author Jing Huang
 */
public class TestIssue0018_NpeInQuickstep extends DrawStuff.dsFunctions {

	public TestIssue0018_NpeInQuickstep() {
		super();

	}

	private void drawGeom(DGeom g) {
		DVector3C pos = g.getPosition();
		DMatrix3C rot = g.getRotation();

		if (g instanceof DBox) {
			if (applyForce) {
				dsSetColor(1., .5, 0.);
			} else {
				dsSetColor(1, 1, 0);
			}
			dsSetTexture(DrawStuff.DS_TEXTURE_NUMBER.DS_WOOD);
			DVector3C lengths = ((DBox) g).getLengths();
			dsDrawBox(pos, rot, lengths);
		}
	}

	//draw order z x y in this 
	private void draw() {

	}

	private void simLoop(boolean pause) {
		if (!pause) {
			if (!pause) {

				final double step = 0.005;
				//world.step(step);
				world.quickStep(step);

			}

			{
				DVector3 p = new DVector3(0, 0, 0);
				dsSetColorAlpha(1, 0, 0, 1);
				dsDrawBox(p.reAdd(new DVector3(0.3, 0, 0)), new DMatrix3().setIdentity(), new DVector3(0.3, 0.05, 0.05));
				dsSetColorAlpha(0, 1, 0, 1);
				dsDrawBox(p.reAdd(new DVector3(0, 0.3, 0)), new DMatrix3().setIdentity(), new DVector3(0.05, 0.3, 0.05));
				dsSetColorAlpha(0, 0, 1, 1);
				dsDrawBox(p.reAdd(new DVector3(0, 0, 0.3)), new DMatrix3().setIdentity(), new DVector3(0.05, 0.05, 0.3));
			}

			// now we draw everything
			int ngeoms = space.getNumGeoms();
			for (int i = 0; i < ngeoms; ++i) {
				DGeom g = space.getGeom(i);
				drawGeom(g);
			}

			if (true) {
				DVector3 a11 = new DVector3(), a12 = new DVector3();
				joint1.getAnchor1(a11);
				joint1.getAnchor2(a12);
				dsSetColor(1, 0, 0);
				dsDrawLine(a11, a12);
			}

			if (true) {
				DVector3 a21 = new DVector3(), a22 = new DVector3();
				joint2.getAnchor1(a21);
				joint2.getAnchor2(a22);
				dsSetColor(0, 1, 0);
				dsDrawLine(a21, a22);
			}
		}
		draw();
	}

	private DWorld world;
	private DSpace space;
	private DBody body1;
	private DBody body2;
	private DDoubleHingeJoint joint1;
	private DDoubleHingeJoint joint2;
	private boolean applyForce = false;
	private DAMotorJoint _motor;
	//private static double[] xyz = {3.8966, -2.0614, 4.0300};
	//private static double[] hpr = {153.5, -16.5, 0};
	private DVector3 _axis0 = new DVector3(1, 0, 0);
	private DVector3 _axis1 = new DVector3(0, 1, 0);
	private DVector3 _axis2 = new DVector3(0, 0, 1);
	
	@Override
	public void start() {
		world = OdeHelper.createWorld();
		world.setGravity(0, 0, 0);

		world.setDamping(1e-4, 1e-5);
		//	    dWorldSetERP(world, 1);

		space = OdeHelper.createSimpleSpace();

		body1 = OdeHelper.createBody(world);
		body2 = OdeHelper.createBody(world);

		body1.setPosition(0, 1, 0);
		body2.setPosition(0, 3, 0);

		DGeom g;
		DMass mass = OdeHelper.createMass();

		g = OdeHelper.createBox(space, 0.2, 1.9, 0.2);
		g.setBody(body1);
		mass.setBox(1, 0.2, 2, 0.2);
		body1.setMass(mass);

		g = OdeHelper.createBox(space, 0.2, 1.9, 0.2);
		g.setBody(body2);
		mass.setBox(1, 0.2, 2, 0.2);
		body2.setMass(mass);

		if (true) {
			joint1 = OdeHelper.createDHingeJoint(world);
			joint1.attach(body1, null);
			joint1.setAxis(0, 0, 1);
			joint1.setAnchor1(0, 0, 0);
			joint1.setAnchor2(0, 0, 0);

		}

		if (true) {
			joint2 = OdeHelper.createDHingeJoint(world);
			joint2.attach(body1, body2);
			joint2.setAxis(0, 1, 0);
			joint2.setAnchor1(0, 2, 0);
			joint2.setAnchor2(0, 2, 0);

			_motor = OdeHelper.createAMotorJoint(world);
			_motor.attach(body1, body2);
			_motor.setMode(DAMotorJoint.AMotorMode.dAMotorEuler);
			_motor.setNumAxes(3);
			_motor.setAxis(0, 1, _axis0.get0(), _axis0.get1(), _axis0.get2());
			_motor.setAxis(1, 1, _axis1.get0(), _axis1.get1(), _axis1.get2());
			_motor.setAxis(2, 1, _axis2.get0(), _axis2.get1(), _axis2.get2());

			// XXX: make these changeable, for example using the XML
			_motor.setParam(DJoint.PARAM_N.dParamFudgeFactor1, 0.8f);
			_motor.setParam(DJoint.PARAM_N.dParamFudgeFactor2, 0.8f);
			_motor.setParam(DJoint.PARAM_N.dParamFudgeFactor3, 0.8f);

			_motor.setParam(DJoint.PARAM_N.dParamStopCFM1, 0.2f);
			_motor.setParam(DJoint.PARAM_N.dParamStopCFM2, 0.2f);
			_motor.setParam(DJoint.PARAM_N.dParamStopCFM3, 0.2f);
		}
	}

	@Override
	public void step(boolean pause) {
		simLoop(pause);
	}

	@Override
	public void command(char cmd) {
		if (cmd == '1') {

		} else if (cmd == 'w') {
			//	         
		} else if (cmd == 's') {
			_motor.addTorques(0, 1.1, 0);
		} else if (cmd == 'a') {

		} else if (cmd == 'd') {

		} else if (cmd == 'e') {

		} else if (cmd == 'b') {

		}
	}

	@Override
	public void stop() {
		//throw new UnsupportedOperationException("Not supported yet.");
	}

	/**
	 * See issue #18. This failed with NPE in quickstep() because of uninitialised DJointWithInfo.
	 * The reason is that the 'nj' used for creation of the jointinfo[] is sometimes bigger than 
	 * the 'nj' used when initialising jointinfo[].
	 * 
	 */
	@Test
	public void test() {
		DrawStuff.dsSetOutputNull();
		// create world
		OdeHelper.initODE();

		start();
		for (int i = 0; i < 1000; i++) {
			step(false);
		}
	}
	
	private void demo(String[] args) {
		// create world
		OdeHelper.initODE();

		// run demo
		dsSimulationLoop(args, 800, 600, this);

		OdeHelper.closeODE();
	}

	/**
	 * @param args
	 */
	public static void main(final String[] args) {
		new TestIssue0018_NpeInQuickstep().demo(args);
	}

}