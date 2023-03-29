package org.ode4j.demo;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.OdeHelper.*;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DRay;
import org.ode4j.ode.DSapSpace.AXES;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

public class RaycastTest extends dsFunctions {

	// @formatter:off
	private static final float[] trimeshBlockVertices = new float[] {
		// top
		-0.5f, -0.5f, 0.5f,
		0.5f, -0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,
		0.5f, 0.5f, 0.5f,

		// bottom
		-0.5f, 0.5f, -0.5f,
		0.5f, 0.5f, -0.5f,
		-0.5f, -0.5f, -0.5f,
		0.5f, -0.5f, -0.5f,

		// front
		-0.5f, 0.5f, 0.5f,
		0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f, -0.5f,
		0.5f, 0.5f, -0.5f,

		// back
		0.5f, -0.5f, 0.5f,
		-0.5f, -0.5f, 0.5f,
		0.5f, -0.5f, -0.5f,
		-0.5f, -0.5f, -0.5f,

		// left
		-0.5f, -0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,
		-0.5f, -0.5f, -0.5f,
		-0.5f, 0.5f, -0.5f,

		// right
		0.5f, 0.5f, 0.5f,
		0.5f, -0.5f, 0.5f,
		0.5f, 0.5f, -0.5f,
		0.5f, -0.5f, -0.5f
	};
	
	private static final int[] trimeshBlockIndices = new int[] {
		// top
		3, 2, 0,
		0, 1, 3,

		// bottom
		7, 6, 4,
		4, 5, 7,

		// front
		11, 10, 8,
		8, 9, 11,

		// back
		15, 14, 12,
		12, 13, 15,

		// left
		19, 18, 16,
		16, 17, 19,

		// right
		23, 22, 20,
		20, 21, 23
	};
	
	private static final DVector3[] blockFaceCenterPositions = new DVector3[] {
		// top
		new DVector3(0, 0, 0.5),
		// bottom
		new DVector3(0, 0, -0.5),
		// front
		new DVector3(0, 0.5, 0),
		// back
		new DVector3(0, -0.5, 0),
		// left
		new DVector3(-0.5, 0, 0),
		// right
		new DVector3(0.5, 0, 0),
	};
	// @formatter:on

	private DWorld world;

	private DSpace space;

	private DJointGroup contactgroup;

	private Set<DGeom> geoms;

	public void run(String[] args) {
		dsSimulationLoop(args, 1024, 768, this);
	}

	@Override
	public void start() {
		initODE();

		this.world = createWorld();
		this.world.setQuickStepNumIterations(60);
		this.world.setContactSurfaceLayer(0.001);
		this.world.setGravity(0, 0, -9.81);

		//this.space = createQuadTreeSpace(new DVector3(0, 0, 0), new DVector3(250, 250, 250), 8);
		//this.space = createSapSpace(AXES.XYZ);
		this.space = createSimpleSpace();

		this.contactgroup = createJointGroup();

		this.geoms = new HashSet<>();

		dsSetViewpoint(new double[] { -1, -5, 12 }, new double[] { 90, -75, 0 });
		
		System.err.println("Use ' ' to create the explosion and destroy matching blocks");
		System.err.println("Use 'b' to reset the stage using boxes");
		System.err.println("Use 't' to reset the stage using trimeshes");
		
		reset(false);
	}

	@Override
	public void step(boolean pause) {
		this.space.collide(null, this.nearCallback);
		this.world.quickStep(0.033);
		this.contactgroup.empty();

		// now we draw everything
		for (DGeom g : geoms) {
			this.drawGeom(g);
		}
	}

	@Override
	public void command(char cmd) {
		switch (cmd) {
		case ' ':
			createExplosion(new DVector3(0, 0, 3), 3);
			break;
		case 'b':
			reset(false);
			break;
		case 't':
			reset(true);
			break;
		}
	}

	@Override
	public void stop() {
		this.contactgroup.destroy();
		this.world.destroy();
		this.space.destroy();

		closeODE();
	}
	
	private void reset(boolean trimesh) {
		// clear the geoms if any
		Set<DGeom> geomsCopy = new HashSet<>(geoms);
		for (DGeom geom : geomsCopy) {
			destroyGeom(geom);
		}
		
		// create some blocks
		for (int x = -5; x < 5; x++) {
			for (int y = -5; y < 5; y++) {
				DVector3 blockPosition = new DVector3(x, y, 0);
				DQuaternion blockRotation = new DQuaternion(1, 0, 0, 0);
				createGeom(blockPosition, blockRotation, trimesh);
			}
		}
	}

	private void createGeom(DVector3 position, DQuaternion rotation, boolean trimesh) {
		DGeom geom;
		if (trimesh) {
			DTriMeshData normalBlockTrimeshData = OdeHelper.createTriMeshData();

			normalBlockTrimeshData.build(trimeshBlockVertices, trimeshBlockIndices);
			normalBlockTrimeshData.preprocess();

			geom = OdeHelper.createTriMesh(this.space, normalBlockTrimeshData, null, null, null);
			
			// I have no idea what this is but still
//			((DTriMesh) geom).enableTC(DSphere.class, false);
//			((DTriMesh) geom).enableTC(DBox.class, false);
//			((DTriMesh) geom).enableTC(DCapsule.class, false);
		} else {
			geom = createBox(this.space, 1, 1, 1);
		}

		geom.setPosition(position);
		geom.setQuaternion(rotation);

		geoms.add(geom);
	}

	private void destroyGeom(DGeom geom) {
		geom.destroy();

		geoms.remove(geom);
	}

	private void createExplosion(DVector3 position, double radius) {
		
		System.out.println("explosion position=" + position);
		
		// first create a sphere intersection
		DSphere explosion = OdeHelper.createSphere(radius);
		explosion.setPosition(position);

		List<DGeom> explosionTargets = new ArrayList<>();
		OdeHelper.spaceCollide2(explosion, this.space, explosionTargets, this.explosionNearCallback);

		// destroy the sphere intersection
		explosion.destroy();

		// now on to the raycast
		final int MAX_CONTACTS = 1;

		// test against the top face
		// all the other faces show the same problem but for the sake of simplicity just use the top face
		DVector3 topFacePosition = blockFaceCenterPositions[0];

		for (DGeom explosionTarget : explosionTargets) 
		{
		//	DGeom explosionTarget = explosionTargets.get(10);
			DContactGeomBuffer contacts = new DContactGeomBuffer(MAX_CONTACTS);
			// let the ray be little longer than the radius so we are sure it penetrates the face
			DRay ray = OdeHelper.createRay(this.space, 6);

			ray.setFirstContact(false);
			ray.setBackfaceCull(false);
			ray.setClosestHit(true);

			double rotationX = explosionTarget.getPosition().get0() + topFacePosition.get0() - position.get0();
			double rotationY = explosionTarget.getPosition().get1() + topFacePosition.get1() - position.get1();
			double rotationZ = explosionTarget.getPosition().get2() + topFacePosition.get2() - position.get2();
			ray.set(position.get0(), position.get1(), position.get2(), rotationX, rotationY, rotationZ);

			int numC = OdeHelper.collide(ray, this.space, MAX_CONTACTS, contacts);
			for (int i = 0; i < numC; i++) {
				DContactGeom contact = contacts.get(i);

				boolean isTarget = contact.g2 == explosionTarget;
				
				// just print some info
				System.out.println("testing explosionTarget=" + explosionTarget + "; explosionFace position="
						+ new DVector3().eqSum(explosionTarget.getPosition(), topFacePosition) + "; CONTACT; position=" + contact.pos + "; normal="
						+ contact.normal + "; depth=" + contact.depth + "; g2=" + contact.g2 + "; g2Position=" + contact.g2.getPosition() + "; isTarget="
						+ isTarget);

				// destroy the geom if matched
				if (isTarget) {
					System.out.println("destroying match");

					destroyGeom(contact.g2);
				}
			}

			geoms.add(ray);
		}
	}

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			final int MAX_CONTACTS = 80;

			DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);

			int numc = OdeHelper.collide(o1, o2, MAX_CONTACTS, contacts.getGeomBuffer());

			for (int i = 0; i < numc; i++) {
				DContact contact = contacts.get(i);

				contact.surface.mode = dContactApprox1;

				DJoint c = OdeHelper.createContactJoint(RaycastTest.this.world, RaycastTest.this.contactgroup, contact);
				c.attach(o1.getBody(), o2.getBody());
			}

		}
	};

	private DNearCallback explosionNearCallback = new DNearCallback() {
		@SuppressWarnings("unchecked")
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {

			final int MAX_CONTACTS = 1;
			DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);

			if (OdeHelper.collide(o1, o2, MAX_CONTACTS, contacts.getGeomBuffer()) != 0) {
				DGeom explosionTarget = o1.getSpace() == RaycastTest.this.space ? o1 : o2;

				((List<DGeom>) data).add(explosionTarget);
			}

		}
	};

	private void drawGeom(DGeom g) {
		if (g instanceof DRay) {
			DRay ray = (DRay) g;

			// draw the ray itself
			dsSetTexture(DS_TEXTURE_NUMBER.DS_NONE);
			DVector3 origin = new DVector3(), dir = new DVector3();
			ray.get(origin, dir);
			dir.eqSum(origin, dir, ray.getLength());
			dsDrawLine(origin, dir);

			// draw start position
			dsSetTexture(DS_TEXTURE_NUMBER.DS_CHECKERED);
			dsDrawSphere(origin, ray.getRotation(), 0.05);
		} else if (g instanceof DBox) {
			DBox box = (DBox) g;
			dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);
			dsDrawBox(g.getPosition(), g.getRotation(), box.getLengths());
		} else if (g instanceof DTriMesh) {
			dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);

			for (int i = 0; i < trimeshBlockIndices.length; i += 3) {
				int i0 = trimeshBlockIndices[i + 0] * 3;
				int i1 = trimeshBlockIndices[i + 1] * 3;
				int i2 = trimeshBlockIndices[i + 2] * 3;
				dsDrawTriangle(g.getPosition(), g.getRotation(), trimeshBlockVertices, i0, i1, i2, true); // single precision draw
			}
		}
	}

	public static void main(String[] args) {
		new RaycastTest().run(args);
	}
}