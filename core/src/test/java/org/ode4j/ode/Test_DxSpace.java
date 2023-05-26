package org.ode4j.ode;

import org.junit.Before;
import org.junit.Test;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.junit.Assert.*;

import java.util.concurrent.atomic.AtomicInteger;

public class Test_DxSpace {

	private DSimpleSpace space;
	private DSphere sphere;
	private DBox box;
	private DCapsule capsule;

	@Before
	public void setUp() {
		OdeHelper.initODE2(0);
		space = OdeHelper.createSimpleSpace();
		sphere = OdeHelper.createSphere(1);
		box = OdeHelper.createBox(1, 2, 3);
		capsule = OdeHelper.createCapsule(1, 2);
	}

	@Test
	public void testCreateDestroy() {
		space.add(sphere);
		space.add(box);
		space.add(capsule);
		assertEquals(3, space.getNumGeoms());
		assertTrue(contains(sphere));
		assertTrue(contains(box));
		assertTrue(contains(capsule));
		space.destroy();
		assertEquals(0, space.getNumGeoms());
		assertFalse(contains(sphere));
		assertFalse(contains(box));
		assertFalse(contains(capsule));
	}

	@Test
	public void testAddRemove() {
		space.add(sphere);
		assertEquals(1, space.getNumGeoms());
		assertTrue(contains(sphere));
		space.add(box);
		space.add(capsule);
		assertEquals(3, space.getNumGeoms());
		assertTrue(contains(sphere));
		assertTrue(contains(box));
		assertTrue(contains(capsule));
		space.remove(box);
		assertEquals(2, space.getNumGeoms());
		assertTrue(contains(sphere));
		assertFalse(contains(box));
		assertTrue(contains(capsule));
		capsule.destroy();
		assertEquals(1, space.getNumGeoms());
		assertTrue(contains(sphere));
		assertFalse(contains(box));
		assertFalse(contains(capsule));
		space.destroy();
		assertEquals(0, space.getNumGeoms());
		assertFalse(contains(sphere));
		assertFalse(contains(box));
		assertFalse(contains(capsule));
	}

	@Test
	public void testAddRemoveCollide() {
		space.add(sphere);
		assertEquals(1, space.getNumGeoms());
		assertTrue(contains(sphere));
		space.add(box);
		space.add(capsule);
		assertEquals(3, space.getNumGeoms());
		assertTrue(contains(sphere));
		assertTrue(contains(box));
		assertTrue(contains(capsule));
		final AtomicInteger collisions = new AtomicInteger(0);
		space.collide(null, (data, o1, o2) -> collisions.incrementAndGet());
		assertEquals(3, collisions.get());
		space.remove(box);
		assertEquals(2, space.getNumGeoms());
		assertTrue(contains(sphere));
		assertFalse(contains(box));
		assertTrue(contains(capsule));
		collisions.set(0);
		space.collide(null, (data, o1, o2) -> collisions.incrementAndGet());
		assertEquals(1, collisions.get());
		capsule.destroy();
		assertEquals(1, space.getNumGeoms());
		assertTrue(contains(sphere));
		assertFalse(contains(box));
		assertFalse(contains(capsule));
		collisions.set(0);
		space.collide(null, (data, o1, o2) -> collisions.incrementAndGet());
		assertEquals(0, collisions.get());
		space.destroy();
		assertEquals(0, space.getNumGeoms());
		assertFalse(contains(sphere));
		assertFalse(contains(box));
		assertFalse(contains(capsule));
	}

	private boolean contains(DGeom geom) {
		for (DGeom g : space.getGeoms()) {
			if (geom == g) {
				return true;
			}
		}
		return false;
	}
}
