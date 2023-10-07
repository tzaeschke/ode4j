/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode;

import java.io.File;
import java.util.List;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DTriMesh.DTriCallback;
import org.ode4j.ode.DTriMesh.DTriRayCallback;
import org.ode4j.ode.internal.*;
import org.ode4j.ode.internal.joints.DxJointConstrainedBall;
import org.ode4j.ode.internal.ragdoll.DxRagdoll;
import org.ode4j.ode.internal.trimesh.DxTriMesh;
import org.ode4j.ode.internal.trimesh.DxTriMeshData;
import org.ode4j.ode.internal.joints.DxJointGroup;
import org.ode4j.ode.internal.joints.OdeJointsFactoryImpl;
import org.ode4j.ode.ragdoll.DRagdoll;
import org.ode4j.ode.ragdoll.DRagdollConfig;

/**
 * This is the general helper class for ode4j.
 * <p>
 * It provides: 
 * <ul>
 * <li> Factory methods for most of the classes in ode4j </li>
 * <li> Initialisation methods ({@code initOde2()} </li>
 * <li> Collision methods </li>   
 * <li> Other helper methods </li>
 * </ul>   
 */
public abstract class OdeHelper {

	private static final OdeFactoryImpl ODE = new OdeFactoryImpl(); 

	//This could later be used to implement a pluggable factory, where the
	//static methods call actual methods in OdeFactoryImpl.
	//	public static OdeFactory getInstance() {
	//		if (ODE == null) {
	//			ODE = new OdeFactoryImpl();
	//		}
	//		return ODE;
	//	}

	//****************************************************************************
	// joints

	//	public abstract dJointBall dJointCreateBall (dWorld w, dJointGroup group);
	//
	//	public abstract dJointHinge dJointCreateHinge (dWorld w, dJointGroup group);
	//
	//	public abstract dJoint dJointCreateSlider (dWorld w, dJointGroup group);
	//
	//	//		dxJoint dJointCreateContact (dWorld w, dJointGroup group,
	//	//				final dContact[] c)
	//	public abstract dJointContact dJointCreateContact (dWorld w, dJointGroup group,
	//			final dContact c);
	//
	//	public abstract dJointHinge2 dJointCreateHinge2 (dWorld w, dJointGroup group);
	//
	//	public abstract dJointUniversal dJointCreateUniversal (dWorld w, dJointGroup group);
	//
	//	public abstract dJoint  dJointCreatePR (dWorld w, dJointGroup group);
	//
	//	public abstract dJoint  dJointCreatePU (dWorld w, dJointGroup group);
	//
	//	public abstract dJoint  dJointCreatePiston (dWorld w, dJointGroup group);
	//	
	//	public abstract dJointFixed dJointCreateFixed (dWorld id, dJointGroup group);
	//
	//	abstract dxJointNull  dJointCreateNull (dWorld w, dJointGroup group);
	//
	//	public abstract dJointAMotor dJointCreateAMotor (dWorld w, dJointGroup group);
	//
	//	public abstract dJointLMotor  dJointCreateLMotor (dWorld w, dJointGroup group);
	//
	//	public abstract dJointPlane2D  dJointCreatePlane2D (dWorld w, dJointGroup group);

	/**
	 * Create a joint group.
	 * @return joint group
	 */
	public static DJointGroup createJointGroup () {
		 //param max_size deprecated. Set to 0.
		return DxJointGroup.dJointGroupCreate(-1);
	}
	/**
	 * Create a new joint feedback.
	 * @return joint feedback
	 */
	public static DJoint.DJointFeedback createJointFeedback() {
		return new DJoint.DJointFeedback();
	}

	// AMotorJoint
	/**
	 * Create a new joint of the A-motor type.
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DAMotorJoint createAMotorJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateAMotor(world, group);
	}
	/**
	 * Create a new joint of the A-motor type.
	 * @param world world
	 * @return new joint
	 */
	public static DAMotorJoint createAMotorJoint (DWorld world) {
		return ODE.dJointCreateAMotor(world, null);
	}

	// BallJoint
	/**
	 * Create a new joint of the ball type.
	 *
	 * <p>REMARK:
	 * The joint is initially in "limbo" (i.e. it has no effect on the simulation)
	 * because it does not connect to any bodies.
	 *
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DBallJoint createBallJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateBall(world, group);
	}
	/**
	 * Create a new joint of the ball type.
	 *
	 * <p>REMARK:
	 * The joint is initially in "limbo" (i.e. it has no effect on the simulation)
	 * because it does not connect to any bodies.
	 * @param world world
	 * @return new joint
	 */
	public static DBallJoint createBallJoint (DWorld world) {
		return ODE.dJointCreateBall(world, null);
	}

	// ConstrainedBallJoint
	/**
	 * Create a new joint of the constrained ball type. This is a custom extension in ode4j.
	 *
	 * <p>REMARK:
	 * The joint is initially in "limbo" (i.e. it has no effect on the simulation)
	 * because it does not connect to any bodies.
	 *
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DxJointConstrainedBall createConstrainedBallJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateConstrainedBall(world, group);
	}
	/**
	 * Create a new joint of the constrained ball type. This is a custom extension in ode4j.
	 *
	 * <p>REMARK:
	 * The joint is initially in "limbo" (i.e. it has no effect on the simulation)
	 * because it does not connect to any bodies.
	 * @param world world
	 * @return new joint
	 */
	public static DxJointConstrainedBall createConstrainedBallJoint (DWorld world) {
		return ODE.dJointCreateConstrainedBall(world, null);
	}

	// ContactJoint
	/**
	 * Create a new joint of the contact type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @param c contact
	 * @return new joint
	 */
	public static DContactJoint createContactJoint (DWorld world, DJointGroup group, DContact c) {
		return ODE.dJointCreateContact(world, group, c);
	}
	/**
	 * Create a new joint of the contact type.
	 * @param world world
	 * @param c contact
	 * @return new joint
	 */
	public static DContactJoint createContactJoint (DWorld world, DContact c) {
		return ODE.dJointCreateContact(world, null, c);
	}

	/**
	 * Create a new joint of the double ball type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DDoubleBallJoint createDBallJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateDBall(world, group);
	}

	/**
	 * Create a new joint of the double ball type.
	 * @param world world
	 * @return new joint
	 */
	public static DDoubleBallJoint createDBallJoint (DWorld world) {
		return ODE.dJointCreateDBall(world, null);
	}

	/**
	 * Create a new joint of the double hinge type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DDoubleHingeJoint createDHingeJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateDHinge(world, group);
	}

	/**
	 * Create a new joint of the double hinge type.
	 * @param world world
	 * @return new joint
	 */
	public static DDoubleHingeJoint createDHingeJoint (DWorld world) {
		return ODE.dJointCreateDHinge(world, null);
	}

	// FixedJoint
	/**
	 * Create a new joint of the fixed type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DFixedJoint createFixedJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateFixed(world, group);
	}
	/**
	 * Create a new joint of the fixed type.
	 * @param world world
	 * @return new joint
	 */
	public static DFixedJoint createFixedJoint (DWorld world) {
		return ODE.dJointCreateFixed(world, null);
	}

	// HingeJoint
	/**
	 * Create a new joint of the hinge type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DHingeJoint createHingeJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateHinge(world, group);
	}
	/**
	 * Create a new joint of the hinge type.
	 * @param world world
	 * @return new joint
	 */
	public static DHingeJoint createHingeJoint (DWorld world) {
		return ODE.dJointCreateHinge(world, null);
	}

	// Hinge2Joint
	/**
	 * Create a new joint of the hinge2 type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DHinge2Joint createHinge2Joint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateHinge2(world, group);
	}
	/**
	 * Create a new joint of the hinge2 type.
	 * @param world world
	 * @return new joint
	 */
	public static DHinge2Joint createHinge2Joint (DWorld world) {
		return ODE.dJointCreateHinge2(world, null);
	}

	// LMotorJoint
	/**
	 * Create a new joint of the L-motor type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DLMotorJoint createLMotorJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateLMotor(world, group);
	}
	/**
	 * Create a new joint of the L-motor type.
	 * @param world world
	 * @return new joint
	 */
	public static DLMotorJoint createLMotorJoint (DWorld world) {
		return ODE.dJointCreateLMotor(world, null);
	}

	// NullJoint
	public static DNullJoint createNullJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateNull(world, group);
	}
	public static DNullJoint createNullJoint (DWorld world) {
		return ODE.dJointCreateNull(world, null);
	}

	// PistonJoint
	/**
	 * Create a new joint of the Piston type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DPistonJoint createPistonJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePiston(world, group);
	}
	/**
	 * Create a new joint of the Piston type.
	 * @param world world
	 * @return new joint
	 */
	public static DPistonJoint createPistonJoint (DWorld world) {
		return ODE.dJointCreatePiston(world, null);
	}

	// Plane2DJoint
	/**
	 * Create a new joint of the plane-2d type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DPlane2DJoint createPlane2DJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePlane2D(world, group);
	}
	/**
	 * Create a new joint of the plane-2d type.
	 * 
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @param world world
	 * @return new joint
	 */
	public static DPlane2DJoint createPlane2DJoint (DWorld world) {
		return ODE.dJointCreatePlane2D(world, null);
	}

	// PRJoint
	/**
	 * Create a new joint of the PR (Prismatic and Rotoide) type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DPRJoint createPRJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePR(world, group);
	}
	/**
	 * Create a new joint of the PR (Prismatic and Rotoide) type.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @param world world
	 * @return new joint
	 */
	public static DPRJoint createPRJoint (DWorld world) {
		return ODE.dJointCreatePR(world, null);
	}

	// PUJoint
	/**
	 * Create a new joint of the PU (Prismatic and Universal) type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DPUJoint createPUJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePU(world, group);
	}
	/**
	 * Create a new joint of the PU (Prismatic and Universal) type.
	 * 
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @param world world
	 * @return new joint
	 */
	public static DPUJoint createPUJoint (DWorld world) {
		return ODE.dJointCreatePU(world, null);
	}

	// SliderJoint
	/**
	 * Create a new joint of the slider type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DSliderJoint createSliderJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateSlider(world, group);
	}
	/**
	 * Create a new joint of the slider type.
	 * 
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @param world world
	 * @return new joint
	 */
	public static DSliderJoint createSliderJoint (DWorld world) {
		return ODE.dJointCreateSlider(world, null);
	}

	/**
	 * Create a new joint of the Transmission type.
	 * 
	 * @param w world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DTransmissionJoint createTransmissionJoint (DWorld w, DJointGroup group) {
		return ODE.dJointCreateTransmission(w, group);
	}

	/**
	 * Create a new joint of the Transmission type.
	 * @param w world
	 * @return new joint
	 */
	public static DTransmissionJoint createTransmissionJoint (DWorld w) {
		return ODE.dJointCreateTransmission(w, null);
	}

	// UniversalJoint
	/**
	 * Create a new joint of the universal type.
	 * 
	 * @param world world
	 * @param group set to null to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return new joint
	 */
	public static DUniversalJoint createUniversalJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateUniversal(world, group);
	}
	/**
	 * Create a new joint of the universal type.
	 * 
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @param world world
	 * @return new joint
	 */
	public static DUniversalJoint createUniversalJoint (DWorld world) {
		return ODE.dJointCreateUniversal(world, null);
	}




	// World
	/**
	 * Create a new, empty world and return its ID number.
	 * @return an identifier
	 */
	public static DWorld createWorld () {
		return DxWorld.dWorldCreate();
	}

	
	public static DMass createMass() {
		return new DxMass();
	}
	
	/**
	 * Create a body in given world.
	 * 
	 * <p>REMARK: 
	 * Default mass parameters are at position (0,0,0).
	 * @param w world
	 * @return new body
	 */
	//ODE_API
	public static DBody createBody (DWorld w){
		return DxBody.dBodyCreate((DxWorld) w);
	}

	//ODE_API 
	public static DSimpleSpace createSimpleSpace () {
		return DxSimpleSpace.dSimpleSpaceCreate(null);
	}
	public static DSimpleSpace createSimpleSpace (DSpace space) {
		return DxSimpleSpace.dSimpleSpaceCreate((DxSpace) space);
	}
	/**
	 * @param axes DSapSpace.AXES
	 * @return SAP Space
	 */
	//ODE_API 
	public static DSapSpace createSapSpace (DSapSpace.AXES axes) {
		return DxSAPSpace.dSweepAndPruneSpaceCreate(null, axes.getCode());
	}
	/**
	 * @param space space
     * @param axes DSapSpace.AXES
	 * @return SAP space
	 */
	//ODE_API 
	public static DSapSpace createSapSpace (DSpace space, DSapSpace.AXES axes) {
		return DxSAPSpace.dSweepAndPruneSpaceCreate((DxSpace) space, axes.getCode());
	}
	/**
	 * @param axes DSapSpace.AXES
	 * @param staticGeomCategoryMask Geoms that are marked as static are not checked
	 * for mutual collision. See SpacePerformanceTest for an example.
	 * @return SAP Space
	 */
	public static DSapSpace createSapSpace2 (DSapSpace.AXES axes, long staticGeomCategoryMask) {
		return DxSAPSpace2.dSweepAndPruneSpaceCreate(null, axes.getCode(), staticGeomCategoryMask);
	}
	/**
	 * @param space space
     * @param axes DSapSpace.AXES
	 * @param staticGeomCategoryMask Geoms that are marked as static are not checked
	 * for mutual collision. See SpacePerformanceTest for an example.
	 * @return SAP space
	 */
	public static DSapSpace createSapSpace2 (DSpace space, DSapSpace.AXES axes, long staticGeomCategoryMask) {
		return DxSAPSpace2.dSweepAndPruneSpaceCreate((DxSpace) space, axes.getCode(), staticGeomCategoryMask);
	}
	//ODE_API 
	public static DHashSpace createHashSpace () {
		return DxHashSpace.dHashSpaceCreate(null);
	}
	//ODE_API 
	public static DHashSpace createHashSpace (DSpace space) {
		return DxHashSpace.dHashSpaceCreate((DxSpace)space);
	}
	//ODE_API 
	public static DQuadTreeSpace createQuadTreeSpace (
			DVector3C Center, DVector3C Extents, int Depth) {
		return DxQuadTreeSpace.dQuadTreeSpaceCreate(null, 
				Center, Extents, Depth);
	}
	//ODE_API 
	public static DQuadTreeSpace createQuadTreeSpace (DSpace space, 
			DVector3C Center, DVector3C Extents, int Depth) {
		return DxQuadTreeSpace.dQuadTreeSpaceCreate((DxSpace) space, 
				Center, Extents, Depth);
	}
	/**
	 * @param staticGeomCategoryMask Geoms that are marked as static are not checked
	 * for mutual collision. See SpacePerformanceTest for an example.
	 * @return SAP Space
	 */
	public static DBhvSpace createBHVSpace (long staticGeomCategoryMask) {
		return DxBVHSpace.bvhSpaceCreate(null, 16, false, 0.2, staticGeomCategoryMask);
	}
	/**
	 * @param space space
	 * @param nodesPerLeaf Suggested default: 4-16
	 * @param highQuality Suggested default: false
	 * @param fatAabbMargin Suggested default: 0.2
	 * @param staticGeomCategoryMask Geoms that are marked as static are not checked
	 * for mutual collision. See SpacePerformanceTest for an example.
	 * @return SAP space
	 */
	public static DBhvSpace createBHVSpace (DSpace space, int nodesPerLeaf, boolean highQuality, 
			double fatAabbMargin, long staticGeomCategoryMask) {
		return DxBVHSpace.bvhSpaceCreate((DxSpace) space, nodesPerLeaf, highQuality, fatAabbMargin, 
				staticGeomCategoryMask);
	}

	/**
	 * Create a box geom with the provided side lengths.
	 * 
	 * <p>REMARK: The point of reference for a box is its center.
	 *
	 * @param lx      the length of the box along the X axis
	 * @param ly      the length of the box along the Y axis
	 * @param lz      the length of the box along the Z axis
	 *
	 * @return A new box geom.
	 *
	 * @see DGeom#destroy()
	 * @see DBox#setLengths(DVector3C)
	 * @see DBox#setLengths(double, double, double)
	 */
	public static DBox createBox(double lx, double ly, double lz) {
		return DxBox.dCreateBox(null, lx, ly, lz);
	}
	/**
	 * @param lxyz lengths
	 * @return new Box
	 * @see #createBox(double, double, double)
	 */
	public static DBox createBox(DVector3 lxyz) {
		return DxBox.dCreateBox(null, lxyz.get0(), lxyz.get1(), lxyz.get2());
	}
	/**
	 * Create a box geom with the provided side lengths.
	 *
	 * <p>REMARK: The point of reference for a box is its center.
	 *
	 * @param space   a space to contain the new geom. May be null.
	 * @param lx      the length of the box along the X axis
	 * @param ly      the length of the box along the Y axis
	 * @param lz      the length of the box along the Z axis
	 *
	 * @return A new box geom.
	 *
	 * @see DGeom#destroy()
	 * @see DBox#setLengths(DVector3C)
	 * @see DBox#setLengths(double, double, double)
	 */
	public static DBox createBox(DSpace space, double lx, double ly, double lz) {
		return DxBox.dCreateBox((DxSpace) space, lx, ly, lz);
	}
	/**
	 * @param space space
	 * @param lxyz lengths
	 * @return new box
	 * @see #createBox(DSpace, double, double, double)
	 */
	public static DBox createBox(DSpace space, DVector3 lxyz) {
		return DxBox.dCreateBox((DxSpace) space, lxyz.get0(), lxyz.get1(), lxyz.get2());
	}

	public static DCapsule createCapsule(double radius, double length) {
		return DxCapsule.dCreateCapsule(null, radius, length);
	}
	public static DCapsule createCapsule(DSpace space, double radius, double length) {
		return DxCapsule.dCreateCapsule((DxSpace) space, radius, length);
	}

	/**
	 * @param planes An array of planes in the form: normal X, normal Y, normal Z,Distance
	 * @param points An array of points X,Y,Z
	 * @param polygons An array of indices to the points of each polygon, it should be the
	 * 	 number of vertices followed by that amount of indices to "points" in
	 * 	 counter clockwise order
	 * @return new DConvex instance
	 */
	public static DConvex createConvex(double[] planes, double[] points, int[] polygons) {
		return DxConvex.dCreateConvex(null, planes, points, polygons);
	}
	/**
	 * @param space the space instance
	 * @param planes An array of planes in the form: normal X, normal Y, normal Z,Distance
	 * @param points An array of points X,Y,Z
	 * @param polygons An array of indices to the points of each polygon, it should be the
	 * 	 number of vertices followed by that amount of indices to "points" in
	 * 	 counter clockwise order
	 * @return new DConvex instance
	 */
	public static DConvex createConvex(DSpace space, double[] planes, double[] points, int[] polygons) {
		return DxConvex.dCreateConvex((DxSpace)space, planes, points, polygons);
	}

	/**
	 * @param planes An array of planes in the form: normal X, normal Y, normal Z,Distance
	 * @param planeCount Amount of planes in `planes`
	 * @param points An array of points X,Y,Z
	 * @param pointCount Amount of points in `points`
	 * @param polygons An array of indices to the points of each polygon, it should be the
	 * 	 number of vertices followed by that amount of indices to "points" in
	 * 	 counter clockwise order
	 * @return new DConvex instance
	 */
	public static DConvex createConvex(double[] planes,
									   int planeCount, double[] points, int pointCount, int[] polygons) {
		return DxConvex.dCreateConvex(null, planes, planeCount, points, pointCount, polygons);
	}
	/**
	 * @param space The space instance
	 * @param planes An array of planes in the form: normal X, normal Y, normal Z,Distance
	 * @param planeCount Amount of planes in `planes`
	 * @param points An array of points X,Y,Z
	 * @param pointCount Amount of points in `points`
	 * @param polygons An array of indices to the points of each polygon, it should be the
	 * 	 number of vertices followed by that amount of indices to "points" in
	 * 	 counter clockwise order
	 * @return new DConvex instance
	 */
	public static DConvex createConvex(DSpace space, double[] planes,
									   int planeCount, double[] points, int pointCount, int[] polygons) {
		return DxConvex.dCreateConvex((DxSpace)space, planes, planeCount, points, pointCount, polygons);
	}

	public static DCylinder createCylinder(double radius, double length) {
		return DxCylinder.dCreateCylinder(null, radius, length);
	}
	public static DCylinder createCylinder(DSpace space, double radius, double length) {
		return DxCylinder.dCreateCylinder((DxSpace)space, radius, length);
	}

	public static DPlane createPlane (DSpace space, double a, double b, double c, double d) {
		return DxPlane.dCreatePlane((DxSpace) space, a, b, c, d);
	}

	public static DRagdoll createRagdoll(DWorld world, DRagdollConfig skeleton) {
		return DxRagdoll.create(world, null, skeleton);
	}
	public static DRagdoll createRagdoll(DWorld world, DSpace space, DRagdollConfig skeleton) {
		return DxRagdoll.create(world, space, skeleton);
	}

	public static DRay createRay(int length) {
		return DxRay.dCreateRay(null, length);
	}
	public static DRay createRay(DSpace space, double length) {
		return DxRay.dCreateRay((DxSpace) space, length);
	}

	/**
	 * Create a sphere geom of the given radius, and return its ID.
	 *
	 * <p>REMARK: The point of reference for a sphere is its center.
	 *
	 * @param radius  the radius of the sphere.
	 *
	 * @return A new sphere geom.
	 *
	 * @see DGeom#destroy()
	 * @see DSphere#setRadius(double)
	 */
	public static DSphere createSphere(double radius) {
		return DxSphere.dCreateSphere(null, radius);
	}
	/**
	 * Create a sphere geom of the given radius, and return its ID.
	 *
	 * <p>REMARK: The point of reference for a sphere is its center.
	 *
	 * @param space   a space to contain the new geom. May be null.
	 * @param radius  the radius of the sphere.
	 *
	 * @return A new sphere geom.
	 *
	 * @see DGeom#destroy()
	 * @see DSphere#setRadius(double)
	 */
	public static DSphere createSphere(DSpace space, double radius) {
		return DxSphere.dCreateSphere((DxSpace)space, radius);
	}

	/**
	 * Initializes ODE library.
	 * <p>
	 * <tt>dInitODE</tt> is obsolete. <tt>dInitODE2</tt> is to be used for library initialization.
	 *
	 * A call to <tt>dInitODE</tt> is equal to the following initialization sequence <p>
	 * <code><br>
	 *     initODE2(0); <br>
	 *     allocateODEDataForThread(dAllocateMaskAll); <br>
	 * </code>
	 *
	 * @see #initODE2(int)
	 * @see #allocateODEDataForThread(int)
	 */
	public static void initODE() {
		OdeInit.dInitODE();
	}

	/**
	 * Initializes ODE library.
	 * <p>
	 * This function must be called to initialize ODE library before first use. If
	 * initialization succeeds the function may not be called again until library is
	 * closed with a call to <tt>dCloseODE</tt>.
	 * <p>
	 * The <tt>uiInitFlags</tt> parameter specifies initialization options to be used. These
	 * can be combination of zero or more <tt>dInitODEFlags</tt> flags.
	 *
	 * <p>NOTE:
	 * If <tt>dInitFlagManualThreadCleanup</tt> flag is used for initialization, 
	 * <tt>dSpaceSetManualCleanup</tt> must be called to set manual cleanup mode for every
	 * space object right after creation. Failure to do so may lead to resource leaks.
	 *
	 * @param uiInitFlags Initialization options bitmask
	 * @return A nonzero if initialization succeeded and zero otherwise.
	 * @see #closeODE()
	 * @see OdeInit
	 * @see DSpace#setManualCleanup(int)
	 */
	//* @see #initODEFlags
	public static int initODE2(int uiInitFlags/*=0*/) {
		return OdeInit.dInitODE2(uiInitFlags) ? 1 : 0;
	}
	
	/**
	 * Close ODE after it is not needed any more.
	 * <p>
	 * The function is required to be called when program does not need ODE features any more.
	 * The call to <tt>dCloseODE</tt> releases all the resources allocated for library
	 * including all the thread local data that might be allocated for all the threads
	 * that were using ODE.
	 * <p>
	 * <tt>dCloseODE</tt> is a paired function for <tt>dInitODE2</tt> and must only be called
	 * after successful library initialization.
	 *
	 * <p>NOTE: Important! <br>
	 * Make sure that all the threads that were using ODE have already terminated
	 * before calling <tt>dCloseODE</tt>. In particular it is not allowed to call
	 * <tt>dCleanupODEAllDataForThread</tt> after <tt>dCloseODE</tt>.
	 *
	 * @see #initODE2(int)
	 */
	//* @see #dCleanupODEAllDataForThread
	public static void closeODE() {
		OdeInit.dCloseODE();
	}
	
	/**
	 *
	 * Given two geoms o1 and o2 that potentially intersect,
	 * generate contact information for them.
	 * <p>
	 * Internally, this just calls the correct class-specific collision
	 * functions for o1 and o2.
	 *
	 * <p>REMARK: If a space is passed as o1 or o2 then this function will collide
	 * all objects contained in o1 with all objects contained in o2, and return
	 * the resulting contact points. This method for colliding spaces with geoms
	 * (or spaces with spaces) provides no user control over the individual
	 * collisions. To get that control, use dSpaceCollide or dSpaceCollide2 instead.
	 *
	 * <p>REMARK: If o1 and o2 are the same geom then this function will do nothing
	 * and return 0. Technically speaking an object intersects with itself, but it
	 * is not useful to find contact points in this case.
	 *
	 * <p>REMARK: This function does not care if o1 and o2 are in the same space or not
	 * (or indeed if they are in any space at all).
	 *
	 * @param o1 The first geom to test.
	 * @param o2 The second geom to test.
	 *
	 * @param flags The flags specify how contacts should be generated if
	 * the geoms touch. The lower 16 bits of flags is an integer that
	 * specifies the maximum number of contact points to generate. You must
	 * ask for at least one contact.
	 * Additionally, following bits may be set:
	 * CONTACTS_UNIMPORTANT -- just generate any contacts (skip contact refining).
	 * All other bits in flags must be set to zero. In the future the other bits
	 * may be used to select from different contact generation strategies.
	 *
	 * @param contacts Points to an array of dContactGeom structures. The array
	 * must be able to hold at least the maximum number of contacts. These
	 * dContactGeom structures may be embedded within larger structures in the
	 * array -- the skip parameter is the byte offset from one dContactGeom to
	 * the next in the array. If skip is sizeof(dContactGeom) then contact
	 * points to a normal (C-style) array. It is an error for skip to be smaller
	 * than sizeof(dContactGeom).
	 *
	 * @return If the geoms intersect, this function returns the number of contact
	 * points generated (and updates the contact array), otherwise it returns 0
	 * (and the contact array is not touched).
	 */
	public static int collide (DGeom o1, DGeom o2, int flags, 
			DContactGeomBuffer contacts) {
		return DxGeom.dCollide((DxGeom)o1, (DxGeom)o2, flags, contacts, 1);
	}
	

	/**
	 * Utility function.
	 * @param b1 b1
	 * @param b2 b2
	 * @return true if the two bodies are connected together by
	 * a joint, otherwise return 0.
	 */
	public static boolean areConnected (DBody b1, DBody b2) {
		return ODE._dAreConnected(b1, b2);
	}

	/**
	 * Utility function.
	 * @param body1 A body to check.
	 * @param body2 A body to check.
	 * @param jointTypes is a set of subclasses of DJoint.
	 * This is useful for deciding whether to add contact joints between two bodies:
	 * if they are already connected by non-contact joints then it may not be
	 * appropriate to add contacts, however it is okay to add more contact between-
	 * bodies that already have contacts.
	 * @return true if the two bodies are connected together by
	 * a joint that does not have type <code>jointType</code>, otherwise return 0.
	 */
 	@SafeVarargs
	public static boolean areConnectedExcluding (DBody body1, DBody body2,
												 Class<? extends DJoint> ... jointTypes) {
        return ODE._dAreConnectedExcluding(body1, body2, jointTypes);
    }
	/**
	 * Utility function.
	 * @param body1 A body to check.
	 * @param body2 A body to check.
	 * @param jointType is a set of subclasses of DJoint.
	 * This is useful for deciding whether to add contact joints between two bodies:
	 * if they are already connected by non-contact joints then it may not be
	 * appropriate to add contacts, however it is okay to add more contact between-
	 * bodies that already have contacts.
	 * @return true if the two bodies are connected together by
	 * a joint that does not have type <code>jointType</code>, otherwise return 0.
	 */
 	@SuppressWarnings("unchecked")
	public static boolean areConnectedExcluding (DBody body1, DBody body2, 
            Class<? extends DJoint> jointType) {
        return ODE._dAreConnectedExcluding(body1, body2, jointType);
    }
	
	
	
	/**
	 * Helper to check for a token in the ODE configuration string.
	 * Caution, this function is case sensitive.
	 *
	 * @param extension A configuration token, see dGetConfiguration for details
	 *
	 * @return <tt>true</tt> if exact token is present, <tt>false</tt> if not present
	 */
	public static boolean checkConfiguration( String extension ) {
		return ODE._dCheckConfiguration(extension);
	}

	
	/**
	 * getConfiguration returns the specific ODE build configuration as
	 * a string of tokens. The string can be parsed in a similar way to
	 * the OpenGL extension mechanism, the naming convention should be
	 * familiar too. The following extensions are reported:
	 *
	 * ODE
	 * ODE_single_precision
	 * ODE_double_precision
	 * ODE_EXT_no_debug
	 * ODE_EXT_trimesh
	 * ODE_EXT_opcode
	 * ODE_EXT_gimpact
	 * ODE_EXT_malloc_not_alloca
	 * ODE_EXT_gyroscopic
	 * ODE_OPC_16bit_indices
	 * ODE_OPC_new_collider
	 * @return String
	 */
	public static String getConfiguration () {
		return ODE._dGetConfiguration();
	}

	/**
	 * 
	 * @return The version String.
	 */
	public static String getVersion() {
		return "0.5.2";
	}
	
	
	/**
	 * 
	 * @param world world
	 * @param f f
	 * @param string string
	 * @deprecated TZ: Currently not implemented.
	 */
	@Deprecated
    public static void worldExportDIF(DWorld world, File f, String string) {
		throw new UnsupportedOperationException(); //TODO
	}

	/**
	 * Allocate thread local data to allow the thread calling ODE.
	 *
	 * The function is required to be called for every thread that is going to use
	 * ODE. This function allocates the data that is required for accessing ODE from
	 * current thread along with optional data required for particular ODE subsystems.
	 *
	 * <tt>uiAllocateFlags</tt> parameter can contain zero or more flags from 
	 * <tt>dAllocateODEDataFlags</tt>
	 * enumerated type. Multiple calls with different allocation flags are allowed.
	 * The flags that are already allocated are ignored in subsequent calls. If zero
	 * is passed as the parameter, it means to only allocate the set of most important
	 * data the library can not operate without.
	 *
	 * If the function returns failure status it means that none of the requested
	 * data has been allocated. The client may retry allocation attempt with the same
	 * flags when more system resources are available.
	 *
     * @param uiAllocateFlags Allocation options bitmask.
     * @return A nonzero if allocation succeeded and zero otherwise.
	 * @deprecated TZ I guess this can be removed?
	 */
//	 * @see #dAllocateODEDataFlags
//	 * @see #dCleanupODEAllDataForThread
	@Deprecated
    public static int allocateODEDataForThread(int uiAllocateFlags) {
		return OdeInit.dAllocateODEDataForThread(uiAllocateFlags) ? 1 : 0;
	}

	/**
	 * Creates a heightfield geom.
	 * <p>
	 * Uses the information in the given dHeightfieldData to construct
	 * a geom representing a heightfield in a collision space.
	 *
	 * @param space The space to add the geom to.
	 * @param data The dHeightfieldData created by dGeomHeightfieldDataCreate and
	 * setup by dGeomHeightfieldDataBuildCallback, dGeomHeightfieldDataBuildByte,
	 * dGeomHeightfieldDataBuildShort or dGeomHeightfieldDataBuildFloat.
	 * @param bPlaceable If non-zero this geom can be transformed in the world using the
	 * usual functions such as dGeomSetPosition and dGeomSetRotation. If the geom is
	 * not set as placeable, then it uses a fixed orientation where the global y axis
	 * represents the dynamic 'height' of the heightfield.
	 *
	 * @return A geom id to reference this geom in other calls.
	 */
	public static DHeightfield createHeightfield( DSpace space,
			DHeightfieldData data, boolean bPlaceable ) {
		return DxHeightfield.dCreateHeightfield((DxSpace)space, (DxHeightfieldData)data, bPlaceable);
	}

	
	/**
	 * Creates a new empty dHeightfieldData.
	 * <p>
	 * Allocates a new dHeightfieldData and returns it. You must call
	 * dGeomHeightfieldDataDestroy to destroy it after the geom has been removed.
	 * The dHeightfieldData value is used when specifying a data format type.
	 *
	 * @return A dHeightfieldData for use with dGeomHeightfieldDataBuildCallback,
	 * dGeomHeightfieldDataBuildByte, dGeomHeightfieldDataBuildShort or
	 * dGeomHeightfieldDataBuildFloat.
	 */
	public static DHeightfieldData createHeightfieldData() {
		return DxHeightfieldData.dGeomHeightfieldDataCreate();
	}


	/**
	 * This is custom heightfield based on a trimesh. It is specific to ode4j.
	 *
	 * Creates a trimesh heightfield geom.
	 * <p>
	 * Uses the information in the given dHeightfieldData to construct
	 * a geom representing a heightfield in a collision space.
	 *
	 * @param space The space to add the geom to.
	 * @param data The dHeightfieldData created by dGeomHeightfieldDataCreate and
	 * setup by dGeomHeightfieldDataBuildCallback, dGeomHeightfieldDataBuildByte,
	 * dGeomHeightfieldDataBuildShort or dGeomHeightfieldDataBuildFloat.
	 * @param bPlaceable If non-zero this geom can be transformed in the world using the
	 * usual functions such as dGeomSetPosition and dGeomSetRotation. If the geom is
	 * not set as placeable, then it uses a fixed orientation where the global y axis
	 * represents the dynamic 'height' of the heightfield.
	 *
	 * @return A geom id to reference this geom in other calls.
	 */
	public static DHeightfield createTrimeshHeightfield( DSpace space,
												  DHeightfieldData data, boolean bPlaceable ) {
		return DxTrimeshHeightfield.dCreateHeightfield((DxSpace)space, (DxHeightfieldData)data, bPlaceable);
	}


//	/**
//	 * Creates a new empty dHeightfieldData.
//	 * <p>
//	 * Allocates a new dHeightfieldData and returns it. You must call
//	 * dGeomHeightfieldDataDestroy to destroy it after the geom has been removed.
//	 * The dHeightfieldData value is used when specifying a data format type.
//	 *
//	 * @return A dHeightfieldData for use with dGeomHeightfieldDataBuildCallback,
//	 * dGeomHeightfieldDataBuildByte, dGeomHeightfieldDataBuildShort or
//	 * dGeomHeightfieldDataBuildFloat.
//	 */
//	public static DHeightfieldData createTrimeshHeightfieldData() {
//		return DxTrimeshHeightfield.dGeomHeightfieldDataCreate();
//	}

	/**
	 * Trimesh class
	 * Construction. Callbacks are optional.
	 * @param space space
	 * @param data user data
	 * @return trimesh
	 */
	public static DTriMesh createTriMesh(DSpace space, DTriMeshData data) {
		return DxTriMesh.dCreateTriMesh((DxSpace)space, (DxTriMeshData)data,null, null, null);
	}

	/**
	 * Trimesh class
	 * Construction. Callbacks are optional.
	 * @param space space
	 * @param data user data
	 * @param callback callback (can be NULL)
	 *                 NOTE: The callback is only called for Box, Capsule, Ray, Sphere and TriMesh. See issue #76.
	 * // @param arrayCallback array callback (can be NULL) ---- Not supported in GIMPACT.
	 * @param rayCallback ray callback (can be NULL)
	 * @return trimesh
	 */
	public static DTriMesh createTriMesh(DSpace space, DTriMeshData data, DTriCallback callback,
										 DTriRayCallback rayCallback) {
		return DxTriMesh.dCreateTriMesh((DxSpace)space, (DxTriMeshData)data, callback, null, rayCallback);
	}

	/**
	 * Trimesh class
	 * Construction. Callbacks are optional.
	 * @param space space
	 * @param data user data
	 * @param callback callback (can be NULL)
	 *                 NOTE: The callback is only called for Box, Capsule, Ray, Sphere and TriMesh. See issue #76.
	 * @param arrayCallback array callback (can be NULL) ---- Not supported in GIMPACT.
	 * @param rayCallback ray callback (can be NULL)
	 * @return trimesh
	 */
	@SuppressWarnings("deprecation") // TODO deprecate this method?!?!? 0.6.0 ?
	public static DTriMesh createTriMesh(DSpace space, DTriMeshData data, DTriCallback callback,
										 org.ode4j.ode.DTriMesh.DTriArrayCallback arrayCallback,
										 DTriRayCallback rayCallback) {
		return DxTriMesh.dCreateTriMesh((DxSpace)space, (DxTriMeshData)data,
				callback, arrayCallback, rayCallback);
	}

	/**
	 * These don't make much sense now, but they will later when we add more
	 * features.
	 * @return new trimesh data object
	 */
	public static DTriMeshData createTriMeshData() {
		return DxTriMeshData.dGeomTriMeshDataCreate();
	}


	/**
	 * Determines which pairs of geoms in a space may potentially intersect,
	 * and calls the callback function for each candidate pair.
	 *
	 * This is equivalent to DSpace.collide(...).
	 *
	 * <p>REMARK: Other spaces that are contained within the colliding space are
	 * not treated specially, i.e. they are not recursed into. The callback
	 * function may be passed these contained spaces as one or both geom
	 * arguments.
	 *
	 * <p>REMARK: dSpaceCollide() is guaranteed to pass all intersecting geom
	 * pairs to the callback function, but may also pass close but
	 * non-intersecting pairs. The number of these calls depends on the
	 * internal algorithms used by the space. Thus you should not expect
	 * that dCollide will return contacts for every pair passed to the
	 * callback.
	 *
	 * @param space The space to test.
	 *
	 * @param data Passed from dSpaceCollide directly to the callback
	 * function. Its meaning is user defined. The o1 and o2 arguments are the
	 * geoms that may be near each other.
	 *
	 * @param callback A callback function is of type {@link DNearCallback}.
	 *
	 * @see #spaceCollide2(DGeom, DGeom, Object, DGeom.DNearCallback)
	 * @see DSpace#collide(Object, DGeom.DNearCallback)
	 */
	public static void spaceCollide (DSpace space, Object data, DNearCallback callback) {
		((DxSpace)space).dSpaceCollide(data, callback);
	}

	/**
	 * Determines which geoms from one space may potentially intersect with 
	 * geoms from another space, and calls the callback function for each candidate 
	 * pair. 
	 *
	 * <p>REMARK: This function can also test a single non-space geom against a 
	 * space. This function is useful when there is a collision hierarchy, i.e. 
	 * when there are spaces that contain other spaces.
	 *
	 * <p>REMARK: Other spaces that are contained within the colliding space are
	 * not treated specially, i.e. they are not recursed into. The callback
	 * function may be passed these contained spaces as one or both geom
	 * arguments.
	 *
	 * <p>REMARK: Sublevel value of space affects how the spaces are iterated.
	 * Both spaces are recursed only if their sublevels match. Otherwise, only
	 * the space with greater sublevel is recursed and the one with lesser sublevel
	 * is used as a geom itself.
	 *
	 * <p>REMARK: dSpaceCollide2() is guaranteed to pass all intersecting geom
	 * pairs to the callback function, but may also pass close but
	 * non-intersecting pairs. The number of these calls depends on the
	 * internal algorithms used by the space. Thus you should not expect
	 * that dCollide will return contacts for every pair passed to the
	 * callback.
	 *
	 * @param space1 The first space to test.
	 *
	 * @param space2 The second space to test.
	 *
	 * @param data Passed from dSpaceCollide directly to the callback
	 * function. Its meaning is user defined. The o1 and o2 arguments are the
	 * geoms that may be near each other.
	 *
	 * @param callback A callback function is of type {@link DNearCallback}.
	 *
	 * @see #spaceCollide(DSpace, Object, DGeom.DNearCallback)
	 * @see DSpace#setSublevel(int)
	 */
	public static void spaceCollide2(DGeom space1, DGeom space2, Object data,
			DNearCallback callback) {
		DxSpace.dSpaceCollide2((DxGeom)space1, (DxGeom)space2, data, callback);
	}
	
	
	/**
	 * Sets a custom collider function for two geom classes.
	 *
	 * @param i The first geom class handled by this collider
	 * @param j The second geom class handled by this collider
	 * @param fn The collider function to use to determine collisions.
	 */
	public static void setColliderOverride (int i, int j, DColliderFn fn) {
		DxGeom.dSetColliderOverride(i, j, fn);
	}

	
	/**
	 * @param b1 b1
	 * @param b2 b2
	 * @return new joint
	 */
	public static DJoint connectingJoint(DBody b1, DBody b2) {
		return OdeJointsFactoryImpl.dConnectingJoint(b1, b2);
	}
	
	
	/**
	 * @param b1 b1
	 * @param b2 b2
	 * @return connectiong joints
	 */
	public static List<DJoint> connectingJointList(DBody b1, DBody b2) {
		return OdeJointsFactoryImpl.dConnectingJointList((DxBody)b1, (DxBody)b2);
	}

	@Deprecated // Make this "private" in 0.6.0
	protected OdeHelper() {}
}
