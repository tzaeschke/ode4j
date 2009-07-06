/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode;

import java.io.File;

import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DTriMesh.dTriArrayCallback;
import org.ode4j.ode.DTriMesh.dTriCallback;
import org.ode4j.ode.DTriMesh.dTriRayCallback;
import org.ode4j.ode.internal.joints.DxJointGroup;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.internal.DxHeightfield;
import org.ode4j.ode.internal.DxHeightfieldData;
import org.ode4j.ode.internal.DxSAPSpace;
import org.ode4j.ode.internal.DxTriMesh;
import org.ode4j.ode.internal.DxTriMeshData;
import org.ode4j.ode.internal.OdeFactoryImpl;
import org.ode4j.ode.internal.OdeInit;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxBox;
import org.ode4j.ode.internal.DxCapsule;
import org.ode4j.ode.internal.DxConvex;
import org.ode4j.ode.internal.DxCylinder;
import org.ode4j.ode.internal.DxGeom;
import org.ode4j.ode.internal.DxGeomTransform;
import org.ode4j.ode.internal.DxHashSpace;
import org.ode4j.ode.internal.DxMass;
import org.ode4j.ode.internal.DxPlane;
import org.ode4j.ode.internal.DxQuadTreeSpace;
import org.ode4j.ode.internal.DxRay;
import org.ode4j.ode.internal.DxSimpleSpace;
import org.ode4j.ode.internal.DxSpace;
import org.ode4j.ode.internal.DxSphere;
import org.ode4j.ode.internal.DxWorld;

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

	// JointGroup
	public static DJointGroup createJointGroup () {
		return DxJointGroup.dJointGroupCreate(-1);
	}
	/**
	 * @brief Create a new joint feedback.
	 * @ingroup joints
	 */
	public static DJoint.DJointFeedback createJointFeedback() {
		return new DJoint.DJointFeedback();
	}

	// AMotorJoint
	public static DAMotorJoint createAMotorJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateAMotor((DxWorld) world, group);
	}
	public static DAMotorJoint createAMotorJoint (DWorld world) {
		return ODE.dJointCreateAMotor((DxWorld) world, null);
	}

	// BallJoint
	public static DBallJoint createBallJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateBall((DxWorld) world, group);
	}
	public static DBallJoint createBallJoint (DWorld world) {
		return ODE.dJointCreateBall((DxWorld) world, null);
	}

	// ContactJoint
	public static DContactJoint createContactJoint (DWorld world, DJointGroup group, DContact c) {
		return ODE.dJointCreateContact((DxWorld) world, group, c);
	}
	public static DContactJoint createContactJoint (DWorld world, DContact c) {
		return ODE.dJointCreateContact((DxWorld) world, null, c);
	}

	// FixedJoint
	public static DFixedJoint createFixedJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateFixed((DxWorld) world, group);
	}
	public static DFixedJoint createFixedJoint (DWorld world) {
		return ODE.dJointCreateFixed((DxWorld) world, null);
	}

	// HingeJoint
	public static DHingeJoint createHingeJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateHinge((DxWorld) world, group);
	}
	public static DHingeJoint createHingeJoint (DWorld world) {
		return ODE.dJointCreateHinge((DxWorld) world, null);
	}

	// Hinge2Joint
	public static DHinge2Joint createHinge2Joint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateHinge2((DxWorld) world, group);
	}
	public static DHinge2Joint createHinge2Joint (DWorld world) {
		return ODE.dJointCreateHinge2((DxWorld) world, null);
	}

	// LMotorJoint
	public static DLMotorJoint createLMotorJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateLMotor((DxWorld) world, group);
	}
	public static DLMotorJoint createLMotorJoint (DWorld world) {
		return ODE.dJointCreateLMotor((DxWorld) world, null);
	}

	// NullJoint
	public static DNullJoint createNullJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateNull((DxWorld) world, group);
	}
	public static DNullJoint createNullJoint (DWorld world) {
		return ODE.dJointCreateNull((DxWorld) world, null);
	}

	// PistonJoint
	public static DPistonJoint createPistonJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePiston((DxWorld) world, group);
	}
	public static DPistonJoint createPistonJoint (DWorld world) {
		return ODE.dJointCreatePiston((DxWorld) world, null);
	}

	// Plane2DJoint
	public static DPlane2DJoint createPlane2DJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePlane2D((DxWorld) world, group);
	}
	public static DPlane2DJoint createPlane2DJoint (DWorld world) {
		return ODE.dJointCreatePlane2D((DxWorld) world, null);
	}

	// PRJoint
	public static DPRJoint createPRJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePR((DxWorld) world, group);
	}
	public static DPRJoint createPRJoint (DWorld world) {
		return ODE.dJointCreatePR((DxWorld) world, null);
	}

	// PUJoint
	public static DPUJoint createPUJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePU((DxWorld) world, group);
	}
	public static DPUJoint createPUJoint (DWorld world) {
		return ODE.dJointCreatePU((DxWorld) world, null);
	}

	// SliderJoint
	public static DSliderJoint createSliderJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateSlider((DxWorld) world, group);
	}
	public static DSliderJoint createSliderJoint (DWorld world) {
		return ODE.dJointCreateSlider((DxWorld) world, null);
	}

	// UniversalJoint
	public static DUniversalJoint createUniversalJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateUniversal((DxWorld) world, group);
	}
	public static DUniversalJoint createUniversalJoint (DWorld world) {
		return ODE.dJointCreateUniversal((DxWorld) world, null);
	}




	// World
	public static DWorld createWorld () {
		return DxWorld.dWorldCreate();
	}

	
	public static DMass createMass() {
		return new DxMass();
	}
	
	/**
	 * @brief Create a body in given world.
	 * @remarks
	 * Default mass parameters are at position (0,0,0).
	 * @ingroup bodies
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
	//ODE_API 
	public static DSapSpace createSapSpace (DSapSpace.AXES axes) {
		return DxSAPSpace.dSweepAndPruneSpaceCreate(null, axes.getCode());
	}
	//ODE_API 
	public static DSapSpace createSapSpace (DSpace space, DSapSpace.AXES axes) {
		return DxSAPSpace.dSweepAndPruneSpaceCreate((DxSpace) space, axes.getCode());
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

	public static DBox createBox(double lx, double ly, double lz) {
		return DxBox.dCreateBox(null, lx, ly, lz);
	}
	public static DBox createBox(DSpace space, double lx, double ly, double lz) {
		return DxBox.dCreateBox((DxSpace) space, lx, ly, lz);
	}

	public static DCapsule createCapsule(double radius, double length) {
		return DxCapsule.dCreateCapsule(null, radius, length);
	}
	public static DCapsule createCapsule(DSpace space, double radius, double length) {
		return DxCapsule.dCreateCapsule((DxSpace) space, radius, length);
	}

	public static DConvex createConvex(double[] planes,
			int planecount, double[] points, int pointcount, int[] polygons) {
		return DxConvex.dCreateConvex(null, planes, planecount, points, pointcount, polygons);
	}
	public static DConvex createConvex(DSpace space, double[] planes,
			int planecount, double[] points, int pointcount, int[] polygons) {
		return DxConvex.dCreateConvex((DxSpace)space, planes, planecount, points, pointcount, polygons);
	}

	public static DCylinder createCylinder(double radius, double length) {
		return DxCylinder.dCreateCylinder(null, radius, length);
	}
	public static DCylinder createCylinder(DSpace space, double radius, double length) {
		return DxCylinder.dCreateCylinder((DxSpace)space, radius, length);
	}

	/** @deprecated TZ: Please do not use DGeomTransform. */
	public static DGeomTransform createGeomTransform() {
		return DxGeomTransform.dCreateGeomTransform(null);
	}
	/** @deprecated TZ: Please do not use DGeomTransform. */
	public static DGeomTransform createGeomTransform(DSpace space) {
		return DxGeomTransform.dCreateGeomTransform((DxSpace) space);
	}

	public static DPlane createPlane (DSpace space, double a, double b, double c, double d) {
		return DxPlane.dCreatePlane((DxSpace) space, a, b, c, d);
	}

	public static DRay createRay(int length) {
		return DxRay.dCreateRay(null, length);
	}
	public static DRay createRay(DSpace space, double length) {
		return DxRay.dCreateRay((DxSpace) space, length);
	}
	
	public static DSphere createSphere(double radius) {
		return DxSphere.dCreateSphere(null, radius);
	}
	public static DSphere createSphere(DSpace space, double radius) {
		return DxSphere.dCreateSphere((DxSpace)space, radius);
	}

	/**
	 * @brief Initializes ODE library.
	 *
	 * @c dInitODE is obsolete. @c dInitODE2 is to be used for library initialization.
	 *
	 * A call to @c dInitODE is equal to the following initialization sequence
	 * @code
	 *     initODE2(0);
	 *     allocateODEDataForThread(dAllocateMaskAll);
	 * @endcode
	 *
	 * @see #initODE2(int)
	 * @see #allocateODEDataForThread(int)
	 * @ingroup init
	 */
	public static void initODE() {
		OdeInit.dInitODE();
	}

	/**
	 * @brief Initializes ODE library.
	 * @param uiInitFlags Initialization options bitmask
	 * @return A nonzero if initialization succeeded and zero otherwise.
	 *
	 * This function must be called to initialize ODE library before first use. If
	 * initialization succeeds the function may not be called again until library is
	 * closed with a call to @c dCloseODE.
	 *
	 * The @a uiInitFlags parameter specifies initialization options to be used. These
	 * can be combination of zero or more @c dInitODEFlags flags.
	 *
	 * @note
	 * If @c dInitFlagManualThreadCleanup flag is used for initialization, 
	 * @c dSpaceSetManualCleanup must be called to set manual cleanup mode for every
	 * space object right after creation. Failure to do so may lead to resource leaks.
	 *
	 * @see #initODEFlags
	 * @see #closeODE()
	 * @see #dSpaceSetManualCleanup
	 * @ingroup init
	 */
	public static int initODE2(int uiInitFlags/*=0*/) {
		return OdeInit.dInitODE2(uiInitFlags);
	}
	
	/**
	 * @brief Close ODE after it is not needed any more.
	 *
	 * The function is required to be called when program does not need ODE features any more.
	 * The call to @c dCloseODE releases all the resources allocated for library
	 * including all the thread local data that might be allocated for all the threads
	 * that were using ODE.
	 *
	 * @c dCloseODE is a paired function for @c dInitODE2 and must only be called
	 * after successful library initialization.
	 *
	 * @note Important!
	 * Make sure that all the threads that were using ODE have already terminated
	 * before calling @c dCloseODE. In particular it is not allowed to call
	 * @c dCleanupODEAllDataForThread after @c dCloseODE.
	 *
	 * @see #initODE2(int)
	 * @see #dCleanupODEAllDataForThread
	 * @ingroup init
	 */
	public static void closeODE() {
		OdeInit.dCloseODE();
	}
	
	/**
	 *
	 * @brief Given two geoms o1 and o2 that potentially intersect,
	 * generate contact information for them.
	 *
	 * Internally, this just calls the correct class-specific collision
	 * functions for o1 and o2.
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
	 * @returns If the geoms intersect, this function returns the number of contact
	 * points generated (and updates the contact array), otherwise it returns 0
	 * (and the contact array is not touched).
	 *
	 * @remarks If a space is passed as o1 or o2 then this function will collide
	 * all objects contained in o1 with all objects contained in o2, and return
	 * the resulting contact points. This method for colliding spaces with geoms
	 * (or spaces with spaces) provides no user control over the individual
	 * collisions. To get that control, use dSpaceCollide or dSpaceCollide2 instead.
	 *
	 * @remarks If o1 and o2 are the same geom then this function will do nothing
	 * and return 0. Technically speaking an object intersects with itself, but it
	 * is not useful to find contact points in this case.
	 *
	 * @remarks This function does not care if o1 and o2 are in the same space or not
	 * (or indeed if they are in any space at all).
	 *
	 * @ingroup collide
	 */
	public static int collide (DGeom o1, DGeom o2, int flags, 
			DContactGeomBuffer contacts) {
		return DxGeom.dCollide((DxGeom)o1, (DxGeom)o2, flags, contacts, 1);
	}
	

	/**
	 * @brief Utility function
	 * @return true if the two bodies are connected together by
	 * a joint, otherwise return 0.
	 * @ingroup joints
	 */
	public static boolean areConnected (DBody b1, DBody b2) {
		return ODE._dAreConnected(b1, b2);
	}

	/**
	 * @brief Utility function
	 * @return true if the two bodies are connected together by
	 * a joint that does not have type @arg{joint_type}, otherwise return 0.
	 * @param body1 A body to check.
	 * @param body2 A body to check.
	 * @param jointType is a set of subclasses of DJoint.
	 * This is useful for deciding whether to add contact joints between two bodies:
	 * if they are already connected by non-contact joints then it may not be
	 * appropriate to add contacts, however it is okay to add more contact between-
	 * bodies that already have contacts.
	 * @ingroup joints
	 */
	public static boolean areConnectedExcluding (DBody body1, DBody body2, 
			Class<? extends DJoint> ... jointType) {
		return ODE._dAreConnectedExcluding(body1, body2, jointType);
	}
//	public static boolean dAreConnectedExcluding (DBody body1, DBody body2, dJointType joint_type) {
//		return ODE._dAreConnectedExcluding(body1, body2, joint_type);
//	}
	
	
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
	 * dGetConfiguration returns the specific ODE build configuration as
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
	 */
	public static String getConfiguration () {
		return ODE._dGetConfiguration();
	}

	/**
	 * 
	 * @return The version String.
	 */
	public static String getVersion() {
		return "0.11.1-pre";
	}
	
	
	/**
	 * 
	 * @param world
	 * @param f
	 * @param string
	 * @deprecated TZ: Currently not implemented.
	 */
	public static void worldExportDIF(DWorld world, File f, String string) {
		throw new UnsupportedOperationException(); //TODO
	}

	/**
	 * @brief Allocate thread local data to allow the thread calling ODE.
	 * @param uiAllocateFlags Allocation options bitmask.
	 * @return A nonzero if allocation succeeded and zero otherwise.
	 *
	 * The function is required to be called for every thread that is going to use
	 * ODE. This function allocates the data that is required for accessing ODE from
	 * current thread along with optional data required for particular ODE subsystems.
	 *
	 * @a uiAllocateFlags parameter can contain zero or more flags from @c dAllocateODEDataFlags
	 * enumerated type. Multiple calls with different allocation flags are allowed.
	 * The flags that are already allocated are ignored in subsequent calls. If zero
	 * is passed as the parameter, it means to only allocate the set of most important
	 * data the library can not operate without.
	 *
	 * If the function returns failure status it means that none of the requested
	 * data has been allocated. The client may retry allocation attempt with the same
	 * flags when more system resources are available.
	 *
	 * @see #dAllocateODEDataFlags
	 * @see #dCleanupODEAllDataForThread
	 * @ingroup init
	 * @deprecated TZ I guess this can be removed?
	 */
	public static int allocateODEDataForThread(int uiAllocateFlags) {
		return OdeInit.dAllocateODEDataForThread(uiAllocateFlags);
	}

	/**
	 * @brief Creates a heightfield geom.
	 *
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
	 *
	 * @ingroup collide
	 */
	public static DHeightfield createHeightfield( DSpace space,
			DHeightfieldData data, boolean bPlaceable ) {
		return DxHeightfield.dCreateHeightfield((DxSpace)space, (DxHeightfieldData)data, bPlaceable);
	}

	
	/**
	 * @brief Creates a new empty dHeightfieldData.
	 *
	 * Allocates a new dHeightfieldData and returns it. You must call
	 * dGeomHeightfieldDataDestroy to destroy it after the geom has been removed.
	 * The dHeightfieldData value is used when specifying a data format type.
	 *
	 * @return A dHeightfieldData for use with dGeomHeightfieldDataBuildCallback,
	 * dGeomHeightfieldDataBuildByte, dGeomHeightfieldDataBuildShort or
	 * dGeomHeightfieldDataBuildFloat.
	 * @ingroup collide
	 */
	public static DHeightfieldData createHeightfieldData() {
		return DxHeightfieldData.dGeomHeightfieldDataCreate();
	}


	/**
	 * Trimesh class
	 * Construction. Callbacks are optional.
	 */
	public static DTriMesh createTriMesh(DSpace space, DTriMeshData Data, dTriCallback Callback, 
			 dTriArrayCallback ArrayCallback, dTriRayCallback RayCallback) {
		return DxTriMesh.dCreateTriMesh((DxSpace)space, (DxTriMeshData)Data, 
				Callback, ArrayCallback, RayCallback);
	}

	/**
	 * These don't make much sense now, but they will later when we add more
	 * features.
	 */
	public static DTriMeshData createTriMeshData() {
		return DxTriMeshData.dGeomTriMeshDataCreate();
	}


	/**
	 * @brief Determines which pairs of geoms in a space may potentially intersect,
	 * and calls the callback function for each candidate pair.
	 *
	 * @param space The space to test.
	 *
	 * @param data Passed from dSpaceCollide directly to the callback
	 * function. Its meaning is user defined. The o1 and o2 arguments are the
	 * geoms that may be near each other.
	 *
	 * @param callback A callback function is of type @ref dNearCallback.
	 *
	 * @remarks Other spaces that are contained within the colliding space are
	 * not treated specially, i.e. they are not recursed into. The callback
	 * function may be passed these contained spaces as one or both geom
	 * arguments.
	 *
	 * @remarks dSpaceCollide() is guaranteed to pass all intersecting geom
	 * pairs to the callback function, but may also pass close but
	 * non-intersecting pairs. The number of these calls depends on the
	 * internal algorithms used by the space. Thus you should not expect
	 * that dCollide will return contacts for every pair passed to the
	 * callback.
	 *
	 * @see #spaceCollide2(DGeom, DGeom, Object, DNearCallback)
	 * @ingroup collide
	 */
	public static void spaceCollide (DSpace space, Object data, DNearCallback callback) {
		((DxSpace)space).dSpaceCollide(data, callback);
	}

	/**
	 * @brief Determines which geoms from one space may potentially intersect with 
	 * geoms from another space, and calls the callback function for each candidate 
	 * pair. 
	 *
	 * @param space1 The first space to test.
	 *
	 * @param space2 The second space to test.
	 *
	 * @param data Passed from dSpaceCollide directly to the callback
	 * function. Its meaning is user defined. The o1 and o2 arguments are the
	 * geoms that may be near each other.
	 *
	 * @param callback A callback function is of type @ref dNearCallback.
	 *
	 * @remarks This function can also test a single non-space geom against a 
	 * space. This function is useful when there is a collision hierarchy, i.e. 
	 * when there are spaces that contain other spaces.
	 *
	 * @remarks Other spaces that are contained within the colliding space are
	 * not treated specially, i.e. they are not recursed into. The callback
	 * function may be passed these contained spaces as one or both geom
	 * arguments.
	 *
	 * @remarks Sublevel value of space affects how the spaces are iterated.
	 * Both spaces are recursed only if their sublevels match. Otherwise, only
	 * the space with greater sublevel is recursed and the one with lesser sublevel
	 * is used as a geom itself.
	 *
	 * @remarks dSpaceCollide2() is guaranteed to pass all intersecting geom
	 * pairs to the callback function, but may also pass close but
	 * non-intersecting pairs. The number of these calls depends on the
	 * internal algorithms used by the space. Thus you should not expect
	 * that dCollide will return contacts for every pair passed to the
	 * callback.
	 *
	 * @see #spaceCollide(DSpace, Object, DNearCallback)
	 * @see #spaceSetSublevel
	 * @ingroup collide
	 */
	public static void spaceCollide2(DGeom space1, DGeom space2, Object data,
			DNearCallback callback) {
		DxSpace.dSpaceCollide2((DxGeom)space1, (DxGeom)space2, data, callback);
	}
}
