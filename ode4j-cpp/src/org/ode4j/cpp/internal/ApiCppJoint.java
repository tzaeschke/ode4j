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
package org.ode4j.cpp.internal;

import org.cpp4j.java.RefDouble;
import org.ode4j.ode.DBallJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointAMotor;
import org.ode4j.ode.internal.joints.DxJointBall;
import org.ode4j.ode.internal.joints.DxJointFixed;
import org.ode4j.ode.internal.joints.DxJointGroup;
import org.ode4j.ode.internal.joints.DxJointHinge;
import org.ode4j.ode.internal.joints.DxJointHinge2;
import org.ode4j.ode.internal.joints.DxJointLMotor;
import org.ode4j.ode.internal.joints.DxJointPR;
import org.ode4j.ode.internal.joints.DxJointPU;
import org.ode4j.ode.internal.joints.DxJointPiston;
import org.ode4j.ode.internal.joints.DxJointPlane2D;
import org.ode4j.ode.internal.joints.DxJointSlider;
import org.ode4j.ode.internal.joints.DxJointUniversal;
import org.ode4j.math.DVector3;
import org.ode4j.ode.internal.DxWorld;

public abstract class ApiCppJoint extends ApiCppOther {

	/* joint type numbers */

	public enum dJointType {
		dJointTypeNone (null),		/* or "unknown" */
		dJointTypeBall (null),
		dJointTypeHinge (null),
		dJointTypeSlider (null),
		dJointTypeContact (null),
		dJointTypeUniversal (null),
		dJointTypeHinge2 (null),
		dJointTypeFixed (null),
		dJointTypeNull (null),
		dJointTypeAMotor (null),
		dJointTypeLMotor (null),
		dJointTypePlane2D (null),
		dJointTypePR (null),
		dJointTypePU (null),
		dJointTypePiston (null);
		final Class<?> _cls;
		private dJointType(Class<?> cls) {
			_cls = cls;
		}
	}

	/**
	 * @brief Create a new joint of the ball type.
	 * @ingroup joints
	 * @remarks
	 * The joint is initially in "limbo" (i.e. it has no effect on the simulation)
	 * because it does not connect to any bodies.
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DBallJoint dJointCreateBall (DWorld w, DJointGroup g) {
		return ODE.dJointCreateBall((DxWorld) w, g);
	}

	/**
	 * @brief Create a new joint of the hinge type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DHingeJoint dJointCreateHinge (DWorld w, DJointGroup g) {
		return ODE.dJointCreateHinge((DxWorld) w, g);
	}


	/**
	 * @brief Create a new joint of the slider type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DSliderJoint dJointCreateSlider (DWorld w, DJointGroup g) {
		return ODE.dJointCreateSlider((DxWorld) w, g);
	}


	/**
	 * @brief Create a new joint of the contact type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	//	dJoint dJointCreateContact (dWorld, dJointGroup, const dContact *);
	public static DJoint dJointCreateContact (DWorld w, DJointGroup g, final DContact c) {
		return ODE.dJointCreateContact((DxWorld) w, (DJointGroup) g, c);
	}

	//TODO by TZ
	/**
	 * @brief Create a new joint feedback.
	 * @ingroup joints
	 */
	public static DJoint.DJointFeedback dJointCreateFeedback() {
		return new DJoint.DJointFeedback();
	}

	/**
	 * @brief Create a new joint of the hinge2 type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DJoint dJointCreateHinge2 (DWorld w, DJointGroup g) {
		return ODE.dJointCreateHinge2((DWorld) w, g);
	}


	/**
	 * @brief Create a new joint of the universal type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DJoint dJointCreateUniversal (DWorld w, DJointGroup g) {
		return ODE.dJointCreateUniversal((DWorld) w, g);
	}


	/**
	 * @brief Create a new joint of the PR (Prismatic and Rotoide) type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DJoint dJointCreatePR (DWorld w, DJointGroup g) {
		return ODE.dJointCreatePR((DWorld)  w, g);
	}


	/**
	 * @brief Create a new joint of the PU (Prismatic and Universal) type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DJoint dJointCreatePU (DWorld w, DJointGroup g) {
		return ODE.dJointCreatePU( (DWorld) w, g);
	}


	/**
	 * @brief Create a new joint of the Piston type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 *                      If it is nonzero the joint is allocated in the given
	 *                      joint group.
	 */
	//ODE_API 
	public static DJoint dJointCreatePiston (DWorld w, DJointGroup g) {
		return ODE.dJointCreatePiston((DWorld) w, g);
	}


	/**
	 * @brief Create a new joint of the fixed type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DJoint dJointCreateFixed (DWorld w, DJointGroup g) {
		return ODE.dJointCreateFixed((DWorld) w, g);
	}


	//ODE_API 
	DJoint dJointCreateNull (DWorld w, DJointGroup g) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Create a new joint of the A-motor type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DJoint dJointCreateAMotor (DWorld w, DJointGroup g) {
		return ODE.dJointCreateAMotor((DxWorld) w, g);
	}


	/**
	 * @brief Create a new joint of the L-motor type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DJoint dJointCreateLMotor (DWorld w, DJointGroup g) {
		return ODE.dJointCreateLMotor((DxWorld) w, g);
	}


	/**
	 * @brief Create a new joint of the plane-2d type.
	 * @ingroup joints
	 * @param DJointGroup set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 */
	//ODE_API 
	public static DJoint dJointCreatePlane2D (DWorld w, DJointGroup g) {
		return ODE.dJointCreatePlane2D((DxWorld) w, g);
	}


	/**
	 * @brief Destroy a joint.
	 * @ingroup joints
	 *
	 * disconnects it from its attached bodies and removing it from the world.
	 * However, if the joint is a member of a group then this function has no
	 * effect - to destroy that joint the group must be emptied or destroyed.
	 */
	//ODE_API 
	void dJointDestroy (DJoint j) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief Create a joint group
	 * @ingroup joints
	 * @param max_size deprecated. Set to 0.
	 */
	//ODE_API 
	public static DJointGroup dJointGroupCreate (int max_size) {
		return DxJointGroup.dJointGroupCreate(max_size);
	}


	/**
	 * @brief Destroy a joint group.
	 * @ingroup joints
	 *
	 * All joints in the joint group will be destroyed.
	 */
	//ODE_API 
	public static void dJointGroupDestroy (DJointGroup g) {
		((DxJointGroup)g).dJointGroupDestroy();
	}


	/**
	 * @brief Empty a joint group.
	 * @ingroup joints
	 *
	 * All joints in the joint group will be destroyed,
	 * but the joint group itself will not be destroyed.
	 */
	//ODE_API 
	public static void dJointGroupEmpty (DJointGroup g) {
		((DxJointGroup)g).dJointGroupEmpty();
	}


	/**
	 * @brief Return the number of bodies attached to the joint
	 * @ingroup joints
	 */
	//ODE_API 
	int dJointGetNumBodies(DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Attach the joint to some new bodies.
	 * @ingroup joints
	 *
	 * If the joint is already attached, it will be detached from the old bodies
	 * first.
	 * To attach this joint to only one body, set body1 or body2 to zero - a zero
	 * body refers to the static environment.
	 * Setting both bodies to zero puts the joint into "limbo", i.e. it will
	 * have no effect on the simulation.
	 * @remarks
	 * Some joints, like hinge-2 need to be attached to two bodies to work.
	 */
	//ODE_API 
	public static void dJointAttach (DJoint j, DBody body1, DBody body2) {
		((DxJoint)j).dJointAttach((DBody)body1, (DBody)body2);
	}


	/**
	 * @brief Set the user-data pointer
	 * @ingroup joints
	 */
	//ODE_API
	//	void dJointSetData (dJoint j, void *data) {
	void dJointSetData (DJoint j, Object data) {
		((DxJoint)j).dJointSetData(data);
	}


	/**
	 * @brief Get the user-data pointer
	 * @ingroup joints
	 */
	//ODE_API 
	//	void *dJointGetData (dJoint j) {
	Object dJointGetData (DJoint j) {
		return ((DxJoint)j).dJointGetData();
	}


	/**
	 * @brief Get the type of the joint
	 * @ingroup joints
	 * @return the type, being one of these:
	 * \li dJointTypeBall
	 * \li dJointTypeHinge
	 * \li dJointTypeSlider
	 * \li dJointTypeContact
	 * \li dJointTypeUniversal
	 * \li dJointTypeHinge2
	 * \li dJointTypeFixed
	 * \li dJointTypeNull
	 * \li dJointTypeAMotor
	 * \li dJointTypeLMotor
	 * \li dJointTypePlane2D
	 * \li dJointTypePR
	 * \li dJointTypePU
	 * \li dJointTypePiston
	 */
	//ODE_API 
	dJointType dJointGetType (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Return the bodies that this joint connects.
	 * @ingroup joints
	 * @param index return the first (0) or second (1) body.
	 * @remarks
	 * If one of these returned body IDs is zero, the joint connects the other body
	 * to the static environment.
	 * If both body IDs are zero, the joint is in ``limbo'' and has no effect on
	 * the simulation.
	 */
	//ODE_API
	public static DBody dJointGetBody (DJoint j, int index) {
		return ((DxJoint)j).dJointGetBody(index);
	}


	/**
	 * @brief Sets the datastructure that is to receive the feedback.
	 *
	 * The feedback can be used by the user, so that it is known how
	 * much force an individual joint exerts.
	 * @ingroup joints
	 */
	//ODE_API 
	//	void dJointSetFeedback (dJoint j, dJointFeedback *);
	public static void dJointSetFeedback (DJoint j, DJoint.DJointFeedback fb) {
		((DxJoint)j).dJointSetFeedback(fb);
	}


	/**
	 * @brief Gets the datastructure that is to receive the feedback.
	 * @ingroup joints
	 */
	//ODE_API 
	//	dJointFeedback *dJointGetFeedback (dJoint j);
	DJoint.DJointFeedback dJointGetFeedback (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the joint anchor point.
	 * @ingroup joints
	 *
	 * The joint will try to keep this point on each body
	 * together. The input is specified in world coordinates.
	 */
	//ODE_API 
	public static void dJointSetBallAnchor (DJoint j, double x, double y, double z) {
		((DxJointBall)j).dJointSetBallAnchor(x, y, z);
	}


	/**
	 * @brief Set the joint anchor point.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointSetBallAnchor2 (DJoint j, double x, double y, double z) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Param setting for Ball joints
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointSetBallParam (DJoint j, int parameter, double value) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set hinge anchor parameter.
	 * @ingroup joints
	 */
	static public //ODE_API 
	void dJointSetHingeAnchor (DJoint j, double x, double y, double z) {
		((DxJointHinge)j).dJointSetHingeAnchor(x, y, z);
	}


	//ODE_API 
	void dJointSetHingeAnchorDelta (DJoint j, double x, double y, double z, double ax, double ay, double az) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set hinge axis.
	 * @ingroup joints
	 */
	static public //ODE_API 
	void dJointSetHingeAxis (DJoint j, double x, double y, double z) {
		((DxJointHinge)j).dJointSetHingeAxis(x, y, z);
	}


	/**
	 * @brief Set the Hinge axis as if the 2 bodies were already at angle appart.
	 * @ingroup joints
	 *
	 * This function initialize the Axis and the relative orientation of each body
	 * as if body1 was rotated around the axis by the angle value. \br
	 * Ex:
	 * <PRE>
	 * dJointSetHingeAxis(jId, 1, 0, 0);
	 * // If you request the position you will have: dJointGetHingeAngle(jId) == 0
	 * dJointSetHingeAxisDelta(jId, 1, 0, 0, 0.23);
	 * // If you request the position you will have: dJointGetHingeAngle(jId) == 0.23
	 * </PRE>

	 * @param j The Hinge joint ID for which the axis will be set
	 * @param x The X component of the axis in world frame
	 * @param y The Y component of the axis in world frame
	 * @param z The Z component of the axis in world frame
	 * @param angle The angle for the offset of the relative orientation.
	 *              As if body1 was rotated by angle when the Axis was set (see below).
	 *              The rotation is around the new Hinge axis.
	 *
	 * @note Usually the function dJointSetHingeAxis set the current position of body1
	 *       and body2 as the zero angle position. This function set the current position
	 *       as the if the 2 bodies where \b angle appart.
	 * @warning Calling dJointSetHingeAnchor or dJointSetHingeAxis will reset the "zero"
	 *          angle position.
	 */
	static public //ODE_API 
	void dJointSetHingeAxisOffset (DJoint j, double x, double y, double z, double angle) {
		((DxJointHinge)j).dJointSetHingeAxisOffset(x, y, z, angle);
	}


	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetHingeParam (DJoint j, int parameter, double value) {
		((DxJointHinge)j).dJointSetHingeParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @brief Applies the torque about the hinge axis.
	 *
	 * That is, it applies a torque with specified magnitude in the direction
	 * of the hinge axis, to body 1, and with the same magnitude but in opposite
	 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointAddHingeTorque(DJoint joint, double torque) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief set the joint axis
	 * @ingroup joints
	 */
	static public //ODE_API 
	void dJointSetSliderAxis (DJoint j, double x, double y, double z) {
		((DxJointSlider)j).dJointSetSliderAxis(x, y, z);
	}


	/**
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointSetSliderAxisDelta (DJoint j, double x, double y, double z, double ax, double ay, double az) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetSliderParam (DJoint j, int parameter, double value) {
		((DxJointSlider)j).dJointSetSliderParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @brief Applies the given force in the slider's direction.
	 *
	 * That is, it applies a force with specified magnitude, in the direction of
	 * slider's axis, to body1, and with the same magnitude but opposite
	 * direction to body2.  This function is just a wrapper for dBodyAddForce().
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointAddSliderForce(DJoint joint, double force) {
		((DxJointSlider)joint).dJointAddSliderForce(force);
	}


	/**
	 * @brief set anchor
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetHinge2Anchor (DJoint j, double x, double y, double z) {
		((DxJointHinge2)j).dJointSetHinge2Anchor(x, y, z);
	}


	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetHinge2Axis1 (DJoint j, double x, double y, double z) {
		((DxJointHinge2)j).dJointSetHinge2Axis1(x, y, z);
	}


	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetHinge2Axis2 (DJoint j, double x, double y, double z) {
		((DxJointHinge2)j).dJointSetHinge2Axis2(x, y, z);
	}


	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 */
	static public //ODE_API 
	void dJointSetHinge2Param (DJoint j, int parameter, double value) {
		((DxJointHinge2)j).dJointSetHinge2Param(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @brief Applies torque1 about the hinge2's axis 1, torque2 about the
	 * hinge2's axis 2.
	 * @remarks  This function is just a wrapper for dBodyAddTorque().
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointAddHinge2Torques(DJoint joint, double torque1, double torque2) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief set anchor
	 * @ingroup joints
	 */
	static public //ODE_API 
	void dJointSetUniversalAnchor (DJoint j, double x, double y, double z) {
		((DxJointUniversal)j).dJointSetUniversalAnchor(x, y, z);
	}


	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetUniversalAxis1 (DJoint j, double x, double y, double z) {
		((DxJointUniversal)j).dJointSetUniversalAxis1(x, y, z);
	}


	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetUniversalAxis2 (DJoint j, double x, double y, double z) {
		((DxJointUniversal)j).dJointSetUniversalAxis2(x, y, z);
	}


	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 */
	static public //ODE_API 
	void dJointSetUniversalParam (DJoint j, int parameter, double value) {
		((DxJointUniversal)j).dJointSetUniversalParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @brief Applies torque1 about the universal's axis 1, torque2 about the
	 * universal's axis 2.
	 * @remarks This function is just a wrapper for dBodyAddTorque().
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointAddUniversalTorques(DJoint joint, double torque1, double torque2) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief set anchor
	 * @ingroup joints
	 */
	//ODE_API 
	static public void dJointSetPRAnchor (DJoint j, double x, double y, double z) {
		((DxJointPR)j).dJointSetPRAnchor(x, y, z);
	}


	/**
	 * @brief set the axis for the prismatic articulation
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetPRAxis1 (DJoint j, double x, double y, double z) {
		((DxJointPR)j).dJointSetPRAxis1(x, y, z);
	}


	/**
	 * @brief set the axis for the rotoide articulation
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetPRAxis2 (DJoint j, double x, double y, double z) {
		((DxJointPR)j).dJointSetPRAxis2(x, y, z);
	}


	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 *
	 * @note parameterX where X equal 2 refer to parameter for the rotoide articulation
	 */
	//ODE_API 
	static public void dJointSetPRParam (DJoint j, int parameter, double value) {
		((DxJointPR)j).dJointSetPRParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @brief Applies the torque about the rotoide axis of the PR joint
	 *
	 * That is, it applies a torque with specified magnitude in the direction 
	 * of the rotoide axis, to body 1, and with the same magnitude but in opposite
	 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointAddPRTorque (DJoint j, double torque) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief set anchor
	 * @ingroup joints
	 */
	static public //ODE_API 
	void dJointSetPUAnchor (DJoint j, double x, double y, double z) {
		((DxJointPU)j).dJointSetPUAnchor(x, y, z);
	}


	/**
	 * @brief set anchor
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointSetPUAnchorDelta (DJoint j, double x, double y, double z,
			double dx, double dy, double dz) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief set the axis for the first axis or the universal articulation
	 * @ingroup joints
	 */
	//ODE_API 
	static public void dJointSetPUAxis1 (DJoint j, double x, double y, double z) {
		((DxJointPU)j).dJointSetPUAxis1(x, y, z);
	}


	/**
	 * @brief set the axis for the second axis or the universal articulation
	 * @ingroup joints
	 */
	//ODE_API 
	static public void dJointSetPUAxis2 (DJoint j, double x, double y, double z) {
		((DxJointPU)j).dJointSetPUAxis2(x, y, z);
	}


	/**
	 * @brief set the axis for the prismatic articulation
	 * @ingroup joints
	 */
	//ODE_API 
	static public void dJointSetPUAxis3 (DJoint j, double x, double y, double z) {
		((DxJointPU)j).dJointSetPUAxis1(x, y, z);
	}


	/**
	 * @brief set the axis for the prismatic articulation
	 * @ingroup joints
	 * @note This function was added for convenience it is the same as
	 *       dJointSetPUAxis3
	 */
	//ODE_API 
	static public void dJointSetPUAxisP (DJoint j, double x, double y, double z) {
		((DxJointPU)j).dJointSetPUAxisP(x, y, z);
	}




	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 *
	 * @note parameterX where X equal 2 refer to parameter for second axis of the
	 *       universal articulation
	 * @note parameterX where X equal 3 refer to parameter for prismatic
	 *       articulation
	 */
	//ODE_API 
	static public void dJointSetPUParam (DJoint j, int parameter, double value) {
		((DxJointPU)j).dJointSetPUParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @brief Applies the torque about the rotoide axis of the PU joint
	 *
	 * That is, it applies a torque with specified magnitude in the direction
	 * of the rotoide axis, to body 1, and with the same magnitude but in opposite
	 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointAddPUTorque (DJoint j, double torque) {
		throw new UnsupportedOperationException();
	}





	/**
	 * @brief set the joint anchor
	 * @ingroup joints
	 */
	static public //ODE_API 
	void dJointSetPistonAnchor (DJoint j, double x, double y, double z) {
		((DxJointPiston)j).dJointSetPistonAnchor(new DVector3(x, y, z));
	}

	/**
	 * @brief Set the Piston anchor as if the 2 bodies were already at [dx,dy, dz] appart.
	 * @ingroup joints
	 *
	 * This function initialize the anchor and the relative position of each body
	 * as if the position between body1 and body2 was already the projection of [dx, dy, dz]
	 * along the Piston axis. (i.e as if the body1 was at its current position - [dx,dy,dy] when the
	 * axis is set).
	 * Ex:
	 * <PRE>
	 * double offset = 3;
	 * dVector3 axis;
	 * dJointGetPistonAxis(jId, axis);
	 * dJointSetPistonAnchor(jId, 0, 0, 0);
	 * // If you request the position you will have: dJointGetPistonPosition(jId) == 0
	 * dJointSetPistonAnchorOffset(jId, 0, 0, 0, axis[X]*offset, axis[Y]*offset, axis[Z]*offset);
	 * // If you request the position you will have: dJointGetPistonPosition(jId) == offset
	 * </PRE>
	 * @param j The Piston joint for which the anchor point will be set
	 * @param x The X position of the anchor point in world frame
	 * @param y The Y position of the anchor point in world frame
	 * @param z The Z position of the anchor point in world frame
	 * @param dx A delta to be substracted to the X position as if the anchor was set
	 *           when body1 was at current_position[X] - dx
	 * @param dx A delta to be substracted to the Y position as if the anchor was set
	 *           when body1 was at current_position[Y] - dy
	 * @param dx A delta to be substracted to the Z position as if the anchor was set
	 *           when body1 was at current_position[Z] - dz
	 */
	//ODE_API 
	void dJointSetPistonAnchorOffset(DJoint j, double x, double y, double z,
			double dx, double dy, double dz) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief set the joint axis
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointSetPistonAxis (DJoint j, double x, double y, double z) {
		throw new UnsupportedOperationException();
	}


	/**
	 * This function set prismatic axis of the joint and also set the position
	 * of the joint.
	 *
	 * @ingroup joints
	 * @param j The joint affected by this function
	 * @param x The x component of the axis
	 * @param y The y component of the axis
	 * @param z The z component of the axis
	 * @param dx The Initial position of the prismatic join in the x direction
	 * @param dy The Initial position of the prismatic join in the y direction
	 * @param dz The Initial position of the prismatic join in the z direction
	 * @deprecated TZ
	 */
	//ODE_API_DEPRECATED ODE_API 
	void dJointSetPistonAxisDelta (DJoint j, double x, double y, double z, double ax, double ay, double az) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetPistonParam (DJoint j, int parameter, double value) {
		((DxJointPiston)j).dJointSetPistonParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @brief Applies the given force in the slider's direction.
	 *
	 * That is, it applies a force with specified magnitude, in the direction of
	 * prismatic's axis, to body1, and with the same magnitude but opposite
	 * direction to body2.  This function is just a wrapper for dBodyAddForce().
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointAddPistonForce (DJoint joint, double force) {
		((DxJointPiston)joint).dJointAddPistonForce(force);
	}



	/**
	 * @brief Call this on the fixed joint after it has been attached to
	 * remember the current desired relative offset and desired relative
	 * rotation between the bodies.
	 * @ingroup joints
	 */
	//ODE_API
	public static void dJointSetFixed (DJoint j) {
		((DxJointFixed)j).dJointSetFixed();
	}


	/*
	 * @brief Sets joint parameter
	 *
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointSetFixedParam (DJoint j, int parameter, double value) {
		((DxJointFixed)j).dJointSetFixedParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @brief set the nr of axes
	 * @param num 0..3
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetAMotorNumAxes (DJoint j, int num) {
		((DxJointAMotor)j).dJointSetAMotorNumAxes(num);
	}


	/**
	 * @brief set axis
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetAMotorAxis (DJoint j, int anum, int rel,
			double x, double y, double z) {
		((DxJointAMotor)j).dJointSetAMotorAxis(anum, rel, x, y, z);
	}


	/**
	 * @brief Tell the AMotor what the current angle is along axis anum.
	 *
	 * This function should only be called in dAMotorUser mode, because in this
	 * mode the AMotor has no other way of knowing the joint angles.
	 * The angle information is needed if stops have been set along the axis,
	 * but it is not needed for axis motors.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointSetAMotorAngle (DJoint j, int anum, double angle) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetAMotorParam (DJoint j, int parameter, double value) {
		((DxJointAMotor)j).dJointSetAMotorParam(
				D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @brief set mode
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetAMotorMode (DJoint j, int mode) {
		((DxJointAMotor)j).dJointSetAMotorMode(DxJointAMotor.AMotorMode.from(mode));
	}


	/**
	 * @brief Applies torque0 about the AMotor's axis 0, torque1 about the
	 * AMotor's axis 1, and torque2 about the AMotor's axis 2.
	 * @remarks
	 * If the motor has fewer than three axes, the higher torques are ignored.
	 * This function is just a wrapper for dBodyAddTorque().
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointAddAMotorTorques (DJoint j, double torque1, double torque2, double torque3) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Set the number of axes that will be controlled by the LMotor.
	 * @param num can range from 0 (which effectively deactivates the joint) to 3.
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetLMotorNumAxes (DJoint j, int num) {
		((DxJointLMotor)j).dJointSetLMotorNumAxes(num);
	}


	/**
	 * @brief Set the AMotor axes.
	 * @param anum selects the axis to change (0,1 or 2).
	 * @param rel Each axis can have one of three ``relative orientation'' modes
	 * \li 0: The axis is anchored to the global frame.
	 * \li 1: The axis is anchored to the first body.
	 * \li 2: The axis is anchored to the second body.
	 * @remarks The axis vector is always specified in global coordinates
	 * regardless of the setting of rel.
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetLMotorAxis (DJoint j, int anum, int rel, double x, double y, double z) {
		((DxJointLMotor)j).dJointSetLMotorAxis(anum, rel, x, y, z);
	}


	/**
	 * @brief set joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetLMotorParam (DJoint j, int parameter, double value) {
		((DxJointLMotor)j).dJointSetLMotorParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointSetPlane2DXParam (DJoint j, int parameter, double value) {
		((DxJointPlane2D)j).dJointSetPlane2DXParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @ingroup joints
	 */

	//ODE_API 
	public static void dJointSetPlane2DYParam (DJoint j, int parameter, double value) {
		((DxJointPlane2D)j).dJointSetPlane2DYParam(D_PARAM_NAMES_N.toEnum(parameter), value);
	}


	/**
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointSetPlane2DAngleParam (DJoint j, int parameter, double value) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 *
	 * This returns the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 */
	//ODE_API 
	void dJointGetBallAnchor (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 *
	 * This returns the point on body 2. You can think of a ball and socket
	 * joint as trying to keep the result of dJointGetBallAnchor() and
	 * dJointGetBallAnchor2() the same.  If the joint is perfectly satisfied,
	 * this function will return the same value as dJointGetBallAnchor() to
	 * within roundoff errors. dJointGetBallAnchor2() can be used, along with
	 * dJointGetBallAnchor(), to see how far the joint has come apart.
	 */
	//ODE_API 
	void dJointGetBallAnchor2 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetBallParam (DJoint j, int parameter) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the hinge anchor point, in world coordinates.
	 *
	 * This returns the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetHingeAnchor (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * @return The point on body 2. If the joint is perfectly satisfied,
	 * this will return the same value as dJointGetHingeAnchor().
	 * If not, this value will be slightly different.
	 * This can be used, for example, to see how far the joint has come apart.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetHingeAnchor2 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get axis
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetHingeAxis (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetHingeParam (DJoint j, int parameter) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the hinge angle.
	 *
	 * The angle is measured between the two bodies, or between the body and
	 * the static environment.
	 * The angle will be between -pi..pi.
	 * Give the relative rotation with respect to the Hinge axis of Body 1 with
	 * respect to Body 2.
	 * When the hinge anchor or axis is set, the current position of the attached
	 * bodies is examined and that position will be the zero angle.
	 * @ingroup joints
	 */
	static public //ODE_API 
	double dJointGetHingeAngle (DJoint j) {
		return ((DxJointHinge)j).dJointGetHingeAngle();
	}


	/**
	 * @brief Get the hinge angle time derivative.
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetHingeAngleRate (DJoint j) {
		return ((DxJointHinge)j).dJointGetHingeAngleRate();
	}


	/**
	 * @brief Get the slider linear position (i.e. the slider's extension)
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.

	 * The position is the distance, with respect to the zero position,
	 * along the slider axis of body 1 with respect to
	 * body 2. (A NULL body is replaced by the world).
	 * @ingroup joints
	 */
	static public //ODE_API 
	double dJointGetSliderPosition (DJoint j) {
		return ((DxJointSlider)j).dJointGetSliderPosition();
	}


	/**
	 * @brief Get the slider linear position's time derivative.
	 * @ingroup joints
	 */
	static public //ODE_API 
	double dJointGetSliderPositionRate (DJoint j) {
		return ((DxJointSlider)j).dJointGetSliderPositionRate();
	}


	/**
	 * @brief Get the slider axis
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetSliderAxis (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetSliderParam (DJoint j, int parameter) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * @return the point on body 1.  If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetHinge2Anchor (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * This returns the point on body 2. If the joint is perfectly satisfied,
	 * this will return the same value as dJointGetHinge2Anchor.
	 * If not, this value will be slightly different.
	 * This can be used, for example, to see how far the joint has come apart.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetHinge2Anchor2 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get joint axis
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetHinge2Axis1 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get joint axis
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetHinge2Axis2 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetHinge2Param (DJoint j, int parameter) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get angle
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetHinge2Angle1 (DJoint j) {
		return ((DxJointHinge2)j).dJointGetHinge2Angle1();
	}


	/**
	 * @brief Get time derivative of angle
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetHinge2Angle1Rate (DJoint j) {
		return ((DxJointHinge2)j).dJointGetHinge2Angle1Rate();
	}


	/**
	 * @brief Get time derivative of angle
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetHinge2Angle2Rate (DJoint j) {
		return ((DxJointHinge2)j).dJointGetHinge2Angle2Rate();
	}


	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * @return the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetUniversalAnchor (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * @return This returns the point on body 2.
	 * @remarks
	 * You can think of the ball and socket part of a universal joint as
	 * trying to keep the result of dJointGetBallAnchor() and
	 * dJointGetBallAnchor2() the same. If the joint is
	 * perfectly satisfied, this function will return the same value
	 * as dJointGetUniversalAnchor() to within roundoff errors.
	 * dJointGetUniversalAnchor2() can be used, along with
	 * dJointGetUniversalAnchor(), to see how far the joint has come apart.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetUniversalAnchor2 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get axis
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointGetUniversalAxis1 (DJoint j, DVector3 result) {
		((DxJointUniversal)j).dJointGetUniversalAxis1(result);
	}


	/**
	 * @brief Get axis
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointGetUniversalAxis2 (DJoint j, DVector3 result) {
		((DxJointUniversal)j).dJointGetUniversalAxis2(result);
	}



	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetUniversalParam (DJoint j, int parameter) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get both angles at the same time.
	 * @ingroup joints
	 *
	 * @param joint   The universal joint for which we want to calculate the angles
	 * @param angle1  The angle between the body1 and the axis 1
	 * @param angle2  The angle between the body2 and the axis 2
	 *
	 * @note This function combine getUniversalAngle1 and getUniversalAngle2 together
	 *       and try to avoid redundant calculation
	 */
	//ODE_API 
	//	void dJointGetUniversalAngles (dJoint j, double *angle1, double *angle2);
	void dJointGetUniversalAngles (DJoint j, RefDouble angle1, RefDouble angle2) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get angle
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetUniversalAngle1 (DJoint j) {
		return ((DxJointUniversal)j).dJointGetUniversalAngle1();
	}


	/**
	 * @brief Get angle
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetUniversalAngle2 (DJoint j) {
		return ((DxJointUniversal)j).dJointGetUniversalAngle2();
	}


	/**
	 * @brief Get time derivative of angle
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetUniversalAngle1Rate (DJoint j) {
		return ((DxJointUniversal)j).dJointGetUniversalAngle1Rate();
	}


	/**
	 * @brief Get time derivative of angle
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetUniversalAngle2Rate (DJoint j) {
		return ((DxJointUniversal)j).dJointGetUniversalAngle2Rate();
	}




	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * @return the point on body 1. If the joint is perfectly satisfied, 
	 * this will be the same as the point on body 2.
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointGetPRAnchor (DJoint j, DVector3 result) {
		((DxJointPR)j).dJointGetPRAnchor(result);
	}


	/**
	 * @brief Get the PR linear position (i.e. the prismatic's extension)
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 *
	 * The position is the "oriented" length between the
	 * position = (Prismatic axis) dot_product [(body1 + offset) - (body2 + anchor2)]
	 *
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetPRPosition (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the PR linear position's time derivative
	 *
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetPRPositionRate (DJoint j) {
		return ((DxJointPR)j).dJointGetPRPositionRate();
	}



	/**
	 * @brief Get the prismatic axis
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetPRAxis1 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the Rotoide axis
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetPRAxis2 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetPRParam (DJoint j, int d_param_names) {
		return ((DxJointPR)j).dJointGetPRParam(D_PARAM_NAMES_N.toEnum(d_param_names));
	}




	/**
	 * @brief Get the joint anchor point, in world coordinates.
	 * @return the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointGetPUAnchor (DJoint j, DVector3 result) {
		((DxJointPU)j).dJointGetPUAnchor(result);
	}


	/**
	 * @brief Get the PU linear position (i.e. the prismatic's extension)
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 *
	 * The position is the "oriented" length between the
	 * position = (Prismatic axis) dot_product [(body1 + offset) - (body2 + anchor2)]
	 *
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetPUPosition (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the PR linear position's time derivative
	 *
	 * @ingroup joints
	 */
	//ODE_API 
	static public double dJointGetPUPositionRate (DJoint j) {
		return ((DxJointPU)j).dJointGetPUPositionRate();
	}


	/**
	 * @brief Get the first axis of the universal component of the joint
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetPUAxis1 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the second axis of the Universal component of the joint
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetPUAxis2 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the prismatic axis
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetPUAxis3 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the prismatic axis
	 * @ingroup joints
	 *
	 * @note This function was added for convenience it is the same as
	 *       dJointGetPUAxis3
	 */
	//ODE_API 
	void dJointGetPUAxisP (DJoint id, DVector3 result) {
		throw new UnsupportedOperationException();
	}





	/**
	 * @brief Get both angles at the same time.
	 * @ingroup joints
	 *
	 * @param joint   The Prismatic universal joint for which we want to calculate the angles
	 * @param angle1  The angle between the body1 and the axis 1
	 * @param angle2  The angle between the body2 and the axis 2
	 *
	 * @note This function combine dJointGetPUAngle1 and dJointGetPUAngle2 together
	 *       and try to avoid redundant calculation
	 */
	//ODE_API 
	//	  void dJointGetPUAngles (dJoint j, double *angle1, double *angle2);
	void dJointGetPUAngles (DJoint j, RefDouble angle1, RefDouble angle2) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get angle
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetPUAngle1 (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief * @brief Get time derivative of angle1
	 *
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetPUAngle1Rate (DJoint j) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief Get angle
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetPUAngle2 (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief * @brief Get time derivative of angle2
	 *
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetPUAngle2Rate (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	static public double dJointGetPUParam (DJoint j, int parameter) {
		return ((DxJointPU)j).dJointGetPUParam(D_PARAM_NAMES_N.toEnum(parameter));
	}






	/**
	 * @brief Get the Piston linear position (i.e. the piston's extension)
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetPistonPosition (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the piston linear position's time derivative.
	 * @ingroup joints
	 */
	//ODE_API 
	static public double dJointGetPistonPositionRate (DJoint j) {
		return ((DxJointPiston)j).dJointGetPistonPositionRate();
	}


	/**
	 * @brief Get the Piston angular position (i.e. the  twist between the 2 bodies)
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetPistonAngle (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the piston angular position's time derivative.
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetPistonAngleRate (DJoint j) {
		throw new UnsupportedOperationException();
	}



	/**
	 * @brief Get the joint anchor
	 *
	 * This returns the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2 in direction perpendicular
	 * to the prismatic axis.
	 *
	 * @ingroup joints
	 */
	//ODE_API 
	public static void dJointGetPistonAnchor (DJoint j, DVector3 result) {
		((DxJointPiston)j).dJointGetPistonAnchor(result);
	}


	/**
	 * @brief Get the joint anchor w.r.t. body 2
	 *
	 * This returns the point on body 2. You can think of a Piston
	 * joint as trying to keep the result of dJointGetPistonAnchor() and
	 * dJointGetPistonAnchor2() the same in the direction perpendicular to the
	 * pirsmatic axis. If the joint is perfectly satisfied,
	 * this function will return the same value as dJointGetPistonAnchor() to
	 * within roundoff errors. dJointGetPistonAnchor2() can be used, along with
	 * dJointGetPistonAnchor(), to see how far the joint has come apart.
	 *
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetPistonAnchor2 (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the prismatic axis (This is also the rotoide axis.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetPistonAxis (DJoint j, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	static public double dJointGetPistonParam (DJoint j, int parameter) {
		return ((DxJointPiston)j).dJointGetPistonParam(D_PARAM_NAMES_N.toEnum(parameter));
	}



	/**
	 * @brief Get the number of angular axes that will be controlled by the
	 * AMotor.
	 * @param num can range from 0 (which effectively deactivates the
	 * joint) to 3.
	 * This is automatically set to 3 in dAMotorEuler mode.
	 * @ingroup joints
	 */
	//ODE_API 
	int dJointGetAMotorNumAxes (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the AMotor axes.
	 * @param anum selects the axis to change (0,1 or 2).
	 * @param rel Each axis can have one of three ``relative orientation'' modes.
	 * \li 0: The axis is anchored to the global frame.
	 * \li 1: The axis is anchored to the first body.
	 * \li 2: The axis is anchored to the second body.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetAMotorAxis (DJoint j, int anum, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get axis
	 * @remarks
	 * The axis vector is always specified in global coordinates regardless
	 * of the setting of rel.
	 * There are two GetAMotorAxis functions, one to return the axis and one to
	 * return the relative mode.
	 *
	 * For dAMotorEuler mode:
	 * \li	Only axes 0 and 2 need to be set. Axis 1 will be determined
		automatically at each time step.
	 * \li	Axes 0 and 2 must be perpendicular to each other.
	 * \li	Axis 0 must be anchored to the first body, axis 2 must be anchored
		to the second body.
	 * @ingroup joints
	 */
	//ODE_API 
	int dJointGetAMotorAxisRel (DJoint j, int anum) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the current angle for axis.
	 * @remarks
	 * In dAMotorUser mode this is simply the value that was set with
	 * dJointSetAMotorAngle().
	 * In dAMotorEuler mode this is the corresponding euler angle.
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetAMotorAngle (DJoint j, int anum) {
		return ((DxJointAMotor)j).dJointGetAMotorAngle(anum);
	}


	/**
	 * @brief Get the current angle rate for axis anum.
	 * @remarks
	 * In dAMotorUser mode this is always zero, as not enough information is
	 * available.
	 * In dAMotorEuler mode this is the corresponding euler angle rate.
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetAMotorAngleRate (DJoint j, int anum) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetAMotorParam (DJoint j, int parameter) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get the angular motor mode.
	 * @param mode must be one of the following constants:
	 * \li dAMotorUser The AMotor axes and joint angle settings are entirely
	 * controlled by the user.  This is the default mode.
	 * \li dAMotorEuler Euler angles are automatically computed.
	 * The axis a1 is also automatically computed.
	 * The AMotor axes must be set correctly when in this mode,
	 * as described below.
	 * When this mode is initially set the current relative orientations
	 * of the bodies will correspond to all euler angles at zero.
	 * @ingroup joints
	 */
	//ODE_API 
	int dJointGetAMotorMode (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get nr of axes.
	 * @ingroup joints
	 */
	//ODE_API 
	int dJointGetLMotorNumAxes (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief Get axis.
	 * @ingroup joints
	 */
	//ODE_API 
	void dJointGetLMotorAxis (DJoint j, int anum, DVector3 result) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	public static double dJointGetLMotorParam (DJoint j, int parameter) {
		return ((DxJointLMotor)j).dJointGetLMotorParam(
				D_PARAM_NAMES_N.toEnum(parameter));
	}


	/**
	 * @brief get joint parameter
	 * @ingroup joints
	 */
	//ODE_API 
	double dJointGetFixedParam (DJoint j, int parameter) {
		throw new UnsupportedOperationException();
	}


	/**
	 * @ingroup joints
	 */
	//ODE_API 
	DJoint dConnectingJoint (DBody b1, DBody b2) {
		throw new UnsupportedOperationException();
	}
}
