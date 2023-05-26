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

import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import org.ode4j.ode.DAMotorJoint.AMotorMode;
import org.ode4j.ode.DJoint.PARAM;
import org.ode4j.ode.DJoint.PARAM_N;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.math.DVector3;

public abstract class ApiCppJoint extends ApiCppOther {

	private static final int P_OFS_1 = 0x000;
	private static final int P_OFS_2 = 0x100;
	private static final int P_OFS_3 = 0x200;

	public static final int dParamGroup = 0;
	//	  /* parameters for limits and motors */ \
	public static final int dParamLoStop = PARAM_N.dParamLoStop1.ordinal();
	public static final int dParamHiStop = 1;
	public static final int dParamVel = 2;
	public static final int dParamLoVel = 3;
	public static final int dParamHiVel = 4;
	public static final int dParamFMax = 5;
	public static final int dParamFudgeFactor  = 6;
	public static final int dParamBounce  = 7;
	public static final int dParamCFM  = 8;
	public static final int dParamStopERP  = 9;
	public static final int dParamStopCFM  = 10;
	/* parameters for suspension */ 
	public static final int dParamSuspensionERP  = 11;
	public static final int dParamSuspensionCFM = 12;
	public static final int dParamERP = 13;

	public static final int dParamGroup1 = 0 + P_OFS_1;
	//	  /* parameters for limits and motors */ \
	public static final int dParamLoStop1 = 0 + P_OFS_1;
	public static final int dParamHiStop1 = 1 + P_OFS_1;
	public static final int dParamVel1 = 2 + P_OFS_1;
	public static final int dParamLoVel1 = 3 + P_OFS_1;
	public static final int dParamHiVel1 = 4 + P_OFS_1;
	public static final int dParamFMax1  = 5 + P_OFS_1;
	public static final int dParamFudgeFactor1  = 6 + P_OFS_1;
	public static final int dParamBounce1  = 7 + P_OFS_1;
	public static final int dParamCFM1  = 8 + P_OFS_1;
	public static final int dParamStopERP1  = 9 + P_OFS_1;
	public static final int dParamStopCFM1  = 10 + P_OFS_1;
	/* parameters for suspension */ 
	public static final int dParamSuspensionERP1  = 11 + P_OFS_1;
	public static final int dParamSuspensionCFM1 = 12 + P_OFS_1;
	public static final int dParamERP1 = 13 + P_OFS_1;

	public static final int dParamGroup2 = 0 + P_OFS_2;
	//	  /* parameters for limits and motors */ \
	public static final int dParamLoStop2 = 0 + P_OFS_2;
	public static final int dParamHiStop2 = 1 + P_OFS_2; 
	public static final int dParamVel2 = 2 + P_OFS_2; 
	public static final int dParamLoVel2 = 3 + P_OFS_1;
	public static final int dParamHiVel2 = 4 + P_OFS_1;
	public static final int dParamFMax2  = 5 + P_OFS_2; 
	public static final int dParamFudgeFactor2  = 6 + P_OFS_2; 
	public static final int dParamBounce2  = 7 + P_OFS_2; 
	public static final int dParamCFM2  = 8 + P_OFS_2; 
	public static final int dParamStopERP2  = 9 + P_OFS_2; 
	public static final int dParamStopCFM2  = 10 + P_OFS_2; 
	/* parameters for suspension */ 
	public static final int dParamSuspensionERP2  = 11 + P_OFS_2; 
	public static final int dParamSuspensionCFM2 = 12 + P_OFS_2;
	public static final int dParamERP2 = 13 + P_OFS_2;

	public static final int dParamGroup3 = 0 + P_OFS_3;
	//	  /* parameters for limits and motors */ \
	public static final int dParamLoStop3 = 0 + P_OFS_3; 
	public static final int dParamHiStop3 = 1 + P_OFS_3; 
	public static final int dParamVel3 = 2 + P_OFS_3; 
	public static final int dParamLoVel3 = 3 + P_OFS_1;
	public static final int dParamHiVel3 = 4 + P_OFS_1;
	public static final int dParamFMax3  = 5 + P_OFS_3; 
	public static final int dParamFudgeFactor3  = 6 + P_OFS_3; 
	public static final int dParamBounce3  = 7 + P_OFS_3; 
	public static final int dParamCFM3  = 8 + P_OFS_3; 
	public static final int dParamStopERP3  = 9 + P_OFS_3; 
	public static final int dParamStopCFM3  = 10 + P_OFS_3; 
	/* parameters for suspension */ 
	public static final int dParamSuspensionERP3  = 11 + P_OFS_3; 
	public static final int dParamSuspensionCFM3 = 12 + P_OFS_3;
	public static final int dParamERP3 = 13 + P_OFS_3;

	
	/* angular motor mode numbers */
	public static final int dAMotorUser = 0;
	public static final int dAMotorEuler = 1;

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
		dJointTypeDBall(null),
		dJointTypeDHinge(null),
		dJointTypeTransmission(null),
		dJointTypePiston (null);
		final Class<?> _cls;
		private dJointType(Class<?> cls) {
			_cls = cls;
		}
	}

	/**
	 * Create a new joint of the ball type.
	 * 
	 * <p>REMARK:
	 * The joint is initially in "limbo" (i.e. it has no effect on the simulation)
	 * because it does not connect to any bodies.
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DBallJoint dJointCreateBall (DWorld w, DJointGroup g) {
		return OdeHelper.createBallJoint(w, g);
	}


	/**
	 * Create a new joint of the double ball type.
	 *
	 * @param w world
	 * @param g set to 0 to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API
	public static DDoubleBallJoint dJointCreateDBall (DWorld w, DJointGroup g) {
		return OdeHelper.createDBallJoint(w, g);
	}

	/**
	 * Create a new joint of the hinge type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DHingeJoint dJointCreateHinge (DWorld w, DJointGroup g) {
		return OdeHelper.createHingeJoint(w, g);
	}


	/**
	 * Create a new joint of the slider type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DSliderJoint dJointCreateSlider (DWorld w, DJointGroup g) {
		return OdeHelper.createSliderJoint(w, g);
	}


	/**
	 * Create a new joint of the contact type.
	 * @param w w
	 * 
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @param c contact
	 * @return contact joint
	 */
	//ODE_API 
	//	dJoint dJointCreateContact (dWorld, dJointGroup, const dContact *);
	public static DContactJoint dJointCreateContact (DWorld w, DJointGroup g, final DContact c) {
		return OdeHelper.createContactJoint(w, g, c);
	}

	/**
	 * Create a new joint feedback.
	 * @return feedback
	 */
	public static DJoint.DJointFeedback dJointCreateFeedback() {
		return new DJoint.DJointFeedback();
	}

	/**
	 * Create a new joint of the hinge2 type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DHinge2Joint dJointCreateHinge2 (DWorld w, DJointGroup g) {
		return OdeHelper.createHinge2Joint(w, g);
	}


	/**
	 * Create a new joint of the universal type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DUniversalJoint dJointCreateUniversal (DWorld w, DJointGroup g) {
		return OdeHelper.createUniversalJoint(w, g);
	}


	/**
	 * Create a new joint of the PR (Prismatic and Rotoide) type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DPRJoint dJointCreatePR (DWorld w, DJointGroup g) {
		return OdeHelper.createPRJoint(w, g);
	}


	/**
	 * Create a new joint of the PU (Prismatic and Universal) type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DPUJoint dJointCreatePU (DWorld w, DJointGroup g) {
		return OdeHelper.createPUJoint(w, g);
	}


	/**
	 * Create a new joint of the Piston type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DPistonJoint dJointCreatePiston (DWorld w, DJointGroup g) {
		return OdeHelper.createPistonJoint(w, g);
	}


	/**
	 * Create a new joint of the fixed type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DFixedJoint dJointCreateFixed (DWorld w, DJointGroup g) {
		return OdeHelper.createFixedJoint(w, g);
	}


	//ODE_API 
	public static DNullJoint dJointCreateNull (DWorld w, DJointGroup g) {
		return OdeHelper.createNullJoint(w, g);
	}


	/**
	 * Create a new joint of the A-motor type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DAMotorJoint dJointCreateAMotor (DWorld w, DJointGroup g) {
		return OdeHelper.createAMotorJoint(w, g);
	}


	/**
	 * Create a new joint of the L-motor type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DLMotorJoint dJointCreateLMotor (DWorld w, DJointGroup g) {
		return OdeHelper.createLMotorJoint(w, g);
	}


	/**
	 * Create a new joint of the plane-2d type.
	 * 
	 * @param w w
	 * @param g set to <tt>null</tt> to allocate the joint normally.
	 * If it is nonzero the joint is allocated in the given joint group.
	 * @return joint
	 */
	//ODE_API 
	public static DPlane2DJoint dJointCreatePlane2D (DWorld w, DJointGroup g) {
		return OdeHelper.createPlane2DJoint(w, g);
	}


	/**
	 * Destroy a joint.
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
	 * Create a joint group.
	 * 
	 * @param max_size deprecated. Set to 0.
	 * @return joint group
	 */
	//ODE_API 
	public static DJointGroup dJointGroupCreate (int max_size) {
		return OdeHelper.createJointGroup();
	}


	/**
	 * Destroy a joint group.
	 * 
	 * All joints in the joint group will be destroyed.
	 * @param g g
	 */
	//ODE_API 
	public static void dJointGroupDestroy (DJointGroup g) {
		g.destroy();
	}


	/**
	 * Empty a joint group.
	 * 
	 * All joints in the joint group will be destroyed,
	 * but the joint group itself will not be destroyed.
	 * @param g g
	 */
	//ODE_API 
	public static void dJointGroupEmpty (DJointGroup g) {
		g.empty();
	}


	/**
	 * Return the number of bodies attached to the joint.
	 * @param j j
	 * @return ret
	 */
	//ODE_API 
	public static int dJointGetNumBodies(DJoint j) {
		return j.getNumBodies();
	}


	/**
	 * Attach the joint to some new bodies.
	 * 
	 * If the joint is already attached, it will be detached from the old bodies
	 * first.
	 * To attach this joint to only one body, set body1 or body2 to zero - a zero
	 * body refers to the static environment.
	 * Setting both bodies to zero puts the joint into "limbo", i.e. it will
	 * have no effect on the simulation.
	 * 
	 * <p>REMARK:
	 * Some joints, like hinge-2 need to be attached to two bodies to work.
	 * @param j j
	 * @param body1 body 1 
	 * @param body2 body 2
	 */
	//ODE_API 
	public static void dJointAttach (DJoint j, DBody body1, DBody body2) {
		j.attach(body1, body2);
	}


	/**
	 * Manually enable a joint.
	 * 
	 * @param j identification of joint.
	 */
	//ODE_API 
	public static void dJointEnable (DJoint j) {
		j.enable();
	}

	/**
	 * Manually disable a joint.
	 * 
	 * <p>REMARK:
	 * A disabled joint will not affect the simulation, but will maintain the anchors and
	 * axes so it can be enabled later.
	 * @param j joint
	 */
	//ODE_API 
	public static void dJointDisable (DJoint j) {
		j.disable();
	}

	/**
	 * Check wether a joint is enabled.
	 * 
	 * @param j joint
	 * @return 1 if a joint is currently enabled or 0 if it is disabled.
	 */
	//ODE_API 
	public static boolean dJointIsEnabled (DJoint j) {
		return j.isEnabled();
	}

	
	/**
	 * Set the user-data pointer.
	 * @param j joint
	 * @param data data
	 */
	//ODE_API
	//	void dJointSetData (dJoint j, void *data) {
	public static void dJointSetData (DJoint j, Object data) {
		j.setData(data);
	}


	/**
	 * Get the user-data pointer.
	 * @param j joint
	 * @return ret
	 */
	//ODE_API 
	//	void *dJointGetData (dJoint j) {
	public static Object dJointGetData (DJoint j) {
		return j.getData();
	}


	/**
	 * Get the type of the joint.
	 * @return the type, being one of these:
	 * <li> dJointTypeBall </li>
	 * <li> dJointTypeHinge </li>
	 * <li> dJointTypeSlider </li>
	 * <li> dJointTypeContact </li>
	 * <li> dJointTypeUniversal </li>
	 * <li> dJointTypeHinge2 </li>
	 * <li> dJointTypeFixed </li>
	 * <li> dJointTypeNull </li>
	 * <li> dJointTypeAMotor </li>
	 * <li> dJointTypeLMotor </li>
	 * <li> dJointTypePlane2D </li>
	 * <li> dJointTypePR </li>
	 * <li> dJointTypePU </li>
	 * <li> dJointTypePiston </li>
	 */
	//ODE_API 
	dJointType dJointGetType (DJoint j) {
		throw new UnsupportedOperationException();
	}


	/**
	 * Return the bodies that this joint connects.
	 * 
	 * @param j joint
	 * @param index return the first (0) or second (1) body.
	 * <p>REMARK:
	 * If one of these returned body IDs is zero, the joint connects the other body
	 * to the static environment.
	 * If both body IDs are zero, the joint is in ``limbo'' and has no effect on
	 * the simulation.
	 * @return ret
	 */
	//ODE_API
	public static DBody dJointGetBody (DJoint j, int index) {
		return j.getBody(index);
	}


	/**
	 * Sets the datastructure that is to receive the feedback.
	 *
	 * The feedback can be used by the user, so that it is known how
	 * much force an individual joint exerts.
	 * @param j j
	 * @param fb fb
	 */
	//ODE_API 
	//	void dJointSetFeedback (dJoint j, dJointFeedback *);
	public static void dJointSetFeedback (DJoint j, DJoint.DJointFeedback fb) {
		j.setFeedback(fb);
	}


	/**
	 * Gets the datastructure that is to receive the feedback.
	 * @param j j
	 * @return ret
	 */
	//ODE_API 
	//	dJointFeedback *dJointGetFeedback (dJoint j);
	public static DJoint.DJointFeedback dJointGetFeedback (DJoint j) {
		return j.getFeedback();
	}


	/**
	 * Set the joint anchor point.
	 *
	 * The joint will try to keep this point on each body
	 * together. The input is specified in world coordinates.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetBallAnchor (DBallJoint j, double x, double y, double z) {
		j.setAnchor(x, y, z);
	}


	/**
	 * Set the joint anchor point.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetBallAnchor2 (DBallJoint j, double x, double y, double z) {
		j.setAnchor2(x, y, z);
	}

	/**
	 * Param setting for Ball joints.
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetBallParam (DBallJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}

	/**
	 * Set the target distance for the double ball joint.
	 * @param j j
	 * @param distance distance
	 */
	//ODE_API
	public static void dJointSetDBallDistance (DDoubleBallJoint j, double distance) {
		j.setDistance(distance);
	}


	/**
	 * Set hinge anchor parameter.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetHingeAnchor (DHingeJoint j, double x, double y, double z) {
		j.setAnchor(x, y, z);
	}


	//ODE_API 
	void dJointSetHingeAnchorDelta (DHingeJoint j, double x, double y, double z, double ax, double ay, double az) {
		throw new UnsupportedOperationException();
	}


	/**
	 * Set hinge axis.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetHingeAxis (DHingeJoint j, double x, double y, double z) {
		j.setAxis(x, y, z);
	}


	/**
	 * Set the Hinge axis as if the 2 bodies were already at angle appart.
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
	 *
	 * <p>NOTE: Usually the function dJointSetHingeAxis set the current position of body1
	 *       and body2 as the zero angle position. This function set the current position
	 *       as the if the 2 bodies where \b angle appart.
	 * <p>WARNING: Calling dJointSetHingeAnchor or dJointSetHingeAxis will reset the "zero"
	 *          angle position.
	 *          
	 * @param j The Hinge joint ID for which the axis will be set
	 * @param x The X component of the axis in world frame
	 * @param y The Y component of the axis in world frame
	 * @param z The Z component of the axis in world frame
	 * @param angle The angle for the offset of the relative orientation.
	 *              As if body1 was rotated by angle when the Axis was set (see below).
	 *              The rotation is around the new Hinge axis.
	 */
	//ODE_API 
	public static void dJointSetHingeAxisOffset (DHingeJoint j, 
			double x, double y, double z, double angle) {
		j.setAxisOffset(x, y, z, angle);
	}


	/**
	 * Set joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetHingeParam (DHingeJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * Applies the torque about the hinge axis.
	 *
	 * That is, it applies a torque with specified magnitude in the direction
	 * of the hinge axis, to body 1, and with the same magnitude but in opposite
	 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
	 * @param joint j
	 * @param torque t
	 */
	//ODE_API 
	public static void dJointAddHingeTorque(DHingeJoint joint, double torque) {
		joint.addTorque(torque);
	}


	/**
	 * Set the joint axis.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetSliderAxis (DSliderJoint j, double x, double y, double z) {
		j.setAxis(x, y, z);
	}


	/**
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 * @param ax ax
	 * @param ay ay
	 * @param az az
	 */
	//ODE_API 
	public static void dJointSetSliderAxisDelta (DSliderJoint j, double x, double y, double z, 
			double ax, double ay, double az) {
		j.setAxisDelta(x, y, z, ax, ay, az);
	}


	/**
	 * Set joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetSliderParam (DSliderJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * Applies the given force in the slider's direction.
	 *
	 * That is, it applies a force with specified magnitude, in the direction of
	 * slider's axis, to body1, and with the same magnitude but opposite
	 * direction to body2.  This function is just a wrapper for dBodyAddForce().
	 * @param joint joint
	 * @param force force
	 */
	//ODE_API 
	public static void dJointAddSliderForce(DSliderJoint joint, double force) {
		joint.addForce(force);
	}


	/**
	 * Set anchor.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetHinge2Anchor (DHinge2Joint j, double x, double y, double z) {
		j.setAnchor(x, y, z);
	}


	/**
	 * Set axis.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API
	@Deprecated // Please use setAxes instead
	public static void dJointSetHinge2Axis1 (DHinge2Joint j, double x, double y, double z) {
		j.setAxis1(x, y, z);
	}


	/**
	 * Set axis.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	@Deprecated // Please use setAxes instead
	public static void dJointSetHinge2Axis2 (DHinge2Joint j, double x, double y, double z) {
		j.setAxis2(x, y, z);
	}


	//ODE_API
	public static void dJointSetHinge2Axes (DHinge2Joint j, DVector3C axis1, DVector3C axis2) {
		j.setAxes(axis1, axis2);
	}


	/**
	 * Set joint parameter.
	 * @param j j
	 * @param parameter p
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetHinge2Param (DHinge2Joint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * Applies torque1 about the hinge2's axis 1, torque2 about the
	 * hinge2's axis 2.
	 * <p>REMARK:
	 * This function is just a wrapper for dBodyAddTorque().
	 * @param joint j
	 * @param torque1 t1 
	 * @param torque2 t2
	 */
	//ODE_API 
	public static void dJointAddHinge2Torques(DUniversalJoint joint, double torque1, double torque2) {
		joint.addTorques(torque1, torque2);
	}


	/**
	 * Set anchor.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetUniversalAnchor (DUniversalJoint j, double x, double y, double z) {
		j.setAnchor(x, y, z);
	}


	/**
	 * Set axis.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetUniversalAxis1 (DUniversalJoint j, double x, double y, double z) {
		j.setAxis1(x, y, z);
	}


	/**
	 * Set the Universal axis1 as if the 2 bodies were already at 
	 *        offset1 and offset2 appart with respect to axis1 and axis2.
	 *
	 * This function initialize the axis1 and the relative orientation of 
	 * each body as if body1 was rotated around the new axis1 by the offset1 
	 * value and as if body2 was rotated around the axis2 by offset2. \br
	 * Ex:
	* <PRE>
	 * dJointSetHuniversalAxis1(jId, 1, 0, 0);
	 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0
	 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0
	 * dJointSetHuniversalAxis1Offset(jId, 1, 0, 0, 0.2, 0.17);
	 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0.2
	 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0.17
	 * </PRE>
	 *
	 * <p>NOTE: Usually the function dJointSetHingeAxis set the current position of body1
	 *       and body2 as the zero angle position. This function set the current position
	 *       as the if the 2 bodies where \b offsets appart.
	 *
	 * <p>NOTE: Any previous offsets are erased.
	 *
	 * <p>WARNING: Calling dJointSetUniversalAnchor, dJointSetUnivesalAxis1, 
	 *          dJointSetUniversalAxis2, dJointSetUniversalAxis2Offset 
	 *          will reset the "zero" angle position.
	 *
	 * @param j The Hinge joint ID for which the axis will be set
	 * @param x The X component of the axis in world frame
	 * @param y The Y component of the axis in world frame
	 * @param z The Z component of the axis in world frame
	 * @param offset1 The angle for the offset of the relative orientation.
	 *              As if body1 was rotated by angle when the Axis was set (see below).
	 *              The rotation is around the new Hinge axis.
	 * @param offset2 offset 2
	 */
	//ODE_API 
	public static void dJointSetUniversalAxis1Offset (DUniversalJoint j, 
			double x, double y, double z, double offset1, double offset2) {
		j.setAxis1Offset(x, y, z, offset1, offset2);
	}

	/**
	 * Set axis.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetUniversalAxis2 (DUniversalJoint j, double x, double y, double z) {
		j.setAxis2(x, y, z);
	}


	/**
	 * Set the Universal axis2 as if the 2 bodies were already at 
	 *        offset1 and offset2 appart with respect to axis1 and axis2.
	 *
	 * This function initialize the axis2 and the relative orientation of 
	 * each body as if body1 was rotated around the axis1 by the offset1 
	 * value and as if body2 was rotated around the new axis2 by offset2. \br
	 * Ex:
	 * <PRE>
	 * dJointSetHuniversalAxis2(jId, 0, 1, 0);
	 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0
	 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0
	 * dJointSetHuniversalAxis2Offset(jId, 0, 1, 0, 0.2, 0.17);
	 * // If you request the position you will have: dJointGetUniversalAngle1(jId) == 0.2
	 * // If you request the position you will have: dJointGetUniversalAngle2(jId) == 0.17
	 * </PRE>
	 *
	 * <p>NOTE: Usually the function dJointSetHingeAxis set the current position of body1
	 *       and body2 as the zero angle position. This function set the current position
	 *       as the if the 2 bodies where \b offsets appart.
	 *
	 * <p>NOTE: Any previous offsets are erased.
	 *
	 * <p>WARNING: Calling dJointSetUniversalAnchor, dJointSetUnivesalAxis1, 
	 *          dJointSetUniversalAxis2, dJointSetUniversalAxis2Offset 
	 *          will reset the "zero" angle position.
	 *          
	 * @param j The Hinge joint ID for which the axis will be set
	 * @param x The X component of the axis in world frame
	 * @param y The Y component of the axis in world frame
	 * @param z The Z component of the axis in world frame
	 * @param offset1 The angle for the offset of the relative orientation.
	 *              As if body1 was rotated by angle when the Axis was set (see below).
	 *              The rotation is around the new Hinge axis.
	 * @param offset2 offset 2
	 */
	//ODE_API 
	public static void dJointSetUniversalAxis2Offset (DUniversalJoint j, 
			double x, double y, double z, double offset1, double offset2) {
		j.setAxis2Offset(x, y, z, offset1, offset2);
	}

	/**
	 * Set joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetUniversalParam (DUniversalJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * Applies torque1 about the universal's axis 1, torque2 about the
	 * universal's axis 2.
	 * <p>REMARK:
	 * This function is just a wrapper for dBodyAddTorque().
	 * @param joint j
	 * @param torque1 t1 
	 * @param torque2 t2
	 */
	//ODE_API 
	public static void dJointAddUniversalTorques(DUniversalJoint joint, 
			double torque1, double torque2) {
		joint.addTorques(torque1, torque2);
	}


	/**
	 * Set anchor.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPRAnchor (DPRJoint j, double x, double y, double z) {
		j.setAnchor(x, y, z);
	}


	/**
	 * Set the axis for the prismatic articulation.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPRAxis1 (DPRJoint j, double x, double y, double z) {
		j.setAxis1(x, y, z);
	}


	/**
	 * Set the axis for the rotoide articulation.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPRAxis2 (DPRJoint j, double x, double y, double z) {
		j.setAxis2(x, y, z);
	}


	/**
	 * Set joint parameter.
	 *
	 * <p>NOTE: parameterX where X equal 2 refer to parameter for the rotoide articulation
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetPRParam (DPRJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * Applies the torque about the rotoide axis of the PR joint.
	 *
	 * That is, it applies a torque with specified magnitude in the direction 
	 * of the rotoide axis, to body 1, and with the same magnitude but in opposite
	 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
	 * @param j j
	 * @param torque t 
	 */
	//ODE_API 
	public static void dJointAddPRTorque (DPRJoint j, double torque) {
		j.addTorque(torque);
	}



	/**
	 * Set anchor.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPUAnchor (DPUJoint j, double x, double y, double z) {
		j.setAnchor(x, y, z);
	}

//deprecated
//	/**
//	 * Set anchor.
//	 */
//	//ODE_API 
//	void dJointSetPUAnchorDelta (DJoint j, double x, double y, double z,
//			double dx, double dy, double dz) {
//		throw new UnsupportedOperationException();
//	}

	 /**
	   * Set the PU anchor as if the 2 bodies were already at [dx, dy, dz] appart.
	   *
	   * This function initialize the anchor and the relative position of each body
	   * as if the position between body1 and body2 was already the projection of [dx, dy, dz]
	   * along the Piston axis. (i.e as if the body1 was at its current position - [dx,dy,dy] when the
	   * axis is set).
	   * Ex:
	   * <PRE>
	   * dReal offset = 3;
	   * dVector3 axis;
	   * dJointGetPUAxis(jId, axis);
	   * dJointSetPUAnchor(jId, 0, 0, 0);
	   * // If you request the position you will have: dJointGetPUPosition(jId) == 0
	   * dJointSetPUAnchorOffset(jId, 0, 0, 0, axis[X]*offset, axis[Y]*offset, axis[Z]*offset);
	   * // If you request the position you will have: dJointGetPUPosition(jId) == offset
	   * </PRE>
	   * @param j The PU joint for which the anchor point will be set
	   * @param x The X position of the anchor point in world frame
	   * @param y The Y position of the anchor point in world frame
	   * @param z The Z position of the anchor point in world frame
	   * @param dx A delta to be substracted to the X position as if the anchor was set
	   *           when body1 was at current_position[X] - dx
	   * @param dy A delta to be substracted to the Y position as if the anchor was set
	   *           when body1 was at current_position[Y] - dy
	   * @param dz A delta to be substracted to the Z position as if the anchor was set
	   *           when body1 was at current_position[Z] - dz
	   */
	  //ODE_API 
	public static void dJointSetPUAnchorOffset (DPUJoint j, double x, double y, double z,
			double dx, double dy, double dz) {
		j.setAnchorOffset(x, y, z, dx, dy, dz);
	}

	  
	/**
	 * Set the axis for the first axis or the universal articulation.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPUAxis1 (DPUJoint j, double x, double y, double z) {
		j.setAxis1(x, y, z);
	}


	/**
	 * Set the axis for the second axis or the universal articulation.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPUAxis2 (DPUJoint j, double x, double y, double z) {
		j.setAxis2(x, y, z);
	}


	/**
	 * Set the axis for the prismatic articulation.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPUAxis3 (DPUJoint j, double x, double y, double z) {
		j.setAxis1(x, y, z);
	}


	/**
	 * Set the axis for the prismatic articulation.
	 * 
	 * <p>NOTE: This function was added for convenience it is the same as
	 *       dJointSetPUAxis3
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPUAxisP (DPUJoint j, double x, double y, double z) {
		j.setAxisP(x, y, z);
	}


	/**
	 * Set joint parameter.
	 *
	 * <p>NOTE: parameterX where X equal 2 refer to parameter for second axis of the
	 *       universal articulation
	 * <p>NOTE: parameterX where X equal 3 refer to parameter for prismatic
	 *       articulation
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetPUParam (DPUJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * Applies the torque about the rotoide axis of the PU joint.
	 *
	 * That is, it applies a torque with specified magnitude in the direction
	 * of the rotoide axis, to body 1, and with the same magnitude but in opposite
	 * direction to body 2. This function is just a wrapper for dBodyAddTorque()}
	 */
	//ODE_API 
	void dJointAddPUTorque (DPUJoint j, double torque) {
		//j.addTorque(torque);
		//TODO there is no implementation. Use dBodyAddTorque???? Or leave unimplemented?
	}


	/**
	 * Set the joint anchor.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPistonAnchor (DPistonJoint j, double x, double y, double z) {
		j.setAnchor(new DVector3(x, y, z));
	}

	
	/**
	 * Set the Piston anchor as if the 2 bodies were already at [dx,dy, dz] appart.
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
	 * @param dy A delta to be substracted to the Y position as if the anchor was set
	 *           when body1 was at current_position[Y] - dy
	 * @param dz A delta to be substracted to the Z position as if the anchor was set
	 *           when body1 was at current_position[Z] - dz
	 */
	//ODE_API 
	public static void dJointSetPistonAnchorOffset(DPistonJoint j, double x, double y, double z,
			double dx, double dy, double dz) {
		j.setAnchorOffset(new DVector3(x, y, z), dx, dy, dz);
	}


	/**
	 * Set the joint axis.
	 * @param j j
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetPistonAxis (DPistonJoint j, double x, double y, double z) {
		j.setAxis(new DVector3(x, y, z));
	}


//	/**
//	 * This function set prismatic axis of the joint and also set the position
//	 * of the joint.
//	 *
//	 * @param j The joint affected by this function
//	 * @param x The x component of the axis
//	 * @param y The y component of the axis
//	 * @param z The z component of the axis
//	 * @param dx The Initial position of the prismatic join in the x direction
//	 * @param dy The Initial position of the prismatic join in the y direction
//	 * @param dz The Initial position of the prismatic join in the z direction
//	 * @deprecated TZ
//	 */
//	//ODE_API_DEPRECATED ODE_API 
//	void dJointSetPistonAxisDelta (DPistonJoint j, double x, double y, double z, double ax, double ay, double az) {
//		throw new UnsupportedOperationException();
//	}


	/**
	 * Set joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetPistonParam (DPistonJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * Applies the given force in the slider's direction.
	 *
	 * That is, it applies a force with specified magnitude, in the direction of
	 * prismatic's axis, to body1, and with the same magnitude but opposite
	 * direction to body2.  This function is just a wrapper for dBodyAddForce().
	 * @param joint j
	 * @param force f
	 */
	//ODE_API 
	public static void dJointAddPistonForce (DPistonJoint joint, double force) {
		joint.addForce(force);
	}



	/**
	 * Call this on the fixed joint after it has been attached to
	 * remember the current desired relative offset and desired relative
	 * rotation between the bodies.
	 * @param j j
	 */
	//ODE_API
	public static void dJointSetFixed (DFixedJoint j) {
		j.setFixed();
	}


	/**
	 * Sets joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetFixedParam (DFixedJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * Set the nr of axes.
	 * @param j j
	 * @param num 0..3
	 */
	//ODE_API 
	public static void dJointSetAMotorNumAxes (DAMotorJoint j, int num) {
		j.setNumAxes(num);
	}


	/**
	 * Set axis.
	 * @param j j
	 * @param anum anum 
	 * @param rel r
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetAMotorAxis (DAMotorJoint j, int anum, int rel,
			double x, double y, double z) {
		j.setAxis(anum, rel, x, y, z);
	}


	/**
	 * Tell the AMotor what the current angle is along axis anum.
	 *
	 * This function should only be called in dAMotorUser mode, because in this
	 * mode the AMotor has no other way of knowing the joint angles.
	 * The angle information is needed if stops have been set along the axis,
	 * but it is not needed for axis motors.
	 * @param j j
	 * @param anum anum
	 * @param angle a
	 */
	//ODE_API 
	public static void dJointSetAMotorAngle (DAMotorJoint j, int anum, double angle) {
		j.setAngle(anum, angle);
	}


	/**
	 * Set joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetAMotorParam (DAMotorJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * Set mode.
	 * @param j j
	 * @param mode mode 
	 */
	//ODE_API 
	public static void dJointSetAMotorMode (DAMotorJoint j, int mode) {
		j.setMode(AMotorMode.from(mode));
	}


	/**
	 * Applies torque0 about the AMotor's axis 0, torque1 about the
	 * AMotor's axis 1, and torque2 about the AMotor's axis 2.
	 * 
	 * <p>REMARK:
	 * If the motor has fewer than three axes, the higher torques are ignored.
	 * This function is just a wrapper for dBodyAddTorque().
	 * @param j j
	 * @param torque1 t1 
	 * @param torque2 t2
	 * @param torque3 t3
	 */
	//ODE_API 
	public static void dJointAddAMotorTorques (DAMotorJoint j, 
			double torque1, double torque2, double torque3) {
		j.addTorques(torque1, torque2, torque3);
	}


	/**
	 * Set the number of axes that will be controlled by the LMotor.
	 * @param j j
	 * @param num can range from 0 (which effectively deactivates the joint) to 3.
	 */
	//ODE_API 
	public static void dJointSetLMotorNumAxes (DLMotorJoint j, int num) {
		j.setNumAxes(num);
	}


	/**
	 * Set the AMotor axes.
	 * @param j j
	 * @param anum selects the axis to change (0,1 or 2).
	 * @param rel Each axis can have one of three ``relative orientation'' modes
	 * <ul>
	 * <li> 0: The axis is anchored to the global frame.
	 * <li> 1: The axis is anchored to the first body.
	 * <li> 2: The axis is anchored to the second body.
	 * </ul>
 	 * <p>REMARK: The axis vector is always specified in global coordinates
	 * regardless of the setting of rel.
	 * @param x x
	 * @param y y
	 * @param z z
	 */
	//ODE_API 
	public static void dJointSetLMotorAxis (DLMotorJoint j, int anum, int rel, double x, double y, double z) {
		j.setAxis(anum, rel, x, y, z);
	}


	/**
	 * Set joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetLMotorParam (DLMotorJoint j, int parameter, double value) {
		j.setParam(PARAM_N.toEnum(parameter), value);
	}


	/**
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetPlane2DXParam (DPlane2DJoint j, int parameter, double value) {
		switch (PARAM.toEnum(parameter)) {
		case dParamFMax:
			j.setXParamFMax(value);
			break;
		case dParamVel:
			j.setXParamVel(value);
			break;
		default:
			break;
		
		}
	}


	/**
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetPlane2DYParam (DPlane2DJoint j, int parameter, double value) {
		switch (PARAM.toEnum(parameter)) {
		case dParamFMax:
			j.setYParamFMax(value);
			break;
		case dParamVel:
			j.setYParamVel(value);
			break;
		default:
			break;
		
		}
	}


	/**
	 * @param j j
	 * @param parameter p 
	 * @param value v
	 */
	//ODE_API 
	public static void dJointSetPlane2DAngleParam (DPlane2DJoint j, int parameter, double value) {
		j.setAngleParam(PARAM.toEnum(parameter), value);
	}


	/**
	 * Get the joint anchor point, in world coordinates.
	 *
	 * This returns the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetBallAnchor (DBallJoint j, DVector3 result) {
		j.getAnchor(result);
	}


	/**
	 * Get the joint anchor point, in world coordinates.
	 *
	 * This returns the point on body 2. You can think of a ball and socket
	 * joint as trying to keep the result of dJointGetBallAnchor() and
	 * dJointGetBallAnchor2() the same.  If the joint is perfectly satisfied,
	 * this function will return the same value as dJointGetBallAnchor() to
	 * within roundoff errors. dJointGetBallAnchor2() can be used, along with
	 * dJointGetBallAnchor(), to see how far the joint has come apart.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetBallAnchor2 (DBallJoint j, DVector3 result) {
		j.getAnchor2(result);
	}

	/**
	 * Get joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetBallParam (DBallJoint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}

	/**
	 * Get the target distance from double ball joint.
	 * @param j joint
	 * @return distance
	 */
	//ODE_API
	public static double dJointGetDBallDistance (DDoubleBallJoint j) {
		return j.getDistance();
	}

	/**
	 * Get the hinge anchor point, in world coordinates.
	 *
	 * This returns the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetHingeAnchor (DHingeJoint j, DVector3 result) {
		j.getAnchor(result);
	}


	/**
	 * Get the joint anchor point, in world coordinates.
	 * Returns the point on body 2. If the joint is perfectly satisfied,
	 * this will return the same value as dJointGetHingeAnchor().
	 * If not, this value will be slightly different.
	 * This can be used, for example, to see how far the joint has come apart.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetHingeAnchor2 (DHingeJoint j, DVector3 result) {
		j.getAnchor2(result);
	}


	/**
	 * Get axis
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetHingeAxis (DHingeJoint j, DVector3 result) {
		j.getAxis(result);
	}


	/**
	 * Get joint parameter
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetHingeParam (DHingeJoint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}


	/**
	 * Get the hinge angle.
	 *
	 * The angle is measured between the two bodies, or between the body and
	 * the static environment.
	 * The angle will be between -pi..pi.
	 * Give the relative rotation with respect to the Hinge axis of Body 1 with
	 * respect to Body 2.
	 * When the hinge anchor or axis is set, the current position of the attached
	 * bodies is examined and that position will be the zero angle.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetHingeAngle (DHingeJoint j) {
		return j.getAngle();
	}


	/**
	 * Get the hinge angle time derivative.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetHingeAngleRate (DHingeJoint j) {
		return j.getAngleRate();
	}


	/**
	 * Get the slider linear position (i.e. the slider's extension)
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.

	 * The position is the distance, with respect to the zero position,
	 * along the slider axis of body 1 with respect to
	 * body 2. (A NULL body is replaced by the world).
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetSliderPosition (DSliderJoint j) {
		return j.getPosition();
	}


	/**
	 * Get the slider linear position's time derivative.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetSliderPositionRate (DSliderJoint j) {
		return j.getPositionRate();
	}


	/**
	 * Get the slider axis
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetSliderAxis (DSliderJoint j, DVector3 result) {
		j.getAxis(result);
	}


	/**
	 * Get joint parameter
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetSliderParam (DSliderJoint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}


	/**
	 * Get the joint anchor point, in world coordinates.
	 * Return the point on body 1.  If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetHinge2Anchor (DHinge2Joint j, DVector3 result) {
		j.getAnchor(result);
	}


	/**
	 * Get the joint anchor point, in world coordinates.
	 * This returns the point on body 2. If the joint is perfectly satisfied,
	 * this will return the same value as dJointGetHinge2Anchor.
	 * If not, this value will be slightly different.
	 * This can be used, for example, to see how far the joint has come apart.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetHinge2Anchor2 (DHinge2Joint j, DVector3 result) {
		j.getAnchor2(result);
	}


	/**
	 * Get joint axis.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetHinge2Axis1 (DHinge2Joint j, DVector3 result) {
		j.getAxis1(result);
	}


	/**
	 * Get joint axis.
	 * @param j j
	 * @param result r
	 */
	//ODE_API 
	public static void dJointGetHinge2Axis2 (DHinge2Joint j, DVector3 result) {
		j.getAxis2(result);
	}


	/**
	 * Get joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetHinge2Param (DHinge2Joint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}


	/**
	 * Get angle.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetHinge2Angle1 (DHinge2Joint j) {
		return j.getAngle1();
	}


	/**
	 * Get time derivative of angle.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetHinge2Angle1Rate (DHinge2Joint j) {
		return j.getAngle1Rate();
	}


	/**
	 * Get time derivative of angle.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetHinge2Angle2Rate (DHinge2Joint j) {
		return j.getAngle2Rate();
	}


	/**
	 * Get the joint anchor point, in world coordinates.
	 * Return the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetUniversalAnchor (DUniversalJoint j, DVector3 result) {
		j.getAnchor(result);
	}


	/**
	 * Get the joint anchor point, in world coordinates.
	 * Return the point on body 2.
	 * 
	 * <p>REMARK:
	 * You can think of the ball and socket part of a universal joint as
	 * trying to keep the result of dJointGetBallAnchor() and
	 * dJointGetBallAnchor2() the same. If the joint is
	 * perfectly satisfied, this function will return the same value
	 * as dJointGetUniversalAnchor() to within roundoff errors.
	 * dJointGetUniversalAnchor2() can be used, along with
	 * dJointGetUniversalAnchor(), to see how far the joint has come apart.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetUniversalAnchor2 (DUniversalJoint j, DVector3 result) {
		j.getAnchor2(result);
	}


	/**
	 * Get axis.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetUniversalAxis1 (DUniversalJoint j, DVector3 result) {
		j.getAxis1(result);
	}


	/**
	 * Get axis.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetUniversalAxis2 (DUniversalJoint j, DVector3 result) {
		j.getAxis2(result);
	}



	/**
	 * Get joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetUniversalParam (DUniversalJoint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}


	/**
	 * Get both angles at the same time.
	 *
	 * <p>NOTE: This function combine getUniversalAngle1 and getUniversalAngle2 together
	 *       and try to avoid redundant calculation
	 *
	 * @param joint   The universal joint for which we want to calculate the angles
	 * @param angle1  The angle between the body1 and the axis 1
	 * @param angle2  The angle between the body2 and the axis 2
	 */
	//ODE_API 
	//	void dJointGetUniversalAngles (dJoint j, double *angle1, double *angle2);
	public static void dJointGetUniversalAngles (DUniversalJoint joint, 
			RefDouble angle1, RefDouble angle2) {
		angle1.d = joint.getAngle1();
		angle2.d = joint.getAngle2();
	}


	/**
	 * Get angle.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetUniversalAngle1 (DUniversalJoint j) {
		return j.getAngle1();
	}


	/**
	 * Get angle
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetUniversalAngle2 (DUniversalJoint j) {
		return j.getAngle2();
	}


	/**
	 * Get time derivative of angle
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetUniversalAngle1Rate (DUniversalJoint j) {
		return j.getAngle1Rate();
	}


	/**
	 * Get time derivative of angle
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetUniversalAngle2Rate (DUniversalJoint j) {
		return j.getAngle2Rate();
	}




	/**
	 * Get the joint anchor point, in world coordinates.
	 * Return the point on body 1. If the joint is perfectly satisfied, 
	 * this will be the same as the point on body 2.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPRAnchor (DPRJoint j, DVector3 result) {
		j.getAnchor(result);
	}


	/**
	 * Get the PR linear position (i.e. the prismatic's extension).
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 *
	 * The position is the "oriented" length between the
	 * position = (Prismatic axis) dot_product [(body1 + offset) - (body2 + anchor2)]
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPRPosition (DPRJoint j) {
		return j.getPosition();
	}


	/**
	 * Get the PR linear position's time derivative.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPRPositionRate (DPRJoint j) {
		return j.getPositionRate();
	}


	/**
	 * Get the PR angular position (i.e. the  twist between the 2 bodies)
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 * @param j j
	 * @return r
	 */
	//ODE_API
	public static double dJointGetPRAngle (DPRJoint j) {
		return j.getAngle();
	}

	
	/**
	 * Get the PR angular position's time derivative
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPRAngleRate (DPRJoint j) {
		return j.getAngleRate();
	}


	/**
	 * Get the prismatic axis
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPRAxis1 (DPRJoint j, DVector3 result) {
		j.getAxis1(result);
	}


	/**
	 * Get the Rotoide axis
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPRAxis2 (DPRJoint j, DVector3 result) {
		j.getAxis2(result);
	}


	/**
	 * Get joint parameter
	 * @param j j
	 * @param d_param_names dpn 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPRParam (DPRJoint j, int d_param_names) {
		return j.getParam(PARAM_N.toEnum(d_param_names));
	}



	/**
	 * Get the joint anchor point, in world coordinates.
	 * Return the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPUAnchor (DPUJoint j, DVector3 result) {
		j.getAnchor(result);
	}


	/**
	 * Get the PU linear position (i.e. the prismatic's extension)
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 *
	 * The position is the "oriented" length between the
	 * position = (Prismatic axis) dot_product [(body1 + offset) - (body2 + anchor2)]
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPUPosition (DPUJoint j) {
		return j.getPosition();
	}


	/**
	 * Get the PR linear position's time derivative
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPUPositionRate (DPUJoint j) {
		return j.getPositionRate();
	}


	/**
	 * Get the first axis of the universal component of the joint
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPUAxis1 (DPUJoint j, DVector3 result) {
		j.getAxis1(result);
	}


	/**
	 * Get the second axis of the Universal component of the joint.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPUAxis2 (DPUJoint j, DVector3 result) {
		j.getAxis2(result);
	}


	/**
	 * Get the prismatic axis
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPUAxis3 (DPUJoint j, DVector3 result) {
		j.getAxis3(result);
	}


	/**
	 * Get the prismatic axis.
	 *
	 * <p>NOTE: This function was added for convenience it is the same as
	 *       dJointGetPUAxis3
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPUAxisP (DPUJoint j, DVector3 result) {
		j.getAxisP(result);
	}





	/**
	 * Get both angles at the same time.
	 *
	 * <p>NOTE: This function combine dJointGetPUAngle1 and dJointGetPUAngle2 together
	 *       and try to avoid redundant calculation
	 *
	 * @param joint   The Prismatic universal joint for which we want to calculate the angles
	 * @param angle1  The angle between the body1 and the axis 1
	 * @param angle2  The angle between the body2 and the axis 2
	 */
	//ODE_API 
	//	  void dJointGetPUAngles (dJoint j, double *angle1, double *angle2);
	public static void dJointGetPUAngles (DPUJoint joint, RefDouble angle1, RefDouble angle2) {
		angle1.d = joint.getAngle1();
		angle2.d = joint.getAngle2();
	}


	/**
	 * Get angle.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPUAngle1 (DPUJoint j) {
		return j.getAngle1();
	}


	/**
	 * Get time derivative of angle1.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPUAngle1Rate (DPUJoint j) {
		return j.getAngle1Rate();
	}



	/**
	 * Get angle.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPUAngle2 (DPUJoint j) {
		return j.getAngle2();
	}


	/**
	 * Get time derivative of angle2.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPUAngle2Rate (DPUJoint j) {
		return j.getAngle2Rate();
	}


	/**
	 * Get joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPUParam (DPUJoint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}


	/**
	 * Get the Piston linear position (i.e. the piston's extension).
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPistonPosition (DPistonJoint j) {
		return j.getPosition();
	}


	/**
	 * Get the piston linear position's time derivative.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPistonPositionRate (DPistonJoint j) {
		return j.getPositionRate();
	}


	/**
	 * Get the Piston angular position (i.e. the  twist between the 2 bodies).
	 *
	 * When the axis is set, the current position of the attached bodies is
	 * examined and that position will be the zero position.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPistonAngle (DPistonJoint j) {
		return j.getAngle();
	}


	/**
	 * Get the piston angular position's time derivative.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPistonAngleRate (DPistonJoint j) {
		return j.getAngleRate();
	}



	/**
	 * Get the joint anchor.
	 *
	 * This returns the point on body 1. If the joint is perfectly satisfied,
	 * this will be the same as the point on body 2 in direction perpendicular
	 * to the prismatic axis.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPistonAnchor (DPistonJoint j, DVector3 result) {
		j.getAnchor(result);
	}


	/**
	 * Get the joint anchor w.r.t. body 2.
	 *
	 * This returns the point on body 2. You can think of a Piston
	 * joint as trying to keep the result of dJointGetPistonAnchor() and
	 * dJointGetPistonAnchor2() the same in the direction perpendicular to the
	 * pirsmatic axis. If the joint is perfectly satisfied,
	 * this function will return the same value as dJointGetPistonAnchor() to
	 * within roundoff errors. dJointGetPistonAnchor2() can be used, along with
	 * dJointGetPistonAnchor(), to see how far the joint has come apart.
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPistonAnchor2 (DPistonJoint j, DVector3 result) {
		j.getAnchor2(result);
	}


	/**
	 * Get the prismatic axis (This is also the rotoide axis).
	 * @param j j
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetPistonAxis (DPistonJoint j, DVector3 result) {
		j.getAxis(result);
	}


	/**
	 * Get joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetPistonParam (DPistonJoint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}



	/**
	 * Get the number of angular axes that will be controlled by the
	 * AMotor.
	 * Num can range from 0 (which effectively deactivates the joint) to 3.
	 * This is automatically set to 3 in dAMotorEuler mode.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static int dJointGetAMotorNumAxes (DAMotorJoint j) {
		return j.getNumAxes();
	}


	/**
	 * Get the AMotor axes.
	 * @param j j
	 * @param anum selects the axis to change (0,1 or 2).
	 * <ul>
	 * <li> 0: The axis is anchored to the global frame. </li>
	 * <li> 1: The axis is anchored to the first body. </li>
	 * <li> 2: The axis is anchored to the second body. </li>
	 * </ul>
	 * @param result Each axis can have one of three ``relative orientation'' modes.
	 */
	//ODE_API 
	public static void dJointGetAMotorAxis (DAMotorJoint j, int anum, DVector3 result) {
		j.getAxis(anum, result);
	}


	/**
	 * Get axis.
	 * 
	 * <p>REMARK:
	 * The axis vector is always specified in global coordinates regardless
	 * of the setting of rel.
	 * There are two GetAMotorAxis functions, one to return the axis and one to
	 * return the relative mode.
	 *
	 * For dAMotorEuler mode:
	 * <ul>
	 * <li>	Only axes 0 and 2 need to be set. Axis 1 will be determined
		automatically at each time step. </li>
	 * <li>	Axes 0 and 2 must be perpendicular to each other. </li>
	 * <li>	Axis 0 must be anchored to the first body, axis 2 must be anchored
		to the second body. </li>
	 * </ul>
	 * @param j j
	 * @param anum a 
	 * @return r
	 */
	//ODE_API 
	public static int dJointGetAMotorAxisRel (DAMotorJoint j, int anum) {
		return j.getAxisRel(anum);
	}


	/**
	 * Get the current angle for axis.
	 * 
	 * <p>REMARK:
	 * In dAMotorUser mode this is simply the value that was set with
	 * dJointSetAMotorAngle().
	 * In dAMotorEuler mode this is the corresponding euler angle.
	 * @param j j
	 * @param anum a 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetAMotorAngle (DAMotorJoint j, int anum) {
		return j.getAngle(anum);
	}


	/**
	 * Get the current angle rate for axis anum.
	 * 
	 * <p>REMARK:
	 * In dAMotorUser mode this is always zero, as not enough information is
	 * available.
	 * In dAMotorEuler mode this is the corresponding euler angle rate.
	 * @param j j
	 * @param anum a 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetAMotorAngleRate (DAMotorJoint j, int anum) {
		return j.getAngleRate(anum);
	}


	/**
	 * Get joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetAMotorParam (DAMotorJoint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}


	/**
	 * Get the angular motor mode.
	 * Mode must be one of the following constants:
	 * <ul>
	 * <li> dAMotorUser The AMotor axes and joint angle settings are entirely
	 * controlled by the user.  This is the default mode.</li>
	 * <li> dAMotorEuler Euler angles are automatically computed.
	 * The axis a1 is also automatically computed.
	 * The AMotor axes must be set correctly when in this mode,
	 * as described below.</li>
	 * </ul>
	 * When this mode is initially set the current relative orientations
	 * of the bodies will correspond to all euler angles at zero.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static AMotorMode dJointGetAMotorMode (DAMotorJoint j) {
		return j.getMode();
	}


	/**
	 * Get nr of axes.
	 * @param j j
	 * @return r
	 */
	//ODE_API 
	public static int dJointGetLMotorNumAxes (DLMotorJoint j) {
		return j.getNumAxes();
	}


	/**
	 * Get axis.
	 * @param j j
	 * @param anum a 
	 * @param result r 
	 */
	//ODE_API 
	public static void dJointGetLMotorAxis (DLMotorJoint j, int anum, DVector3 result) {
		j.getAxis(anum, result);
	}


	/**
	 * Get joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetLMotorParam (DLMotorJoint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}


	/**
	 * Get joint parameter.
	 * @param j j
	 * @param parameter p 
	 * @return r
	 */
	//ODE_API 
	public static double dJointGetFixedParam (DFixedJoint j, int parameter) {
		return j.getParam(PARAM_N.toEnum(parameter));
	}


	/**
	 * @param b1 b1
	 * @param b2 b2
	 * @return r
	 */
	//ODE_API 
	public static DJoint dConnectingJoint (DBody b1, DBody b2) {
		return OdeHelper.connectingJoint(b1, b2);
	}

	protected ApiCppJoint() {}
}
