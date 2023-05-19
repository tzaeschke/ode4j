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

import org.ode4j.math.DVector3;

/**
 * In real life a joint is something like a hinge, that is used to connect two
 * objects.
 * In ODE a joint is very similar: It is a relationship that is enforced between
 * two bodies so that they can only have certain positions and orientations
 * relative to each other.
 * This relationship is called a constraint -- the words joint and
 * constraint are often used interchangeably.
 * <p>
 * A joint has a set of parameters that can be set. These include:
 * 
 *<ul>
 * <li>  dParamLoStop Low stop angle or position. Setting this to
 *	-dInfinity (the default value) turns off the low stop.
 *	For rotational joints, this stop must be greater than -pi to be
 *	effective.
 * <li>  dParamHiStop High stop angle or position. Setting this to
 *	dInfinity (the default value) turns off the high stop.
 *	For rotational joints, this stop must be less than pi to be
 *	effective.
 *	If the high stop is less than the low stop then both stops will
 *	be ineffective.
 * <li>  dParamVel Desired motor velocity (this will be an angular or
 *	linear velocity).
 * <li>  dParamFMax The maximum force or torque that the motor will use to
 *	achieve the desired velocity.
 *	This must always be greater than or equal to zero.
 *	Setting this to zero (the default value) turns off the motor.
 * <li>  dParamFudgeFactor The current joint stop/motor implementation has
 *	a small problem:
 *	when the joint is at one stop and the motor is set to move it away
 *	from the stop, too much force may be applied for one time step,
 *	causing a ``jumping'' motion.
 *	This fudge factor is used to scale this excess force.
 *	It should have a value between zero and one (the default value).
 *	If the jumping motion is too visible in a joint, the value can be
 *	reduced.
 *	Making this value too small can prevent the motor from being able to
 *	move the joint away from a stop.
 * <li>  dParamBounce The bouncyness of the stops.
 *	This is a restitution parameter in the range 0..1.
 *	0 means the stops are not bouncy at all, 1 means maximum bouncyness.
 * <li>  dParamCFM The constraint force mixing (CFM) value used when not
 *	at a stop.
 * <li>  dParamStopERP The error reduction parameter (ERP) used by the
 *	stops.
 * <li>  dParamStopCFM The constraint force mixing (CFM) value used by the
 *	stops. Together with the ERP value this can be used to get spongy or
 *	soft stops.
 *	Note that this is intended for unpowered joints, it does not really
 *	work as expected when a powered joint reaches its limit.
 * <li>  dParamSuspensionERP Suspension error reduction parameter (ERP).
 *	Currently this is only implemented on the hinge-2 joint.
 * <li>  dParamSuspensionCFM Suspension constraint force mixing (CFM) value.
 *	Currently this is only implemented on the hinge-2 joint.
 * </ul>
 *
 * If a particular parameter is not implemented by a given joint, setting it
 * will have no effect.
 * These parameter names can be optionally followed by a digit (2 or 3)
 * to indicate the second or third set of parameters, e.g. for the second axis
 * in a hinge-2 joint, or the third axis in an AMotor joint.
 */
public interface DJoint {

	class DJointFeedback {
		/** force applied to body 1 */
		public DVector3 f1 = new DVector3();		
		/** torque applied to body 1 */
		public DVector3 t1 = new DVector3();		
		/** force applied to body 2 */
		public DVector3 f2 = new DVector3();		
		/** torque applied to body 2 */
		public DVector3 t2 = new DVector3();

		public DJointFeedback() {}
	}

	//virtual ~dJoint() // :
	void DESTRUCTOR();

//	/**
//	 * Destroy a joint.
//	 *
//	 * disconnects it from its attached bodies and removing it from the world.
//	 * However, if the joint is a member of a group then this function has no
//	 * effect - to destroy that joint the group must be emptied or destroyed.
//	 */
//	void destroy();
	
	/**
	 * Return the number of bodies attached to the joint.
	 * @return number of bodies
	 */
	int getNumBodies();

	
	/**
	 * Attach the joint to some new bodies.
	 * <p>
	 * If the joint is already attached, it will be detached from the old bodies
	 * first.
	 * To attach this joint to only one body, set body1 or body2 to zero - a zero
	 * body refers to the static environment.
	 * Setting both bodies to zero puts the joint into "limbo", i.e. it will
	 * have no effect on the simulation.
	 * 
	 * <p>REMARK:
	 * Some joints, like hinge-2 need to be attached to two bodies to work.
	 * @param body1 body 1
	 * @param body2 body 2
	 */
	void attach (DBody body1, DBody body2);
	

	/**
	 * Set the user-data pointer.
	 * @param data User data
	 */
	void setData (Object data);
	
	
	/**
	 * Get the user-data pointer.
	 * @return user data
	 */
	Object getData();

	//public dJointType getType();

	/**
	 * Return the bodies that this joint connects.
	 * 
	 * <p> REMARK:
	 * If one of these returned body IDs is zero, the joint connects the other body
	 * to the static environment.
	 * If both body IDs are zero, the joint is in ``limbo'' and has no effect on
	 * the simulation.
	 * 
	 * @param index return the first (0) or second (1) body.
	 * @return Body at connection 'index'
	 */
	DBody getBody (int index);
	

	/**
	 * Sets the datastructure that is to receive the feedback.
	 * <p>
	 * The feedback can be used by the user, so that it is known how
	 * much force an individual joint exerts.
	 * @param fb Joint feedback
	 */
	void setFeedback(DJoint.DJointFeedback fb);
	
	
	/**
	 * Gets the datastructure that is to receive the feedback.
	 * @return Joint feedback
	 */
	DJoint.DJointFeedback getFeedback();

	/** If not implemented it will do nothing as describe in the doc. 
	 * @param type Parameter selector
	 * @param value value 
	 */
	void setParam (PARAM_N type, double value);
	
	
	/**
	 * Get joint parameter.
	 * <p>
	 * If not implemented it will do nothing as describe in the doc. 
	 * @param type parameter selector
	 * @return value
	 */
	double getParam (PARAM_N type);

	
	/**
	 * Manually enable a joint.
	 */
	void enable();
	
	
	/**
	 * Manually disable a joint.
	 * <p> REMARK: 
	 * A disabled joint will not affect the simulation, but will maintain the anchors and
	 * axes so it can be enabled later.
	 */
	void disable();
	
	
	/**
	 * Check whether a joint is enabled.
	 * @return 1 if a joint is currently enabled or 0 if it is disabled.
	 */
	boolean isEnabled();

	
	//
	//// If not implemented it will do nothing as describe in the doc
	//virtual void setParam (int, dReal) {};
	//virtual dReal getParam (int) const { return 0; }

	static final int P_OFS_1 = 0x000;
	static final int P_OFS_2 = 0x100;
	static final int P_OFS_3 = 0x200;

	public enum PARAM {
		//		dParamGroup(0),
		//	  /* parameters for limits and motors */ \
		dParamLoStop(0), 
		dParamHiStop(1), 
		dParamVel(2), 
		dParamLoVel(3),
		dParamHiVel(4),
		dParamFMax (5), 
		dParamFudgeFactor (6), 
		dParamBounce (7), 
		dParamCFM (8), 
		dParamStopERP (9), 
		dParamStopCFM (10), 
		/* parameters for suspension */ 
		dParamSuspensionERP (11), 
		dParamSuspensionCFM(12),
		dParamERP(13);
		//		public static int START = 0x000; 
		private final int _x;
		private PARAM(int x) {
			_x = x;
		}

		public PARAM and(int i) {
			int n = _x & i;
			for (PARAM param: values()) {
				if (param._x == n) {
					return param;
				}
			}
			throw new IllegalArgumentException(name() + "->"+ _x + " & " + i + " = n");
		}

		public static PARAM toEnum(int n) {
			for (PARAM param: values()) {
				if (param._x == n) {
					return param;
				}
			}
			throw new IllegalArgumentException("n = " + n);
		}
	}

	public enum PARAM_N {
		//		dParamGroup(0),
		//		//	  /* parameters for limits and motors */ \
		//		dParamLoStop(0), 
		//		dParamHiStop(1), 
		//		dParamVel(2), 
		//		dParamFMax (3), 
		//		dParamFudgeFactor (4), 
		//		dParamBounce (5), 
		//		dParamCFM (6), 
		//		dParamStopERP (7), 
		//		dParamStopCFM (8), 
		//		/* parameters for suspension */ 
		//		dParamSuspensionERP (9), 
		//		dParamSuspensionCFM(10),
		//		dParamERP(11),

		//		dParamGroup1(0, P_OFS_1),
		//	  /* parameters for limits and motors */ \
		dParamLoStop1(0, P_OFS_1), 
		dParamHiStop1(1, P_OFS_1), 
		dParamVel1(2, P_OFS_1), 
		dParamLoVel(3, P_OFS_1),
		dParamHiVel(4, P_OFS_1),
		dParamFMax1 (5, P_OFS_1), 
		dParamFudgeFactor1 (6, P_OFS_1), 
		dParamBounce1 (7, P_OFS_1), 
		dParamCFM1 (8, P_OFS_1), 
		dParamStopERP1 (9, P_OFS_1), 
		dParamStopCFM1 (10, P_OFS_1), 
		/* parameters for suspension */ 
		dParamSuspensionERP1 (11, P_OFS_1), 
		dParamSuspensionCFM1(12, P_OFS_1),
		dParamERP1(13, P_OFS_1),

		//		dParamGroup2(0, P_OFS_2),
		//	  /* parameters for limits and motors */ \
		dParamLoStop2(0, P_OFS_2), 
		dParamHiStop2(1, P_OFS_2), 
		dParamVel2(2, P_OFS_2), 
		dParamLoVel2(3, P_OFS_2),
		dParamHiVel2(4, P_OFS_2),
		dParamFMax2 (5, P_OFS_2), 
		dParamFudgeFactor2 (6, P_OFS_2), 
		dParamBounce2 (7, P_OFS_2), 
		dParamCFM2 (8, P_OFS_2), 
		dParamStopERP2 (9, P_OFS_2), 
		dParamStopCFM2 (10, P_OFS_2), 
		/* parameters for suspension */ 
		dParamSuspensionERP2 (11, P_OFS_2), 
		dParamSuspensionCFM2(12, P_OFS_2),
		dParamERP2(13, P_OFS_2),

		//		dParamGroup3(0, P_OFS_3),
		//	  /* parameters for limits and motors */ \
		dParamLoStop3(0, P_OFS_3), 
		dParamHiStop3(1, P_OFS_3), 
		dParamVel3(2, P_OFS_3), 
		dParamLoVel3(3, P_OFS_3),
		dParamHiVel3(4, P_OFS_3),
		dParamFMax3 (5, P_OFS_3), 
		dParamFudgeFactor3 (6, P_OFS_3), 
		dParamBounce3 (7, P_OFS_3), 
		dParamCFM3 (8, P_OFS_3), 
		dParamStopERP3 (9, P_OFS_3), 
		dParamStopCFM3 (10, P_OFS_3), 
		/* parameters for suspension */ 
		dParamSuspensionERP3 (11, P_OFS_3), 
		dParamSuspensionCFM3(12, P_OFS_3),
		dParamERP3(13, P_OFS_3);
		//		public static int START = 0x000; 
		private final int _x;
		private final PARAM_GROUPS _group;
		private final PARAM _sub;
		private PARAM_N(int x, int g) {
			_x = x + g;

			switch (g) {
			case P_OFS_1: _group = PARAM_GROUPS.dParamGroup1; break;
			case P_OFS_2: _group = PARAM_GROUPS.dParamGroup2; break;
			case P_OFS_3: _group = PARAM_GROUPS.dParamGroup3; break;
			default: throw new IllegalArgumentException(name() + " g=" + g);
			}

			_sub = PARAM.toEnum(x);			
		}

		public PARAM_GROUPS toGROUP() {
			return _group;
		}

		public PARAM toSUB() {
			return _sub;
		}

		public boolean isGroup1() {
			return _group == PARAM_GROUPS.dParamGroup1;
		}

		public boolean isGroup2() {
			return _group == PARAM_GROUPS.dParamGroup2;
		}

		public boolean isGroup3() {
			return _group == PARAM_GROUPS.dParamGroup3;
		}

		public static PARAM_N toEnum(int n) {
			for (PARAM_N param: values()) {
				if (param._x == n) {
					return param;
				}
			}
			throw new IllegalArgumentException("n = " + n);
		}
	}

	public enum PARAM_GROUPS {

		dParamGroup1(P_OFS_1, 0),
		dParamGroup2(P_OFS_2, 1),
		dParamGroup3(P_OFS_3, 2);

		private final int _index;
		private PARAM_GROUPS(int x, int index) {
			_index = index;
		}

		public int getIndex() {
			return _index;
		}
	}

	/* transmission joint mode numbers */

	public enum TRANSMISSION {
	  dTransmissionParallelAxes, // = 0,
	  dTransmissionIntersectingAxes, // = 1,
	  dTransmissionChainDrive, // = 2
	}

	void destroy();

}