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

import java.util.List;

import org.ode4j.ode.DBody;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.OdeHelper;

public abstract class ApiCppOther extends ApiCppMass {
	
	/**
	 * @param b1 b1
	 * @param b2 b2
	 * @return ret
	 */
	//ODE_API 
//	int dConnectingJointList (dBody b1, dBody b2, dJoint*);
	public static List<DJoint> dConnectingJointList (DBody b1, DBody b2) {
		return OdeHelper.connectingJointList(b1, b2);
	}

	/**
	 * Utility function.
	 * @param b1 b1
	 * @param b2 b2
	 * @return 1 if the two bodies are connected together by
	 * a joint, otherwise return 0.
	 */
	//ODE_API 
	public static boolean dAreConnected (DBody b1, DBody b2) {
		return OdeHelper.areConnected(b1, b2);
	}

	/**
	 * Utility function.
	 * @return 1 if the two bodies are connected together by
	 * a joint that does not have type {@code joint_type}, otherwise return 0.
	 * @param body1 A body to check.
	 * @param body2 A body to check.
	 * @param joint_type is a dJointTypeXXX constant.
	 * This is useful for deciding whether to add contact joints between two bodies:
	 * if they are already connected by non-contact joints then it may not be
	 * appropriate to add contacts, however it is okay to add more contact between-
	 * bodies that already have contacts.
	 */
	//ODE_API 
	@SafeVarargs
	@SuppressWarnings("varargs")
	public static boolean dAreConnectedExcluding (DBody body1, DBody body2,
												  Class<? extends DJoint> ... joint_type) {
		return OdeHelper.areConnectedExcluding(body1, body2, joint_type);
	}
	/**
	 * Utility function.
	 * @return 1 if the two bodies are connected together by
	 * a joint that does not have type {@code joint_type}, otherwise return 0.
	 * @param body1 A body to check.
	 * @param body2 A body to check.
	 * @param joint_type is a dJointTypeXXX constant.
	 * This is useful for deciding whether to add contact joints between two bodies:
	 * if they are already connected by non-contact joints then it may not be
	 * appropriate to add contacts, however it is okay to add more contact between-
	 * bodies that already have contacts.
	 */
	public static boolean dAreConnectedExcluding (DBody body1, DBody body2, 
			Class<? extends DJoint> joint_type) {
		return OdeHelper.areConnectedExcluding(body1, body2, joint_type);
	}

	protected ApiCppOther() {}
}
