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

import org.ode4j.math.DVector3;
import org.ode4j.ode.internal.Common.D_PARAM_NAMES_N;

/**
 * @defgroup joints Joints
 *
 * In real life a joint is something like a hinge, that is used to connect two
 * objects.
 * In ODE a joint is very similar: It is a relationship that is enforced between
 * two bodies so that they can only have certain positions and orientations
 * relative to each other.
 * This relationship is called a constraint -- the words joint and
 * constraint are often used interchangeably.
 * <p>
 * A joint has a set of parameters that can be set. These include:
 * <p>
 *
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
 *
 * If a particular parameter is not implemented by a given joint, setting it
 * will have no effect.
 * These parameter names can be optionally followed by a digit (2 or 3)
 * to indicate the second or third set of parameters, e.g. for the second axis
 * in a hinge-2 joint, or the third axis in an AMotor joint.
 */
public interface DJoint {

	public static class DJointFeedback {
		public DVector3 f1 = new DVector3();		/* force applied to body 1 */
		public DVector3 t1 = new DVector3();		/* torque applied to body 1 */
		public DVector3 f2 = new DVector3();		/* force applied to body 2 */
		public DVector3 t2 = new DVector3();		/* torque applied to body 2 */
	}
	
	//virtual ~dJoint() // :
	void DESTRUCTOR();

	int getNumBodies();

	void attach (DBody body1, DBody body2);

	void setData (Object data);
	Object getData();

	//public dJointType getType();

	DBody getBody (int index);

	void setFeedback(DJoint.DJointFeedback fb);
	DJoint.DJointFeedback getFeedback();

	// If not implemented it will do nothing as describe in the doc
	void setParam (D_PARAM_NAMES_N type, double value);
	double getParam (D_PARAM_NAMES_N type);

	void enable();
	void disable();
	boolean isEnabled();

	//	private:
	//		// intentionally undefined, don't use these
	//		dJoint (const dJoint &) ;
	//void operator= (const dJoint &);
	//
	//protected:
	//	dJoint _id;
	//
	//dJoint() // don't let user construct pure dJoint objects
	//{ _id = 0; }
	//
	//public:
	//	virtual ~dJoint() // :( Destructor must be virtual to suppress compiler warning "class XXX has virtual functions but non-virtual destructor"
	//	{ if (_id) dJointDestroy (_id); }
	//
	//dJoint id() const
	//{ return _id; }
	//operator dJoint() const
	//{ return _id; }
	//
	//int getNumBodies() const
	//{ return dJointGetNumBodies(_id); }
	//
	//void attach (dBody body1, dBody body2)
	//{ dJointAttach (_id, body1, body2); }
	//void attach (dBody& body1, dBody& body2)
	//{ attach(body1.id(), body2.id()); }
	//
	//void setData (void *data)
	//{ dJointSetData (_id, data); }
	//void *getData() const
	//{ return dJointGetData (_id); }
	//
	//dJointType getType() const
	//{ return dJointGetType (_id); }
	//
	//dBody getBody (int index) const
	//{ return dJointGetBody (_id, index); }
	//
	//void setFeedback(dJointFeedback *fb)
	//{ dJointSetFeedback(_id, fb); }
	//dJointFeedback *getFeedback() const
	//{ return dJointGetFeedback(_id); }
	//
	//// If not implemented it will do nothing as describe in the doc
	//virtual void setParam (int, dReal) {};
	//virtual dReal getParam (int) const { return 0; }
}

