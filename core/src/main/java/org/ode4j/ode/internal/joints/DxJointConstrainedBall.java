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
package org.ode4j.ode.internal.joints;

import static org.ode4j.ode.OdeMath.dCalcVectorCross3;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Rotation.dRFromAxisAndAngle;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DConstrainedBallJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.DxWorld;

/**
 * Ball and socket joint with constraints as described here
 * <a href="http://www.monsterden.net/software/ragdoll-pyode-tutorial">
 *     http://www.monsterden.net/software/ragdoll-pyode-tutorial</a>
 */
public class DxJointConstrainedBall extends DxJointBall implements DConstrainedBallJoint {

	private final DVector3 baseAxis;
	private final DVector3 body2Axis;
	private final DVector3 twistUpAxis1;
	private final DVector3 twistUpAxis2;
	private final DVector3 axis1;
	private final DVector3 axis2;
	private final DxJointLimitMotor limotFlex;
	private final DxJointLimitMotor limotTwist;

	public DxJointConstrainedBall(DWorld w) {
		super((DxWorld) w);
		baseAxis = new DVector3();
		body2Axis = new DVector3();
		twistUpAxis1 = new DVector3();
		twistUpAxis2 = new DVector3();
		limotFlex = new DxJointLimitMotor();
		limotFlex.init((DxWorld) w);
		limotTwist = new DxJointLimitMotor();
		limotTwist.init((DxWorld) w);
		axis1 = new DVector3();
		axis2 = new DVector3();
	}

	@Override
	public void getInfo1(DxJoint.Info1 info) {
		info.setNub(3);
		info.setM(3);
		boolean limitingFlex = (limotFlex.lostop >= -M_PI || limotFlex.histop <= M_PI)
				&& limotFlex.lostop <= limotFlex.histop;
		boolean limitingTwist = (limotTwist.lostop >= -M_PI || limotTwist.histop <= M_PI)
				&& limotTwist.lostop <= limotTwist.histop;
		DVector3 flex = new DVector3();
		getAxis(flex, baseAxis);
		DVector3 twist = new DVector3();
		getAxis2(twist, body2Axis);
		dCalcVectorCross3(axis1, twist, flex);
		axis1.safeNormalize();
		double angle = limitingFlex ? Math.acos(flex.dot(twist)) : 0;
		if (limotFlex.testRotationalLimit(angle)) {
			info.incM();
		}
		if (limitingTwist) {
			DVector3 twistUpB1 = new DVector3();
			getAxis(twistUpB1, twistUpAxis1);
			DVector3 twistUpB2 = new DVector3();
			getAxis2(twistUpB2, twistUpAxis2);
			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle(R, axis1, angle);
			DVector3 projected = new DVector3();
			OdeMath.dMultiply0_133(projected, twistUpB1, R);
			double angle2 = Math.acos(projected.dot(twistUpB2));
			if (limotTwist.testRotationalLimit(angle2)) {
				dCalcVectorCross3(axis2, twistUpB2, projected);
				axis2.safeNormalize();
				info.incM();
			}
		} else {
			limotTwist.testRotationalLimit(0.0);
		}
	}


	/**
	 * @param baseAxis - base axis for constraints
	 * @param body2Axis - main axis of the 2nd body
	 */
	@Override
	public void setAxes(DVector3C baseAxis, DVector3C body2Axis) {
		DVector3 baseAx = baseAxis.copy();
		DVector3 body2Ax = body2Axis.copy();

		baseAx.safeNormalize();
		body2Ax.safeNormalize();
		DVector3 twistUpAxis = new DVector3();
		if (Math.abs(body2Ax.dot(new DVector3(0.0, 1.0, 0.0))) < 0.7) {
			twistUpAxis.set(0, 1, 0);
		} else {
			twistUpAxis.set(0, 0, 1);
		}
		DVector3 cross = new DVector3();
		cross.eqCross(body2Ax, twistUpAxis);
		cross.safeNormalize();
		twistUpAxis.eqCross(cross, body2Ax);
		twistUpAxis.safeNormalize();
		setAxes(baseAx, this.baseAxis, null);
		setAxes(body2Ax, null, this.body2Axis);
		setAxes(twistUpAxis, twistUpAxis1, twistUpAxis2);

	}

	@Override
	public DVector3C getBaseAxis() {
		return baseAxis;
	}

	@Override
	public DVector3C getSecondBodyAxis() {
		return body2Axis;
	}

	@Override
	public void setLimits(double flexLimit, double twistLimit) {
		limotFlex.set(PARAM.dParamLoStop, -flexLimit);
		limotFlex.set(PARAM.dParamHiStop, flexLimit);
		limotTwist.set(PARAM.dParamLoStop, -twistLimit);
		limotTwist.set(PARAM.dParamHiStop, twistLimit);
	}

	@Override
	public double getFlexLimit() {
		return limotFlex.get(PARAM.dParamHiStop);
	}

	@Override
	public double getTwistLimit() {
		return limotTwist.get(PARAM.dParamHiStop);
	}
}
