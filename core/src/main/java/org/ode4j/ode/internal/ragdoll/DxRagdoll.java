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
package org.ode4j.ode.internal.ragdoll;

import static org.ode4j.ode.internal.Rotation.dQMultiply1;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import org.ode4j.ode.DJoint.PARAM_N;
import org.ode4j.ode.internal.joints.DxJointConstrainedBall;
import org.ode4j.ode.ragdoll.DRagdoll;
import org.ode4j.ode.ragdoll.DRagdollBoneConfig;
import org.ode4j.ode.ragdoll.DRagdollConfig;
import org.ode4j.ode.ragdoll.DRagdollJointConfig;

public class DxRagdoll implements DRagdoll {

    public static class DxRagdollBody implements DRagdoll.DRagdollBody {
        private final DBody body;
        private final double length;
        private final double radius;
        private final DVector3 position;
        private final DQuaternion quaternion;
        private DVector3[] pBuffer;
        private DQuaternion[] qBuffer;

        public DxRagdollBody(DBody body, double length, double radius, DVector3C pinitial, DQuaternionC qinitial) {
            this.body = body;
            this.length = length;
            this.radius = radius;
            this.position = new DVector3(pinitial);
            this.quaternion = new DQuaternion(qinitial.get0(), qinitial.get1(), qinitial.get2(), qinitial.get3());
        }

        public double getLength() {
            return length;
        }

        public double getRadius() {
            return radius;
        }

        public DVector3 getPosition() {
            return position;
        }

        public DQuaternion getQuaternion() {
            return quaternion;
        }

        public DBody getBody() {
            return body;
        }

    }

    private final DWorld world;
    private final DSpace space;
    private final List<DxRagdollBody> bones = new ArrayList<>(16);
    private final List<DJoint> joints = new ArrayList<>();
    private double totalMass;
    private boolean autoDisabled;
    private double autoDisableLinearAverageThreshold;
    private double autoDisableAngularAverageThreshold;
    private int autoDisableAverageSamples;
    private int autoDisableBufferIndex;
    private boolean autoDisableBufferReady;

    public static DRagdoll create(DWorld world, DSpace space, DRagdollConfig skeleton) {
        return new DxRagdoll(world, space, skeleton);
    }

    @Deprecated // This will be made private in 0.6.0. Please use OdeHelper.createRagdoll() instead
    public DxRagdoll(DWorld world, DSpace space, DRagdollConfig skeleton) {
        this.world = world;
        this.space = space;
        autoDisableLinearAverageThreshold = 0.1;
        autoDisableAngularAverageThreshold = 0.1;
        for (DRagdollBoneConfig bone : skeleton.getBones()) {
            addBody(bone.getStart(), bone.getEnd(), bone.getRadius());
        }
        adjustMass(skeleton);
        for (DRagdollJointConfig joint : skeleton.getJoints()) {
            DxRagdollBody bone1 = bones.get(joint.getBone());
            DxRagdollBody bone2 = bones.get(joint.getBone2());
            switch (joint.getType()) {
            case CONSTRAINED_BALL:
                addConstrainedBallJoint(bone1, bone2, joint.getAnchor(), joint.getAxis(), joint.getAxis2(), joint.getLimitMax(), joint.getLimitMax2());
                break;
            case FIXED:
                addFixedJoint(bone1, bone2);
                break;
            case HINGE:
                addHingeJoint(bone1, bone2, joint.getAnchor(), joint.getAxis(), joint.getLimitMin(), joint.getLimitMax());
                break;
            case UNIVERSAL:
                addUniversalJoint(bone1, bone2, joint.getAnchor(), joint.getAxis(), joint.getAxis2(), joint.getLimitMin(), joint.getLimitMax(), joint.getLimitMin2(), joint.getLimitMax2());
                break;
            }
        }
    }

    private DxRagdollBody addBody(DVector3 p1, DVector3 p2, double radius) {
        p1 = new DVector3(p1);
        p2 = new DVector3(p2);
        double cyllen = p1.distance(p2) - radius;
        DBody body = OdeHelper.createBody(world);
        DMass m = OdeHelper.createMass();
        m.setCapsule(1, 3, radius, cyllen);
        body.setMass(m);
        DCapsule geom = OdeHelper.createCapsule(space, radius, cyllen);
        geom.setBody(body);
        DVector3 za = new DVector3(p2);
        (za.sub(p1)).safeNormalize();
        DVector3 xa;
        DVector3 ya;
        if (Math.abs(za.dot(new DVector3(1.0, 0.0, 0.0))) < 0.7) {
            xa = new DVector3(1.0, 0.0, 0.0);
            ya = new DVector3();
            ya.eqCross(za, xa);
            ya.safeNormalize();
            xa.eqCross(ya, za);
        } else {
            ya = new DVector3(0.0, 1.0, 0.0);
            xa = new DVector3();
            xa.eqCross(ya, za);
            xa.safeNormalize();
            ya.eqCross(za, xa);
        }
        body.setPosition(p1.add(p2).scale(0.5));
        body.setRotation(new DMatrix3(xa.get0(), ya.get0(), za.get0(), xa.get1(), ya.get1(), za.get1(), xa.get2(), ya.get2(), za.get2()));
        totalMass += m.getMass();
        DxRagdollBody bone = new DxRagdollBody(body, cyllen, radius, body.getPosition(), body.getQuaternion());
        bones.add(bone);
        return bone;
    }

    private void adjustMass(DRagdollConfig skeleton) {
        for (DRagdollBody bone : bones) {
            DMass mass = ((DMass)bone.getBody().getMass());
            mass.adjust(bone.getBody().getMass().getMass() * skeleton.getMass() / totalMass);
            bone.getBody().setMass(mass);
        }
    }

    private DJoint addFixedJoint(DxRagdollBody body1, DxRagdollBody body2) {
        DFixedJoint joint = OdeHelper.createFixedJoint(world);
        joint.attach(body1.body, body2.body);
        joint.setFixed();
        joints.add(joint);
        return joint;
    }

    private DJoint addHingeJoint(DxRagdollBody body1, DxRagdollBody body2, DVector3 anchor, DVector3 axis, double loStop, double hiStop) {
        DHingeJoint joint = OdeHelper.createHingeJoint(world);
        joint.attach(body1.body, body2.body);
        joint.setAnchor(anchor);
        joint.setAxis(axis);
        joint.setParam(PARAM_N.dParamLoStop1, loStop);
        joint.setParam(PARAM_N.dParamHiStop1, hiStop);
        joints.add(joint);
        return joint;
    }

    private DJoint addUniversalJoint(DxRagdollBody body1, DxRagdollBody body2, DVector3 anchor, DVector3 axis1, DVector3 axis2,
            double loStop1, double hiStop1, double loStop2, double hiStop2) {
        DUniversalJoint joint = OdeHelper.createUniversalJoint(world);
        joint.attach(body1.body, body2.body);
        joint.setAnchor(anchor);
        joint.setAxis1(axis1);
        joint.setAxis2(axis2);
        joint.setParam(PARAM_N.dParamLoStop1, loStop1);
        joint.setParam(PARAM_N.dParamHiStop1, hiStop1);
        joint.setParam(PARAM_N.dParamLoStop2, loStop2);
        joint.setParam(PARAM_N.dParamHiStop2, hiStop2);
        joints.add(joint);
        return joint;
    }

    private DJoint addConstrainedBallJoint(DxRagdollBody body1, DxRagdollBody body2, DVector3 anchor, DVector3 flexAxis, DVector3 twistAxis,
            double flexLimit, double twistLimit) {
        DxJointConstrainedBall joint = new DxJointConstrainedBall(world);
        joint.attach(body1.body, body2.body);
        joint.setAnchor(anchor);
        joint.setAxes(flexAxis, twistAxis);
        joint.setLimits(flexLimit, twistLimit);
        joints.add(joint);
        return joint;
    }

    public void setAngularDamping(double damping) {
        for (DRagdollBody bone : bones) {
            bone.getBody().setAngularDamping(damping);
        }
    }

    @Deprecated // to be removed in 0.6.0. Please use getBoneIter() or getBone()
    public List<DxRagdollBody> getBones() {
        return bones;
    }

    public DRagdollBody getBone(int i) {
        return bones.get(i);
    }

    public List<DRagdollBody> getBoneIter() {
        return Collections.unmodifiableList(bones);
    }

    public List<DJoint> getJoints() {
        return joints;
    }

    public boolean isIdle() {
        return autoDisabled;
    }

    public void setAutoDisableAverageSamplesCount(int samples) {
        if (samples != autoDisableAverageSamples) {
            autoDisableAverageSamples = samples;
            for (DxRagdollBody bone : bones) {
                bone.pBuffer = new DVector3[samples];
                bone.qBuffer = new DQuaternion[samples];
                for (int i = 0; i < samples; i++) {
                    bone.pBuffer[i] = new DVector3();
                    bone.qBuffer[i] = new DQuaternion();
                }
            }
        }
        autoDisableBufferIndex = 0;
        autoDisableBufferReady = false;
    }

    public void setAutoDisableLinearAverageThreshold(double linearAverageThreshold) {
        this.autoDisableLinearAverageThreshold = linearAverageThreshold;
    }

    public void setAutoDisableAngularAverageThreshold(double angularAverageThreshold) {
        this.autoDisableAngularAverageThreshold = angularAverageThreshold;
    }

    public void autoDisable(double stepsize) {
        if (autoDisableAverageSamples < 1) {
            return;
        }
        double linearThreashold = autoDisableAverageSamples * stepsize * autoDisableLinearAverageThreshold;
        linearThreashold *= linearThreashold;
        double angularThreashold = autoDisableAverageSamples * stepsize * autoDisableAngularAverageThreshold;
        angularThreashold = Math.cos(angularThreashold);
        boolean allIdle = false;
        if (autoDisableBufferReady) {
            allIdle = true;
            for (DxRagdollBody bone : bones) {
                DBody body = bone.getBody();
                DVector3 avgPos = new DVector3();
                DQuaternion avgQuat = new DQuaternion();
                for (int i = 0; i < autoDisableAverageSamples; i++) {
                    DVector3C prevPos = bone.pBuffer[i];
                    DQuaternion prevQuat = bone.qBuffer[i];
                    avgPos.add(prevPos);
                    avgQuat.add(prevQuat);
                }
                avgPos.scale(1.0 / autoDisableAverageSamples);
                avgQuat.safeNormalize4();
                avgPos.sub(body.getPosition());
                double ldiff = avgPos.dot(avgPos);
                DQuaternion adiff = new DQuaternion();
                dQMultiply1(adiff, avgQuat, body.getQuaternion());
                if (ldiff > linearThreashold || adiff.get0() < angularThreashold) {
                    allIdle = false;
                    break;
                }
            }
        }
        autoDisabled = allIdle;
        for (DxRagdollBody bone : bones) {
            bone.pBuffer[autoDisableBufferIndex].set(bone.body.getPosition());
            bone.qBuffer[autoDisableBufferIndex].set(bone.body.getQuaternion());
            if (autoDisabled) {
                bone.body.disable();
                bone.body.setLinearVel(0, 0, 0);
                bone.body.setAngularVel(0, 0, 0);
                bone.body.setPosition(bone.position);
                bone.body.setQuaternion(bone.quaternion);
            } else {
                bone.position.set(bone.body.getPosition());
                bone.quaternion.set(bone.body.getQuaternion());
            }
        }
        autoDisableBufferIndex++;
        if (autoDisableBufferIndex == autoDisableAverageSamples) {
            autoDisableBufferIndex = 0;
            autoDisableBufferReady = true;
        }
    }

    public void destroy() {
        for (DJoint joint : joints) {
            joint.destroy();
        }
        for (DRagdollBody bone : bones) {
            bone.getBody().getFirstGeom().destroy();
            bone.getBody().destroy();
        }
    }

}
