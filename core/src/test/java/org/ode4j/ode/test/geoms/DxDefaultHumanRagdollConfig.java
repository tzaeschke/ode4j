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
package org.ode4j.ode.test.geoms;

import org.ode4j.math.DVector3;
import org.ode4j.ode.internal.ragdoll.DxRagdollBoneConfig;
import org.ode4j.ode.internal.ragdoll.DxRagdollJointConfig;
import org.ode4j.ode.ragdoll.DRagdollBoneConfig;
import org.ode4j.ode.ragdoll.DRagdollConfig;
import org.ode4j.ode.ragdoll.DRagdollJointConfig;
import org.ode4j.ode.ragdoll.DRagdollJointConfig.JointType;

/**
 * Human rag doll configuration based on http://www.monsterden.net/software/ragdoll-pyode-tutorial with tiny fixes.  
 */
public class DxDefaultHumanRagdollConfig implements DRagdollConfig {

    public final static int CHEST = 0;
    public final static int BELLY = 1;
    public final static int PELVIS = 2;
    public final static int HEAD = 3;
    public final static int RIGHT_UPPER_LEG = 4;
    public final static int RIGHT_LOWER_LEG = 5;
    public final static int RIGHT_FOOT = 6;
    public final static int LEFT_UPPER_LEG = 7;
    public final static int LEFT_LOWER_LEG = 8;
    public final static int LEFT_FOOT = 9;
    public final static int RIGHT_UPPER_ARM = 10;
    public final static int RIGHT_FORE_ARM = 11;
    public final static int RIGHT_HAND = 12;
    public final static int LEFT_UPPER_ARM = 13;
    public final static int LEFT_FORE_ARM = 14;
    public final static int LEFT_HAND = 15;

    public final static int MID_SPINE = 0;
    public final static int LOW_SPINE = 1;
    public final static int NECK = 2;
    public final static int RIGHT_HIP = 3;
    public final static int RIGHT_KNEE = 4;
    public final static int RIGHT_ANKLE = 5;
    public final static int LEFT_HIP = 6;
    public final static int LEFT_KNEE = 7;
    public final static int LEFT_ANKLE = 8;
    public final static int RIGHT_SHOULDER = 9;
    public final static int RIGHT_ELBOW = 10;
    public final static int RIGHT_WRIST = 11;
    public final static int LEFT_SHOULDER = 12;
    public final static int LEFT_ELBOW = 13;
    public final static int LEFT_WRIST = 14;

    public double mass = 80;
    public double thickness = 1.0;
    public double upperArmLen = 0.30;
    public double foreArmLen = 0.25;
    public double handLen = 0.13; // wrist to mid-fingers only
    public double footLen = 0.18; // ankles to base of ball of foot only
    public double heelLen = 0.05; // ankles to base of ball of foot only
    public double chestW = 0.36; // actually wider, but we want narrower than shoulders (esp. with large radius)
    public double chestH = 1.35;
    public double chestRadius = 0.13;
    public double bellyO = 0.1;
    public double bellyR = 0.125;
    public double hipH = 0.86;
    public double hipR = 0.125;
    public double pelvisW = 0.25; // actually wider, but we want smaller than hip width
    public double pelvisR = 0.125;
    public double browH = 1.68;
    public double mouthH = 1.53;
    public double headO = 0.03;
    public double headR = 0.11;
    public double legW = 0.28; // between middles of upper legs
    public double kneeH = 0.48;
    public double ankleH = 0.08;
    public double upperLegR = 0.11;
    public double lowerLegR = 0.09;
    public double footR = 0.09;
    public double shoulderW = 0.41;
    public double shoulderH = 1.37;
    public double upperArmR = 0.08;
    public double foreArmR = 0.075;
    public double handR = 0.075;
    public double neckH = 1.5;
    public final DVector3 offset = new DVector3(0, 0, 0);
    public final DVector3 scale = new DVector3(1, 1, 1);

    @Override
    public double getMass() {
        return mass;
    }

    @Override
    public DRagdollBoneConfig[] getBones() {
        DRagdollBoneConfig[] bones = new DRagdollBoneConfig[LEFT_HAND + 1];
        bones[CHEST] = new DxRagdollBoneConfig(getRightChestPos(), getLeftChestPos(), getChestRadius());
        bones[BELLY] = new DxRagdollBoneConfig(getBellyTopPos(), getBellyBottomPos(), getBellyRadius());
        bones[PELVIS] = new DxRagdollBoneConfig(getRightPelvisPos(), getLeftPelvisPos(), getPelvisRadius());
        bones[HEAD] = new DxRagdollBoneConfig(getHeadTopPos(), getHeadBottomPos(), getHeadRadius());
        DVector3 rHipPos = getRightHipPos();
        DVector3 rKneePos = getRightKneePos();
        DVector3 rAnklePos = getRightAnklePos();
        bones[RIGHT_UPPER_LEG] = new DxRagdollBoneConfig(rHipPos, rKneePos, getUpperLegRadius());
        bones[RIGHT_LOWER_LEG] = new DxRagdollBoneConfig(rKneePos, rAnklePos, getLowerLegRadius());
        bones[RIGHT_FOOT] = new DxRagdollBoneConfig(getRightHeelPos(), getRightToesPos(), getFootRadius());
        DVector3 lHipPos = getLeftHipPos();
        DVector3 lKneePos = getLeftKneePos();
        DVector3 lAnklePos = getLeftAnklePos();
        bones[LEFT_UPPER_LEG] = new DxRagdollBoneConfig(lHipPos, lKneePos, getUpperLegRadius());
        bones[LEFT_LOWER_LEG] = new DxRagdollBoneConfig(lKneePos, lAnklePos, getLowerLegRadius());
        bones[LEFT_FOOT] = new DxRagdollBoneConfig(getLeftHeelPos(), getLeftToesPos(), getFootRadius());
        DVector3 rShoulderPos = getRightShoulderPos();
        DVector3 rElbowPos = getRightElbowPos();
        DVector3 rWristPos = getRightWristPos();
        bones[RIGHT_UPPER_ARM] = new DxRagdollBoneConfig(rShoulderPos, rElbowPos, getUpperArmRadius());
        bones[RIGHT_FORE_ARM] = new DxRagdollBoneConfig(rElbowPos, rWristPos, getForeArmRadius());
        bones[RIGHT_HAND] = new DxRagdollBoneConfig(rWristPos, getRightFingersPos(), getHandRadius());
        DVector3 lShoulderPos = getLeftShoulderPos();
        DVector3 lElbowPos = getLeftElbowPos();
        DVector3 lWristPos = getLeftWristPos();
        bones[LEFT_UPPER_ARM] = new DxRagdollBoneConfig(lShoulderPos, lElbowPos, getUpperArmRadius());
        bones[LEFT_FORE_ARM] = new DxRagdollBoneConfig(lElbowPos, lWristPos, getForeArmRadius());
        bones[LEFT_HAND] = new DxRagdollBoneConfig(lWristPos, getLeftFingersPos(), getHandRadius());
        return bones;
    }

    private final static DVector3 rightAxis = new DVector3(1.0, 0.0, 0.0);
    private final static DVector3 leftAxis = new DVector3(-1.0, 0.0, 0.0);
    private final static DVector3 upAxis = new DVector3(0.0, 1.0, 0.0);
    private final static DVector3 downAxis = new DVector3(0.0, -1.0, 0.0);
    private final static DVector3 bkwdAxis = new DVector3(0.0, 0.0, 1.0);
    private final static DVector3 fwdAxis = new DVector3(0.0, 0.0, -1.0);

    @Override
    public DRagdollJointConfig[] getJoints() {
        DRagdollJointConfig[] joints = new DRagdollJointConfig[LEFT_WRIST + 1];
        joints[MID_SPINE] = new DxRagdollJointConfig(JointType.FIXED, CHEST, BELLY, null, null, null, 0, 0, 0, 0);
        joints[LOW_SPINE] = new DxRagdollJointConfig(JointType.FIXED, BELLY, PELVIS, null, null, null, 0, 0, 0, 0);
        joints[NECK] = new DxRagdollJointConfig(JointType.CONSTRAINED_BALL, CHEST, HEAD, getNeckPos(), upAxis, upAxis, -Math.PI * 0.25,
                Math.PI * 0.25, -Math.PI * 0.3, Math.PI * 0.3);
        joints[RIGHT_HIP] = new DxRagdollJointConfig(JointType.UNIVERSAL, PELVIS, RIGHT_UPPER_LEG, getRightHipPos(), bkwdAxis, rightAxis,
                -Math.PI * 0.1, Math.PI * 0.3, -Math.PI * 0.15, Math.PI * 0.5);
        joints[RIGHT_KNEE] = new DxRagdollJointConfig(JointType.HINGE, RIGHT_UPPER_LEG, RIGHT_LOWER_LEG, getRightKneePos(), leftAxis, null,
                0.0, Math.PI * 0.75, 0.0, 0.0);
        joints[RIGHT_ANKLE] = new DxRagdollJointConfig(JointType.HINGE, RIGHT_LOWER_LEG, RIGHT_FOOT, getRightAnklePos(), rightAxis, null,
                -Math.PI * 0.1, Math.PI * 0.05, 0.0, 0.0);
        joints[LEFT_HIP] = new DxRagdollJointConfig(JointType.UNIVERSAL, PELVIS, LEFT_UPPER_LEG, getLeftHipPos(), fwdAxis, rightAxis,
                -Math.PI * 0.1, Math.PI * 0.3, -Math.PI * 0.15, Math.PI * 0.5);
        joints[LEFT_KNEE] = new DxRagdollJointConfig(JointType.HINGE, LEFT_UPPER_LEG, LEFT_LOWER_LEG, getLeftKneePos(), leftAxis, null, 0.0,
                Math.PI * 0.75, 0.0, 0.0);
        joints[LEFT_ANKLE] = new DxRagdollJointConfig(JointType.HINGE, LEFT_LOWER_LEG, LEFT_FOOT, getLeftAnklePos(), rightAxis, null,
                -Math.PI * 0.1, Math.PI * 0.05, 0.0, 0.0);

        DVector3 axis = new DVector3(-1.0, -1.0, 4.0);
        axis.normalize();
        joints[RIGHT_SHOULDER] = new DxRagdollJointConfig(JointType.CONSTRAINED_BALL, CHEST, RIGHT_UPPER_ARM, getRightShoulderPos(), axis,
                leftAxis, -Math.PI * 0.45, Math.PI * 0.45, -Math.PI * 0.25, Math.PI * 0.25);
        joints[RIGHT_ELBOW] = new DxRagdollJointConfig(JointType.HINGE, RIGHT_UPPER_ARM, RIGHT_FORE_ARM, getRightElbowPos(), downAxis, null,
                0.0, Math.PI * 0.6, 0.0, 0.0);
        joints[RIGHT_WRIST] = new DxRagdollJointConfig(JointType.HINGE, RIGHT_FORE_ARM, RIGHT_HAND, getRightWristPos(), fwdAxis, null,
                -Math.PI * 0.1, Math.PI * 0.2, 0.0, 0.0);
        axis = new DVector3(1.0, -1.0, 4.0);
        axis.normalize();
        joints[LEFT_SHOULDER] = new DxRagdollJointConfig(JointType.CONSTRAINED_BALL, CHEST, LEFT_UPPER_ARM, getLeftShoulderPos(), axis,
                rightAxis, -Math.PI * 0.45, Math.PI * 0.45, -Math.PI * 0.25, Math.PI * 0.25);
        joints[LEFT_ELBOW] = new DxRagdollJointConfig(JointType.HINGE, LEFT_UPPER_ARM, LEFT_FORE_ARM, getLeftElbowPos(), upAxis, null, 0.0,
                Math.PI * 0.6, 0.0, 0.0);
        joints[LEFT_WRIST] = new DxRagdollJointConfig(JointType.HINGE, LEFT_FORE_ARM, LEFT_HAND, getLeftWristPos(), bkwdAxis, null,
                -Math.PI * 0.1, Math.PI * 0.2, 0.0, 0.0);
        return joints;
    }

    public DVector3 getRightChestPos() {
        return scaleAndOffset(new DVector3(-chestW * 0.5, chestH, 0.0));
    }

    public DVector3 getLeftChestPos() {
        return scaleAndOffset(new DVector3(chestW * 0.5, chestH, 0.0));
    }

    public DVector3 getBellyTopPos() {
        return scaleAndOffset(new DVector3(0.0, chestH - bellyO, 0.0));
    }

    public DVector3 getBellyBottomPos() {
        return scaleAndOffset(new DVector3(0.0, hipH + bellyO, 0.0));
    }

    public DVector3 getRightPelvisPos() {
        return scaleAndOffset(new DVector3(-pelvisW * 0.5, hipH, 0.0));
    }

    public DVector3 getLeftPelvisPos() {
        return scaleAndOffset(new DVector3(pelvisW * 0.5, hipH, 0.0));
    }

    public DVector3 getHeadTopPos() {
        return scaleAndOffset(new DVector3(0.0, browH, headO));
    }

    public DVector3 getHeadBottomPos() {
        return scaleAndOffset(new DVector3(0.0, mouthH, headO));
    }

    public DVector3 getRightHipPos() {
        return scaleAndOffset(new DVector3(-legW * 0.5, hipH, 0.0));
    }

    public DVector3 getRightKneePos() {
        return scaleAndOffset(new DVector3(-legW * 0.5, kneeH, 0.0));
    }

    public DVector3 getRightAnklePos() {
        return scaleAndOffset(new DVector3(-legW * 0.5, ankleH, 0.0));
    }

    public DVector3 getRightHeelPos() {
        return scaleAndOffset(new DVector3(-legW * 0.5, ankleH, -heelLen));
    }

    public DVector3 getRightToesPos() {
        return scaleAndOffset(new DVector3(-legW * 0.5, ankleH, footLen));
    }

    public DVector3 getLeftHipPos() {
        return scaleAndOffset(new DVector3(legW * 0.5, hipH, 0.0));
    }

    public DVector3 getLeftKneePos() {
        return scaleAndOffset(new DVector3(legW * 0.5, kneeH, 0.0));
    }

    public DVector3 getLeftAnklePos() {
        return scaleAndOffset(new DVector3(legW * 0.5, ankleH, 0.0));
    }

    public DVector3 getLeftHeelPos() {
        return scaleAndOffset(new DVector3(legW * 0.5, ankleH, -heelLen));
    }

    public DVector3 getLeftToesPos() {
        return scaleAndOffset(new DVector3(legW * 0.5, ankleH, footLen));
    }

    public DVector3 getNeckPos() {
        return scaleAndOffset(new DVector3(0.0, neckH, 0.0));
    }

    public DVector3 getRightShoulderPos() {
        return scaleAndOffset(new DVector3(-shoulderW * 0.5, shoulderH, 0.0));
    }

    public DVector3 getRightElbowPos() {
        return scaleAndOffset(new DVector3(-shoulderW * 0.5 - upperArmLen, shoulderH, 0.0));
    }

    public DVector3 getRightWristPos() {
        return scaleAndOffset(new DVector3(-shoulderW * 0.5 - upperArmLen - foreArmLen, shoulderH, 0.0));
    }

    public DVector3 getRightFingersPos() {
        return scaleAndOffset(new DVector3(-shoulderW * 0.5 - upperArmLen - foreArmLen - handLen, shoulderH, 0.0));
    }

    public DVector3 getLeftShoulderPos() {
        return scaleAndOffset(new DVector3(shoulderW * 0.5, shoulderH, 0.0));
    }

    public DVector3 getLeftElbowPos() {
        return scaleAndOffset(new DVector3(shoulderW * 0.5 + upperArmLen, shoulderH, 0.0));
    }

    public DVector3 getLeftWristPos() {
        return scaleAndOffset(new DVector3(shoulderW * 0.5 + upperArmLen + foreArmLen, shoulderH, 0.0));
    }

    public DVector3 getLeftFingersPos() {
        return scaleAndOffset(new DVector3(shoulderW * 0.5 + upperArmLen + foreArmLen + handLen, shoulderH, 0.0));
    }

    private DVector3 scaleAndOffset(DVector3 anchor) {
        return new DVector3(anchor).scale(scale).add(offset);
    }

    public double getChestRadius() {
        return chestRadius * thickness;
    }

    public double getBellyRadius() {
        return bellyR * thickness;
    }

    public double getPelvisRadius() {
        return pelvisR * thickness;
    }

    public double getHeadRadius() {
        return headR * thickness;
    }

    public double getUpperLegRadius() {
        return upperLegR * thickness;
    }

    public double getLowerLegRadius() {
        return lowerLegR * thickness;
    }

    public double getFootRadius() {
        return footR * thickness;
    }

    public double getUpperArmRadius() {
        return upperArmR * thickness;
    }

    public double getHandRadius() {
        return handR * thickness;
    }

    public double getForeArmRadius() {
        return foreArmR * thickness;
    }

}