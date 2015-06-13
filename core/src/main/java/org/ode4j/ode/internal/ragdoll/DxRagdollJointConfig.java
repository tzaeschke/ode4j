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

import org.ode4j.math.DVector3;
import org.ode4j.ode.ragdoll.DRagdollJointConfig;

public class DxRagdollJointConfig implements DRagdollJointConfig {

    private final JointType type;
    private final int bone;
    private final int bone2;
    private final DVector3 anchor;
    private final DVector3 axis;
    private final DVector3 axis2;
    private final double limitMin;
    private final double limitMax;
    private final double limitMin2;
    private final double limitMax2;

    public DxRagdollJointConfig(JointType type, int bone, int bone2, DVector3 anchor, DVector3 axis, DVector3 axis2, double limitMin,
            double limitMax, double limitMin2, double limitMax2) {
        this.type = type;
        this.bone = bone;
        this.bone2 = bone2;
        this.anchor = anchor;
        this.axis = axis;
        this.axis2 = axis2;
        this.limitMin = limitMin;
        this.limitMax = limitMax;
        this.limitMin2 = limitMin2;
        this.limitMax2 = limitMax2;
    }

    @Override
    public JointType getType() {
        return type;
    }

    @Override
    public int getBone() {
        return bone;
    }

    @Override
    public int getBone2() {
        return bone2;
    }

    @Override
    public DVector3 getAnchor() {
        return anchor;
    }

    @Override
    public DVector3 getAxis() {
        return axis;
    }

    @Override
    public DVector3 getAxis2() {
        return axis2;
    }

    @Override
    public double getLimitMin() {
        return limitMin;
    }

    @Override
    public double getLimitMax() {
        return limitMax;
    }

    @Override
    public double getLimitMin2() {
        return limitMin2;
    }

    @Override
    public double getLimitMax2() {
        return limitMax2;
    }
}
