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
package org.ode4j.ode.ragdoll;

import org.ode4j.math.*;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.internal.ragdoll.DxRagdoll;

import java.util.List;

/**
 * ode4j has generic support for rag dolls (or any other entities made of multiple bodies connected with joints).
 * The patch adds a new type of joint - DxJointConstrainedBall, it is an extension to the regular ball and socket
 * joint and additionally provides a way to constrain the movement as described at
 * <a href="http://www.monsterden.net/software/ragdoll-pyode-tutorial">
 *     http://www.monsterden.net/software/ragdoll-pyode-tutorial</a>.
 * It is useful to implement more realistic shoulders of a human rag doll for instance.
 * DxRagdoll class builds the actual rag doll based on the given list of "bones" and "joints".
 * Additionally it provides a simple autoDisable method that can be used to freeze the whole rag doll
 * when the movement of the bodies is minimal. Usually rag doll bodies tend to oscillate due to the high
 * number of hard constraints and disabling only some of them does not work. Additionally, there is a demo
 * with an example of a humanoid rag doll based on the tutorial mentioned above.
 * The demo is extremely simple, you can only press space to apply some force.
 */
public interface DRagdoll {

    interface DRagdollBody {
        double getLength();

        double getRadius();

        DVector3C getPosition();

        DQuaternionC getQuaternion();

        DBody getBody();
    }

    void setAngularDamping(double damping);

    @Deprecated // to be removed in 0.6.0. Please use getBoneIter()
    List<DxRagdoll.DxRagdollBody> getBones();

    DRagdollBody getBone(int i);
    List<DRagdollBody> getBoneIter();

    List<DJoint> getJoints();

    boolean isIdle();

    void setAutoDisableAverageSamplesCount(int samples);

    void setAutoDisableLinearAverageThreshold(double linearAverageThreshold);

    void setAutoDisableAngularAverageThreshold(double angularAverageThreshold);

    void autoDisable(double stepsize);

    void destroy();

}
