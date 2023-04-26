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
package org.ode4j.ode.internal.trimesh;

interface IFaceAngleStorageControl {
    //    class IFaceAngleStorageControl
    //    {
    //        public:
    //        virtual void disposeStorage() = 0;
    //
    //        virtual bool areNegativeAnglesStored() const = 0;
    //
    //        // This is to store angles between neighbor triangle normals as positive value for convex and negative for concave edges
    //        virtual void assignFacesAngleIntoStorage(unsigned triangleIndex, dMeshTriangleVertex vertexIndex, dReal dAngleValue) = 0;
    //    };
    //

    void disposeStorage();
    boolean areNegativeAnglesStored();

    // This is to store angles between neighbor triangle normals as positive value for convex and negative for concave edges
    void assignFacesAngleIntoStorage(int triangleIndex, int vertexIndex, double dAngleValue);
}