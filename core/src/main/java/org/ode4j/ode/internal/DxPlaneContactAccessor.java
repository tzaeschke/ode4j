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
package org.ode4j.ode.internal;

import org.ode4j.math.DVector4;
import org.ode4j.math.DVector4C;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.internal.cpp4j.java.ObjArray;
import org.ode4j.ode.internal.gimpact.GimGeometry;

public class DxPlaneContactAccessor implements DxGImpactContactsExportHelper.GImpactContactAccessorI {
    //#ifndef _ODE_GIMPACT_PLANE_CONTACT_ACCESSOR_H_
    //#define _ODE_GIMPACT_PLANE_CONTACT_ACCESSOR_H_

    //    struct dxPlaneContactAccessor
    //    {
    //        dxPlaneContactAccessor(const vec4f *planecontact_results, const dReal *plane, dGeomID g1, dGeomID g2) : m_planecontact_results(planecontact_results), m_plane(plane), m_g1(g1), m_g2(g2) {}
    public DxPlaneContactAccessor(final GimGeometry.vec4f[] planecontact_results, final DVector4C plane, DxGeom g1, DxGeom g2) {
        m_planecontact_results = planecontact_results;
        m_plane = plane;
        m_g1 = g1;
        m_g2 = g2;
    }

    //dReal RetrieveDepthByIndex(unsigned index) const { return m_planecontact_results[index][3]; }
    public double RetrieveDepthByIndex(int index) {
        return m_planecontact_results[index].f[3];
    }

    public void ExportContactGeomByIndex(DContactGeom pcontact, int index) {
        final GimGeometry.vec4f planecontact = m_planecontact_results[index];

        pcontact.pos.set(planecontact.f[0], planecontact.f[1], planecontact.f[2]);
        pcontact.pos3(1.0); //pos[3] = 1.0;

        final DVector4C plane = m_plane;
        pcontact.normal.set(plane.get0(), plane.get1(), plane.get2());
        pcontact.normal3(0); //normal[3] = 0;

        pcontact.depth = planecontact.f[3];
        pcontact.g1 = m_g1; // trimesh geom
        pcontact.g2 = m_g2; // plane geom
        pcontact.side1 = -1; // note: don't have the triangle index, but OPCODE *does* do this properly
        pcontact.side2 = -1;
    }

    final GimGeometry.vec4f[] m_planecontact_results;
    final DVector4C m_plane;
    DxGeom m_g1, m_g2;
}
