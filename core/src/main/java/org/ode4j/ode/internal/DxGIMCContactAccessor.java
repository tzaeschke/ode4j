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

import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.internal.gimpact.GimContact;

public class DxGIMCContactAccessor implements DxGImpactContactsExportHelper.GImpactContactAccessorI {
    // #ifndef _ODE_GIMPACT_GIM_CONTACT_ACCESSOR_H_
    // #define _ODE_GIMPACT_GIM_CONTACT_ACCESSOR_H_

    //    struct dxGIMCContactAccessor
    //    {
    //        dxGIMCContactAccessor(GIM_CONTACT *ptrimeshcontacts, dGeomID g1, dGeomID g2) : m_ptrimeshcontacts(ptrimeshcontacts), m_g1(g1), m_g2(g2), m_gotside2ovr(false), m_side2ovr() {}
    public DxGIMCContactAccessor(GimContact[] ptrimeshcontacts, DxGeom g1, DxGeom g2) {
        m_ptrimeshcontacts = ptrimeshcontacts;
        m_g1 = g1;
        m_g2 = g2;
        m_gotside2ovr = false;
        m_side2ovr = 0;
    }


    //        dxGIMCContactAccessor(GIM_CONTACT *ptrimeshcontacts, dGeomID g1, dGeomID g2, int side2ovr) : m_ptrimeshcontacts(ptrimeshcontacts), m_g1(g1), m_g2(g2), m_gotside2ovr(true), m_side2ovr(side2ovr) {}
    public DxGIMCContactAccessor(GimContact[] ptrimeshcontacts, DxGeom g1, DxGeom g2, int side2ovr) {
        m_ptrimeshcontacts = ptrimeshcontacts;
        m_g1 = g1;
        m_g2 = g2;
        m_gotside2ovr = true;
        m_side2ovr = side2ovr;
    }


    public double RetrieveDepthByIndex(int index) {
        return m_ptrimeshcontacts[index].getDepth();
    }

    public void ExportContactGeomByIndex(DContactGeom pcontact, int index) {
        final GimContact ptrimeshcontact = m_ptrimeshcontacts[index];
        //        pcontact.pos[0] = ptrimeshcontact.m_point[0];
        //        pcontact.pos[1] = ptrimeshcontact.m_point[1];
        //        pcontact.pos[2] = ptrimeshcontact.m_point[2];
        //        pcontact.pos[3] = 1.0;
        pcontact.pos.set0(ptrimeshcontact.getPoint().f[0]);
        pcontact.pos.set1(ptrimeshcontact.getPoint().f[1]);
        pcontact.pos.set2(ptrimeshcontact.getPoint().f[2]);
        pcontact.pos3(1.0); //pos[3] = 1.0;

        pcontact.normal.set0(ptrimeshcontact.getNormal().f[0]);
        pcontact.normal.set1(ptrimeshcontact.getNormal().f[1]);
        pcontact.normal.set2(ptrimeshcontact.getNormal().f[2]);
        pcontact.normal3(0); //normal[3] = 0.0;

        pcontact.depth = ptrimeshcontact.getDepth();
        pcontact.g1 = m_g1;
        pcontact.g2 = m_g2;
        pcontact.side1 = ptrimeshcontact.getFeature1();
        pcontact.side2 = !m_gotside2ovr ? ptrimeshcontact.getFeature2() : m_side2ovr;
    }

    private final GimContact[] m_ptrimeshcontacts;
    private final DxGeom m_g1;
    private final DxGeom m_g2;
    private final boolean m_gotside2ovr;
    private final int m_side2ovr;
//    };

}
