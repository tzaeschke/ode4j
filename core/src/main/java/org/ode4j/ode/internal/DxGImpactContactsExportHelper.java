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
import org.ode4j.ode.DContactGeomBuffer;

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dNextAfter;
import static org.ode4j.ode.internal.DxGeom.NUMC_MASK;

class DxGImpactContactsExportHelper {
    interface GImpactContactAccessorI {
        double RetrieveDepthByIndex(int index);

        void ExportContactGeomByIndex(DContactGeom pcontact, int index);
    }


    //#ifndef _ODE_GIMPACT_CONTACT_EXPORT_HELPER_H_
    //#define _ODE_GIMPACT_CONTACT_EXPORT_HELPER_H_
    //
    //    #ifndef ALLOCA
    //    #define ALLOCA(x) dALLOCA16(x)
    //    #endif


    //    struct dxGImpactContactsExportHelper
    //    {
    //        public:
    //        template<class dxGImpactContactAccessor>
    //        static unsigned ExportMaxDepthGImpactContacts(dxGImpactContactAccessor &srccontacts, unsigned contactcount,
    //        int Flags, dContactGeom* Contacts, int Stride)
    public static int ExportMaxDepthGImpactContacts(GImpactContactAccessorI srccontacts, int contactcount,
                                                    int Flags, DContactGeomBuffer Contacts, int Stride) {
        int result;

        int maxcontacts = Flags & NUMC_MASK;
        if (contactcount > maxcontacts) {
            ExportExcesssiveContacts(srccontacts, contactcount, Flags, Contacts, Stride);
            result = maxcontacts;
        } else {
            ExportFitContacts(srccontacts, contactcount, Flags, Contacts, Stride);
            result = contactcount;
        }

        return result;
    }

    //private:
    //        template<class dxGImpactContactAccessor>
    //        static void ExportExcesssiveContacts(dxGImpactContactAccessor &srccontacts, unsigned contactcount,
    //        int Flags, dContactGeom* Contacts, int Stride);
    //        template<class dxGImpactContactAccessor>
    //        static void ExportFitContacts(dxGImpactContactAccessor &srccontacts, unsigned contactcount,
    //        int Flags, dContactGeom* Contacts, int Stride);
    //        template<class dxGImpactContactAccessor>
    //        static dReal FindContactsMarginalDepth(dxGImpactContactAccessor &srccontacts, unsigned contactcount, unsigned maxcontacts);
    //        static dReal FindContactsMarginalDepth(dReal *pdepths, unsigned contactcount, unsigned maxcontacts, dReal mindepth, dReal maxdepth);
    //    };


    //    template<class dxGImpactContactAccessor>
    //    /*static */
    //    void dxGImpactContactsExportHelper::ExportExcesssiveContacts(dxGImpactContactAccessor &srccontacts, unsigned contactcount,
    //                                                                 int Flags, dContactGeom* Contacts, int Stride)
    /*static */
    private static void ExportExcesssiveContacts(GImpactContactAccessorI srccontacts, int contactcount,
                                                 int Flags, DContactGeomBuffer Contacts, int Stride) {
        int maxcontacts = Flags & NUMC_MASK;
        double marginaldepth = FindContactsMarginalDepth(srccontacts, contactcount, maxcontacts);

        int contactshead = 0, contacttail = maxcontacts;
        for (int i = 0; i < contactcount; i++) {
            double depth = srccontacts.RetrieveDepthByIndex(i);

            if (depth > marginaldepth) {
                DContactGeom pcontact = Contacts.getSafe(Flags, contactshead);//, Stride);
                srccontacts.ExportContactGeomByIndex(pcontact, i);

                if (++contactshead == maxcontacts) {
                    break;
                }
            } else if (depth == marginaldepth && contactshead < contacttail) {
                --contacttail;

                DContactGeom pcontact = Contacts.getSafe(Flags, contacttail);//, Stride);
                srccontacts.ExportContactGeomByIndex(pcontact, i);
            }
        }
    }

    //    template<class dxGImpactContactAccessor>
    //    /*static */
    //    void dxGImpactContactsExportHelper::ExportFitContacts(dxGImpactContactAccessor &srccontacts, unsigned contactcount,
    //                                                          int Flags, dContactGeom* Contacts, int Stride)
    /*static */
    private static void ExportFitContacts(GImpactContactAccessorI srccontacts, int contactcount,
                                          int Flags, DContactGeomBuffer Contacts, int Stride) {
        for (int i = 0; i < contactcount; i++) {
            DContactGeom pcontact = Contacts.getSafe(Flags, i);//, Stride);

            srccontacts.ExportContactGeomByIndex(pcontact, i);
        }
    }

    //    template<class dxGImpactContactAccessor>
    //    /*static */
    //    dReal dxGImpactContactsExportHelper::FindContactsMarginalDepth(dxGImpactContactAccessor &srccontacts, unsigned contactcount, unsigned maxcontacts)
    /*static */
    private static double FindContactsMarginalDepth(GImpactContactAccessorI srccontacts, int contactcount, int maxcontacts) {
        double result;

        double[] pdepths = new double[contactcount]; //(dReal *)ALLOCA(contactcount * sizeof(dReal));
        int marginindex = 0;
        int highindex = marginindex;

        double firstdepth = srccontacts.RetrieveDepthByIndex(0);
        double mindepth = firstdepth, maxdepth = firstdepth;
        dIASSERT(contactcount > 1);

        for (int i = 1; i < contactcount; i++) {
            double depth = srccontacts.RetrieveDepthByIndex(i);

            if (depth < firstdepth) {
                double temp = pdepths[marginindex];
                pdepths[highindex++] = temp;
                pdepths[marginindex++] = depth;
                if (depth < mindepth) {
                    mindepth = depth;
                }
            } else if (depth > firstdepth) {
                pdepths[highindex++] = depth;
                if (maxdepth < depth) {
                    maxdepth = depth;
                }
            }
        }

        int countabove = highindex - marginindex;
        if (maxcontacts < countabove) {
            result = FindContactsMarginalDepth(pdepths, marginindex, countabove, maxcontacts, firstdepth, maxdepth);
        } else if (maxcontacts == countabove) {
            result = dNextAfter(firstdepth, dInfinity);
        } else {
            int countbelow = marginindex;
            if (maxcontacts <= contactcount - countbelow) {
                result = firstdepth;
            } else {
                result = FindContactsMarginalDepth(pdepths, 0, countbelow, maxcontacts - (contactcount - countbelow), mindepth, firstdepth);
            }
        }

        return result;
    }

    /*static */
    //dReal dxGImpactContactsExportHelper::FindContactsMarginalDepth(dReal *pdepths, unsigned contactcount, unsigned maxcontacts, dReal mindepth, dReal maxdepth)
    private static double FindContactsMarginalDepth(double[] pdepthsA, int pdepthsP, int contactcount, int maxcontacts, double mindepth, double maxdepth) {
        double result;

        while (true) {
            double firstdepth = 0.5 * (mindepth + maxdepth);
            double lowdepth = maxdepth, highdepth = mindepth;

            int marginindex = 0;
            int highindex = marginindex;
            dIASSERT(contactcount != 0);

            for (int i = 0; i < contactcount; i++) {
                double depth = pdepthsA[pdepthsP + i];

                if (depth < firstdepth) {
                    double temp = pdepthsA[pdepthsP + marginindex];
                    pdepthsA[pdepthsP + highindex++] = temp;
                    pdepthsA[pdepthsP + marginindex++] = depth;
                    if (highdepth < depth) {
                        highdepth = depth;
                    }
                } else if (depth > firstdepth) {
                    pdepthsA[pdepthsP + highindex++] = depth;
                    if (depth < lowdepth) {
                        lowdepth = depth;
                    }
                }
            }

            int countabove = highindex - marginindex;
            if (maxcontacts < countabove) {
                contactcount = countabove;
                pdepthsP += marginindex;
                mindepth = lowdepth;
            } else if (maxcontacts == countabove) {
                result = dNextAfter(firstdepth, dInfinity);
                break;
            } else {
                int countbelow = marginindex;
                if (maxcontacts <= contactcount - countbelow) {
                    result = firstdepth;
                    break;
                }

                maxcontacts -= contactcount - countbelow;
                contactcount = countbelow;
                maxdepth = highdepth;
            }
        }

        return result;
    }

    private DxGImpactContactsExportHelper() {}
}
