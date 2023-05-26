/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010,2011 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2009-2014 Tilmann Zaeschke<ode4j@gmx.de>  
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */
package org.ode4j.ode.internal.libccd;

import static org.ode4j.ode.internal.libccd.CCDVec3.*;

public class CCDCustomVec3 {

    // # define CCD_ATAN2(x, y) (atan2((x), (y))) /*!< atan2 of two floats */
    public static double CCD_ATAN2(double x, double y) {
        return Math.atan2(x, y);
    }

    /**
     * d = v + w
     * @param d d
     * @param v v
     * @param w w
     */
    public static void ccdVec3Add2(ccd_vec3_t d, final ccd_vec3_t v, final ccd_vec3_t w) {
//#ifndef dLIBCCD_USE_SYSTEM
//			d->v[0] = v->v[0] + w->v[0];
//			d->v[1] = v->v[1] + w->v[1];
//			d->v[2] = v->v[2] + w->v[2];
//#else
        ccdVec3Copy(d, v);
        ccdVec3Add(d, w);
//#endif
    }

    /**
     * d = s * k;
     * @param d d
     * @param s s
     * @param k k
     */
    public static void ccdVec3CopyScaled(ccd_vec3_t d, final ccd_vec3_t s, double k) {
//#ifndef dLIBCCD_USE_SYSTEM
//			d->v[0] = s->v[0] * k;
//			d->v[1] = s->v[1] * k;
//			d->v[2] = s->v[2] * k;
//#else
        ccdVec3Copy(d, s);
        ccdVec3Scale(d, k);
//#endif
    }

    /**
     * d = v + s * k;
     * @param d d
     * @param v v
     * @param s s
     * @param k k
     */
    public static void ccdVec3AddScaled(ccd_vec3_t d, final ccd_vec3_t v, final ccd_vec3_t s, double k) {
//#ifndef dLIBCCD_USE_SYSTEM
//			d->v[0] = v->v[0] + s->v[0] * k;
//			d->v[1] = v->v[1] + s->v[1] * k;
//			d->v[2] = v->v[2] + s->v[2] * k;
//#else
        ccdVec3Copy(d, s);
        ccdVec3Scale(d, k);
        ccdVec3Add(d, v);
//#endif
    }

    /**
     * Normalizes given vector to unit length.
     * @param d d
     * @return 0 for success or -1 for error
     */
    public static int ccdVec3SafeNormalize(ccd_vec3_t d) {
        int result = -1;

        double len = CCD_SQRT(ccdVec3Len2(d));
        if (len >= CCD_EPS) {
            ccdVec3Scale(d, CCD_ONE / len);
            result = 0;
        }

        return result;
    }

    private CCDCustomVec3() {}
}
