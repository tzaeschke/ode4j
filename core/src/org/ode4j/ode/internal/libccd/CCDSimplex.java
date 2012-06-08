/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2007-2012 Tilmann Zäschke <ode4j@gmx.de>  
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

import static org.ode4j.ode.internal.libccd.CCDSupport.*;

public class CCDSimplex {


	static class ccd_simplex_t {
		private final ccd_support_t[] ps = new ccd_support_t[4];
		int last; //!< index of last added point
		{
			ps[0] = new ccd_support_t();
			ps[1] = new ccd_support_t();
			ps[2] = new ccd_support_t();
			ps[3] = new ccd_support_t();
		}
	};


	/**** INLINES ****/

	static final void ccdSimplexInit(ccd_simplex_t s)
	{
		s.last = -1;
	}

	static final int ccdSimplexSize(final ccd_simplex_t s)
	{
		return s.last + 1;
	}

	static final ccd_support_t ccdSimplexLast(final ccd_simplex_t s)
	{
		return ccdSimplexPoint(s, s.last);
	}

	static final ccd_support_t ccdSimplexPoint(final ccd_simplex_t s, int idx)
	{
		// here is no check on boundaries
		return s.ps[idx];
	}
	static final ccd_support_t ccdSimplexPointW(ccd_simplex_t s, int idx)
	{
		return s.ps[idx];
	}

	static final void ccdSimplexAdd(ccd_simplex_t s, final ccd_support_t v)
	{
		// here is no check on boundaries in sake of speed
		++s.last;
		ccdSupportCopy(s.ps[s.last], v);
	}

	static final void ccdSimplexSet(ccd_simplex_t s, int pos, final ccd_support_t a)
	{
		ccdSupportCopy(s.ps[pos], a);
	}

	static final void ccdSimplexSetSize(ccd_simplex_t s, int size)
	{
		s.last = size - 1;
	}

	static final void ccdSimplexSwap(ccd_simplex_t s, int pos1, int pos2)
	{
		ccd_support_t supp = new ccd_support_t();

		ccdSupportCopy(supp, s.ps[pos1]);
		ccdSupportCopy(s.ps[pos1], s.ps[pos2]);
		ccdSupportCopy(s.ps[pos2], supp);
	}

}