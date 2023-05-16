/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
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

import static org.ode4j.ode.internal.libccd.CCDSupport.*;

/**
 *
 * LibCCD simplex class.
 */
public class CCDSimplex {


	static class ccd_simplex_t {
//		private final ccd_support_t[] ps = new ccd_support_t[4];
		int last; //!< index of last added point
//		{
//			ps[0] = new ccd_support_t();
//			ps[1] = new ccd_support_t();
//			ps[2] = new ccd_support_t();
//			ps[3] = new ccd_support_t();
//		}
		private final ccd_support_t ps0 = new ccd_support_t();
		private final ccd_support_t ps1 = new ccd_support_t();
		private final ccd_support_t ps2 = new ccd_support_t();
		private final ccd_support_t ps3 = new ccd_support_t();
	};


	/**** INLINES ****/

	static void ccdSimplexInit(ccd_simplex_t s)
	{
		s.last = -1;
	}

	static int ccdSimplexSize(final ccd_simplex_t s)
	{
		return s.last + 1;
	}

	static ccd_support_t ccdSimplexLast(final ccd_simplex_t s)
	{
		//return ccdSimplexPoint(s, s.last);
		switch (s.last){
		case 0: return ccdSimplexPoint0(s);
		case 1: return ccdSimplexPoint1(s);
		case 2: return ccdSimplexPoint2(s);
		case 3: return ccdSimplexPoint3(s);
		}
		throw new IllegalArgumentException();
	}

//	static final ccd_support_t ccdSimplexPoint(final ccd_simplex_t s, int idx)
//	{
//		// here is no check on boundaries
//		return s.ps[idx];
//	}
	static ccd_support_t ccdSimplexPoint0(final ccd_simplex_t s) {
		return s.ps0;
	}
	static ccd_support_t ccdSimplexPoint1(final ccd_simplex_t s) {
		return s.ps1;
	}
	static ccd_support_t ccdSimplexPoint2(final ccd_simplex_t s) {
		return s.ps2;
	}
	static ccd_support_t ccdSimplexPoint3(final ccd_simplex_t s) {
		return s.ps3;
	}
//	static final ccd_support_t ccdSimplexPointW(ccd_simplex_t s, int idx)
//	{
//		return s.ps[idx];
//	}
	static ccd_support_t ccdSimplexPointW0(final ccd_simplex_t s) {
		return s.ps0;
	}
	static ccd_support_t ccdSimplexPointW1(final ccd_simplex_t s) {
		return s.ps1;
	}
	static ccd_support_t ccdSimplexPointW2(final ccd_simplex_t s) {
		return s.ps2;
	}
	static ccd_support_t ccdSimplexPointW3(final ccd_simplex_t s) {
		return s.ps3;
	}

	static void ccdSimplexAdd(ccd_simplex_t s, final ccd_support_t v)
	{
		// here is no check on boundaries in sake of speed
		++s.last;
		//ccdSupportCopy(s.ps[s.last], v);
		switch(s.last){
		case 0: ccdSupportCopy(s.ps0, v); break;
		case 1: ccdSupportCopy(s.ps1, v); break;
		case 2: ccdSupportCopy(s.ps2, v); break;
		case 3: ccdSupportCopy(s.ps3, v); break;
		default: throw new IllegalArgumentException();
		}
	}

//	static final void ccdSimplexSet(ccd_simplex_t s, int pos, final ccd_support_t a)
//	{
//		ccdSupportCopy(s.ps[pos], a);
//	}
	static void ccdSimplexSet0(ccd_simplex_t s, final ccd_support_t a) {
		ccdSupportCopy(s.ps0, a);
	}
	static void ccdSimplexSet1(ccd_simplex_t s, final ccd_support_t a) {
		ccdSupportCopy(s.ps1, a);
	}
	static void ccdSimplexSet2(ccd_simplex_t s, final ccd_support_t a) {
		ccdSupportCopy(s.ps2, a);
	}
	static void ccdSimplexSet3(ccd_simplex_t s, final ccd_support_t a) {
		ccdSupportCopy(s.ps3, a);
	}

	static void ccdSimplexSetSize(ccd_simplex_t s, int size)
	{
		s.last = size - 1;
	}

//	static final void ccdSimplexSwap(ccd_simplex_t s, int pos1, int pos2)
//	{
//		ccd_support_t supp = new ccd_support_t();
//
//		ccdSupportCopy(supp, s.ps[pos1]);
//		ccdSupportCopy(s.ps[pos1], s.ps[pos2]);
//		ccdSupportCopy(s.ps[pos2], supp);
//	}
	/**
	 * swaps position 1 and 2.
	 */
	static void ccdSimplexSwap12(ccd_simplex_t s)
	{
		ccd_support_t supp = new ccd_support_t();

		ccdSupportCopy(supp, s.ps1);
		ccdSupportCopy(s.ps1, s.ps2);
		ccdSupportCopy(s.ps2, supp);
	}

	private CCDSimplex() {}
}