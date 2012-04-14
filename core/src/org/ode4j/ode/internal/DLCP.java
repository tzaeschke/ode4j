/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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


import static org.cpp4j.Cstdio.*;
import static org.ode4j.ode.OdeMath.*;

import org.ode4j.math.DMatrixN;
import org.ode4j.ode.OdeConfig;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.DStopwatch;
import org.ode4j.ode.internal.DxUtil.BlockPointer;
import org.ode4j.ode.internal.DxWorldProcessMemArena.DxStateSave;



/**

given (A,b,lo,hi), solve the LCP problem: A*x = b+w, where each x(i),w(i)
satisfies one of
	(1) x = lo, w >= 0
	(2) x = hi, w <= 0
	(3) lo < x < hi, w = 0
A is a matrix of dimension n*n, everything else is a vector of size n*1.
lo and hi can be +/- dInfinity as needed. the first `nub' variables are
unbounded, i.e. hi and lo are assumed to be +/- dInfinity.

we restrict lo(i) <= 0 and hi(i) >= 0.

the original data (A,b) may be modified by this function.

if the `findex' (friction index) parameter is nonzero, it points to an array
of index values. in this case constraints that have findex[i] >= 0 are
special. all non-special constraints are solved for, then the lo and hi values
for the special constraints are set:
  hi[i] = abs( hi[i] * x[findex[i]] )
  lo[i] = -hi[i]
and the solution continues. this mechanism allows a friction approximation
to be implemented. the first `nub' variables are assumed to have findex < 0.



THE ALGORITHM
-------------

solve A*x = b+w, with x and w subject to certain LCP conditions.
each x(i),w(i) must lie on one of the three line segments in the following
diagram. each line segment corresponds to one index set :

     w(i)
     /|\      |           :
      |       |           :
      |       |i in N     :
  w>0 |       |state[i]=0 :
      |       |           :
      |       |           :  i in C
  w=0 +       +-----------------------+
      |                   :           |
      |                   :           |
  w<0 |                   :           |i in N
      |                   :           |state[i]=1
      |                   :           |
      |                   :           |
      +-------|-----------|-----------|----------> x(i)
             lo           0           hi

the Dantzig algorithm proceeds as follows:
  for i=1:n
 * if (x(i),w(i)) is not on the line, push x(i) and w(i) positive or
      negative towards the line. as this is done, the other (x(j),w(j))
      for j<i are constrained to be on the line. if any (x,w) reaches the
      end of a line segment then it is switched between index sets.
 * i is added to the appropriate index set depending on what line segment
      it hits.

we restrict lo(i) <= 0 and hi(i) >= 0. this makes the algorithm a bit
simpler, because the starting point for x(i),w(i) is always on the dotted
line x=0 and x will only ever increase in one direction, so it can only hit
two out of the three line segments.


NOTES
-----

this is an implementation of "lcp_dantzig2_ldlt.m" and "lcp_dantzig_lohi.m".
the implementation is split into an LCP problem object (dLCP) and an LCP
driver function. most optimization occurs in the dLCP object.

a naive implementation of the algorithm requires either a lot of data motion
or a lot of permutation-array lookup, because we are constantly re-ordering
rows and columns. to avoid this and make a more optimized algorithm, a
non-trivial data structure is used to represent the matrix A (this is
implemented in the fast version of the dLCP object).

during execution of this algorithm, some indexes in A are clamped (set C),
some are non-clamped (set N), and some are "don't care" (where x=0).
A,x,b,w (and other problem vectors) are permuted such that the clamped
indexes are first, the unclamped indexes are next, and the don't-care
indexes are last. this permutation is recorded in the array `p'.
initially p = 0..n-1, and as the rows and columns of A,x,b,w are swapped,
the corresponding elements of p are swapped.

because the C and N elements are grouped together in the rows of A, we can do
lots of work with a fast dot product function. if A,x,etc were not permuted
and we only had a permutation array, then those dot products would be much
slower as we would have a permutation array lookup in some inner loops.

A is accessed through an array of row pointers, so that element (i,j) of the
permuted matrix is A[i][j]. this makes row swapping fast. for column swapping
we still have to actually move the data.

during execution of this algorithm we maintain an L*D*L' factorization of
the clamped submatrix of A (call it `AC') which is the top left nC*nC
submatrix of A. there are two ways we could arrange the rows/columns in AC.

(1) AC is always permuted such that L*D*L' = AC. this causes a problem
    when a row/column is removed from C, because then all the rows/columns of A
    between the deleted index and the end of C need to be rotated downward.
    this results in a lot of data motion and slows things down.
(2) L*D*L' is actually a factorization of a *permutation* of AC (which is
    itself a permutation of the underlying A). this is what we do - the
    permutation is recorded in the vector C. call this permutation A[C,C].
    when a row/column is removed from C, all we have to do is swap two
    rows/columns and manipulate C.

 */
public class DLCP {

	//***************************************************************************
	// code generation parameters

	// LCP debugging (mostly for fast dLCP) - this slows things down a lot
	//#define DEBUG_LCP

	//#define dLCP_FAST		// use fast dLCP object
	private static final boolean dLCP_FAST = true;

	// option 1 : matrix row pointers (less data copying)
	//#define ROWPTRS
	//#define ATYPE dReal **
	//#define AROW(i) (m_A[i])
	//private final double AROW(int i) { return A[i]; };
	
	// option 2 : no matrix row pointers (slightly faster inner loops)
	//#define NOROWPTRS
	//#define ATYPE dReal *
	//#define AROW(i) (A+(i)*nskip)

    //TODO 
    //TODO This was true!!! (TZ)
	// see options above (TZ)
    //protected static final boolean ROWPTRS = true;
    protected static final boolean ROWPTRS = false;

	
	
	//#define NUB_OPTIMIZATIONS
	protected static final boolean NUB_OPTIMIZATIONS = true;

	//TZ: remove TODO
	//protected static final boolean DEBUG_LCP = true;
	protected static final boolean DEBUG_LCP = false;

	//***************************************************************************

	//abstract methods by TZ
//	protected abstract double AROW(int i, int j);
//	abstract int getNub();
//	abstract double Aii (int i);
//	abstract double AiC_times_qC (int i, double[] q);
//	abstract double AiN_times_qN (int i, double[] q);
//	abstract int numC();
//	abstract int numN();
//	abstract int indexC (int i);
//	abstract int indexN (int i);
//	abstract void transfer_i_to_N (int i);
//	abstract void transfer_i_to_C (int i);
//	abstract void transfer_i_from_N_to_C (int i);
//	abstract void transfer_i_from_C_to_N (int i);
//	abstract void pC_plusequals_s_times_qC (double[] p, double s, double[] q);
//	abstract void pN_plusequals_s_times_qN (double[] p, double s, double[] q);
//	abstract void pN_equals_ANC_times_qC (double[] p, double[] q);
//	abstract void pN_plusequals_ANi (double[] p, int i, int sign);
//	void pN_plusequals_ANi (double[] p, int i) { pN_plusequals_ANi(p, i, 1); } ;
//	abstract void solve1 (double[] a, int i, int dir, boolean only_transfer);
//	void solve1(double[] a, int i, int dir) { solve1(a, i, dir, false); }
//	void solve1(double[] a, int i) { solve1(a, i, 1, false); }
//	abstract void unpermute();
	

	//#endif

	//***************************************************************************
	// dLCP manipulator object. this represents an n*n LCP problem.
	//
	// two index sets C and N are kept. each set holds a subset of
	// the variable indexes 0..n-1. an index can only be in one set.
	// initially both sets are empty.
	//
	// the index set C is special: solutions to A(C,C)\A(C,i) can be generated.



//***************************************************************************
// fast implementation of dLCP. see the above definition of dLCP for
// interface comments.
//
// `p' records the permutation of A,x,b,w,etc. p is initially 1:n and is
// permuted as the other vectors/matrices are permuted.
//
// A,x,b,w,lo,hi,state,findex,p,c are permuted such that sets C,N have
// contiguous indexes. the don't-care indexes follow N.
//
// an L*D*L' factorization is maintained of A(C,C), and whenever indexes are
// added or removed from the set C the factorization is updated.
// thus L*D*L'=A[C,C], i.e. a permuted top left nC*nC submatrix of A.
// the leading dimension of the matrix L is always `nskip'.
//
// at the start there may be other indexes that are unbounded but are not
// included in `nub'. dLCP will permute the matrix so that absolutely all
// unbounded vectors are at the start. thus there may be some initial
// permutation.
//
// the algorithms here assume certain patterns, particularly with respect to
// index transfer.

//#ifdef dLCP_FAST

    private final int m_n;
    private final int m_nskip;
    private int m_nub;
    private int m_nC, m_nN;              // size of each index set
	//  ATYPE A;				// A rows
	//TODO use [][] ???
	private double[] m_A;				// A rows
	//  dReal *Adata,*x,*b,*w,*lo,*hi;	// permuted LCP problem data
	//  dReal *L,*d;				// L*D*L' factorization of set C
	//  dReal *Dell,*ell,*tmp;
	//  int *state,*findex,*p,*C;
	private double[] m_x,m_b,m_w,m_lo,m_hi;	// permuted LCP problem data
	private double[] m_L,m_d;				// L*D*L' factorization of set C
	private double[] m_Dell,m_ell,m_tmp;
	private boolean[] m_state;
	private int[] m_findex, m_p, m_C;

	private final int AROWp(int i) { return i*m_nskip; };
	protected final double AROW(int i, int j) { return m_A[i*m_nskip+j]; };
	private void pN_plusequals_ANi (double[] p, int i) { pN_plusequals_ANi(p, i, 1); } ;
	void solve1(double[] a, int i, int dir) { solve1(a, i, dir, false); }
	void solve1(double[] a, int i) { solve1(a, i, 1, false); }
	
	
	//TODO
	//  dLCP (int _n, int _nub, dReal *_Adata, dReal *_x, dReal *_b, dReal *_w,
	//	dReal *_lo, dReal *_hi, dReal *_L, dReal *_d,
	//	dReal *_Dell, dReal *_ell, dReal *_tmp,
	//	int *_state, int *_findex, int *_p, int *_C, dReal **Arows);
	private int getNub() { return m_nub; }
	//  void transfer_i_to_C (int i);
	private void transfer_i_to_N (int i) { m_nN++; } // because we can assume C and N span 1:i-1
	//  void transfer_i_from_N_to_C (int i);
	//  void transfer_i_from_C_to_N (int i, void*tmpbuf);
	static int estimate_transfer_i_from_C_to_N_mem_req(int nC, int nskip) { 
	    return Matrix.dEstimateLDLTRemoveTmpbufSize(nC, nskip); }
	private int numC() { return m_nC; }
	private int numN() { return m_nN; }
	private int indexC (int i) { return i; }
	private int indexN (int i) { return i+m_nC; }
	private double Aii (int i) { return AROW(i,i); }
	private double AiC_times_qC (int i, double[] q) { return FastDot.dDot (m_A, AROWp(i),q,0,m_nC); }
	private double AiN_times_qN (int i, double[] q) { return FastDot.dDot (m_A, AROWp(i)+m_nC,q,m_nC,m_nN); }
	//  void pN_equals_ANC_times_qC (dReal *p, dReal *q);
	//  void pN_plusequals_ANi (dReal *p, int i, int sign=1);
//	private void pC_plusequals_s_times_qC (double[] p, double s, double[] q)
//	{ for (int i=0; i<nC; i++) p[i] += s*q[i]; }
//	private void pN_plusequals_s_times_qN (double[] p, double s, double[] q)
//	{ for (int i=0; i<nN; i++) p[i+nC] += s*q[i+nC]; }
	//  void solve1 (dReal *a, int i, int dir=1, int only_transfer=0);
	//  void unpermute();
	//};


	
	
	//  dLCP::dLCP (int _n, int _nub, dReal *_Adata, dReal *_x, dReal *_b, dReal *_w,
	//		    dReal *_lo, dReal *_hi, dReal *_L, dReal *_d,
	//		    dReal *_Dell, dReal *_ell, dReal *_tmp,
	//		    int *_state, int *_findex, int *_p, int *_C, dReal **Arows)
	DLCP (int _n, int _nskip, int _nub, double []_Adata, double[] _x, double[] _b, double[] _w,
			double[] _lo, double[] _hi, double[] _L, double[] _d,
			double[] _Dell, double[] _ell, double[] _tmp,
			boolean []_state, int []_findex, int []_p, int []_C, double[][]Arows)
			{
	    m_n = _n;
	    m_nskip = _nskip;
	    m_nub = _nub;
	    if (ROWPTRS) {//# ifdef ROWPTRS
	        //m_A(Arows),
	        throw new UnsupportedOperationException();
	    } else { //#else
	        m_A = _Adata;//m_A(_Adata),
	    } //#endif
	    m_x = _x;
	    m_b = _b;
	    m_w = _w;
	    m_lo = _lo;
	    m_hi = _hi;
	    m_L = _L;
	    m_d = _d;
	    m_Dell = _Dell;
	    m_ell = _ell;
	    m_tmp = _tmp;
	    m_state = _state;
	    m_findex = _findex;
	    m_p = _p;
	    m_C = _C;

	    OdeMath.dSetZero (m_x, m_n);

		if (ROWPTRS) {//# ifdef ROWPTRS
			// make matrix row pointers
		    //		    dReal *aptr = _Adata;
		    //		    ATYPE A = m_A;
		    //		    const int n = m_n, nskip = m_nskip;
		    //		    for (int k=0; k<n; aptr+=nskip, ++k) A[k] = aptr;
			throw new UnsupportedOperationException();
		}//# endif

		{
		    int []p = m_p;
		    final int n = m_n;
		    for (int k=0; k<n; ++k) p[k]=k;     // initially unpermuted
		}

		/*
		  // for testing, we can do some random swaps in the area i > nub
		  {
		    const int n = m_n;
		    const int nub = m_nub;
		    if (nub < n) {
		    for (int k=0; k<100; k++) {
		      int i1,i2;
		      do {
		        i1 = dRandInt(n-nub)+nub;
		        i2 = dRandInt(n-nub)+nub;
		      }
		      while (i1 > i2); 
		      //printf ("--> %d %d\n",i1,i2);
		      swapProblem (m_A,m_x,m_b,m_w,m_lo,m_hi,m_p,m_state,m_findex,n,i1,i2,m_nskip,0);
		    }
		  }
		 */

		// permute the problem so that *all* the unbounded variables are at the
		// start, i.e. look for unbounded variables not included in `nub'. we can
		// potentially push up `nub' this way and get a bigger initial factorization.
		// note that when we swap rows/cols here we must not just swap row pointers,
		// as the initial factorization relies on the data being all in one chunk.
		// variables that have findex >= 0 are *not* considered to be unbounded even
		// if lo=-inf and hi=inf - this is because these limits may change during the
		// solution process.

		{
		    int[] findex = m_findex;
		    double[] lo = m_lo, hi = m_hi;
		    final int n = m_n;
		    for (int k = m_nub; k<n; ++k) {
		        if (findex!=null && findex[k] >= 0) continue;
		        if (lo[k]==-dInfinity && hi[k]==dInfinity) {
		            swapProblem (m_A,m_x,m_b,m_w,lo,hi,m_p,m_state,findex,n,m_nub,k,m_nskip,false);
		            m_nub++;
		        }
		    }
		}

		// if there are unbounded variables at the start, factorize A up to that
		// point and solve for x. this puts all indexes 0..nub-1 into C.
		if (m_nub > 0) {
		    final int nub = m_nub;
		    {
		        int Lrow = 0;//m_L;
		        final int nskip = m_nskip;
		        for (int j=0; j<nub; Lrow+=nskip, ++j) { 
		            //memcpy(Lrow,AROW(j),(j+1)*sizeof(dReal));
		            memcpy(m_L, Lrow, m_A, AROWp(j), j+1);
		        }
		    }
		    dFactorLDLT (m_L,m_d,nub,m_nskip);
		    memcpy (m_x,m_b,nub);//*sizeof(dReal));
		    dSolveLDLT (m_L,m_d,m_x,nub,m_nskip);
		    dSetZero (m_w,nub);
		    {
		        int[] C = m_C;
		        for (int k=0; k<nub; ++k) C[k] = k;
		    }
		    m_nC = nub;
		}

		// permute the indexes > nub such that all findex variables are at the end
		if (m_findex!=null) {
		    final int nub = m_nub;
		    int[] findex = m_findex;
		    int num_at_end = 0;
		    for (int k=m_n-1; k >= nub; k--) {
		        if (findex[k] >= 0) {
		            swapProblem (m_A,m_x,m_b,m_w,m_lo,m_hi,m_p,m_state,findex,m_n,k,m_n-1-num_at_end,m_nskip,true);
		            num_at_end++;
		        }
		    }
		}

		// print info about indexes
		/*
		  {
		    const int n = m_n;
		    const int nub = m_nub;
		    for (int k=0; k<n; k++) {
		      if (k<nub) printf ("C");
		      else if (m_lo[k]==-dInfinity && m_hi[k]==dInfinity) printf ("c");
		      else printf (".");
		    }
		    printf ("\n");
		  }
		 */
			}


	void transfer_i_to_C (int i)
	{
	    {
	        if (m_nC > 0) {
	            // ell,Dell were computed by solve1(). note, ell = D \ L1solve (L,A(i,C))
	            {
	                final int nC = m_nC;
	                int Ltgt_p = nC*m_nskip;//m_L + nC*m_nskip;
	                double[] ell = m_ell;
	                for (int j=0; j<nC; ++j) m_L[Ltgt_p+j] = ell[j];
	            }
	            final int nC = m_nC;
	            m_d[nC] = dRecip (AROW(i,i) - dDot(m_ell,m_Dell,nC));
	        }
	        else {
	            m_d[0] = dRecip (AROW(i,i));
	        }	    
	    }

	    swapProblem (m_A,m_x,m_b,m_w,m_lo,m_hi,m_p,m_state,m_findex,m_n,m_nC,i,m_nskip,true);
	    final int nC = m_nC;
	    m_C[nC] = nC;
	    m_nC = nC + 1; // nC value is outdated after this line

	    if (DEBUG_LCP) {//# ifdef DEBUG_LCP
	        checkFactorization (m_A,m_L,m_d,m_nC,m_C,m_nskip);
	        if (i < (m_n-1)) checkPermutations (i+1,m_n,m_nC,m_nN,m_p,m_C);
	    }//# endif
	}


	void transfer_i_from_N_to_C (int i)
	{
	    {
	        if (m_nC > 0) {
	            {
	                //dReal *const aptr = AROW(i);
	                int aPos = AROWp(i);
	                double[] Dell = m_Dell;
	                final int[] C = m_C;
	                if (NUB_OPTIMIZATIONS) {//#   ifdef NUB_OPTIMIZATIONS
	                    // if nub>0, initial part of aptr unpermuted
	                    final int nub = m_nub;
	                    int j = 0;
	                    for ( ; j<nub; ++j) Dell[j] = m_A[aPos+j];//aptr[j];
	                    final int nC = m_nC;
	                    for ( ; j<nC; ++j) Dell[j] = m_A[aPos+C[j]];//aptr[C[j]];
	                } else {//#   else
	                    final int nC = m_nC;
	                    for (int j=0; j<nC; ++j) Dell[j] = m_A[aPos+C[j]];//aptr[C[j]];
	                }//#   endif
	            }
	            Matrix.dSolveL1 (m_L,m_Dell,m_nC,m_nskip);
	            {
	                final int nC = m_nC;
	                final int Ltgt = nC*m_nskip;
	                double[] ell = m_ell, Dell = m_Dell, d = m_d;
	                for (int j=0; j<nC; ++j) m_L[Ltgt+j] = ell[j] = Dell[j] * d[j];
	            }
	            final int nC = m_nC;
	            m_d[nC] = dRecip (AROW(i,i) - dDot(m_ell,m_Dell,nC));
	        }
	        else {
	            m_d[0] = dRecip (AROW(i,i));
	        }
	        
	        swapProblem (m_A,m_x,m_b,m_w,m_lo,m_hi,m_p,m_state,m_findex,m_n,m_nC,i,m_nskip,true);
	        final int nC = m_nC;
	        m_C[nC] = nC;
	        m_nN--;
	        m_nC = nC + 1; // nC value is outdated after this line
	    }
	    
		// @@@ TO DO LATER
		// if we just finish here then we'll go back and re-solve for
		// delta_x. but actually we can be more efficient and incrementally
		// update delta_x here. but if we do this, we wont have ell and Dell
		// to use in updating the factorization later.

		if (DEBUG_LCP) {//# ifdef DEBUG_LCP
			checkFactorization (m_A,m_L,m_d,m_nC,m_C,m_nskip);
		}//# endif
	}


	void transfer_i_from_C_to_N (int i, BlockPointer tmpbuf)
	{
	    {
	        int[] C = m_C;
    		// remove a row/column from the factorization, and adjust the
    		// indexes (black magic!)
	        int last_idx = -1;
	        final int nC = m_nC;
	        int j = 0;
	        for ( ; j<nC; ++j) {
	            if (C[j]==nC-1) {
	                last_idx = j;
	            }
	            if (C[j]==i) {
	                Matrix.dLDLTRemove (m_A,C,m_L,m_d,m_n,nC,j,m_nskip,tmpbuf);
	                int k;
	                if (last_idx == -1) {
	                    for (k=j+1 ; k<nC; ++k) {
	                        if (C[k]==nC-1) {
	                            break;
	                        }
	                    }
	                    dIASSERT (k < nC);
	                }
	                else {
	                    k = last_idx;
	                }
	                C[k] = C[j];
	                if (j < (nC-1)) memmove (C,j,C,j+1,(nC-j-1));//*sizeof(int));
	                break;
	            }
	        }
	        dIASSERT (j < nC);
	        swapProblem (m_A,m_x,m_b,m_w,m_lo,m_hi,m_p,m_state,m_findex,m_n,i,nC-1,m_nskip,true);

	        m_nN++;
	        m_nC = nC - 1; // nC value is outdated after this line
	    }

		if (DEBUG_LCP) {//# ifdef DEBUG_LCP
		    checkFactorization (m_A,m_L,m_d,m_nC,m_C,m_nskip);
		}//# endif
	}


	void pN_equals_ANC_times_qC (double[] p, double[] q)
	{
		// we could try to make this matrix-vector multiplication faster using
		// outer product matrix tricks, e.g. with the dMultidotX() functions.
		// but i tried it and it actually made things slower on random 100x100
		// problems because of the overhead involved. so we'll stick with the
		// simple method for now.
	    final int nC = m_nC;
	    int ptgt_p = nC;
	    final int nN = m_nN;
	    for (int i=0; i<nN; ++i) {
	        p[ptgt_p+i] = dDot (m_A, AROWp(i+nC),q,0,nC);
	    }
	}


	void pN_plusequals_ANi (double[] p, int ii, int sign)
	{
	    final int nC = m_nC;
		//double[] aptr = AROW(ii)+nC;
		int aPos = AROWp(ii)+nC;
		int ptgt_p = nC;
		if (sign > 0) {
		    final int nN = m_nN;
		    for (int j=0; j<nN; ++j) p[ptgt_p+j] += m_A[aPos+j];
		  }
		  else {
		    final int nN = m_nN;
		    for (int j=0; j<nN; ++j) p[ptgt_p+j] -= m_A[aPos+j];
		  }
	}


	void pC_plusequals_s_times_qC (double[] p, double s, double[] q)
	{
	    final int nC = m_nC;
	    for (int i=0; i<nC; ++i) {
	        p[i] += s*q[i];
	    }
	}

	
	void pN_plusequals_s_times_qN (double[] p, double s, double[] q)
	{
	  final int nC = m_nC;
      //dReal *int ptgt_P = p + nC, qsrc_p = q + nC;
	  int ptgt_p = nC, qsrc_p = nC;
	  final int nN = m_nN;
	  for (int i=0; i<nN; ++i) {
	    p[ptgt_p+i] += s*q[qsrc_p+i];
	  }
	}

	
	void solve1 (double[] a, int i, int dir, boolean only_transfer)
	{
		// the `Dell' and `ell' that are computed here are saved. if index i is
		// later added to the factorization then they can be reused.
		//
		// @@@ question: do we need to solve for entire delta_x??? yes, but
		//     only if an x goes below 0 during the step.

		if (m_nC > 0) {
		    double[] Dell = m_Dell;
		    int[] C = m_C;
			//double[] aptr = AROW(i);
			int aPos = AROWp(i);
			if (NUB_OPTIMIZATIONS) {//#   ifdef NUB_OPTIMIZATIONS
				// if nub>0, initial part of aptr[] is guaranteed unpermuted
			    final int nub = m_nub;
			    int j=0;
				for ( ; j<nub; ++j) Dell[j] = m_A[aPos+j];//aptr[j];
				final int nC = m_nC;
				for ( ; j<nC; ++j) Dell[j] = m_A[aPos+C[j]];//aptr[C[j]];
			} else {//#   else
                final int nC = m_nC;
				for (int j=0; j<nC; j++) Dell[j] = m_A[aPos+C[j]];//aptr[C[j]];
			}//#   endif
		}
		Matrix.dSolveL1 (m_L,m_Dell,m_nC,m_nskip);
		{
		    double[] ell = m_ell, Dell = m_Dell, d = m_d;
		    final int nC = m_nC;
		    for (int j=0; j<nC; j++) ell[j] = Dell[j] * d[j];
		}

		if (!only_transfer) {
		    double[] tmp = m_tmp, ell = m_ell;
		    {
		        final int nC = m_nC;
		        for (int j=0; j<nC; ++j) tmp[j] = ell[j];
		    }
		    Matrix.dSolveL1T (m_L,tmp,m_nC,m_nskip);
		    if (dir > 0) {
		        int[] C = m_C;
		        //double[] tmp = m_tmp;
		        final int nC = m_nC;
		        for (int j=0; j<nC; ++j) a[C[j]] = -tmp[j];
		    } else {
		        int[] C = m_C;
		        //double[] tmp = m_tmp;
		        final int nC = m_nC;
		        for (int j=0; j<nC; ++j) a[C[j]] = tmp[j];
		    }
		}
	}


	void unpermute()
	{
		// now we have to un-permute x and w
	    {
	        memcpy (m_tmp,m_x,m_n);//*sizeof(dReal));
	        double[] x = m_x, tmp = m_tmp;
	        final int[] p = m_p;
	        final int n = m_n;
	        for (int j=0; j<n; ++j) x[p[j]] = tmp[j];
	    }
	    {
	        memcpy (m_tmp,m_w,m_n);//*sizeof(dReal));
	        double[] w = m_w, tmp = m_tmp;
	        final int[] p = m_p;
	        final int n = m_n;
	        for (int j=0; j<n; ++j) w[p[j]] = tmp[j];
	    }
	}

	//***************************************************************************
	// an optimized Dantzig LCP driver routine for the lo-hi LCP problem.

	static void dSolveLCP (DxWorldProcessMemArena memarena,
	        int n, double[] A, double[] x, double[] b,
			double[] outer_w, int nub, double[] lo, double[] hi, int []findex)
	{
		//	  dAASSERT (n>0 && A && x && b && w && lo && hi && nub >= 0 && nub <= n);
		dAASSERT (n>0 && nub >= 0 && nub <= n);
		dAASSERT (A, x, b, lo, hi);
		if(!dNODEBUG) {//# ifndef dNODEBUG
		  {
		    // check restrictions on lo and hi
		    for (int k=0; k<n; ++k) dIASSERT (lo[k] <= 0 && hi[k] >= 0);
		  }
		}//# endif

		// if all the variables are unbounded then we can just factor, solve,
		// and return
		if (nub >= n) {
		    double[] d = memarena.AllocateArrayDReal(n);
		    dSetZero (d, n);

		    int nskip = dPAD(n);
			Matrix.dFactorLDLT (A,d,n,nskip);		// use w for d
			Matrix.dSolveLDLT (A,d,b,n,nskip);
			memcpy (x,b,n);//*sizeof(dReal));

			return;
		}
		
		final int nskip = dPAD(n);
		double[] L =  memarena.AllocateArrayDReal(n*nskip);//ALLOCA (dReal,L,n*nskip*sizeof(dReal));
		double[] d =  memarena.AllocateArrayDReal(n);//ALLOCA (dReal,d,n*sizeof(dReal));
		double[] w = outer_w != null ? outer_w : memarena.AllocateArrayDReal(n);
		double[] delta_x =  memarena.AllocateArrayDReal(n);//ALLOCA (dReal,delta_x,n*sizeof(dReal));
		double[] delta_w =  memarena.AllocateArrayDReal(n);//ALLOCA (dReal,delta_w,n*sizeof(dReal));
		double[] Dell =  memarena.AllocateArrayDReal(n);//ALLOCA (dReal,Dell,n*sizeof(dReal));
		double[] ell =  memarena.AllocateArrayDReal(n);//ALLOCA (dReal,ell,n*sizeof(dReal));
		double[][] Arows = null;
		if (ROWPTRS) {
		    Arows =  memarena.AllocateArrayDRealDReal(n);//ALLOCA (dReal*,Arows,n*sizeof(dReal*));
		}
		int[] p = memarena.AllocateArrayInt(n);//ALLOCA (int,p,n*sizeof(int));
		int[] C = memarena.AllocateArrayInt(n);//ALLOCA (int,C,n*sizeof(int));

		// for i in N, state[i] is 0 if x(i)==lo(i) or 1 if x(i)==hi(i)
		boolean[]state = memarena.AllocateArrayBool(n);//ALLOCA (int,state,n*sizeof(int));

		// create LCP object. note that tmp is set to delta_w to save space, this
		// optimization relies on knowledge of how tmp is used, so be careful!
		DLCP lcp = new DLCP(n,nskip,nub,A,x,b,w,lo,hi,L,d,Dell,ell,delta_w,state,findex,p,C,Arows);
		int adj_nub = lcp.getNub();

		// loop over all indexes adj_nub..n-1. for index i, if x(i),w(i) satisfy the
		// LCP conditions then i is added to the appropriate index set. otherwise
		// x(i),w(i) is driven either +ve or -ve to force it to the valid region.
		// as we drive x(i), x(C) is also adjusted to keep w(C) at zero.
		// while driving x(i) we maintain the LCP conditions on the other variables
		// 0..i-1. we do this by watching out for other x(i),w(i) values going
		// outside the valid region, and then switching them between index sets
		// when that happens.

		boolean hit_first_friction_index = false;
		for (int i=adj_nub; i<n; ++i) {
		    boolean s_error = false;
		    // the index i is the driving index and indexes i+1..n-1 are "dont care",
			// i.e. when we make changes to the system those x's will be zero and we
			// don't care what happens to those w's. in other words, we only consider
			// an (i+1)*(i+1) sub-problem of A*x=b+w.

			// if we've hit the first friction index, we have to compute the lo and
			// hi values based on the values of x already computed. we have been
			// permuting the indexes, so the values stored in the findex vector are
			// no longer valid. thus we have to temporarily unpermute the x vector. 
			// for the purposes of this computation, 0*infinity = 0 ... so if the
			// contact constraint's normal force is 0, there should be no tangential
			// force applied.

			if (!hit_first_friction_index && findex!=null && findex[i] >= 0) {
				// un-permute x into delta_w, which is not being used at the moment
			    for (int j=0; j<n; ++j) delta_w[p[j]] = x[j];

			    // set lo and hi values
			    for (int k=i; k<n; ++k) {
					double wfk = delta_w[findex[k]];
					if (wfk == 0) {
						hi[k] = 0;
						lo[k] = 0;
					}
					else {
						hi[k] = dFabs (hi[k] * wfk);
						lo[k] = -hi[k];
					}
				}
				hit_first_friction_index = true;
			}

			// thus far we have not even been computing the w values for indexes
			// greater than i, so compute w[i] now.
			w[i] = lcp.AiC_times_qC (i,x) + lcp.AiN_times_qN (i,x) - b[i];

			// if lo=hi=0 (which can happen for tangential friction when normals are
			// 0) then the index will be assigned to set N with some state. however,
			// set C's line has zero size, so the index will always remain in set N.
			// with the "normal" switching logic, if w changed sign then the index
			// would have to switch to set C and then back to set N with an inverted
			// state. this is pointless, and also computationally expensive. to
			// prevent this from happening, we use the rule that indexes with lo=hi=0
			// will never be checked for set changes. this means that the state for
			// these indexes may be incorrect, but that doesn't matter.

			// see if x(i),w(i) is in a valid region
			if (lo[i]==0 && w[i] >= 0) {
				lcp.transfer_i_to_N (i);
				state[i] = false;
			}
			else if (hi[i]==0 && w[i] <= 0) {
				lcp.transfer_i_to_N (i);
				state[i] = true;
			}
			else if (w[i]==0) {
				// this is a degenerate case. by the time we get to this test we know
				// that lo != 0, which means that lo < 0 as lo is not allowed to be +ve,
				// and similarly that hi > 0. this means that the line segment
				// corresponding to set C is at least finite in extent, and we are on it.
				// NOTE: we must call lcp.solve1() before lcp.transfer_i_to_C()
				lcp.solve1 (delta_x,i,0,true);

				lcp.transfer_i_to_C (i);
			}
			else {
				// we must push x(i) and w(i)
				for (;;) {
			        int dir;
			        double dirf;
					// find direction to push on x(i)
					if (w[i] <= 0) {
						dir = 1;
						dirf = (1.0);
					}
					else {
						dir = -1;
						dirf = (-1.0);
					}

					// compute: delta_x(C) = -dir*A(C,C)\A(C,i)
					lcp.solve1 (delta_x,i,dir);

					// note that delta_x[i] = dirf, but we wont bother to set it

					// compute: delta_w = A*delta_x ... note we only care about
					// delta_w(N) and delta_w(i), the rest is ignored
			        lcp.pN_equals_ANC_times_qC (delta_w,delta_x);
			        lcp.pN_plusequals_ANi (delta_w,i,dir);
			        delta_w[i] = lcp.AiC_times_qC (i,delta_x) + lcp.Aii(i)*dirf;

					// find largest step we can take (size=s), either to drive x(i),w(i)
					// to the valid LCP region or to drive an already-valid variable
					// outside the valid region.

			        int cmd = 1;        // index switching command
			        int si = 0;     // si = index to switch if cmd>3
			        double s = -w[i]/delta_w[i];
			        if (dir > 0) {
			            if (hi[i] < dInfinity) {
			                double s2 = (hi[i]-x[i])*dirf;   // was (hi[i]-x[i])/dirf    // step to x(i)=hi(i)
			                if (s2 < s) {
			                    s = s2;
			                    cmd = 3;
			                }
			            }
			        }
			        else {
			            if (lo[i] > -dInfinity) {
			                double s2 = (lo[i]-x[i])*dirf;   // was (lo[i]-x[i])/dirf    // step to x(i)=lo(i)
			                if (s2 < s) {
			                    s = s2;
			                    cmd = 2;
			                }
			            }
			        }


					{
					    final int numN = lcp.numN();
					    for (int k=0; k < numN; ++k) {
					        final int indexN_k = lcp.indexN(k);
					        if (!state[indexN_k] ? delta_w[indexN_k] < 0 : delta_w[indexN_k] > 0) {
					            // don't bother checking if lo=hi=0
					            if (lo[indexN_k] == 0 && hi[indexN_k] == 0) continue;
					            double s2 = -w[indexN_k] / delta_w[indexN_k];
					            if (s2 < s) {
					                s = s2;
					                cmd = 4;
					                si = indexN_k;
					            }
					        }
					    }
					}

					{
					    final int numC = lcp.numC();
					    for (int k=adj_nub; k < numC; ++k) {
					        final int indexC_k = lcp.indexC(k);
					        if (delta_x[indexC_k] < 0 && lo[indexC_k] > -dInfinity) {
					            double s2 = (lo[indexC_k]-x[indexC_k]) / delta_x[indexC_k];
					            if (s2 < s) {
					                s = s2;
					                cmd = 5;
					                si = indexC_k;
					            }
					        }
					        if (delta_x[indexC_k] > 0 && hi[indexC_k] < dInfinity) {
					            double s2 = (hi[indexC_k]-x[indexC_k]) / delta_x[indexC_k];
					            if (s2 < s) {
					                s = s2;
					                cmd = 6;
					                si = indexC_k;
					            }
					        }
					    }
					}

			        //static char* cmdstring[8] = {0,"->C","->NL","->NH","N->C",
					//			     "C->NL","C->NH"};
					//printf ("cmd=%d (%s), si=%d\n",cmd,cmdstring[cmd],(cmd>3) ? si : i);

					// if s <= 0 then we've got a problem. if we just keep going then
					// we're going to get stuck in an infinite loop. instead, just cross
					// our fingers and exit with the current solution.
					if (s <= 0) {
						dMessage (d_ERR_LCP, "LCP internal error, s <= 0 (s=%.4e)",s);
						if (i < n) {
						    //dSetZero (x+i,n-i);
						    //dSetZero (w+i,n-i);
						    for (int ii = i; ii < n; ii++) x[ii] = 0;
						    for (int ii = i; ii < n; ii++) w[ii] = 0;
						}
						s_error = true;
						break;
					}

				       // apply x = x + s * delta_x
			        lcp.pC_plusequals_s_times_qC (x, s, delta_x);
			        x[i] += s * dirf;

			        // apply w = w + s * delta_w
			        lcp.pN_plusequals_s_times_qN (w, s, delta_w);
			        w[i] += s * delta_w[i];

			        BlockPointer tmpbuf;
			        // switch indexes between sets if necessary
			        switch (cmd) {
			        case 1:     // done
			          w[i] = 0;
			          lcp.transfer_i_to_C (i);
			          break;
			        case 2:     // done
			          x[i] = lo[i];
			          state[i] = false;
			          lcp.transfer_i_to_N (i);
			          break;
			        case 3:     // done
			          x[i] = hi[i];
			          state[i] = true;
			          lcp.transfer_i_to_N (i);
			          break;
			        case 4:     // keep going
			          w[si] = 0;
			          lcp.transfer_i_from_N_to_C (si);
			          break;
			        case 5:     // keep going
			          x[si] = lo[si];
			          state[si] = false;
			          tmpbuf = memarena.PeekBufferRemainder();
			          lcp.transfer_i_from_C_to_N (si, tmpbuf);
			          break;
			        case 6:     // keep going
			          x[si] = hi[si];
			          state[si] = true;
			          tmpbuf = memarena.PeekBufferRemainder();
			          lcp.transfer_i_from_C_to_N (si, tmpbuf);
			          break;
			        }

					if (cmd <= 3) break;
			      } // for (;;)
		    } // else

		    if (s_error) {
		      break;
		    }
		  } // for (int i=adj_nub; i<n; ++i)

		  lcp.unpermute();
	}

	private int sizeof(Class<?> cls) {
	    return -1;
//	    if (cls == double.class) {
//	        return 8;
//	    } else if
	}
	
	static int dEstimateSolveLCPMemoryReq(int n, boolean outer_w_avail)
	{
//	  final int nskip = dPAD(n);
//
//	  int res = 0;
//
//	  res += dEFFICIENT_SIZE(sizeof(double.class) * (n * nskip)); // for L
//	  res += 5 * dEFFICIENT_SIZE(sizeof(double.class) * n); // for d, delta_w, delta_x, Dell, ell
//	  if (!outer_w_avail) {
//	    res += dEFFICIENT_SIZE(sizeof(double.class) * n); // for w
//	  }
//	  if (ROWPTRS) {//#ifdef ROWPTRS
//	      res += dEFFICIENT_SIZE(sizeof(double[].class) * n); // for Arows
//	  } //#endif
//	  res += 2 * dEFFICIENT_SIZE(sizeof(int.class) * n); // for p, C
//	  res += dEFFICIENT_SIZE(sizeof(boolean.class) * n); // for state
//
//	  // Use n instead of nC as nC varies at runtime while n is greater or equal to nC
//	  int lcp_transfer_req = dLCP.estimate_transfer_i_from_C_to_N_mem_req(n, nskip);
//	  res += dEFFICIENT_SIZE(lcp_transfer_req); // for dLCP::transfer_i_from_C_to_N
//
//	  return res;
	    return -1;
	}


	//***************************************************************************
	// accuracy and timing test

	static int EstimateTestSolveLCPMemoryReq(int n)
	{
//	  final int nskip = dPAD(n);
//
//	  int res = 0;
//
//	  res += 2 * dEFFICIENT_SIZE(sizeof(double.class) * (n * nskip)); // for A, A2
//	  res += 10 * dEFFICIENT_SIZE(sizeof(double.class) * n); // for x, b, w, lo, hi, b2, lo2, hi2, tmp1, tmp2
//
//	  res += dEstimateSolveLCPMemoryReq(n, true);
//
//	  return res;
	    return -1;
	}

    //TODO API?
    //extern "C" ODE_API
    public static int dTestSolveLCP()
    {
        final int n = 100;
        
        int memreq = EstimateTestSolveLCPMemoryReq(n);
        DxWorldProcessMemArena arena = 
            //dxAllocateTemporaryWorldProcessMemArena(memreq, null, null);
            DxWorldProcessMemArena.allocateTemporary(memreq, null, null);
        if (arena == null) {
          return 0;
        }

        int i,nskip = dPAD(n);
        //  #ifdef dDOUBLE
        //    const dReal tol = REAL(1e-9);
        //  #endif
        //  #ifdef dSINGLE
        //    const dReal tol = REAL(1e-4);
        //  #endif
        double tol;
        if (OdeConfig.isDoublePrecision()) {//)#ifdef dDOUBLE
            tol = 1e-9;
        } else {
            tol = 1e-4f;
        }
        System.out.println ("dTestSolveLCP()");

//        double[] A = new double[n*nskip];//ALLOCA (dReal,A,n*nskip*sizeof(dReal));
//        double[] x = new double[n];//ALLOCA (dReal,x,n*sizeof(dReal));
//        double[] b = new double[n];//ALLOCA (dReal,b,n*sizeof(dReal));
//        double[] w = new double[n];//ALLOCA (dReal,w,n*sizeof(dReal));
//        double[] lo = new double[n];//ALLOCA (dReal,lo,n*sizeof(dReal));
//        double[] hi = new double[n];//ALLOCA (dReal,hi,n*sizeof(dReal));
//
//        double[] A2 = new double[n*nskip];//ALLOCA (dReal,A2,n*nskip*sizeof(dReal));
//        double[] b2 = new double[n];//ALLOCA (dReal,b2,n*sizeof(dReal));
//        double[] lo2 = new double[n];//ALLOCA (dReal,lo2,n*sizeof(dReal));
//        double[] hi2 = new double[n];//ALLOCA (dReal,hi2,n*sizeof(dReal));
//        double[] tmp1 = new double[n];//ALLOCA (dReal,tmp1,n*sizeof(dReal));
//        double[] tmp2 = new double[n];//ALLOCA (dReal,tmp2,n*sizeof(dReal));
        double[] A = arena.AllocateArrayDReal(n*nskip);//ALLOCA (dReal,A,n*nskip*sizeof(dReal));
        double[] x = arena.AllocateArrayDReal(n);//ALLOCA (dReal,x,n*sizeof(dReal));
        double[] b = arena.AllocateArrayDReal(n);//ALLOCA (dReal,b,n*sizeof(dReal));
        double[] w = arena.AllocateArrayDReal(n);//ALLOCA (dReal,w,n*sizeof(dReal));
        double[] lo = arena.AllocateArrayDReal(n);//ALLOCA (dReal,lo,n*sizeof(dReal));
        double[] hi = arena.AllocateArrayDReal(n);//ALLOCA (dReal,hi,n*sizeof(dReal));

        double[] A2 = arena.AllocateArrayDReal(n*nskip);//ALLOCA (dReal,A2,n*nskip*sizeof(dReal));
        double[] b2 = arena.AllocateArrayDReal(n);//ALLOCA (dReal,b2,n*sizeof(dReal));
        double[] lo2 = arena.AllocateArrayDReal(n);//ALLOCA (dReal,lo2,n*sizeof(dReal));
        double[] hi2 = arena.AllocateArrayDReal(n);//ALLOCA (dReal,hi2,n*sizeof(dReal));
        double[] tmp1 = arena.AllocateArrayDReal(n);//ALLOCA (dReal,tmp1,n*sizeof(dReal));
        double[] tmp2 = arena.AllocateArrayDReal(n);//ALLOCA (dReal,tmp2,n*sizeof(dReal));

        double total_time = 0;
        for (int count=0; count < 1000; count++) {
            DxWorldProcessMemArena.DxStateSave saveInner = 
                arena.BEGIN_STATE_SAVE(); 
            {

            // form (A,b) = a random positive definite LCP problem
            dMakeRandomMatrix (A2,n,n,1.0);
            Matrix.dMultiply2 (A,A2,A2,n,n,n);
            dMakeRandomMatrix (x,n,1,1.0);
            Matrix.dMultiply0 (b,A,x,n,n,1);
            for (i=0; i<n; i++) b[i] += (dRandReal()*(0.2))-(0.1); //REAL

            // choose `nub' in the range 0..n-1
            int nub = 50; //dRandInt (n);

            // make limits
            for (i=0; i<nub; i++) lo[i] = -dInfinity;
            for (i=0; i<nub; i++) hi[i] = dInfinity;
            //for (i=nub; i<n; i++) lo[i] = 0;
            //for (i=nub; i<n; i++) hi[i] = dInfinity;
            //for (i=nub; i<n; i++) lo[i] = -dInfinity;
            //for (i=nub; i<n; i++) hi[i] = 0;
            for (i=nub; i<n; i++) lo[i] = -dRandReal()-0.01; //REAL
            for (i=nub; i<n; i++) hi[i] =  dRandReal()+0.01; //REAL

            // set a few limits to lo=hi=0
            /*
//    for (i=0; i<10; i++) {
//      int j = dRandInt (n-nub) + nub;
//      lo[j] = 0;
//      hi[j] = 0;
//    }
             */

            // solve the LCP. we must make copy of A,b,lo,hi (A2,b2,lo2,hi2) for
            // SolveLCP() to permute. also, we'll clear the upper triangle of A2 to
            // ensure that it doesn't get referenced (if it does, the answer will be
            // wrong).

            memcpy (A2,A,n*nskip);//*sizeof(dReal));
            dClearUpperTriangle (A2,n);
            memcpy (b2,b,n);//*sizeof(dReal));
            memcpy (lo2,lo,n);//*sizeof(dReal));
            memcpy (hi2,hi,n);//*sizeof(dReal));
            Matrix.dSetZero (x);
            Matrix.dSetZero (w);

            DStopwatch sw = new DStopwatch();
            Timer.dStopwatchReset (sw);
            Timer.dStopwatchStart (sw);

            DLCP.dSolveLCP (arena, n,A2,x,b2,w,nub,lo2,hi2,null);

            Timer.dStopwatchStop (sw);
            double time = Timer.dStopwatchTime(sw);
            total_time += time;
            double average = total_time / ((double)(count+1.)) * 1000.0;

            // check the solution

            Matrix.dMultiply0 (tmp1,A,x,n,n,1);
            for (i=0; i<n; i++) tmp2[i] = b[i] + w[i];
            double diff = dMaxDifference (tmp1,tmp2,n,1);
            // printf ("\tA*x = b+w, maximum difference = %.6e - %s (1)\n",diff,
            //      diff > tol ? "FAILED" : "passed");
            if (diff > tol) dDebug (0,"A*x = b+w, maximum difference = %.6e",diff);
            int n1=0,n2=0,n3=0;
            double xi, wi;
            for (i=0; i<n; i++) {
                xi=x[i];
                wi=w[i];
                //if (x[i]==lo[i] && w[i] >= 0) {
                if (xi==lo[i] && wi >= 0) {
                    n1++;   // ok
                }
                //else if (x[i]==hi[i] && w[i] <= 0) {
                else if (xi==hi[i] && wi <= 0) {
                    n2++;   // ok
                }
                //else if (x[i] >= lo[i] && x[i] <= hi[i] && w[i] == 0) {
                else if (xi >= lo[i] && xi <= hi[i] && wi == 0) {
                    n3++;   // ok
                }
                else {
                    dDebug (0,"FAILED: i=%d x=%.4e w=%.4e lo=%.4e hi=%.4e",i,
                            x[i],w[i],lo[i],hi[i]);
                }
            }

            // pacifier
            printf ("passed: NL=%3d NH=%3d C=%3d   ",n1,n2,n3);
            printf ("time=%10.3f ms  avg=%10.4f\n",time * 1000.0,average);
            } 
            arena.END_STATE_SAVE(saveInner);
        }

        DxWorldProcessMemArena.freeTemporary(arena);
        return 1;
    }

    
    /** 
	 * swap row/column i1 with i2 in the n*n matrix A. the leading dimension of
	 * A is nskip. this only references and swaps the lower triangle.
	 * if `do_fast_row_swaps' is nonzero and row pointers are being used, then
	 * rows will be swapped by exchanging row pointers. otherwise the data will
	 * be copied.
	 */
	private static void swapRowsAndCols (double[] A, int n, int i1, int i2, int nskip,
			boolean do_fast_row_swaps)
	{
		//dAASSERT (A);
		dAASSERT (n > 0 && i1 >= 0 && i2 >= 0 && i1 < n && i2 < n &&
				nskip >= n && i1 < i2);

		if (ROWPTRS) {//# ifdef ROWPTRS
//		    dReal *A_i1 = A[i1];
//		    dReal *A_i2 = A[i2];
//		    for (int i=i1+1; i<i2; ++i) {
//		      dReal *A_i_i1 = A[i] + i1;
//		      A_i1[i] = *A_i_i1;
//		      *A_i_i1 = A_i2[i];
//		    }
//		    A_i1[i2] = A_i1[i1];
//		    A_i1[i1] = A_i2[i1];
//		    A_i2[i1] = A_i2[i2];
//		    // swap rows, by swapping row pointers
//		    if (do_fast_row_swaps) {
//		      A[i1] = A_i2;
//		      A[i2] = A_i1;
//		    }
//		    else {
//		      // Only swap till i2 column to match A plain storage variant.
//		      for (int k = 0; k <= i2; ++k) {
//		        dReal tmp = A_i1[k];
//		        A_i1[k] = A_i2[k];
//		        A_i2[k] = tmp;
//		      }
//		    }
//		    // swap columns the hard way
//		    for (int j=i2+1; j<n; ++j) {
//		      dReal *A_j = A[j];
//		      dReal tmp = A_j[i1];
//		      A_j[i1] = A_j[i2];
//		      A_j[i2] = tmp;
//		    }
			throw new UnsupportedOperationException();
		} else { // ROWPTRS # else
		    int A_i1p = i1*nskip;//dReal *A_i1 = A+i1*nskip;
		    int A_i2p = i2*nskip;//dReal *A_i2 = A+i2*nskip;
		    for (int k = 0; k < i1; ++k) {
		      double tmp = A[A_i1p+k];//A_i1[k];
		      A[A_i1p+k] = A[A_i2p+k];//A_i1[k] = A_i2[k];
		      A[A_i2p+k] = tmp;//A_i2[k] = tmp;
		    }
		    int A_ip = A_i1p + nskip;//dReal *A_i = A_i1 + nskip;
		    for (int i=i1+1; i<i2; A_ip+=nskip, ++i) {
		      double tmp = A[A_i2p+i];//A_i2[i];
		      A[A_i2p+i] = A[A_ip+i1];//A_i2[i] = A_i[i1];
		      A[A_ip+i1] = tmp;//A_i[i1] = tmp;
		    }
		    {
		      double tmp = A[A_i1p+i1];//A_i1[i1];
		      A[A_i1p+i1] = A[A_i2p+i2];//A_i1[i1] = A_i2[i2];
		      A[A_i2p+i2] = tmp;//A_i2[i2] = tmp;
		    }
		    int A_jp = A_i2p + nskip;//dReal *A_j = A_i2 + nskip;
		    for (int j=i2+1; j<n; A_jp+=nskip, ++j) {
		      double tmp = A[A_jp+i1];//A_j[i1];
		      A[A_jp+i1] = A[A_jp+i2];//A_j[i1] = A_j[i2];
		      A[A_jp+i2] = tmp;//A_j[i2] = tmp;
		    }
		}// else ROWPTRS # endif
	}


	// swap two indexes in the n*n LCP problem. i1 must be <= i2.

	//private static void swapProblem (ATYPE A, dReal *x, dReal *b, dReal *w, dReal *lo,
	//			 dReal *hi, int *p, int *state, int *findex,
	//			 int n, int i1, int i2, int nskip,
	//			 int do_fast_row_swaps)
	protected static void swapProblem (double[] A, double[] x, double[]b, double[] w, 
			double[] lo, double[] hi, int[] p, boolean []state, int[] findex,
			int n, int i1, int i2, int nskip,
			boolean do_fast_row_swaps)
	{
		double tmpr;
		int tmpi;
		boolean tmpb;
		dIASSERT (n>0 && i1 >=0 && i2 >= 0 && i1 < n && i2 < n 
				&& nskip >= n && i1 <= i2);
		if (i1==i2) return;
		swapRowsAndCols (A,n,i1,i2,nskip,do_fast_row_swaps);
		tmpr = x[i1];
		x[i1] = x[i2];
		x[i2] = tmpr;
		
		tmpr = b[i1];
		b[i1] = b[i2];
		b[i2] = tmpr;
		
		tmpr = w[i1];
		w[i1] = w[i2];
		w[i2] = tmpr;
		
		tmpr = lo[i1];
		lo[i1] = lo[i2];
		lo[i2] = tmpr;
		
		tmpr = hi[i1];
		hi[i1] = hi[i2];
		hi[i2] = tmpr;
		
		tmpi = p[i1];
		p[i1] = p[i2];
		p[i2] = tmpi;
		
		tmpb = state[i1];
		state[i1] = state[i2];
		state[i2] = tmpb;
		
		if (findex!=null) {
			tmpi = findex[i1];
			findex[i1] = findex[i2];
			findex[i2] = tmpi;
		}
	}


	// for debugging - check that L,d is the factorization of A[C,C].
	// A[C,C] has size nC*nC and leading dimension nskip.
	// L has size nC*nC and leading dimension nskip.
	// d has size nC.

	//#ifdef DEBUG_LCP -> TZ see first 'return'

	//static void checkFactorization (ATYPE A, dReal *_L, dReal *_d,
	//		int nC, int *C, int nskip)
	protected void checkFactorization (double[] A, double[]_L, double[]_d,
			int nC, int[] C, int nskip)
	{
		if (!DEBUG_LCP) return;
		int i,j;
		if (nC==0) return;

		// get A1=A, copy the lower triangle to the upper triangle, get A2=A[C,C]
		DMatrixN A1 = new DMatrixN(nC,nC);
		for (i=0; i<nC; i++) {
			for (j=0; j<=i; j++) { //A1(i,j) = A1(j,i) = AROW(i)[j];
				A1.set(i, j, AROW(i,j));
				A1.set(j, i, AROW(i,j));
			}
		}
		DMatrixN A2 = A1.newSubMatrix (nC,C,nC,C);

		// printf ("A1=\n"); A1.print(); printf ("\n");
		// printf ("A2=\n"); A2.print(); printf ("\n");

		// compute A3 = L*D*L'
		DMatrixN L = new DMatrixN(nC, nC);
		L.set(_L, nskip, 1);//new dMatrixN(nC,nC,_L,nskip,1);
		DMatrixN D = new DMatrixN(nC,nC);
		for (i=0; i<nC; i++) D.set(i, i, 1/_d[i]);//D(i,i) = 1/_d[i];
		L.clearUpperTriangle();
		for (i=0; i<nC; i++) L.set(i, i, 1);//L(i,i) = 1;
		//TODO is this correct? dMatrixN A3 = L * D * L.transpose();
		DMatrixN A3 = L.mulNew( D.mulNew( L.reTranspose() ));
//		dMatrixN A3 = L.mulNew(D).mulNew( L.transposeNew() );


		// compare A2 and A3
		double diff = A2.maxDifference (A3);
		if (diff > 1e-8)
			dDebug (0,"L*D*L' check, maximum difference = %.6e\n",diff);
	}

	//#endif


	// for debugging

	//#ifdef DEBUG_LCP -> TZ see first 'return'

	//static void checkPermutations (int i, int n, int nC, int nN, int *p, int *C)
	protected static void checkPermutations (int i, int n, int nC, int nN, 
			int[] p, int[] C)
	{
		if (!DEBUG_LCP) return;
		int j,k;
		dIASSERT (nC>=0 && nN>=0 && (nC+nN)==i && i < n);
		for (k=0; k<i; k++) dIASSERT (p[k] >= 0 && p[k] < i);
		for (k=i; k<n; k++) dIASSERT (p[k] == k);
		for (j=0; j<nC; j++) {
			int C_is_bad = 1;
			for (k=0; k<nC; k++) if (C[k]==j) C_is_bad = 0;
			dIASSERT (C_is_bad==0);
		}
	}
	//#endif // dLCP_FAST



}
