/**
 * ----------------------------------------------------------------------------
 * This source file is part of the ODE4J library (ported to
 * Java from the GIMPACT Library).
 * 
 * For the latest info on ODE4J, see http://www.ode4j.org/
 * For the latest info on GIMPACT, see http://gimpact.sourceforge.net/
 * 
 * Copyright of GIMPACT (c) 2006 Francisco Leon. C.C. 80087371.
 * email: projectileman@yahoo.com
 * Copyright of ODE4J (c) 2009-2014 Tilmann Zäschke.
 * email: ode4j.gmx.de
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this library in the
 *       file GIMPACT-LICENSE-LGPL.TXT and LICENSE.TXT.
 *   (2) The BSD-style license that is included with this library in
 *       the file GIMPACT-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-BSD.TXT, LICENSE.TXT and 
 * ODE4J-LICENSE-BSD.TXT for more details.
 * 
 * ----------------------------------------------------------------------------
 */
package org.ode4j.ode.internal.gimpact;



/**
 * Based on the work of Michael Herf : "fast floating-point radix sort"
 * Avaliable on http://www.stereopsis.com/radix.html
 * 
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
public class GimRadixSort {

	/**
	 * Macros for sorting.
	 */
	static class GIM_RSORT_TOKEN
	{
		//	    GUINT32 m_key;
		//	    GUINT32 m_value;
		long m_key;
		int m_value;
	};
	//typedef struct _GIM_RSORT_TOKEN GIM_RSORT_TOKEN;

	//comparator for sorting
	//#define RSORT_TOKEN_COMPARATOR(x, y) ((int)((x.m_key) - (y.m_key)))
	private static long RSORT_TOKEN_COMPARATOR(GIM_RSORT_TOKEN x, GIM_RSORT_TOKEN y) { return x.m_key - y.m_key; }
	interface GimRSortTokenComparator {
		long run(GIM_RSORT_TOKEN x, GIM_RSORT_TOKEN y);
	}
	static final GimRSortTokenComparator RSORT_TOKEN_COMPARATOR = new GimRSortTokenComparator() {
		@Override public long run(GIM_RSORT_TOKEN x, GIM_RSORT_TOKEN y) {
			return RSORT_TOKEN_COMPARATOR(x, y);
		}
	};

	// ---- utils for accessing 11-bit quantities
	//	#define D11_0(x)	(x & 0x7FF)
	//	#define D11_1(x)	(x >> 11 & 0x7FF)
	//	#define D11_2(x)	(x >> 22 )
	private static int D11_0(long x) { return (int) (x & 0x7FFl); }
	private static int D11_1(long x) { return (int) (x >>> 11 & 0x7FFl); }
	private static int D11_2(long x) { return (int) (x >>> 22 & 0x3FFl ); }


	//COMMON FUNCTIONS FOR ACCESSING THE KEY OF AN ELEMENT


	//For the type of your array, you need to declare a macro for obtaining the key, like these:
	//#define SIMPLE_GET_FLOAT32KEY(e,key) {key =(GREAL)(e);}
//	private float SIMPLE_GET_FLOAT32KEY(int e) {return (float)e;}
//
//	//#define SIMPLE_GET_INTKEY(e,key) {key =(GINT32)(e);}
//	private int SIMPLE_GET_INTKEY(float e) {return (int)e;}
//
//	//#define SIMPLE_GET_UINTKEY(e,key) {key =(GUINT32)(e);}
//	private long SIMPLE_GET_UINTKEY(float e) {return (long)e;}

	//For the type of your array, you need to declare a macro for copy elements, like this:

	//#define SIMPLE_COPY_ELEMENTS(dest,src) {dest = src;}
	//TZ not used? private Object SIMPLE_COPY_ELEMENTS(Object src) {return src.clone();}

	//#define kHist 2048
	private static final short kHist = 2048;

	///Radix sort for unsigned integer keys

	//#define GIM_RADIX_SORT_RTOKENS(array,sorted,element_count)\
	static void GIM_RADIX_SORT_RTOKENS(final GIM_RSORT_TOKEN[] array, final GIM_RSORT_TOKEN[] sorted, 
			final int element_count)
	{
//                        TreeMap<Long,GIM_RSORT_TOKEN> tm = new TreeMap<Long, GIM_RSORT_TOKEN>();
//            for(GIM_RSORT_TOKEN t : array)
//                tm.put(t.m_key, t);
//            int i = 0;
//            for(GIM_RSORT_TOKEN t : tm.values())
//                sorted[i++] = t;

		int i;
		//		int[] b0 = new int[kHist * 3];
		//		IntArray b1 = new IntArray( b0, kHist );
		//		IntArray b2 = new IntArray( b1, kHist );
		int[] b0 = new int[kHist];
		int[] b1 = new int[kHist];
		int[] b2 = new int[kHist];
		//		for (i = 0; i < kHist * 3; i++)
		//		{
		//			b0[i] = 0;
		//		}
		long fi;
		int pos;
		for (i = 0; i < element_count; i++)
		{
			fi = array[i].m_key;
			b0[D11_0(fi)] ++;
			b1[D11_1(fi)] ++;
			b2[D11_2(fi)] ++;
		}
		{
			int sum0 = 0, sum1 = 0, sum2 = 0;
			int tsum;
			for (i = 0; i < kHist; i++)
			{
				tsum = b0[i] + sum0;
				b0[i] = sum0 - 1;
				sum0 = tsum;
				tsum = b1[i] + sum1;
				b1[i] = sum1 - 1;
				sum1 = tsum;
				tsum = b2[i] + sum2;
				b2[i] = sum2 - 1;
				sum2 = tsum;
			}
		}
		for (i = 0; i < element_count; i++)
		{
			fi = array[i].m_key;
			pos = D11_0(fi);
			pos = ++b0[pos];
			sorted[pos].m_key = array[i].m_key;
			sorted[pos].m_value = array[i].m_value;
		}
		for (i = 0; i < element_count; i++)
		{
			fi = sorted[i].m_key;
			pos = D11_1(fi);
			pos = ++b1[pos];
			array[pos].m_key = sorted[i].m_key;
			array[pos].m_value = sorted[i].m_value;
		}
		for (i = 0; i < element_count; i++)
		{
			fi = array[i].m_key;
			pos = D11_2(fi);
			pos = ++b2[pos];
			sorted[pos].m_key = array[i].m_key;
			sorted[pos].m_value = array[i].m_value;
		}
	}

	//	/// Get the sorted tokens from an array. For generic use. Tokens are GIM_RSORT_TOKEN
	//	//#define GIM_RADIX_SORT_ARRAY_TOKENS(array, sorted_tokens, element_count, get_uintkey_macro)\
	//	private void GIM_RADIX_SORT_ARRAY_TOKENS(final Object[] array, final boolean sorted_tokens, 
	//			final int element_count, XXX get_uintkey_macro)
	//	{
	//	    //GIM_RSORT_TOKEN[] _unsorted = (GIM_RSORT_TOKEN *) gim_alloc(sizeof(GIM_RSORT_TOKEN )*element_count);
	//	    GIM_RSORT_TOKEN[] _unsorted = new GIM_RSORT_TOKEN[element_count];
	//	    for (int _i=0;_i<element_count;_i++)
	//	    {
	//	        get_uintkey_macro(array[_i],_unsorted[_i].m_key);
	//	        _unsorted[_i].m_value = _i;
	//	    }
	//	    GIM_RADIX_SORT_RTOKENS(_unsorted,sorted_tokens,element_count);
	//	    gim_free(_unsorted,sizeof(GIM_RSORT_TOKEN )*element_count);
	//	}
	//
	//	/// Sorts array in place. For generic use
	//	//#define GIM_RADIX_SORT(type,array,element_count,get_uintkey_macro,copy_elements_macro)\
	//	private void GIM_RADIX_SORT(Class type, final Object[] array, final int element_count,
	//			XXX get_uintkey_macro, XXX copy_elements_macro)
	//	{
	//	    //GIM_RSORT_TOKEN * _sorted = (GIM_RSORT_TOKEN *) gim_alloc(sizeof(GIM_RSORT_TOKEN )*element_count);
	//		GIM_RSORT_TOKEN[] _sorted = new GIM_RSORT_TOKEN[element_count];
	//	    GIM_RADIX_SORT_ARRAY_TOKENS(array,_sorted,element_count,get_uintkey_macro);
	//	    //type * _original_array = (type *) gim_alloc(sizeof(type)*element_count);
	//	    type[] _original_array = new type[element_count];
	//	    memcpy(_original_array,array,sizeof(type)*element_count);
	//	    GUINT32 _i;
	//	    for (_i=0;_i<element_count;_i++)
	//	    {
	//	        copy_elements_macro(array[_i],_original_array[_sorted[_i].m_value]);
	//	    }
	//	    gim_free(_original_array,sizeof(type)*element_count);
	//	    gim_free(_sorted,sizeof(GIM_RSORT_TOKEN )*element_count);
	//	}

	interface GimCompMacro {
		int run(final int x, final int y);
	}
	interface GimExchangeMacro {
		void run(GIM_RSORT_TOKEN[] array, final int i, final int j);
	}

	/// Sorts array in place using quick sort
	//#define GIM_QUICK_SORT_ARRAY(type, array, array_count, comp_macro, exchange_macro) \
	static void GIM_QUICK_SORT_ARRAY(final GIM_RSORT_TOKEN[] array, final int array_count, 
			final GimRSortTokenComparator comp_macro, final GimExchangeMacro exchange_macro) 
	{
		int   _i_, _j_, _p_, _stack_index_, _start_, _end_;
		int[]   _start_stack_ = new int[64]; 
		int[]   _end_stack_ = new int[64];
		_start_stack_[0] = 0;
		_end_stack_[0] = (array_count);
		_stack_index_ = 1;
		while (_stack_index_ > 0)
		{
			_stack_index_ --;
			_start_ = _start_stack_[_stack_index_];
			_end_ = _end_stack_[_stack_index_];
			while (_end_ - _start_ > 2)
			{
				_p_ = _start_;
				_i_ = _start_ + 1;
				_j_ = _end_ - 1;
				while (_i_<_j_) 
				{
					for(; _i_<=_j_ && comp_macro.run(array[_i_], array[_p_] )<=0; _i_++) ;
					if (_i_ > _j_) 
					{
						exchange_macro.run(array, _j_, _p_);
						_i_ = _j_;
					}
					else
					{
						for(; _i_<=_j_ && comp_macro.run( array[_j_], array[_p_])>=0; _j_--) ;
						if (_i_ > _j_) 
						{
							exchange_macro.run(array, _j_, _p_);
							_i_ = _j_;
						}
						else if (_i_ < _j_)
						{
							exchange_macro.run(array, _i_, _j_);
							if (_i_+2 < _j_) {_i_++; _j_--;}
							else if (_i_+1 < _j_) _i_++;
						}
					}
				}
				if (_i_-_start_ > 1 && _end_-_j_ > 1) 
				{
					if (_i_-_start_ < _end_-_j_-1) 
					{
						_start_stack_[_stack_index_] = _j_+1;
						_end_stack_[_stack_index_] = _end_;
						_stack_index_ ++;
						_end_ = _i_;
					}
					else
					{
						_start_stack_[_stack_index_] = _start_;
						_end_stack_[_stack_index_] = _i_;
						_stack_index_ ++;
						_start_ = _j_+1;
					}
				}
				else
				{
					if (_i_-_start_ > 1)
					{
						_end_ = _i_;
					}
					else 
					{
						_start_ = _j_+1;
					}
				}
			}
			if (_end_ - _start_ == 2) 
			{
				if (comp_macro.run( array[_start_], array[_end_-1]) > 0) 
				{
					exchange_macro.run(array, _start_, _end_-1);
				}
			}
		}
	}

	//#define GIM_DEF_EXCHANGE_MACRO(type, _array, _i, _j)\
	private static void GIM_DEF_EXCHANGE_MACRO(GIM_RSORT_TOKEN[] _array, final int _i, final int _j)
	{
		GIM_RSORT_TOKEN _e_tmp_ = _array[_i];
		_array[_i] = _array[_j];
		_array[_j] = _e_tmp_;
	}

	static final GimExchangeMacro GIM_DEF_EXCHANGE_MACRO = new GimExchangeMacro() {
		@Override public void run(GIM_RSORT_TOKEN[] array, int i, int j) {
			GIM_DEF_EXCHANGE_MACRO(array, i, j);
		}
	};

	//#define GIM_COMP_MACRO(x, y) ((GINT32)((x) - (y)))
	private static int GIM_COMP_MACRO(final int x, final int y) { return x - y; }
	//! @}
	//#endif // GIM_RADIXSORT_H_INCLUDED

	static final GimCompMacro GIM_COMP_MACRO = new GimCompMacro() {
		@Override public int run(int x, int y) {
			return GIM_COMP_MACRO(x, y);
		}
	};

	private GimRadixSort() {}
}
