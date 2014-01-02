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

import java.util.BitSet;

/**
 * Bitsets, based on \ref DYNAMIC_ARRAYS .
 * <ul>
 * <li> For initializes a bitset array, use \ref GIM_BITSET_CREATE or \ref GIM_BITSET_CREATE_SIZED.
 * <li> When the bitset is no longer used, must be terminated with the macro \ref GIM_DYNARRAY_DESTROY.
 * <li> For putting a mark on the bitset, call \ref GIM_BITSET_SET
 * <li> For clearing a mark on the bitset, call \ref GIM_BITSET_CLEAR
 * <li> For retrieving a bit value from a bitset, call \ref GIM_BITSET_GET-
 * </ul>
 * 
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
*/
public class GimBitSet {//extends GimDynArrayInt { //implements GimConstants {

	private final BitSet _data;
	
	
	//! Creates a bitset
//	public GimBitSet GIM_BITSET_CREATE(GimBitSet array_data) { GIM_DYNARRAY_CREATE(GUINT, array_data, G_ARRAY_GROW_SIZE); }

	private GimBitSet(int size) {
		//private !
		_data = new BitSet(size);
	}
	

	/** Creates a bitset, with their bits set to 0. */
	static GimBitSet GIM_BITSET_CREATE_SIZED(int bits_count) { 
		return new GimBitSet(bits_count);
	} 

	
	/** Sets all bitset bit to 0. */
	void GIM_BITSET_CLEAR_ALL() 	{ 
		_data.clear();
	} 

	
	/** Sets the desired bit to 1. */
	void GIM_BITSET_SET(int bit_index) { 
		_data.set(bit_index);
	} 

	
	/** Return 0 or 1. */
	boolean GIM_BITSET_GET(int bit_index) {
		return _data.get(bit_index);
	} 


	/** Sets the desired bit to 0.  */
	void GIM_BITSET_CLEAR(int bit_index) {
		_data.clear(bit_index);
	}


	public void GIM_DYNARRAY_DESTROY() {
		// Nothing to do
	}

	
	
//	/** Creates a bitset, with their bits set to 0. */
//	//static GimBitSet GIM_BITSET_CREATE_SIZED(GimBitSet array_data, int bits_count) 
//	static GimBitSet GIM_BITSET_CREATE_SIZED(int bits_count)
//	{ 
//	    int array_size = bits_count / GUINT_BIT_COUNT + 1; 
//	    
//	    
//	    //GIM_DYNARRAY_CREATE(array_size);//GUINT, array_data, array_size); 
//	    //->	    
//		GimBitSet a = new GimBitSet();
//	    //(array_data).m_pdata = (char *)gim_alloc((reserve_size) * sizeof(type));
//		a.m_pdata = new int[array_size];//gim_alloc((reserve_size));// * sizeof(type));
//	    a.m_size = 0; 
//	    a.m_reserve_size = array_size;
//	    return a;
//	    
//	    //int[] _pt = a.GIM_DYNARRAY_POINTER(); 
//	    //memset(_pt, 0, sizeof(GUINT) * ((array_data).m_size));  //TZ no need to set to 0; 
//	} 
//
//	/** Gets the bitset bit count. */
//	int GIM_BITSET_SIZE() { return (m_size * GUINT_BIT_COUNT); }
//
//	/** Resizes a bitset, with their bits set to 0. */
//	//#define GIM_BITSET_RESIZE(array_data, new_bits_count) \
//	void GIM_BITSET_RESIZE(int new_bits_count) 
//	{ 
//	    int _oldsize = m_size; 
//	    m_size = new_bits_count / GUINT_BIT_COUNT + 1; 
//	    if (_oldsize < m_size) 
//	    { 
//	        if (m_size > m_reserve_size) 
//	        { 
//	            GIM_DYNARRAY_RESERVE_SIZE(_oldsize, m_size + G_ARRAY_GROW_SIZE); 
//	        } 
//	        //int[] _pt = GIM_DYNARRAY_POINTER(); 
//	        //TZ TODO ? memset(_pt[_oldsize], 0, m_size - _oldsize);//sizeof(GUINT) * ((array_data).m_size - _oldsize)); 
//	    } 
//	} 
//
//	/** Sets all bitset bit to 0. */
//	//#define GIM_BITSET_CLEAR_ALL(array_data) \
//	void GIM_BITSET_CLEAR_ALL() 
//	{ 
//	    //memset(m_pdata, 0, m_size);//sizeof(GUINT) * (array_data).m_size);
//		Arrays.fill(m_pdata, 0);  //TODO that does not set it to NULL !!!!  Use Java BitSet or 'long'!
//	} 
//
//	//! Sets all bitset bit to 1.
//	//#define GIM_BITSET_SET_ALL(array_data) \
////	public void GIM_BITSET_SET_ALL(GimBitSet array_data) 
////	{ 
////	    memset((array_data).m_pdata, 0xFF, sizeof(GUINT) * (array_data).m_size); 
////	} 
//
//	/** Sets the desired bit to 1. */
//	//#define GIM_BITSET_SET(array_data, bit_index) \
//	void GIM_BITSET_SET(int bit_index) 
//	{ 
//	    if (bit_index >= GIM_BITSET_SIZE()) 
//		{ 
//		    GIM_BITSET_RESIZE(bit_index); 
//		} 
//		int[] _pt = GIM_DYNARRAY_POINTER(); 
//		_pt[bit_index >>> GUINT_EXPONENT] |= (1 << (bit_index & (GUINT_BIT_COUNT - 1))); 
//	} 
//
//	/** Return 0 or 1. */
//	//#define GIM_BITSET_GET(array_data, bit_index, get_value) \
//	boolean GIM_BITSET_GET(int bit_index) 
//	{ 
//	    if (bit_index >= GIM_BITSET_SIZE()) 
//		{ 
//		    return false; 
//		} 
//		else 
//		{ 
//	        int[] _pt = GIM_DYNARRAY_POINTER(); 
//	        return 0 != (_pt[bit_index >>> GUINT_EXPONENT] & (1 << (bit_index & (GUINT_BIT_COUNT - 1)))); 
//		} 
//	} 
//
//	/** Sets the desired bit to 0.  */
//	//#define GIM_BITSET_CLEAR(array_data, bit_index) \
//	void GIM_BITSET_CLEAR(int bit_index) 
//	{ 
//	    if (bit_index < GIM_BITSET_SIZE()) 
//		{ 
//	        int[] _pt = GIM_DYNARRAY_POINTER(); 
//	        _pt[bit_index >>> GUINT_EXPONENT] &= ~(1 << (bit_index & (GUINT_BIT_COUNT - 1))); 
//		} 
//	}
}
