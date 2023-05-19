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

import java.util.Arrays;

/**
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
public class GimDynArrayInt {

	protected int[] m_pdata;// = new int[0];
	protected int G_ARRAY_GROW_SIZE;
	protected int m_size = 0;
	protected int m_reserve_size = 0;

	//! Creates a dynamic array zero sized
	//#define GIM_DYNARRAY_CREATE(type, array_data, reserve_size) \
	static GimDynArrayInt GIM_DYNARRAY_CREATE(int reserve_size) 
	{ 
		GimDynArrayInt a = new GimDynArrayInt();
	    //(array_data).m_pdata = (char *)gim_alloc((reserve_size) * sizeof(type));
		a.m_pdata = new int[reserve_size];//gim_alloc((reserve_size));// * sizeof(type));
	    a.m_size = 0; 
	    a.m_reserve_size = reserve_size;
	    return a;
	} 

	public static GimDynArrayInt GIM_CREATE_BOXQUERY_LIST() {
		GimDynArrayInt a = new GimDynArrayInt();
		a.G_ARRAY_GROW_SIZE = GimDynArray.G_ARRAY_GROW_SIZE;
		a.m_reserve_size = a.G_ARRAY_GROW_SIZE;
		a.m_pdata = new int[a.m_reserve_size];
		a.m_size = 0;
		return a;
	}

	private GimDynArrayInt() {}

	/** Inserts an element at the last position. */
	//#define GIM_DYNARRAY_PUSH_ITEM(type, array_data, item)\
	void GIM_DYNARRAY_PUSH_ITEM(final int item)
	{ 
	    if (m_reserve_size <= m_size)
	    {
	        GIM_DYNARRAY_RESERVE_SIZE(m_size, m_size + G_ARRAY_GROW_SIZE); 
	    }
//	    int[] _pt = GIM_DYNARRAY_POINTER(); 
//	    memcpy(_pt[(array_data).m_size], (item), 1);//sizeof(type));
	    m_pdata[m_size] = item;
	    m_size++; 
	} 
	
	/** Reserves memory for a dynamic array. */
	//#define GIM_DYNARRAY_RESERVE_SIZE(type, array_data, old_size, reserve_size) \
	void GIM_DYNARRAY_RESERVE_SIZE(final int old_size, final int reserve_size) 
	{ 
	    if (reserve_size > m_reserve_size) 
	    { 
	        //m_pdata = new int[reserve_size]; 
	        	//(char *) gim_realloc((array_data).m_pdata, (old_size) * sizeof(type), (reserve_size) * sizeof(type)); 
	        m_pdata = Arrays.copyOf(m_pdata, reserve_size);
	        m_reserve_size = reserve_size; 
	    } 
	}

	/** TODO remove? (TZ) */
	public void GIM_DYNARRAY_DESTROY() {
		// TODO Auto-generated method stub
		
	}

	public int[] GIM_DYNARRAY_POINTER() {
		return m_pdata;
	}


	//! Removes an element at the last position
	//#define GIM_DYNARRAY_POP_ITEM(array_data) \
	void GIM_DYNARRAY_POP_ITEM() 
	{ 
	    if (m_size > 0) 
	    { 
	        m_size--; 
	    } 
	}

	
	public int size() {
		return m_size;
	} 

	/**
	 * Sets size to 0. Does not clean any elements.
	 */
	public void clear() {
		m_size = 0;
	}
}
