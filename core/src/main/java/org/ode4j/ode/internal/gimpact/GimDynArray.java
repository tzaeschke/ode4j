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

import java.lang.reflect.Array;
import java.util.Arrays;

import org.ode4j.ode.internal.cpp4j.java.ObjArray;

/**
 * Dynamic Arrays. Allocated from system memory.
 * <ul>
 * <li> For initializes a dynamic array, use GIM_DYNARRAY_CREATE or GIM_DYNARRAY_CREATE_SIZED.
 * <li> When an array is no longer used, must be terminated with the macro GIM_DYNARRAY_DESTROY.
 * </ul>
 * 
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 * @param <T> Type
 */
public class GimDynArray<T> {//extends GimBufferArray<T> {
	//! @{
	//#define G_ARRAY_GROW_SIZE 64
	static final int G_ARRAY_GROW_SIZE = 64;
	//#define G_ARRAY_BUFFERMANAGER_INIT_SIZE 2
	static final int G_ARRAY_BUFFERMANAGER_INIT_SIZE = 2;

	//! Dynamic array handle.
//	static class GDYNAMIC_ARRAY<T>
//	{
//	    char * m_pdata;
//	    GUINT m_size;
//	    GUINT m_reserve_size;
	    private T[] m_pdata;
	    protected int m_size;
	    private int m_reserve_size;
//	}
	//typedef  struct _GDYNAMIC_ARRAY GDYNAMIC_ARRAY;

	private GimDynArray() {
		//private!
	}
	    
	//! Creates a dynamic array zero sized
	//#define GIM_DYNARRAY_CREATE(type, array_data, reserve_size) \
	@SuppressWarnings("unchecked")
	static <T> GimDynArray<T> GIM_DYNARRAY_CREATE(Class<T> clazz, int reserve_size)
	{ 
		GimDynArray<T> a = new GimDynArray<T>();
	    //(array_data).m_pdata = (char *)gim_alloc((reserve_size) * sizeof(type));
		a.m_pdata = (T[]) Array.newInstance(clazz, reserve_size); //gim_alloc((reserve_size));// * sizeof(type));
	    a.m_size = 0; 
	    a.m_reserve_size = reserve_size;
	    return a;
	} 

	/**
	 * Creates a dynamic array with n = size elements.
	 * TODO (TZ) should be deprecated because it won't initialize the array with elements,
	 * but still sets the 'size' to n.  
	 */
	//#define GIM_DYNARRAY_CREATE_SIZED(type, array_data, size) \
	
	@SuppressWarnings("unchecked")
	static <T> GimDynArray<T> GIM_DYNARRAY_CREATE_SIZED(int size) 
	{ 
		GimDynArray<T> a = new GimDynArray<T>();
	    //(array_data).m_pdata = (char *)gim_alloc((size) * sizeof(type)); 
	    a.m_pdata = (T[]) new Object[size];//gim_alloc(size);// * sizeof(type)); 
	    a.m_size = size; 
	    a.m_reserve_size = size;
	    return a;
	} 

	//! Reserves memory for a dynamic array.
	//#define GIM_DYNARRAY_RESERVE_SIZE(type, array_data, old_size, reserve_size) \
	void GIM_DYNARRAY_RESERVE_SIZE(int old_size, int reserve_size) 
	{ 
	    if (reserve_size > m_reserve_size) 
	    { 
//	        m_pdata = //(T[]) new Object[reserve_size]; //TODO add old_size ??? TZ
//	        	//(char *) gim_realloc((array_data).m_pdata, (old_size) * sizeof(type), (reserve_size) * sizeof(type)); 
//	        gim_realloc(m_pdata, old_size, reserve_size);
	    	m_pdata = Arrays.copyOf(m_pdata, reserve_size);
	        m_reserve_size = reserve_size; 
	    } 
	} 

//	//! Set the size of the array
//	//#define GIM_DYNARRAY_SET_SIZE(type, array_data, size) \
//	void GIM_DYNARRAY_SET_SIZE(int size) 
//	{ 
//	    GIM_DYNARRAY_RESERVE_SIZE(m_size, size); 
//	    m_size = size; 
//	} 

	//! Gets a pointer from the beginning of the array
	//#define GIM_DYNARRAY_POINTER(type, array_data) ((type *)((array_data).m_pdata))
	public T[] GIM_DYNARRAY_POINTER() { return m_pdata; }
	public static <T> T[] GIM_DYNARRAY_POINTER(GimDynArray<T> array) { return array.m_pdata; }
	public ObjArray<T> GIM_DYNARRAY_POINTER_V() { return new ObjArray<T>(m_pdata); }

	//! Gets a pointer from the last elemento of the array
	//#define GIM_DYNARRAY_POINTER_LAST(type, array_data) (((type *)(array_data).m_pdata) + ((array_data).m_size - 1))
	T GIM_DYNARRAY_POINTER_LAST() { return m_pdata[m_size - 1]; }

	//! Inserts an element at the last position
	//#define GIM_DYNARRAY_PUSH_ITEM(type, array_data, item)\
	/** Renamed, because it now inserts the element instead of cloning it. TZ */
	void GIM_DYNARRAY_PUSH_ITEM_TZ(T item)
	{ 
	    if (m_reserve_size <= m_size)
	    {
	        GIM_DYNARRAY_RESERVE_SIZE(m_size, m_size + G_ARRAY_GROW_SIZE); 
	    }
	    T[] _pt = GIM_DYNARRAY_POINTER(); 
	    //memcpy(_pt[m_size], (item), 1);//sizeof(type));
	    _pt[m_size] = item;//.clone();
	    m_size++; 
	} 

	//! Inserts an element at the last position
	//#define GIM_DYNARRAY_PUSH_EMPTY(type, array_data) \
	void GIM_DYNARRAY_PUSH_EMPTY() 
	{ 
	    if (m_reserve_size <= m_size) 
	    { 
	        GIM_DYNARRAY_RESERVE_SIZE(m_size, m_size + G_ARRAY_GROW_SIZE); 
	    } 
	    m_size++; 
	} 

	//! Inserts an element
	//#define GIM_DYNARRAY_INSERT_ITEM(type, array_data, item, index) \
//	void GIM_DYNARRAY_INSERT_ITEM(T item, int index) 
//	{ 
//	    if (m_reserve_size <= m_size) 
//	    { 
//	        GIM_DYNARRAY_RESERVE_SIZE( m_size, m_size + G_ARRAY_GROW_SIZE); 
//	    } 
//	    T[] _pt = GIM_DYNARRAY_POINTER(); 
//	    if (index < m_size - 1) 
//	    { 
//	        memmove(_pt[(index) + 1], _pt[(index)], (m_size - (index)) * sizeof(type)); 
//	    } 
//	    memcpy(_pt[(index)], (item), sizeof(type)); 
//	    m_size++; 
//	} 

	//! Removes an element
	//#define GIM_DYNARRAY_DELETE_ITEM(type, array_data, index) \
//	void GIM_DYNARRAY_DELETE_ITEM(int index) 
//	{ 
//	    if (index < m_size - 1) 
//	    { 
//	        T[] _pt = GIM_DYNARRAY_POINTER();
//	        memmove(_pt[(index)], _pt[(index) + 1], (m_size - (index) - 1) * sizeof(type)); 
//	    } 
//	    m_size--; 
//	} 
	public void GIM_DYNARRAY_DELETE_ITEM(int index)
	{
	    if (index < m_size - 1) {
	        T[] _pt = GIM_DYNARRAY_POINTER();
			System.arraycopy(_pt, index + 1, _pt, index, m_size - index - 1);
	    }
	    m_size--;
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

	//! Destroys the array
	//void GIM_DYNARRAY_DESTROY(GDYNAMIC_ARRAY & array_data);
	public void GIM_DYNARRAY_DESTROY()
	{
	    if(m_pdata != null)
	    {
	        m_pdata = null;//TZ gim_free(m_pdata,0);
	        m_reserve_size = 0;
	        m_size = 0;
	        m_pdata = null;
	    }
	}
	//! @}
	
	public int size() {
		return m_size;
	}
	
	public boolean isActive() {
		return m_pdata != null;
	}

	public void makeInactive() {
		m_pdata = null;
	}
}
