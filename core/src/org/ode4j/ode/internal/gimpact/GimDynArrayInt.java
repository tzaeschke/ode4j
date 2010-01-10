package org.ode4j.ode.internal.gimpact;

import java.util.Arrays;

public class GimDynArrayInt {

	protected int[] m_pdata = new int[0];
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
	    a.m_reserve_size = (reserve_size);
	    return a;
	} 

	public void GIM_CREATE_BOXQUERY_LIST() {
		G_ARRAY_GROW_SIZE = GimDynArray.G_ARRAY_GROW_SIZE;
	}

	/** Inserts an element at the last position. */
	//#define GIM_DYNARRAY_PUSH_ITEM(type, array_data, item)\
	void GIM_DYNARRAY_PUSH_ITEM(int item)
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
	void GIM_DYNARRAY_RESERVE_SIZE(int old_size, int reserve_size) 
	{ 
	    if (reserve_size > m_reserve_size) 
	    { 
	        //m_pdata = new int[reserve_size]; 
	        	//(char *) gim_realloc((array_data).m_pdata, (old_size) * sizeof(type), (reserve_size) * sizeof(type)); 
	        m_pdata = Arrays.copyOf(m_pdata, reserve_size);
	        m_reserve_size = reserve_size; 
	    } 
	}

	/** @deprecated remove? */
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


}
