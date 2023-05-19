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

import org.ode4j.ode.internal.cpp4j.java.IntArray;

/**
 * Buffered Arrays, for manip elements on a buffer and treat it as an array.
 * <ul>
 * <li> Before using buffer arrays you must initializes GIMPACT buffer managers by calling gimpact_init.
 * <li> Before creating buffer arrays, you must create a buffer. see \ref BUFFERS.
 * <li> Create a buffer narray by calling \ref GIM_BUFFER_ARRAY_INIT_TYPE, \ref GIM_BUFFER_ARRAY_INIT_TYPE_OFFSET or \ref GIM_BUFFER_ARRAY_INIT_OFFSET_STRIDE.
 * <li> For accessing to the array elements, you must call \ref gim_buffer_array_lock, and then \ref gim_buffer_array_unlock for finish the access.
 * <li> When a buffer array is no longer needed, you must free it by calling \ref GIM_BUFFER_ARRAY_DESTROY.
 * </ul>
 * The following example shows how Buffer arrays can be used:
 *
 * <pre>
int main()
{
    //init gimpact
    gimpact_init();

    //Buffer handle to use
    GBUFFER_ID bufferhandle;

    //Create a memory buffer of 100 float numbers
    gim_create_common_buffer(100*sizeof(float), &amp;bufferhandle);

    //Create a buffer array from the bufferhandle
    GBUFFER_ARRAY buffer_float_array;
    GIM_BUFFER_ARRAY_INIT_TYPE(float,buffer_float_array,bufferhandle,100);

    ////Access to the buffer data, set all elements of the array

    int i, count;
    count = buffer_float_array.m_element_count;
    //Locks the array
    gim_buffer_array_lock(&amp; buffer_float_array,G_MA_READ_WRITE);
    float  * pelements = GIM_BUFFER_ARRAY_POINTER(float, buffer_float_array, 0); // A pointer to the buffer memory

    //fill the array with random numbers
    for (i = 0;i &lt; count;i++ )
    {
        pelements[i] = gim_unit_random();
    }
    //unlock buffer
    gim_buffer_array_unlock(&amp; buffer_float_array);

    //Program code
        ....
        ....

    //Destroy array
    GIM_BUFFER_ARRAY_DESTROY(buffer_float_array);

    //terminate gimpact
    gimpact_terminate();
}
 * </pre>
 * 
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
public class GimBufferArrayInt implements GimConstants { //formerly GBUFFER_ARRAY

	  //! Buffer managed array struct.
//	struct GBUFFER_ARRAY
//	{
//	  GBUFFER_ID m_buffer_id;
//	  char * m_buffer_data;
//	  char m_byte_stride;
//	  GUINT m_byte_offset;
//	  GUINT m_element_count;
//	  final GBUFFER_ID<vec3f> m_buffer_id = new GBUFFER_ID<vec3f>(this);
	  private int[] m_buffer_data; //= {new vec3f()};  //TODO make final ? TZ  //TODO Why init? Was pointer!!??? TZ
		/** @deprecated TZ remove */
	  @Deprecated
      private final int m_byte_stride = 1;
	  //private final int m_byte_offset = 0;
	  /** GUINT m_element_count; */
	  private int m_element_count;
//	};
	//typedef  struct _GBUFFER_ARRAY GBUFFER_ARRAY;


	private GimBufferArrayInt() {}

	//!Return a pointer of the element at the _index
	//#define GIM_BUFFER_ARRAY_POINTER(_type,_array_data,_index) (_type *)((_array_data).m_buffer_data + (_index)*(_array_data).m_byte_stride)
	IntArray GIM_BUFFER_ARRAY_POINTER(int _index) { return new IntArray(m_buffer_data, _index*m_byte_stride ); }




	//! Destroys an GBUFFER_ARRAY object
	/*!
	\post Attemps to destroy the buffer, decreases reference counting
	*/
//	void GIM_BUFFER_ARRAY_DESTROY(GBUFFER_ARRAY & array_data);
//	void GIM_BUFFER_ARRAY_DESTROY(GimBufferArray<T> array_data)
//	{
//	    gim_buffer_array_unlock(array_data);
//	    gim_buffer_free(array_data.m_buffer_id);
//	}
	void GIM_BUFFER_ARRAY_DESTROY() //TZ
	{
//	    gim_buffer_array_unlock();
//	    gim_buffer_free(m_buffer_id);
	}

	

	public GimBufferArrayInt cloneValues() {
		GimBufferArrayInt c = new GimBufferArrayInt();
		c.m_buffer_data = Arrays.copyOf(m_buffer_data, m_buffer_data.length);
		c.m_element_count = m_element_count;
//		c.m_byte_offset = m_byte_offset;
//		c.m_byte_stride = m_byte_stride;
		return c;
	}

	public static GimBufferArrayInt createCopy(int[] array) {
    	GimBufferArrayInt c = new GimBufferArrayInt();
    	c.m_buffer_data = Arrays.copyOf(array, array.length);
    	c.m_element_count = array.length;
		return c;
	}

	public static GimBufferArrayInt createRef(int[] array) {
    	GimBufferArrayInt c = new GimBufferArrayInt();
    	c.m_buffer_data = array;
    	c.m_element_count = array.length;
		return c;
	}


	public int getElementCount() {
		return m_element_count;
	}

}
