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
 * @param <T> Type
 */
class GimBufferArray<T> implements GimConstants { //formerly GBUFFER_ARRAY

	  //! Buffer managed array struct.
//	struct GBUFFER_ARRAY
//	{
//	  GBUFFER_ID m_buffer_id;
//	  char * m_buffer_data;
//	  char m_byte_stride;
//	  GUINT m_byte_offset;
//	  GUINT m_element_count;
//	  final GBUFFER_ID<vec3f> m_buffer_id = new GBUFFER_ID<vec3f>(this);
	  Object[] m_buffer_data;//{new vec3f()};  //TODO make final ? TZ  //TODO Why init? Was pointer!!??? TZ
		/** @deprecated TZ remove */
        @Deprecated
        int m_byte_stride = 1;
	  int m_byte_offset = 0;
	  /** GUINT m_element_count; */
	  int m_element_count;
//	};
	//typedef  struct _GBUFFER_ARRAY GBUFFER_ARRAY;


//	public GimBufferArray<T> cloneRefs() {
//		GimBufferArray<T> c = new GimBufferArray<T>();
//		c.m_buffer_data = m_buffer_data;
//		c.m_element_count = m_element_count;
//		c.m_byte_offset = m_byte_offset;
//		c.m_byte_stride = m_byte_stride;
//		return c;
//	}
//	
//
//	public GimBufferArray<T> cloneValues() {
//		GimBufferArray<T> c = new GimBufferArray<T>();
//		c.m_buffer_data = new vec3f[m_buffer_data.length];
//		//complete clone, not only used elements. Simply to avoid NullPointerExceptions
//		for (int i = 0; i < m_buffer_data.length; i++) {
//			T v2 = m_buffer_data[i].clone();
//			c.m_buffer_data[i] = v2;
//		}
//		c.m_element_count = m_element_count;
//		c.m_byte_offset = m_byte_offset;
//		c.m_byte_stride = m_byte_stride;
//		return c;
//	}
//
//	public static GimBufferArray<T> createCopy(T[] array) {
//    	System.out.println("XYZZ THis is expensive!");
//    	GimBufferArray<T> c = new GimBufferArray<T>();
//		vec3f[] va = new vec3f[array.length/3];
//    	for (int i = 0; i < va.length; i++) {
//			T v2 = m_buffer_data[i].clone();
//			c.m_buffer_data[i] = v2;
//    		va[i] = v;
//    	}
//    	c.m_buffer_data = va;
//    	c.m_element_count = va.length;
//		return c;
//	}
//
//	public static GimBufferArray createRef(float[] array) {
//    	System.out.println("XYZZ THis is expensive!");
//    	System.out.println("WARNING creating ref-copies of float[] is not supported.!");
//    	GimBufferArray c = new GimBufferArray();
//		vec3f[] va = new vec3f[array.length/3];
//    	for (int i = 0; i < va.length; i++) {
//    		vec3f v = new vec3f();
//    		v.f[0] = array[i*3];
//    		v.f[1] = array[i*3+1];
//    		v.f[2] = array[i*3+2];
//    		va[i] = v;
//    	}
//    	c.m_buffer_data = va;
//    	c.m_element_count = va.length;
//		return c;
//	}
	
}
