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

import org.ode4j.ode.internal.cpp4j.java.ObjArray;
import org.ode4j.ode.internal.gimpact.GimGeometry.mat4f;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec3f;

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
    gim_create_common_buffer(100*sizeof(float), bufferhandle);

    //Create a buffer array from the bufferhandle
    GBUFFER_ARRAY buffer_float_array;
    GIM_BUFFER_ARRAY_INIT_TYPE(float,buffer_float_array,bufferhandle,100);

    ////Access to the buffer data, set all elements of the array

    int i, count;
    count = buffer_float_array.m_element_count;
    //Locks the array
    gim_buffer_array_lock(buffer_float_array,G_MA_READ_WRITE);
    float  * pelements = GIM_BUFFER_ARRAY_POINTER(float, buffer_float_array, 0); // A pointer to the buffer memory

    //fill the array with random numbers
    for (i = 0;i &lt; count;i++ )
    {
        pelements[i] = gim_unit_random();
    }
    //unlock buffer
    gim_buffer_array_unlock(buffer_float_array);

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
public class GimBufferArrayFloat implements GimConstants { //formerly GBUFFER_ARRAY

	  //! Buffer managed array struct.
//	struct GBUFFER_ARRAY
//	{
//	  GBUFFER_ID m_buffer_id;
//	  char * m_buffer_data;
//	  char m_byte_stride;
//	  GUINT m_byte_offset;
//	  GUINT m_element_count;
//	  final GBUFFER_ID<vec3f> m_buffer_id = new GBUFFER_ID<vec3f>(this);
	  private vec3f[] m_buffer_data = {new vec3f()};  //TODO make final ? TZ  //TODO Why init? Was pointer!!??? TZ
		/** @deprecated TZ remove */
	  @Deprecated
      private int m_byte_stride = 1;
	  private int m_byte_offset = 0;
	  /** GUINT m_element_count; */
	  private int m_element_count;
//	};
	//typedef  struct _GBUFFER_ARRAY GBUFFER_ARRAY;


	//!Return a pointer of the element at the _index
	//#define GIM_BUFFER_ARRAY_POINTER(_type,_array_data,_index) (_type *)((_array_data).m_buffer_data + (_index)*(_array_data).m_byte_stride)
	ObjArray<vec3f> GIM_BUFFER_ARRAY_POINTER(int _index) { return new ObjArray<vec3f>(m_buffer_data, _index*m_byte_stride ); }



	//!Kernel function prototype for process streams, given a buffered array as source and
	/*!
	\param 1 the uniform arguments
	\param 2 the source stream
	\param 3 the destination stream
	*/
	//typedef void (* gim_kernel_func)(void *,GimBufferArray *,GimBufferArray *);
	interface gim_kernel_func {
		void run(Object o, GimBufferArrayFloat b1, GimBufferArrayFloat b2);
	}

	//! Generic Stream Processingp loop
	/*!

	This macro executes a kernel macro or function for each element of the streams
	\pre _src_array->m_count <= _dst_array->m_count

	\param _uniform_data An argument to be passed to the Kernel function
	\param _src_array An GBUFFER_ARRAY structure passed as the source stream
	\param _dst_array An GBUFFER_ARRAY  structure passed as the source stream
	\param _kernel Macro or function of the kernel
	\param _src_type Required. Type of all elements of the source stream
	\param _dst_type Required. Type of all elements of the dest stream
	*/
	//#define GIM_PROCESS_BUFFER_ARRAY(_uniform_data, _src_array, _dst_array, _kernel, _src_type, _dst_type) 
	static void GIM_PROCESS_BUFFER_ARRAY(mat4f _uniform_data, GimBufferArrayFloat _src_array, 
			GimBufferArrayFloat _dst_array, 
			GIM_PROCESS_BUFFER_ARRAY_FN _kernel)//, Object _src_type, Object _dst_type)
	{ 
	
//		_src_array.gim_buffer_array_lock(G_MA_READ_ONLY); 
//		_dst_array.gim_buffer_array_lock(G_MA_WRITE_ONLY); 
//	
	    int _count_=_src_array.m_element_count;
	
//	    Ref<_src_type> _source_vert_; 
//	    Ref<_dst_type> _dest_vert_; 
	    ObjArray<vec3f> _source_vert_; 
	    ObjArray<vec3f> _dest_vert_; 
	    //if (GIM_BUFFER_ARRAY_IS_ALIGNED(_src_type, _src_array) && GIM_BUFFER_ARRAY_IS_ALIGNED(_dst_type, _dst_array)) 
//	    if (_src_array.GIM_BUFFER_ARRAY_IS_ALIGNED() && _dst_array.GIM_BUFFER_ARRAY_IS_ALIGNED())
//	    { 
	
	        _source_vert_ = _src_array.GIM_BUFFER_ARRAY_POINTER(0); 
	        _dest_vert_ = _dst_array.GIM_BUFFER_ARRAY_POINTER(0); 
	        for (int _i_ = 0;_i_< _count_; _i_++) 
	        { 
	            _kernel.run(_uniform_data, _source_vert_.at0(), _dest_vert_.at0()); 
	            _source_vert_.inc();//++; 
	            _dest_vert_.inc();//++; 
	        } 
//	    } 
//	    else 
//	    { 
//	        for (_i_ = 0; _i_ < _count_; _i_++) 
//	        { 
//	            _source_vert_ = _src_array.GIM_BUFFER_ARRAY_POINTER(_i_); 
//	            _dest_vert_ = _dst_array.GIM_BUFFER_ARRAY_POINTER(_i_); 
//	            _kernel.run(_uniform_data, _source_vert_.at0(), _dest_vert_.at0()); 
//	        } 
//	        throw new IllegalStateException("Stride should always be 1"); //TODO remove?
//	    } 
//	    _src_array.gim_buffer_array_unlock(); 
//	    _dst_array.gim_buffer_array_unlock();
	} 

	public interface GIM_PROCESS_BUFFER_ARRAY_FN {
		void run(mat4f m, vec3f src, vec3f dst);
	}
	
	//! @}

	
	
	GimBufferArrayFloat() {}


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


	public GimBufferArrayFloat cloneRefs() {
		GimBufferArrayFloat c = new GimBufferArrayFloat();
		c.m_buffer_data = m_buffer_data;
		c.m_element_count = m_element_count;
		c.m_byte_offset = m_byte_offset;
		c.m_byte_stride = m_byte_stride;
		return c;
	}
	

	public GimBufferArrayFloat cloneValues() {
		GimBufferArrayFloat c = new GimBufferArrayFloat();
		c.m_buffer_data = new vec3f[m_buffer_data.length];
		//complete clone, not only used elements. Simply to avoid NullPointerExceptions
		for (int i = 0; i < m_buffer_data.length; i++) {
			vec3f v2 = new vec3f();
			vec3f v = m_buffer_data[i];
			v2.f[0] = v.f[0];
			v2.f[1] = v.f[1];
			v2.f[2] = v.f[2];
			c.m_buffer_data[i] = v2;
		}
		c.m_element_count = m_element_count;
		c.m_byte_offset = m_byte_offset;
		c.m_byte_stride = m_byte_stride;
		return c;
	}

	public static GimBufferArrayFloat createCopy(float[] array) {
		GimBufferArrayFloat c = new GimBufferArrayFloat();
		vec3f[] va = new vec3f[array.length/3];
    	for (int i = 0; i < va.length; i++) {
    		vec3f v = new vec3f();
    		v.f[0] = array[i*3];
    		v.f[1] = array[i*3+1];
    		v.f[2] = array[i*3+2];
    		va[i] = v;
    	}
    	c.m_buffer_data = va;
    	c.m_element_count = va.length;
		return c;
	}

	public static GimBufferArrayFloat createRef(float[] array) {
    	//System.out.println("WARNING creating ref-copies of float[] is not supported.!");//TODO
		GimBufferArrayFloat c = new GimBufferArrayFloat();
		vec3f[] va = new vec3f[array.length/3];
    	for (int i = 0; i < va.length; i++) {
    		vec3f v = new vec3f();
    		v.f[0] = array[i*3];
    		v.f[1] = array[i*3+1];
    		v.f[2] = array[i*3+2];
    		va[i] = v;
    	}
    	c.m_buffer_data = va;
    	c.m_element_count = va.length;
		return c;
	}


	public int size() {
		return m_element_count;
	}
	
}
