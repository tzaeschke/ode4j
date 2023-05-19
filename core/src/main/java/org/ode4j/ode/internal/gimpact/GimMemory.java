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
 * Function prototypes to allocate and free memory.
 * 
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
class GimMemory {
//	typedef void * gim_alloc_function (size_t size);
//	typedef void * gim_alloca_function (size_t size);//Allocs on the heap
//	typedef void * gim_realloc_function (void *ptr, size_t oldsize, size_t newsize);
//	typedef void gim_free_function (void *ptr, size_t size);
	interface gim_alloc_function { Object[] run(int size); };
	interface gim_alloca_function { Object[] run(int size); };//Allocs on the heap
	interface gim_realloc_function { Object[] run(Object[] ptr, int oldsize, int newsize); };
	interface gim_free_function { Object[] run(Object[] ptr, int size); };
}
