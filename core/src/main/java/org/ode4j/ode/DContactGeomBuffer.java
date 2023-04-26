/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
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
package org.ode4j.ode;

import org.ode4j.ode.internal.DxGeom;


/**
 * A collection of DContactGeom objects.
 */
public final class DContactGeomBuffer {
	
	private final DContactBuffer _buf;
	private int _ofs = 0;

	public DContactGeomBuffer(int size) {
		_buf = new DContactBuffer(size);
	}

	public DContactGeomBuffer(DContactBuffer contactBuffer) {
		_buf = contactBuffer;
	}

	private DContactGeomBuffer(DContactGeomBuffer contactBuffer, int ofs) {
		_buf = contactBuffer._buf;
		_ofs = contactBuffer._ofs + ofs;
	}

	/** 
	 * 
	 * @return The first contact.
	 */
	public DContactGeom get() {
		return get(0);
	}

	public DContactGeom get(int i) {
		return _buf.get(i+_ofs).geom;
	}

	public DContactGeomBuffer createView(int skip) {
		return new DContactGeomBuffer(this, skip);
	}

	public DContactGeom getSafe(int flags, int index) {
		if (!(index >= 0 && index < (flags & DxGeom.NUMC_MASK))) {
			throw new IllegalStateException("Index="+index + "; flags="+flags);
		}
		return get(index);
	}

	public void swap(int pos1, int pos2) {
		_buf.swap(pos1 + _ofs, pos2 + _ofs);
	}

	//DO NOT IMPLEMENT! This would affect the objects in the calling class!!!
	//	/**
	//	 * Increments the offset of the current buffer by <tt>skip</tt>.
	//	 * @param skip
	//	 * @return Pointer to the current buffer.
	//	 */
	//	public dContactGeomBuffer inc(int skip) {
	//		_ofs += skip;
	//		return this;
	//	}
}
