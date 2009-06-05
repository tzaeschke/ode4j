package org.ode4j.ode;

import java.util.ArrayList;

import org.ode4j.ode.internal.DxGeom;


/**
 * @brief A set of dContactGeom objects (TZ).
 *
 * @ingroup collide
 */
public final class DContactGeomBuffer {

	private final ArrayList<DContactGeom> _buf;
	private int _ofs = 0;

	public DContactGeomBuffer() {
		_buf = new ArrayList<DContactGeom>();
	}

	public DContactGeomBuffer(int size) {
		_buf = new ArrayList<DContactGeom>();
		for (int i = 0; i < size; i++) {
			_buf.add(new DContactGeom());
		}
	}

	public DContactGeomBuffer(DContactBuffer contactBuffer) {
		_buf = new ArrayList<DContactGeom>();
		for (DContact c: contactBuffer) {
			_buf.add(c.geom);
		}
	}

	private DContactGeomBuffer(DContactGeomBuffer contactBuffer, int ofs) {
		_buf = contactBuffer._buf;
		_ofs = contactBuffer._ofs + ofs;
	}

	public DContactGeom get() {
		return get(_ofs);
	}

	public DContactGeom get(int i) {
		return _buf.get(i+_ofs);
	}

	public DContactGeomBuffer createView(int skip) {
		return new DContactGeomBuffer(this, skip);
	}

	public DContactGeom getSafe(int flags, int index) {
		if (!(index >= 0 && index < (flags & DxGeom.NUMC_MASK))) {
			throw new IllegalStateException("Index="+index + "; flags="+flags);
		}
		return _buf.get(index+_ofs);
	}

	//TODO remove, this would affect the objects in the calling class!!!
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
