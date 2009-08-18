package org.ode4j.ode;

import org.ode4j.ode.internal.DxGeom;


/**
 * @brief A collection of DContactGeom objects.
 *
 * @ingroup collide
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
