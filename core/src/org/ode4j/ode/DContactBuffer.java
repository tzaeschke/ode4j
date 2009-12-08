package org.ode4j.ode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;


/**
 * A collection of DContact objects.
 *
 * @ingroup collide
 */
public class DContactBuffer implements Iterable<DContact> {
	
	private ArrayList<DContact> _buf = new ArrayList<DContact>();

	public DContactBuffer(int size) {
		for (int i = 0; i < size; i++) {
			_buf.add(new DContact());
		}
	}
	
	public DContact get(int i) {
		return _buf.get(i);
	}
	
	public DContactGeomBuffer getGeomBuffer() {
		return new DContactGeomBuffer(this);
	}

	@Override
	public Iterator<DContact> iterator() {
		return Collections.unmodifiableList(_buf).iterator();
	}
}
