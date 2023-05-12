package org.ode4j.ode;

import org.junit.Test;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DSapSpace.AXES;

public class Test_DxSAPSpaceAddAssert {

    @Test
    public void test() {
        OdeHelper.initODE2(0);
        DSpace space = OdeHelper.createSapSpace(AXES.XZY); 
        DBox geom = OdeHelper.createBox(space, 1, 1, 1);
        space.collide(null, new DNearCallback() {
			@Override
			public void call(Object data, DGeom o1, DGeom o2) {
			}
		});
        space.remove(geom);
        space.add(geom);
    }    
    

}
