package ewbik.processing.doublePrecision.sceneGraph;

import math.doubleV.sgRayd;

/**
 * 
 * extremely basic implementation of a Ray 
 * object to serve as an example for how to 
 * extend this library
 * 
 * @author Eron Gjoni
 *
 */
public class dRay extends sgRayd {
	
	public dRay(DVector p1, DVector p2) {
		super(p1, p2);
	}

	public DVector p1() {
		if(p1 == null) p1 = new DVector();
		return (DVector) this.p1;
	}
	
	public DVector p2() {
		if(p2 == null) p2 = new DVector();
		return (DVector) this.p2;
	}
	
}
