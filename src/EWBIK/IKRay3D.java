/*

Copyright (c) 2016 Eron Gjoni

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
associated documentation files (the "Software"), to deal in the Software without restriction, including 
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION 
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 

 */

package EWBIK;

/**
 * @author Eron Gjoni
 */
public class IKRay3D {
    public static final int X = 0, Y = 1, Z = 2;
    protected IKVector3 p1;
    protected IKVector3 p2;
    protected IKVector3 workingVector;
    IKVector3 tta, ttb, ttc;
    IKVector3 I, u, v, n, dir, w0;
    IKVector3 m, at, bt, ct, pt;
    IKVector3 bc, ca, ac;

    public IKRay3D(IKVector3 p1, IKVector3 p2) {
        this.workingVector = p1.copy();
        this.p1 = p1.copy();
        if (p2 != null)
            this.p2 = p2.copy();
    }

    public static float triArea2D(float x1, float y1, float x2, float y2, float x3, float y3) {
        return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
    }

    public IKVector3 heading() {
        if (this.p2 == null) {
            if (p1 == null)
                p1 = new IKVector3();
            p2 = p1.copy();
            p2.set(0f, 0f, 0f);
            return p2;
        } else {
            workingVector.set(p2);
            return workingVector.subCopy(p1);
        }
    }

    /**
     * Returns a Vector3 representing where the tip
     * of this ray would be if multiply() was called on the ray
     * with scalar as the parameter.
     *
     * @param scalar
     * @return
     */
    public IKVector3 getMultipledBy(float scalar) {
        IKVector3 result = this.heading();
        result.multiply(scalar);
        result.add(p1);
        return result;
    }

    /**
     * Returns a Vector3 representing where the tip
     * of this ray would be if mag(scale) was called on the ray
     * with scalar as the parameter.
     *
     * @param scalar
     * @return
     */
    public IKVector3 getScaledTo(float scale) {
        IKVector3 result = this.heading().copy();
        result.normalize();
        result.multiply(scale);
        result.add(p1);
        return result;
    }

    /**
     * adds the specified length to the ray in both directions.
     */
    public void elongate(float amt) {
        IKVector3 midPoint = p1.addCopy(p2).multCopy(0.5f);
        IKVector3 p1Heading = p1.subCopy(midPoint);
        IKVector3 p2Heading = p2.subCopy(midPoint);
        IKVector3 p1Add = (IKVector3) p1Heading.copy().normalize().multiply(amt);
        IKVector3 p2Add = (IKVector3) p2Heading.copy().normalize().multiply(amt);

        this.p1.set((IKVector3) p1Heading.addCopy(p1Add).addCopy(midPoint));
        this.p2.set((IKVector3) p2Heading.addCopy(p2Add).addCopy(midPoint));
    }

    public IKRay3D copy() {
        return new IKRay3D(this.p1, this.p2);
    }

    public IKRay3D getRayScaledTo(float scalar) {
        return new IKRay3D(p1, this.getScaledTo(scalar));
    }

    public IKRay3D getRayScaledBy(float scalar) {
        return new IKRay3D(p1, this.getMultipledBy(scalar));
    }

    /*
     * public Vector3 intercepts2D(sgRay r) {
     * Vector3 result = new Vector3();
     * 
     * float a1 = p2.y - p1.y;
     * float b1 = p1.x - p2.x;
     * float c1 = a1*p1.x + b1*p1.y;
     * 
     * float a2 = r.p2.y - r.p1.y;
     * float b2 = r.p1.x - r.p2.y;
     * float c2 = a2* + b2* r.p1.y;
     * 
     * float det = a1*b2 - a2*b1;
     * if(det == 0){
     * // Lines are parallel
     * return null;
     * }
     * else {
     * result.x = (b2*c1 - b1*c2)/det;
     * result.y = (a1*c2 - a2*c1)/det;
     * }
     * return result;
     * }
     */

    /**
     * sets the values of the given vector to where the
     * tip of this Ray would be if the ray were inverted
     *
     * @param vec
     * @return the vector that was passed in after modification (for chaining)
     */
    public IKVector3 setToInvertedTip(IKVector3 vec) {
        vec.x = (p1.x - p2.x) + p1.x;
        vec.y = (p1.y - p2.y) + p1.y;
        vec.z = (p1.z - p2.z) + p1.z;
        return vec;
    }

    /*
     * public Vector3 closestPointToSegment3DStrict(sgRay r) {
     * 
     * }
     */

    /**
     * @param ta the first vertex of a triangle on the plane
     * @param tb the second vertex of a triangle on the plane
     * @param tc the third vertex of a triangle on the plane
     * @return the point where this ray intersects the plane specified by the
     *         triangle ta,tb,tc.
     */
    public IKVector3 intersectsPlane(IKVector3 ta, IKVector3 tb, IKVector3 tc) {
        float[] uvw = new float[3];
        return intersectsPlane(ta, tb, tc, uvw);
    }

    public IKVector3 intersectsPlane(IKVector3 ta, IKVector3 tb, IKVector3 tc, float[] uvw) {
        if (tta == null) {
            tta = ta.copy();
            ttb = tb.copy();
            ttc = tc.copy();
        } else {
            tta.set(ta);
            ttb.set(tb);
            ttc.set(tc);
        }
        tta.sub(p1);
        ttb.sub(p1);
        ttc.sub(p1);

        IKVector3 result = (IKVector3) planeIntersectTest(tta, ttb, ttc, uvw).copy();
        return result.add(this.p1);
    }

    /**
     * @param ta     the first vertex of a triangle on the plane
     * @param tb     the second vertex of a triangle on the plane
     * @param tc     the third vertex of a triangle on the plane
     * @param result the variable in which to hold the result
     */
    public void intersectsPlane(IKVector3 ta, IKVector3 tb, IKVector3 tc, IKVector3 result) {
        float[] uvw = new float[3];
        result.set(intersectsPlane(ta, tb, tc, uvw));
    }

    private IKVector3 planeIntersectTest(IKVector3 ta, IKVector3 tb, IKVector3 tc, float[] uvw) {

        if (u == null) {
            u = tb.copy();
            v = tc.copy();
            dir = this.heading();
            w0 = p1.copy();
            w0.set(0, 0, 0);
            I = p1.copy();
        } else {
            u.set(tb);
            v.set(tc);
            n.set(0, 0, 0);
            dir.set(this.heading());
            w0.set(0, 0, 0);
        }
        // Vector3 w = new Vector3();
        float r, a, b;
        u.sub(ta);
        v.sub(ta);

        n = u.crossCopy(v);

        w0.sub(ta);
        a = -(n.dot(w0));
        b = n.dot(dir);
        r = a / b;
        I.set(0, 0, 0);
        I.set(dir);
        I.multiply(r);
        // float[] barycentric = new float[3];
        barycentric(ta, tb, tc, I, uvw);

        return (IKVector3) I.copy();
    }

    /*
     * Find where this ray intersects a sphere
     * 
     * @param Vector3 the center of the sphere to test against.
     * 
     * @param radius radius of the sphere
     * 
     * @param S1 reference to variable in which the first intersection will be
     * placed
     * 
     * @param S2 reference to variable in which the second intersection will be
     * placed
     * 
     * @return number of intersections found;
     */
    public int intersectsSphere(IKVector3 sphereCenter, float radius, IKVector3 S1, IKVector3 S2) {
        IKVector3 tp1 = p1.subCopy(sphereCenter);
        IKVector3 tp2 = p2.subCopy(sphereCenter);
        int result = intersectsSphere(tp1, tp2, radius, S1, S2);
        S1.add(sphereCenter);
        S2.add(sphereCenter);
        return result;
    }

    /*
     * Find where this ray intersects a sphere
     * 
     * @param radius radius of the sphere
     * 
     * @param S1 reference to variable in which the first intersection will be
     * placed
     * 
     * @param S2 reference to variable in which the second intersection will be
     * placed
     * 
     * @return number of intersections found;
     */
    public int intersectsSphere(IKVector3 rp1, IKVector3 rp2, float radius, IKVector3 S1, IKVector3 S2) {
        IKVector3 direction = (IKVector3) rp2.subCopy(rp1);
        IKVector3 e = (IKVector3) direction.copy(); // e=ray.dir
        e.normalize(); // e=g/|g|
        IKVector3 h = (IKVector3) p1.copy();
        h.set(0f, 0f, 0f);
        h = (IKVector3) h.sub(rp1); // h=r.o-c.M
        float lf = e.dot(h); // lf=e.h
        float radpow = radius * radius;
        float hdh = h.magSq();
        float lfpow = lf * lf;
        float s = radpow - hdh + lfpow; // s=r^2-h^2+lf^2
        if (s < 0.0f)
            return 0; // no intersection points ?
        s = IKMathUtils.sqrt(s); // s=sqrt(r^2-h^2+lf^2)

        int result = 0;
        if (lf < s) { // S1 behind A ?
            if (lf + s >= 0) { // S2 before A ?}
                s = -s; // swap S1 <-> S2}
                result = 1; // one intersection point
            }
        } else
            result = 2; // 2 intersection points

        S1.set(e.multCopy(lf - s));
        S1.add(rp1); // S1=A+e*(lf-s)
        S2.set(e.multCopy(lf + s));
        S2.add(rp1); // S2=A+e*(lf+s)
        return result;
    }

    public void barycentric(IKVector3 a, IKVector3 b, IKVector3 c, IKVector3 p, float[] uvw) {
        if (m == null) {
            bc = b.copy();
            ca = c.copy();
            at = a.copy();
            bt = b.copy();
            ct = c.copy();
            pt = p.copy();
        } else {
            bc.set(b);
            ca.set(a);
            at.set(a);
            bt.set(b);
            ct.set(c);
            pt.set(p);
        }

        m = new IKVector3(((IKVector3) bc.subCopy(ct)).crossCopy((IKVector3) ca.subCopy(at)));

        float nu;
        float nv;
        float ood;

        float x = IKMathUtils.abs(m.x);
        float y = IKMathUtils.abs(m.y);
        float z = IKMathUtils.abs(m.z);

        if (x >= y && x >= z) {
            nu = triArea2D(pt.y, pt.z, bt.y, bt.z, ct.y, ct.z);
            nv = triArea2D(pt.y, pt.z, ct.y, ct.z, at.y, at.z);
            ood = 1.0f / m.x;
        } else if (y >= x && y >= z) {
            nu = triArea2D(pt.x, pt.z, bt.x, bt.z, ct.x, ct.z);
            nv = triArea2D(pt.x, pt.z, ct.x, ct.z, at.x, at.z);
            ood = 1.0f / -m.y;
        } else {
            nu = triArea2D(pt.x, pt.y, bt.x, bt.y, ct.x, ct.y);
            nv = triArea2D(pt.x, pt.y, ct.x, ct.y, at.x, at.y);
            ood = 1.0f / m.z;
        }
        uvw[0] = nu * ood;
        uvw[1] = nv * ood;
        uvw[2] = 1.0f - uvw[0] - uvw[1];
    }

    @Override
    public String toString() {
        String result = "sgRay " + System.identityHashCode(this) + "\n"
                + "(" + this.p1.x + " ->  " + this.p2.x + ") \n "
                + "(" + this.p1.y + " ->  " + this.p2.y + ") \n "
                + "(" + this.p1.z + " ->  " + this.p2.z + ") \n ";
        return result;
    }

    public IKVector3 p2() {
        return p2;
    }

    public void set(IKRay3D r) {
        this.p1.set(r.p1);
        this.p2.set(r.p2);
    }

    public void setP2(IKVector3 p2) {
        this.p2 = p2;
    }

    public IKVector3 p1() {
        return p1;
    }

    public void setP1(IKVector3 p1) {
        this.p1 = p1;
    }
}
