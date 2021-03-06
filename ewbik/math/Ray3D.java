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

package ewbik.math;

import data.agnosticsavior.CanLoad;
import ewbik.asj.data.JSONObject;

/**
 * @author Eron Gjoni
 */
public class Ray3D implements CanLoad {
    public static final int X = 0, Y = 1, Z = 2;
    protected Vector3 p1;
    protected Vector3 p2;
    protected Vector3 workingVector;
    Vector3 tta, ttb, ttc;
    Vector3 I, u, v, n, dir, w0;
    boolean inUse = false;
    Vector3 m, at, bt, ct, pt;
    Vector3 bc, ca, ac;

    public Ray3D() {
        workingVector = new Vector3();
    }

    public Ray3D(Vector3 origin) {
        this.workingVector = origin.copy();
        this.p1 = origin.copy();
    }

    public Ray3D(Vector3 p1, Vector3 p2) {
        this.workingVector = p1.copy();
        this.p1 = p1.copy();
        if (p2 != null)
            this.p2 = p2.copy();
    }

    /**
     * Given two planes specified by a1,a2,a3 and b1,b2,b3 returns a
     * ray representing the line along which the two planes intersect
     *
     * @param a1 the first vertex of a triangle on the first plane
     * @param a2 the second vertex of a triangle on the first plane
     * @param a3 the third vertex od a triangle on the first plane
     * @param b1 the first vertex of a triangle on the second plane
     * @param b2 the second vertex of a triangle on the second plane
     * @param b3 the third vertex od a triangle on the second plane
     * @return a sgRay along the line of intersection of these two planes, or null
     *         if inputs are coplanar
     */
    public static ewbik.math.Ray3D planePlaneIntersect(Vector3 a1, Vector3 a2, Vector3 a3, Vector3 b1, Vector3 b2,
            Vector3 b3) {
        ewbik.math.Ray3D a1a2 = new ewbik.math.Ray3D(a1, a2);
        ewbik.math.Ray3D a1a3 = new ewbik.math.Ray3D(a1, a3);
        ewbik.math.Ray3D a2a3 = new ewbik.math.Ray3D(a2, a3);

        Vector3 interceptsa1a2 = a1a2.intersectsPlane(b1, b2, b3);
        Vector3 interceptsa1a3 = a1a3.intersectsPlane(b1, b2, b3);
        Vector3 interceptsa2a3 = a2a3.intersectsPlane(b1, b2, b3);

        Vector3[] notNullCandidates = { interceptsa1a2, interceptsa1a3, interceptsa2a3 };
        Vector3 notNull1 = null;
        Vector3 notNull2 = null;

        for (int i = 0; i < notNullCandidates.length; i++) {
            if (notNullCandidates[i] != null) {
                if (notNull1 == null)
                    notNull1 = notNullCandidates[i];
                else {
                    notNull2 = notNullCandidates[i];
                    break;
                }
            }
        }
        if (notNull1 != null && notNull2 != null)
            return new ewbik.math.Ray3D(notNull1, notNull2);
        else
            return null;
    }

    public static float triArea2D(float x1, float y1, float x2, float y2, float x3, float y3) {
        return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
    }

    public float distTo(Vector3 point) {

        Vector3 inPoint = point.copy();
        inPoint.sub(this.p1);
        Vector3 heading = this.heading();
        float scale = (inPoint.dot(heading) / (heading.mag() * inPoint.mag())) * (inPoint.mag() / heading.mag());

        return point.dist(this.getRayScaledBy(scale).p2);
    }

    /**
     * returns the distance between the input point and the point on this ray
     * (treated as a lineSegment) to which the input is closest.
     *
     * @param point
     * @return
     */
    public float distToStrict(Vector3 point) {

        Vector3 inPoint = point.copy();
        inPoint.sub(this.p1);
        Vector3 heading = this.heading();
        float scale = (inPoint.dot(heading) / (heading.mag() * inPoint.mag())) * (inPoint.mag() / heading.mag());
        if (scale < 0) {
            return point.dist(this.p1);
        } else if (scale > 1) {
            return point.dist(this.p2);
        } else {
            return point.dist(this.getRayScaledBy(scale).p2);
        }

    }

    /**
     * returns the distance between this ray treated as a line and the input ray
     * treated as a line.
     *
     * @param r
     * @return
     */
    public float distTo(ewbik.math.Ray3D r) {
        Vector3 closestOnThis = this.closestPointToRay3D(r);
        return r.distTo(closestOnThis);
    }

    /**
     * returns the distance between this ray as a line segment, and the input ray
     * treated as a line segment
     */
    public float distToStrict(ewbik.math.Ray3D r) {
        Vector3 closestOnThis = this.closestPointToSegment3D(r);
        return closestOnThis.dist(r.closestPointToStrict(closestOnThis));
    }

    /**
     * returns the point on this sgRay which is closest to the input point
     *
     * @param point
     * @return
     */
    public Vector3 closestPointTo(Vector3 point) {

        workingVector.set(point);
        workingVector.sub(this.p1);
        Vector3 heading = this.heading();
        heading.mag();
        workingVector.mag();
        // workingVector.normalize();
        heading.normalize();
        float scale = workingVector.dot(heading);

        return (Vector3) this.getScaledTo(scale);
    }

    public Vector3 closestPointToStrict(Vector3 point) {
        Vector3 inPoint = (Vector3) point.copy();
        inPoint.sub(this.p1);
        Vector3 heading = (Vector3) this.heading();
        float scale = (inPoint.dot(heading) / (heading.mag() * inPoint.mag())) * (inPoint.mag() / heading.mag());

        if (scale <= 0)
            return this.p1;
        else if (scale >= 1)
            return this.p2;
        else
            return this.getMultipledBy(scale);
    }

    public Vector3 heading() {
        if (this.p2 == null) {
            if (p1 == null)
                p1 = new Vector3();
            p2 = p1.copy();
            p2.set(0f, 0f, 0f);
            return p2;
        } else {
            workingVector.set(p2);
            return workingVector.subCopy(p1);
        }
    }

    /**
     * manually sets the raw variables of this
     * ray to be equivalent to the raw variables of the
     * target ray. Such that the two rays align without
     * creating a new variable.
     *
     * @param target
     */
    public void alignTo(ewbik.math.Ray3D target) {
        p1.set(target.p1);
        p2.set(target.p2);
    }

    public void heading(float[] newHead) {
        if (p2 == null)
            p2 = p1.copy();
        p2.set(newHead);
        p2.set(p1);
    }

    public void heading(Vector3 newHead) {
        if (p2 == null)
            p2 = p1.copy();
        p2.set(p1);
        p2.add(newHead);
    }

    /**
     * sets the input vector equal to this sgRay's heading.
     *
     * @param setTo
     */
    public void getHeading(Vector3 setTo) {
        setTo.set(p2);
        setTo.sub(this.p1);
    }

    /**
     * @return a copy of this ray with its z-component set to 0;
     */
    public ewbik.math.Ray3D get2DCopy() {
        return this.get2DCopy(ewbik.math.Ray3D.Z);
    }

    /**
     * gets a copy of this ray, with the component specified by
     * collapseOnAxis set to 0.
     *
     * @param collapseOnAxis the axis on which to collapse the ray.
     * @return
     */
    public ewbik.math.Ray3D get2DCopy(int collapseOnAxis) {
        ewbik.math.Ray3D result = this.copy();
        if (collapseOnAxis == ewbik.math.Ray3D.X) {
            result.p1.setX_(0);
            result.p2.setX_(0);
        }
        if (collapseOnAxis == ewbik.math.Ray3D.Y) {
            result.p1.setY_(0);
            result.p2.setY_(0);
        }
        if (collapseOnAxis == ewbik.math.Ray3D.Z) {
            result.p1.setZ_(0);
            result.p2.setZ_(0);
        }

        return result;
    }

    public Vector3 origin() {
        return p1.copy();
    }

    public float mag() {
        workingVector.set(p2);
        return (workingVector.sub(p1)).mag();
    }

    public void mag(float newMag) {
        workingVector.set(p2);
        Vector3 dir = workingVector.sub(p1);
        dir.setMag(newMag);
        this.heading(dir);
    }

    /**
     * Returns the scalar projection of the input vector on this
     * ray. In other words, if this ray goes from (5, 0) to (10, 0),
     * and the input vector is (7.5, 7), this function
     * would output 0.5. Because that is amount the ray would need
     * to be scaled by so that its tip is where the vector would project onto
     * this ray.
     * <p>
     * Due to floating point errors, the intended properties of this function might
     * not be entirely consistent with its output under summation.
     * <p>
     * To help spare programmer cognitive cycles debugging in such circumstances,
     * the intended properties
     * are listed for reference here (despite their being easily inferred).
     * <p>
     * 1. calling scaledProjection(someVector) should return the same value as
     * calling
     * scaledProjection(closestPointTo(someVector).
     * 2. calling getMultipliedBy(scaledProjection(someVector)) should return the
     * same
     * vector as calling closestPointTo(someVector)
     *
     * @param input a vector to project onto this ray
     */
    public float scaledProjection(Vector3 input) {
        workingVector.set(input);
        workingVector.sub(this.p1);
        Vector3 heading = this.heading();
        float headingMag = heading.mag();
        float workingVectorMag = workingVector.mag();
        if (workingVectorMag == 0 || headingMag == 0)
            return 0;
        else
            return (workingVector.dot(heading) / (headingMag * workingVectorMag)) * (workingVectorMag / headingMag);
    }

    /**
     * divides the ray by the amount specified by divisor, such that the
     * base of the ray remains where it is, and the tip
     * is scaled accordinly.
     *
     * @param divisor
     */
    public void div(float divisor) {
        p2.sub(p1);
        p2.divide(divisor);
        p2.add(p1);
    }

    /**
     * multiples the ray by the amount specified by scalar, such that the
     * base of the ray remains where it is, and the tip
     * is scaled accordinly.
     *
     * @param divisor
     */
    public void multiply(float scalar) {
        p2.sub(p1);
        p2.multiply(scalar);
        p2.add(p1);
    }

    /**
     * Returns a Vector3 representing where the tip
     * of this ray would be if multiply() was called on the ray
     * with scalar as the parameter.
     *
     * @param scalar
     * @return
     */
    public Vector3 getMultipledBy(float scalar) {
        Vector3 result = this.heading();
        result.multiply(scalar);
        result.add(p1);
        return result;
    }

    /**
     * Returns a Vector3 representing where the tip
     * of this ray would be if div() was called on the ray
     * with scalar as the parameter.
     *
     * @param scalar
     * @return
     */
    public Vector3 getDivideddBy(float divisor) {
        Vector3 result = this.heading().copy();
        result.multiply(divisor);
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
    public Vector3 getScaledTo(float scale) {
        Vector3 result = this.heading().copy();
        result.normalize();
        result.multiply(scale);
        result.add(p1);
        return result;
    }

    /**
     * adds the specified length to the ray in both directions.
     */
    public void elongate(float amt) {
        Vector3 midPoint = p1.addCopy(p2).multCopy(0.5f);
        Vector3 p1Heading = p1.subCopy(midPoint);
        Vector3 p2Heading = p2.subCopy(midPoint);
        Vector3 p1Add = (Vector3) p1Heading.copy().normalize().multiply(amt);
        Vector3 p2Add = (Vector3) p2Heading.copy().normalize().multiply(amt);

        this.p1.set((Vector3) p1Heading.addCopy(p1Add).addCopy(midPoint));
        this.p2.set((Vector3) p2Heading.addCopy(p2Add).addCopy(midPoint));
    }

    public ewbik.math.Ray3D copy() {
        return new ewbik.math.Ray3D(this.p1, this.p2);
    }

    public void reverse() {
        Vector3 temp = this.p1;
        this.p1 = this.p2;
        this.p2 = temp;
    }

    public ewbik.math.Ray3D getReversed() {
        return new ewbik.math.Ray3D(this.p2, this.p1);
    }

    public ewbik.math.Ray3D getRayScaledTo(float scalar) {
        return new ewbik.math.Ray3D(p1, this.getScaledTo(scalar));
    }

    /*
     * reverses this ray's direction so that it
     * has a positive dot product with the heading of r
     * if dot product is already positive, does nothing.
     */
    public void pointWith(ewbik.math.Ray3D r) {
        if (this.heading().dot(r.heading()) < 0)
            this.reverse();
    }

    public void pointWith(Vector3 heading) {
        if (this.heading().dot(heading) < 0)
            this.reverse();
    }

    public ewbik.math.Ray3D getRayScaledBy(float scalar) {
        return new ewbik.math.Ray3D(p1, this.getMultipledBy(scalar));
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
    public Vector3 setToInvertedTip(Vector3 vec) {
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

    public void contractTo(float percent) {
        // contracts both ends of a ray toward its center such that the total length of
        // the ray
        // is the percent % of its current length;
        float halfPercent = 1 - ((1 - percent) / 2f);

        p1 = p1.lerp(p2, halfPercent);// )new Vector3(p1Tempx, p1Tempy, p1Tempz);
        p2 = p2.lerp(p1, halfPercent);// new Vector3(p2Tempx, p2Tempy, p2Tempz);
    }

    public void translateTo(Vector3 newLocation) {

        workingVector.set(p2);
        workingVector.sub(p1);
        workingVector.add(newLocation);
        p2.set(workingVector);
        p1.set(newLocation);
    }

    public void translateTipTo(Vector3 newLocation) {
        workingVector.set(newLocation);
        Vector3 transBy = workingVector.sub(p2);
        this.translateBy(transBy);
    }

    public void translateBy(Vector3 toAdd) {
        p1.add(toAdd);
        p2.add(toAdd);
    }

    public void normalize() {
        this.mag(1);
    }

    public Vector3 intercepts2D(ewbik.math.Ray3D r) {
        Vector3 result = p1.copy();

        float p0_x = this.p1.x;
        float p0_y = this.p1.y;
        float p1_x = this.p2.x;
        float p1_y = this.p2.y;

        float p2_x = r.p1.x;
        float p2_y = r.p1.y;
        float p3_x = r.p2.x;
        float p3_y = r.p2.y;

        float s1_x, s1_y, s2_x, s2_y;
        s1_x = p1_x - p0_x;
        s1_y = p1_y - p0_y;
        s2_x = p3_x - p2_x;
        s2_y = p3_y - p2_y;

        float t;
        t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

        // if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        // Collision detected
        return result.set(p0_x + (t * s1_x), p0_y + (t * s1_y), 0f);
        // }

        // return null; // No collision
    }

    /**
     * If the closest point to this sgRay on the input sgRay lies
     * beyond the bounds of that input sgRay, this returns closest point
     * to the input Rays bound;
     *
     * @param r
     * @return
     */
    public Vector3 closestPointToSegment3D(ewbik.math.Ray3D r) {
        Vector3 closestToThis = r.closestPointToRay3DStrict(this);
        return this.closestPointTo(closestToThis);
    }

    /**
     * returns the point on this ray which is closest to the input ray
     *
     * @param r
     * @return
     */

    public Vector3 closestPointToRay3D(ewbik.math.Ray3D r) {
        Vector3 result = null;

        workingVector.set(p2);
        Vector3 u = workingVector.sub(this.p1);
        workingVector.set(r.p2);
        Vector3 v = workingVector.sub(r.p1);
        workingVector.set(this.p1);
        Vector3 w = workingVector.sub(r.p1);
        float a = u.dot(u); // always >= 0
        float b = u.dot(v);
        float c = v.dot(v); // always >= 0
        float d = u.dot(w);
        float e = v.dot(w);
        float D = a * c - b * b; // always >= 0
        float sc; // tc

        // compute the line parameters of the two closest points
        if (D < Float.MIN_VALUE) { // the lines are almost parallel
            sc = 0.0f;
            // tc = (b>c ? d/b : e/c); // use the largest denominator
        } else {
            sc = (b * e - c * d) / D;
            // tc = (a*e - b*d) / D;
        }

        result = this.getRayScaledBy(sc).p2;
        return result;
    }

    public Vector3 closestPointToRay3DStrict(ewbik.math.Ray3D r) {
        Vector3 result = null;

        workingVector.set(p2);
        Vector3 u = workingVector.sub(this.p1);
        workingVector.set(r.p2);
        Vector3 v = workingVector.sub(r.p1);
        workingVector.set(this.p1);
        Vector3 w = workingVector.sub(r.p1);
        float a = u.dot(u); // always >= 0
        float b = u.dot(v);
        float c = v.dot(v); // always >= 0
        float d = u.dot(w);
        float e = v.dot(w);
        float D = a * c - b * b; // always >= 0
        float sc; // tc

        // compute the line parameters of the two closest points
        if (D < Float.MIN_VALUE) { // the lines are almost parallel
            sc = 0.0f;
            // tc = (b>c ? d/b : e/c); // use the largest denominator
        } else {
            sc = (b * e - c * d) / D;
            // tc = (a*e - b*d) / D;
        }

        if (sc < 0)
            result = this.p1;
        else if (sc > 1)
            result = this.p2;
        else
            result = this.getRayScaledBy(sc).p2;

        return result;
    }

    /**
     * returns the point on this ray which is closest to
     * the input sgRay. If that point lies outside of the bounds
     * of this ray, returns null.
     *
     * @param r
     * @return
     */
    public Vector3 closestPointToRay3DBounded(ewbik.math.Ray3D r) {
        Vector3 result = null;

        workingVector.set(p2);
        Vector3 u = workingVector.sub(this.p1);
        workingVector.set(r.p2);
        Vector3 v = workingVector.sub(r.p1);
        workingVector.set(this.p1);
        Vector3 w = workingVector.sub(r.p1);
        float a = u.dot(u); // always >= 0
        float b = u.dot(v);
        float c = v.dot(v); // always >= 0
        float d = u.dot(w);
        float e = v.dot(w);
        float D = a * c - b * b; // always >= 0
        float sc; // tc

        // compute the line parameters of the two closest points
        if (D < Float.MIN_VALUE) { // the lines are almost parallel
            sc = 0.0f;
            // tc = (b>c ? d/b : e/c); // use the largest denominator
        } else {
            sc = (b * e - c * d) / D;
            // tc = (a*e - b*d) / D;
        }

        if (sc < 0)
            result = null;
        else if (sc > 1)
            result = null;
        else
            result = this.getRayScaledBy(sc).p2;

        return result;
    }

    // returns a ray perpendicular to this ray on the XY plane;
    public ewbik.math.Ray3D getPerpendicular2D() {
        Vector3 heading = this.heading();
        workingVector.set(heading.x - 1f, heading.x, 0f);
        return new ewbik.math.Ray3D(this.p1, workingVector.add(this.p1));
    }

    public Vector3 intercepts2DStrict(ewbik.math.Ray3D r) {
        // will also return null if the intersection does not occur on the
        // line segment specified by the ray.
        Vector3 result = p1.copy();

        // boolean over = false;
        float a1 = p2.y - p1.y;
        float b1 = p1.x - p2.x;
        float c1 = a1 * p1.x + b1 * p1.y;

        float a2 = r.p2.y - r.p1.y;
        float b2 = r.p1.x - r.p2.y;
        float c2 = a2 * +b2 * r.p1.y;

        float det = a1 * b2 - a2 * b1;
        if (det == 0) {
            // Lines are parallel
            return null;
        } else {
            result.setX_((b2 * c1 - b1 * c2) / det);
            result.setY_((a1 * c2 - a2 * c1) / det);

        }

        float position = result.dot(this.heading());
        if (position > 1 || position < 0)
            return null;

        return result;
    }

    /**
     * @param ta the first vertex of a triangle on the plane
     * @param tb the second vertex of a triangle on the plane
     * @param tc the third vertex of a triangle on the plane
     * @return the point where this ray intersects the plane specified by the
     *         triangle ta,tb,tc.
     */
    public Vector3 intersectsPlane(Vector3 ta, Vector3 tb, Vector3 tc) {
        float[] uvw = new float[3];
        return intersectsPlane(ta, tb, tc, uvw);
    }

    public Vector3 intersectsPlane(Vector3 ta, Vector3 tb, Vector3 tc, float[] uvw) {
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

        Vector3 result = (Vector3) planeIntersectTest(tta, ttb, ttc, uvw).copy();
        return result.add(this.p1);
    }

    /**
     * @param ta     the first vertex of a triangle on the plane
     * @param tb     the second vertex of a triangle on the plane
     * @param tc     the third vertex of a triangle on the plane
     * @param result the variable in which to hold the result
     */
    public void intersectsPlane(Vector3 ta, Vector3 tb, Vector3 tc, Vector3 result) {
        float[] uvw = new float[3];
        result.set(intersectsPlane(ta, tb, tc, uvw));
    }

    /**
     * Similar to intersectsPlane, but returns false if intersection does not occur
     * on the triangle strictly defined by ta, tb, and tc
     *
     * @param ta     the first vertex of a triangle on the plane
     * @param tb     the second vertex of a triangle on the plane
     * @param tc     the third vertex of a triangle on the plane
     * @param result the variable in which to hold the result
     */
    public boolean intersectsTriangle(Vector3 ta, Vector3 tb, Vector3 tc, Vector3 result) {
        float[] uvw = new float[3];
        result.set(intersectsPlane(ta, tb, tc, uvw));
        return !Float.isNaN(uvw[0]) && !Float.isNaN(uvw[1]) && !Float.isNaN(uvw[2]) && !(uvw[0] < 0) && !(uvw[1] < 0)
                && !(uvw[2] < 0);
    }

    private Vector3 planeIntersectTest(Vector3 ta, Vector3 tb, Vector3 tc, float[] uvw) {

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

        return (Vector3) I.copy();
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
    public int intersectsSphere(Vector3 sphereCenter, float radius, Vector3 S1, Vector3 S2) {
        Vector3 tp1 = p1.subCopy(sphereCenter);
        Vector3 tp2 = p2.subCopy(sphereCenter);
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
    public int intersectsSphere(Vector3 rp1, Vector3 rp2, float radius, Vector3 S1, Vector3 S2) {
        Vector3 direction = (Vector3) rp2.subCopy(rp1);
        Vector3 e = (Vector3) direction.copy(); // e=ray.dir
        e.normalize(); // e=g/|g|
        Vector3 h = (Vector3) p1.copy();
        h.set(0f, 0f, 0f);
        h = (Vector3) h.sub(rp1); // h=r.o-c.M
        float lf = e.dot(h); // lf=e.h
        float radpow = radius * radius;
        float hdh = h.magSq();
        float lfpow = lf * lf;
        float s = radpow - hdh + lfpow; // s=r^2-h^2+lf^2
        if (s < 0.0f)
            return 0; // no intersection points ?
        s = MathUtils.sqrt(s); // s=sqrt(r^2-h^2+lf^2)

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

    public void barycentric(Vector3 a, Vector3 b, Vector3 c, Vector3 p, float[] uvw) {
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

        m = new Vector3(((Vector3) bc.subCopy(ct)).crossCopy((Vector3) ca.subCopy(at)));

        float nu;
        float nv;
        float ood;

        float x = MathUtils.abs(m.x);
        float y = MathUtils.abs(m.y);
        float z = MathUtils.abs(m.z);

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

    public void p1(Vector3 in) {
        this.p1 = in.copy();
    }

    public void p2(Vector3 in) {
        this.p2 = in.copy();
    }

    public float lerp(float a, float b, float t) {
        return (1 - t) * a + t * b;
    }

    public Vector3 p2() {
        return p2;
    }

    public void set(Ray3D r) {
        this.p1.set(r.p1);
        this.p2.set(r.p2);
    }

    public void setP2(Vector3 p2) {
        this.p2 = p2;
    }

    public Vector3 p1() {
        return p1;
    }

    public void setP1(Vector3 p1) {
        this.p1 = p1;
    }

    @Override
    public CanLoad populateSelfFromJSON(JSONObject j) {
        if (this.p1 != null)
            this.p2 = this.p1.copy();
        if (this.p2 != null)
            this.p1 = this.p2.copy();

        if (this.p1 == null)
            this.p1 = new Vector3(j.getJSONObject("p1"));
        else {
            this.p1.set(new Vector3(j.getJSONObject("p1")));
        }

        if (this.p2 == null)
            this.p2 = new Vector3(j.getJSONObject("p2"));
        else
            this.p2.set(new Vector3(j.getJSONObject("p2")));

        return this;
    }

    @Override
    public JSONObject toJSONObject() {
        JSONObject result = new JSONObject();
        result.setJSONObject("p1", p1.toJSONObject());
        result.setJSONObject("p2", p2.toJSONObject());
        return result;
    }

}
