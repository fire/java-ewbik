/*

Copyright (c) 2015 Eron Gjoni

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

package InverseKinematics;

import ewbik.math.*;
import processing.core.PVector;

public class LimitCone implements Saveable {

    public Kusudama parentKusudama;
    public Vector3 tangentCircleCenterNext1;
    public Vector3 tangentCircleCenterNext2;
    public float tangentCircleRadiusNext;
    public float tangentCircleRadiusNextCos;
    public Vector3 tangentCircleCenterPrevious1;
    public Vector3 tangentCircleCenterPrevious2;
    public float tangentCircleRadiusPrevious;
    public float tangentCircleRadiusPreviousCos;
    // softness of 0 means completely hard.
    // any softness higher than 0f means that
    // as the softness value is increased
    // the is more penalized for moving
    // further from the center of the channel
    public float softness = 0f;
    /**
     * a triangle where the [1] is th tangentCircleNext_n, and [0] and [2]
     * are the points at which the tangent circle intersects this limitCone and the
     * next limitCone
     */
    public Vector3[] firstTriangleNext = new Vector3[3];
    public Vector3[] secondTriangleNext = new Vector3[3];
    Vector3 controlPoint;
    Vector3 radialPoint;
    // radius stored as cosine to save on the acos call necessary for angleBetween.
    private float radiusCosine;
    private float radius;

    // default constructor required for file loading to work
    public LimitCone() {
    }

    public LimitCone(PVector location, float rad, Kusudama attachedTo) {
        Vector3 location1 = Node3D.toVec3f(location);
        setControlPoint(location1);
        LimitCone.this.tangentCircleCenterNext1 = location1.getOrthogonal();
        LimitCone.this.tangentCircleCenterNext2 = Vector3
                .multiply(LimitCone.this.tangentCircleCenterNext1, -1);
        this.setRadius(rad);
        LimitCone.this.parentKusudama = attachedTo;
    }

    /**
     * @param next
     * @param input
     * @param collisionPoint will be set to the rectified (if necessary) position of
     *                       the input after accounting for collisions
     * @return
     */
    public boolean inBoundsFromThisToNext(LimitCone next, Vector3 input,
                                          Vector3 collisionPoint) {
        boolean isInBounds = false;
        Vector3 closestCollision = getClosestCollision(next, input);
        if (closestCollision == null) {
            /**
             * getClosestCollision returns null if the point is already in bounds,
             * so we set isInBounds to true.
             */
            isInBounds = true;
            collisionPoint.x = input.x;
            collisionPoint.y = input.y;
            collisionPoint.z = input.z;
        } else {
            collisionPoint.x = closestCollision.x;
            collisionPoint.y = closestCollision.y;
            collisionPoint.z = closestCollision.z;
        }
        return isInBounds;
    }

    /**
     * @param next
     * @param input
     * @return null if the input point is already in bounds, or the point's
     *         rectified position
     *         if the point was out of bounds.
     */
    public Vector3 getClosestCollision(LimitCone next,
                                       Vector3 input) {
        Vector3 result = getOnGreatTangentTriangle(next, input);
        if (result == null) {
            boolean[] inBounds = { false };
            result = closestPointOnClosestCone(next, input, inBounds);
        }
        return result;
    }

    public Vector3 getClosestPathPoint(LimitCone next,
                                       Vector3 input) {
        Vector3 result = getOnPathSequence(next, input);
        if (result == null) {
            result = closestCone(next, input);
        }
        return result;
    }

    /**
     * Determines if a ray emanating from the origin to given point in local space
     * lies withing the path from this cone to the next cone. This function relies
     * on
     * an optimization trick for a performance boost, but the trick ruins everything
     * if the input isn't normalized. So it is ABSOLUTELY VITAL
     * that @param input have unit length in order for this function to work
     * correctly.
     *
     * @param next
     * @param input
     * @return
     */
    public boolean determineIfInBounds(LimitCone next, Vector3 input) {

        /**
         * Procedure : Check if input is contained in this cone, or the next cone
         * if it is, then we're finished and in bounds. otherwise,
         * check if the point is contained within the tangent radii,
         * if it is, then we're out of bounds and finished, otherwise
         * in the tangent triangles while still remaining outside of the tangent radii
         * if it is, then we're finished and in bounds. otherwise, we're out of bounds.
         */

        if (controlPoint.dot(input) >= radiusCosine || next.controlPoint.dot(input) >= next.radiusCosine) {
            return true;
        } else {
            boolean inTan1Rad = tangentCircleCenterNext1.dot(input) > tangentCircleRadiusNextCos;
            if (inTan1Rad)
                return false;
            boolean inTan2Rad = tangentCircleCenterNext2.dot(input) > tangentCircleRadiusNextCos;
            if (inTan2Rad)
                return false;

            /*
             * if we reach this point in the code, we are either on the path between two
             * limitCones, or on the path extending out from between them
             * but outside of their radii.
             * To determine which , we take the cross product of each control point with
             * each tangent center.
             * The direction of each of the resultant vectors will represent the normal of a
             * plane.
             * Each of these four planes define part of a boundary which determines if our
             * point is in bounds.
             * If the dot product of our point with the normal of any of these planes is
             * negative, we must be out
             * of bounds.
             *
             * Older version of this code relied on a triangle intersection algorithm here,
             * which I think is slightly less efficient on average
             * as it didn't allow for early termination. .
             */

            // Vector3 planeNormal = controlPoint.crossCopy(tangentCircleCenterNext1);
            Vector3 c1xc2 = controlPoint.crossCopy(next.controlPoint);
            float c1c2fir = input.dot(c1xc2);

            if (c1c2fir < 0.0) {
                Vector3 c1xt1 = controlPoint.crossCopy(tangentCircleCenterNext1);
                Vector3 t1xc2 = tangentCircleCenterNext1.crossCopy(next.controlPoint);
                return input.dot(c1xt1) > 0 && input.dot(t1xc2) > 0;
            } else {
                Vector3 t2xc1 = tangentCircleCenterNext2.crossCopy(controlPoint);
                Vector3 c2xt2 = next.controlPoint.crossCopy(tangentCircleCenterNext2);
                return input.dot(t2xc1) > 0 && input.dot(c2xt2) > 0;
            }
        }
    }

    public Vector3 closestCone(LimitCone next, Vector3 input) {
        if (input.dot(controlPoint) > input.dot(next.controlPoint))
            return this.controlPoint.copy();
        else
            return next.controlPoint.copy();
    }

    public Vector3 getOnPathSequence(LimitCone next, Vector3 input) {
        Vector3 c1xc2 = controlPoint.crossCopy(next.controlPoint);
        float c1c2fir = input.dot(c1xc2);
        if (c1c2fir < 0.0) {
            Vector3 c1xt1 = controlPoint.crossCopy(tangentCircleCenterNext1);
            Vector3 t1xc2 = tangentCircleCenterNext1.crossCopy(next.controlPoint);
            if (input.dot(c1xt1) > 0 && input.dot(t1xc2) > 0) {
                Ray3D tan1ToInput = new Ray3D(tangentCircleCenterNext1, input);
                Vector3 result = new Vector3();
                tan1ToInput.intersectsPlane(new Vector3(0, 0, 0), controlPoint, next.controlPoint, result);
                return result.normalize();
            } else {
                return null;
            }
        } else {
            Vector3 t2xc1 = tangentCircleCenterNext2.crossCopy(controlPoint);
            Vector3 c2xt2 = next.controlPoint.crossCopy(tangentCircleCenterNext2);
            if (input.dot(t2xc1) > 0 && input.dot(c2xt2) > 0) {
                Ray3D tan2ToInput = new Ray3D(tangentCircleCenterNext2, input);
                Vector3 result = new Vector3();
                tan2ToInput.intersectsPlane(new Vector3(0, 0, 0), controlPoint, next.controlPoint, result);
                return result.normalize();
            } else {
                return null;
            }
        }

    }

    public Vector3 getOnGreatTangentTriangle(LimitCone next,
                                             Vector3 input) {
        Vector3 c1xc2 = controlPoint.crossCopy(next.controlPoint);
        float c1c2fir = input.dot(c1xc2);
        if (c1c2fir < 0.0) {
            Vector3 c1xt1 = controlPoint.crossCopy(tangentCircleCenterNext1);
            Vector3 t1xc2 = tangentCircleCenterNext1.crossCopy(next.controlPoint);
            if (input.dot(c1xt1) > 0 && input.dot(t1xc2) > 0) {
                if (input.dot(tangentCircleCenterNext1) > tangentCircleRadiusNextCos) {
                    Vector3 planeNormal = tangentCircleCenterNext1.crossCopy(input);
                    Quaternion rotateAboutBy = new Quaternion(planeNormal, tangentCircleRadiusNext);
                    return rotateAboutBy.applyToCopy(tangentCircleCenterNext1);
                } else {
                    return input.copy();
                }
            } else {
                return null;
            }
        } else {
            Vector3 t2xc1 = tangentCircleCenterNext2.crossCopy(controlPoint);
            Vector3 c2xt2 = next.controlPoint.crossCopy(tangentCircleCenterNext2);
            if (input.dot(t2xc1) > 0 && input.dot(c2xt2) > 0) {
                if (input.dot(tangentCircleCenterNext2) > tangentCircleRadiusNextCos) {
                    Vector3 planeNormal = tangentCircleCenterNext2.crossCopy(input);
                    Quaternion rotateAboutBy = new Quaternion(planeNormal, tangentCircleRadiusNext);
                    return rotateAboutBy.applyToCopy(tangentCircleCenterNext2);
                } else {
                    return input.copy();
                }
            } else {
                return null;
            }
        }

    }

    /**
     * returns null if no rectification is required.
     *
     * @param next
     * @param input
     * @param inBounds
     * @return
     */
    public Vector3 closestPointOnClosestCone(LimitCone next,
                                             Vector3 input,
                                             boolean[] inBounds) {
        Vector3 closestToFirst = this.closestToCone(input, inBounds);
        if (inBounds[0]) {
            return closestToFirst;
        }
        Vector3 closestToSecond = next.closestToCone(input, inBounds);
        if (inBounds[0]) {
            return closestToSecond;
        }

        float cosToFirst = input.dot(closestToFirst);
        float cosToSecond = input.dot(closestToSecond);

        if (cosToFirst > cosToSecond) {
            return closestToFirst;
        } else {
            return closestToSecond;
        }

    }

    /**
     * returns null if no rectification is required.
     *
     * @param input
     * @param inBounds
     * @return
     */
    public Vector3 closestToCone(Vector3 input, boolean[] inBounds) {
        float controlPointDotProduct = input.dot(this.getControlPoint());
        float radiusCosine = this.getRadiusCosine();
        if (controlPointDotProduct > radiusCosine) {
            inBounds[0] = true;
            return null;
        } else {
            Vector3 axis = this.getControlPoint().crossCopy(input);
            Quaternion rotTo = new Quaternion(axis, this.getRadius());
            Vector3 result = rotTo.applyToCopy(this.getControlPoint());
            inBounds[0] = false;
            return result;
        }
    }

    public void updateTangentHandles(LimitCone next) {
        this.controlPoint.normalize();
        if (next != null) {
            float radA = this.getRadius();
            float radB = next.getRadius();

            Vector3 A = this.getControlPoint().copy();
            Vector3 B = next.getControlPoint().copy();

            Vector3 arcNormal = A.crossCopy(B);

            /**
             * There are an infinite number of circles co-tangent with A and B, every other
             * one of which had a unique radius.
             *
             * However, we want the radius of our tangent circles to obey the following
             * properties:
             * 1) When the radius of A + B == 0, our tangent circle's radius should = 90.
             * In other words, the tangent circle should span a hemisphere.
             * 2) When the radius of A + B == 180, our tangent circle's radius should = 0.
             * In other words, when A + B combined are capable of spanning the entire
             * sphere,
             * our tangentCircle should be nothing.
             *
             * Another way to think of this is -- whatever the maximum distance can be
             * between the
             * borders of A and B (presuming their centers are free to move about the circle
             * but their radii remain constant), we want our tangentCircle's diameter to be
             * precisely that distance,
             * and so, our tangent circles radius should be precisely half of that distance.
             */
            float tRadius = ((MathUtils.PI) - (radA + radB)) / 2f;

            /**
             * Once we have the desired radius for our tangent circle, we may find the
             * solution for its
             * centers (usually, there are two).
             */
            float boundaryPlusTangentRadiusA = radA + tRadius;
            float boundaryPlusTangentRadiusB = radB + tRadius;

            // the axis of this cone, scaled to minimize its distance to the tangent contact
            // points.
            Vector3 scaledAxisA = Vector3.multiply(A, MathUtils.cos(boundaryPlusTangentRadiusA));
            // a point on the plane running through the tangent contact points
            Vector3 planeDir1A = new Quaternion(arcNormal, boundaryPlusTangentRadiusA).applyToCopy(A);
            // another point on the same plane
            Vector3 planeDir2A = new Quaternion(A, MathUtils.PI / 2f).applyToCopy(planeDir1A);

            Vector3 scaledAxisB = Vector3.multiply(B, MathUtils.cos(boundaryPlusTangentRadiusB));
            // a point on the plane running through the tangent contact points
            Vector3 planeDir1B = new Quaternion(arcNormal, boundaryPlusTangentRadiusB).applyToCopy(B);
            // another poiint on the same plane
            Vector3 planeDir2B = new Quaternion(B, MathUtils.PI / 2f).applyToCopy(planeDir1B);

            // ray from scaled center of next cone to half way point between the
            // circumference of this cone and the next cone.
            Ray3D r1B = new Ray3D(planeDir1B, scaledAxisB);
            Ray3D r2B = new Ray3D(planeDir1B, planeDir2B);

            r1B.elongate(99);
            r2B.elongate(99);

            Vector3 intersection1 = r1B.intersectsPlane(scaledAxisA, planeDir1A, planeDir2A);
            Vector3 intersection2 = r2B.intersectsPlane(scaledAxisA, planeDir1A, planeDir2A);

            Ray3D intersectionRay = new Ray3D(intersection1, intersection2);
            intersectionRay.elongate(99);

            Vector3 sphereIntersect1 = new Vector3();
            Vector3 sphereIntersect2 = new Vector3();
            Vector3 sphereCenter = new Vector3();
            intersectionRay.intersectsSphere(sphereCenter, 1f, sphereIntersect1, sphereIntersect2);

            this.tangentCircleCenterNext1 = sphereIntersect1;
            this.tangentCircleCenterNext2 = sphereIntersect2;
            this.tangentCircleRadiusNext = tRadius;

            next.tangentCircleCenterPrevious1 = sphereIntersect1;
            next.tangentCircleCenterPrevious2 = sphereIntersect2;
            next.tangentCircleRadiusPrevious = tRadius;
        }

        this.tangentCircleRadiusNextCos = MathUtils.cos(tangentCircleRadiusNext);
        this.tangentCircleRadiusPreviousCos = MathUtils.cos(tangentCircleRadiusPrevious);

        if (tangentCircleCenterNext1 == null)
            tangentCircleCenterNext1 = controlPoint.getOrthogonal().normalize();
        if (tangentCircleCenterNext2 == null)
            tangentCircleCenterNext2 = Vector3.multiply(tangentCircleCenterNext1, -1).normalize();
        if (next != null)
            computeTriangles(next);
    }

    private void computeTriangles(LimitCone next) {
        firstTriangleNext[1] = this.tangentCircleCenterNext1.normalize();
        firstTriangleNext[0] = this.getControlPoint().normalize();
        firstTriangleNext[2] = next.getControlPoint().normalize();

        secondTriangleNext[1] = this.tangentCircleCenterNext2.normalize();
        secondTriangleNext[0] = this.getControlPoint().normalize();
        secondTriangleNext[2] = next.getControlPoint().normalize();
    }

    public Vector3 getControlPoint() {
        return controlPoint;
    }

    public void setControlPoint(Vector3 controlPoint) {
        this.controlPoint = controlPoint.copy();
        this.controlPoint.normalize();
        if (this.parentKusudama != null)
            this.parentKusudama.constraintUpdateNotification();
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = MathUtils.max(Float.MIN_VALUE, radius);
        this.radiusCosine = MathUtils.cos(this.radius);
        if (this.parentKusudama != null)
            this.parentKusudama.constraintUpdateNotification();
    }

    public float getRadiusCosine() {
        return this.radiusCosine;
    }

    public Kusudama getParentKusudama() {
        return parentKusudama;
    }

    @Override
    public void makeSaveable(SaveManager saveManager) {
        saveManager.addToSaveState(this);
    }

    @Override
    public ewbik.asj.data.JSONObject getSaveJSON(SaveManager saveManager) {
        ewbik.asj.data.JSONObject saveJSON = new ewbik.asj.data.JSONObject();
        saveJSON.setString("identityHash", this.getIdentityHash());
        saveJSON.setString("parentKusudama", this.getParentKusudama().getIdentityHash());
        saveJSON.setJSONObject("controlPoint", this.controlPoint.toJSONObject());
        saveJSON.setFloat("radius", this.radius);
        return saveJSON;
    }

    public void loadFromJSONObject(ewbik.asj.data.JSONObject j, LoadManager l) {
        this.parentKusudama = (Kusudama) l.getObjectFromClassMaps(
                Kusudama.class,
                j.getString("parentKusudama"));
        Vector3 controlPointJ = null;
        try {
            controlPointJ = new Vector3(j.getJSONObject("controlPoint"));
        } catch (Exception e) {
            controlPointJ = new Vector3(j.getJSONArray("controlPoint"));
        }

        controlPointJ.normalize();

        this.controlPoint = controlPointJ;
        this.setRadius(j.getFloat("radius"));
    }

    @Override
    public void notifyOfSaveIntent(SaveManager saveManager) {

    }

    @Override
    public void notifyOfSaveCompletion(SaveManager saveManager) {
    }

    @Override
    public boolean isLoading() {
        return false;
    }

    @Override
    public void setLoading(boolean loading) {
    }
}
