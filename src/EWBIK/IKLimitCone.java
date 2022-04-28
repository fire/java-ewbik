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

package EWBIK;

public class IKLimitCone {

    public IKKusudama parentKusudama;
    public IKVector3 tangentCircleCenterNext1;
    public IKVector3 tangentCircleCenterNext2;
    public float tangentCircleRadiusNext;
    public float tangentCircleRadiusNextCos;
    public IKVector3 tangentCircleCenterPrevious1;
    public IKVector3 tangentCircleCenterPrevious2;
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
    public IKVector3[] firstTriangleNext = new IKVector3[3];
    public IKVector3[] secondTriangleNext = new IKVector3[3];
    IKVector3 controlPoint;
    IKVector3 radialPoint;
    // radius stored as cosine to save on the acos call necessary for angleBetween.
    private float radiusCosine;
    private float radius;

    // default constructor required for file loading to work
    public IKLimitCone() {
    }

    public IKLimitCone(IKVector3 location, float rad, IKKusudama attachedTo) {
        IKVector3 location1 = location;
        setControlPoint(location1);
        IKLimitCone.this.tangentCircleCenterNext1 = location1.getOrthogonal();
        IKLimitCone.this.tangentCircleCenterNext2 = IKVector3
                .multiply(IKLimitCone.this.tangentCircleCenterNext1, -1);
        this.setRadius(rad);
        IKLimitCone.this.parentKusudama = attachedTo;
    }

    /**
     * @param next
     * @param input
     * @param collisionPoint will be set to the rectified (if necessary) position of
     *                       the input after accounting for collisions
     * @return
     */
    public boolean inBoundsFromThisToNext(IKLimitCone next, IKVector3 input,
                                          IKVector3 collisionPoint) {
        boolean isInBounds = false;
        IKVector3 closestCollision = getClosestCollision(next, input);
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
    public IKVector3 getClosestCollision(IKLimitCone next,
                                         IKVector3 input) {
        IKVector3 result = getOnGreatTangentTriangle(next, input);
        if (result == null) {
            boolean[] inBounds = { false };
            result = closestPointOnClosestCone(next, input, inBounds);
        }
        return result;
    }

    public IKVector3 getClosestPathPoint(IKLimitCone next,
                                         IKVector3 input) {
        IKVector3 result = getOnPathSequence(next, input);
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
    public boolean determineIfInBounds(IKLimitCone next, IKVector3 input) {

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
            IKVector3 c1xc2 = controlPoint.crossCopy(next.controlPoint);
            float c1c2fir = input.dot(c1xc2);

            if (c1c2fir < 0.0) {
                IKVector3 c1xt1 = controlPoint.crossCopy(tangentCircleCenterNext1);
                IKVector3 t1xc2 = tangentCircleCenterNext1.crossCopy(next.controlPoint);
                return input.dot(c1xt1) > 0 && input.dot(t1xc2) > 0;
            } else {
                IKVector3 t2xc1 = tangentCircleCenterNext2.crossCopy(controlPoint);
                IKVector3 c2xt2 = next.controlPoint.crossCopy(tangentCircleCenterNext2);
                return input.dot(t2xc1) > 0 && input.dot(c2xt2) > 0;
            }
        }
    }

    public IKVector3 closestCone(IKLimitCone next, IKVector3 input) {
        if (input.dot(controlPoint) > input.dot(next.controlPoint))
            return this.controlPoint.copy();
        else
            return next.controlPoint.copy();
    }

    public IKVector3 getOnPathSequence(IKLimitCone next, IKVector3 input) {
        IKVector3 c1xc2 = controlPoint.crossCopy(next.controlPoint);
        float c1c2fir = input.dot(c1xc2);
        if (c1c2fir < 0.0) {
            IKVector3 c1xt1 = controlPoint.crossCopy(tangentCircleCenterNext1);
            IKVector3 t1xc2 = tangentCircleCenterNext1.crossCopy(next.controlPoint);
            if (input.dot(c1xt1) > 0 && input.dot(t1xc2) > 0) {
                IKRay3D tan1ToInput = new IKRay3D(tangentCircleCenterNext1, input);
                IKVector3 result = new IKVector3();
                tan1ToInput.intersectsPlane(new IKVector3(0, 0, 0), controlPoint, next.controlPoint, result);
                return result.normalize();
            } else {
                return null;
            }
        } else {
            IKVector3 t2xc1 = tangentCircleCenterNext2.crossCopy(controlPoint);
            IKVector3 c2xt2 = next.controlPoint.crossCopy(tangentCircleCenterNext2);
            if (input.dot(t2xc1) > 0 && input.dot(c2xt2) > 0) {
                IKRay3D tan2ToInput = new IKRay3D(tangentCircleCenterNext2, input);
                IKVector3 result = new IKVector3();
                tan2ToInput.intersectsPlane(new IKVector3(0, 0, 0), controlPoint, next.controlPoint, result);
                return result.normalize();
            } else {
                return null;
            }
        }

    }

    public IKVector3 getOnGreatTangentTriangle(IKLimitCone next,
                                               IKVector3 input) {
        IKVector3 c1xc2 = controlPoint.crossCopy(next.controlPoint);
        float c1c2fir = input.dot(c1xc2);
        if (c1c2fir < 0.0) {
            IKVector3 c1xt1 = controlPoint.crossCopy(tangentCircleCenterNext1);
            IKVector3 t1xc2 = tangentCircleCenterNext1.crossCopy(next.controlPoint);
            if (input.dot(c1xt1) > 0 && input.dot(t1xc2) > 0) {
                if (input.dot(tangentCircleCenterNext1) > tangentCircleRadiusNextCos) {
                    IKVector3 planeNormal = tangentCircleCenterNext1.crossCopy(input);
                    IKQuaternion rotateAboutBy = new IKQuaternion(planeNormal, tangentCircleRadiusNext);
                    return rotateAboutBy.applyToCopy(tangentCircleCenterNext1);
                } else {
                    return input.copy();
                }
            } else {
                return null;
            }
        } else {
            IKVector3 t2xc1 = tangentCircleCenterNext2.crossCopy(controlPoint);
            IKVector3 c2xt2 = next.controlPoint.crossCopy(tangentCircleCenterNext2);
            if (input.dot(t2xc1) > 0 && input.dot(c2xt2) > 0) {
                if (input.dot(tangentCircleCenterNext2) > tangentCircleRadiusNextCos) {
                    IKVector3 planeNormal = tangentCircleCenterNext2.crossCopy(input);
                    IKQuaternion rotateAboutBy = new IKQuaternion(planeNormal, tangentCircleRadiusNext);
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
    public IKVector3 closestPointOnClosestCone(IKLimitCone next,
                                               IKVector3 input,
                                               boolean[] inBounds) {
        IKVector3 closestToFirst = this.closestToCone(input, inBounds);
        if (inBounds[0]) {
            return closestToFirst;
        }
        IKVector3 closestToSecond = next.closestToCone(input, inBounds);
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
    public IKVector3 closestToCone(IKVector3 input, boolean[] inBounds) {
        float controlPointDotProduct = input.dot(this.getControlPoint());
        float radiusCosine = this.getRadiusCosine();
        if (controlPointDotProduct > radiusCosine) {
            inBounds[0] = true;
            return null;
        } else {
            IKVector3 axis = this.getControlPoint().crossCopy(input);
            IKQuaternion rotTo = new IKQuaternion(axis, this.getRadius());
            IKVector3 result = rotTo.applyToCopy(this.getControlPoint());
            inBounds[0] = false;
            return result;
        }
    }

    public void updateTangentHandles(IKLimitCone next) {
        this.controlPoint.normalize();
        if (next != null) {
            float radA = this.getRadius();
            float radB = next.getRadius();

            IKVector3 A = this.getControlPoint().copy();
            IKVector3 B = next.getControlPoint().copy();

            IKVector3 arcNormal = A.crossCopy(B);

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
            float tRadius = ((IKMathUtils.PI) - (radA + radB)) / 2f;

            /**
             * Once we have the desired radius for our tangent circle, we may find the
             * solution for its
             * centers (usually, there are two).
             */
            float boundaryPlusTangentRadiusA = radA + tRadius;
            float boundaryPlusTangentRadiusB = radB + tRadius;

            // the axis of this cone, scaled to minimize its distance to the tangent contact
            // points.
            IKVector3 scaledAxisA = IKVector3.multiply(A, IKMathUtils.cos(boundaryPlusTangentRadiusA));
            // a point on the plane running through the tangent contact points
            IKVector3 planeDir1A = new IKQuaternion(arcNormal, boundaryPlusTangentRadiusA).applyToCopy(A);
            // another point on the same plane
            IKVector3 planeDir2A = new IKQuaternion(A, IKMathUtils.PI / 2f).applyToCopy(planeDir1A);

            IKVector3 scaledAxisB = IKVector3.multiply(B, IKMathUtils.cos(boundaryPlusTangentRadiusB));
            // a point on the plane running through the tangent contact points
            IKVector3 planeDir1B = new IKQuaternion(arcNormal, boundaryPlusTangentRadiusB).applyToCopy(B);
            // another poiint on the same plane
            IKVector3 planeDir2B = new IKQuaternion(B, IKMathUtils.PI / 2f).applyToCopy(planeDir1B);

            // ray from scaled center of next cone to half way point between the
            // circumference of this cone and the next cone.
            IKRay3D r1B = new IKRay3D(planeDir1B, scaledAxisB);
            IKRay3D r2B = new IKRay3D(planeDir1B, planeDir2B);

            r1B.elongate(99);
            r2B.elongate(99);

            IKVector3 intersection1 = r1B.intersectsPlane(scaledAxisA, planeDir1A, planeDir2A);
            IKVector3 intersection2 = r2B.intersectsPlane(scaledAxisA, planeDir1A, planeDir2A);

            IKRay3D intersectionRay = new IKRay3D(intersection1, intersection2);
            intersectionRay.elongate(99);

            IKVector3 sphereIntersect1 = new IKVector3();
            IKVector3 sphereIntersect2 = new IKVector3();
            IKVector3 sphereCenter = new IKVector3();
            intersectionRay.intersectsSphere(sphereCenter, 1f, sphereIntersect1, sphereIntersect2);

            this.tangentCircleCenterNext1 = sphereIntersect1;
            this.tangentCircleCenterNext2 = sphereIntersect2;
            this.tangentCircleRadiusNext = tRadius;

            next.tangentCircleCenterPrevious1 = sphereIntersect1;
            next.tangentCircleCenterPrevious2 = sphereIntersect2;
            next.tangentCircleRadiusPrevious = tRadius;
        }

        this.tangentCircleRadiusNextCos = IKMathUtils.cos(tangentCircleRadiusNext);
        this.tangentCircleRadiusPreviousCos = IKMathUtils.cos(tangentCircleRadiusPrevious);

        if (tangentCircleCenterNext1 == null)
            tangentCircleCenterNext1 = controlPoint.getOrthogonal().normalize();
        if (tangentCircleCenterNext2 == null)
            tangentCircleCenterNext2 = IKVector3.multiply(tangentCircleCenterNext1, -1).normalize();
        if (next != null)
            computeTriangles(next);
    }

    private void computeTriangles(IKLimitCone next) {
        firstTriangleNext[1] = this.tangentCircleCenterNext1.normalize();
        firstTriangleNext[0] = this.getControlPoint().normalize();
        firstTriangleNext[2] = next.getControlPoint().normalize();

        secondTriangleNext[1] = this.tangentCircleCenterNext2.normalize();
        secondTriangleNext[0] = this.getControlPoint().normalize();
        secondTriangleNext[2] = next.getControlPoint().normalize();
    }

    public IKVector3 getControlPoint() {
        return controlPoint;
    }

    public void setControlPoint(IKVector3 controlPoint) {
        this.controlPoint = controlPoint.copy();
        this.controlPoint.normalize();
        if (this.parentKusudama != null)
            this.parentKusudama.constraintUpdateNotification();
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = IKMathUtils.max(Float.MIN_VALUE, radius);
        this.radiusCosine = IKMathUtils.cos(this.radius);
        if (this.parentKusudama != null)
            this.parentKusudama.constraintUpdateNotification();
    }

    public float getRadiusCosine() {
        return this.radiusCosine;
    }

    public IKKusudama getParentKusudama() {
        return parentKusudama;
    }

}
