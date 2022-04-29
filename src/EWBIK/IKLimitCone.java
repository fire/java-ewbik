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

    public IKKusudama parent_kusudama;
    public IKVector3 tangent_circle_center_next_1;
    public IKVector3 tangent_circle_center_next_2;
    public float tangent_circle_radius_next;
    public float tangent_circle_radius_next_cos;
    public IKVector3 tangent_circle_center_previous_1;
    public IKVector3 tangent_circle_center_previous_2;
    public float tangent_circle_radius_previous;
    public float tangent_circle_radius_previous_cos;
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
    public IKVector3[] first_triangle_next = new IKVector3[3];
    public IKVector3[] second_triangle_next = new IKVector3[3];
    IKVector3 control_point;
    IKVector3 radialPoint;
    // radius stored as cosine to save on the acos call necessary for angleBetween.
    private float radius_cosine;
    private float radius;

    // default constructor required for file loading to work
    public IKLimitCone() {
    }

    public IKLimitCone(IKVector3 location, float rad, IKKusudama attachedTo) {
        IKVector3 location1 = location;
        setControl_point(location1);
        IKLimitCone.this.tangent_circle_center_next_1 = location1.getOrthogonal();
        IKLimitCone.this.tangent_circle_center_next_2 = IKVector3
                .multiply(IKLimitCone.this.tangent_circle_center_next_1, -1);
        this.setRadius(rad);
        IKLimitCone.this.parent_kusudama = attachedTo;
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
     * rectified position
     * if the point was out of bounds.
     */
    public IKVector3 getClosestCollision(IKLimitCone next,
                                         IKVector3 input) {
        IKVector3 result = getOnGreatTangentTriangle(next, input);
        if (result == null) {
            boolean[] inBounds = {false};
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

        if (control_point.dot(input) >= radius_cosine || next.control_point.dot(input) >= next.radius_cosine) {
            return true;
        } else {
            boolean inTan1Rad = tangent_circle_center_next_1.dot(input) > tangent_circle_radius_next_cos;
            if (inTan1Rad)
                return false;
            boolean inTan2Rad = tangent_circle_center_next_2.dot(input) > tangent_circle_radius_next_cos;
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
            IKVector3 c1xc2 = control_point.crossCopy(next.control_point);
            float c1c2fir = input.dot(c1xc2);

            if (c1c2fir < 0.0) {
                IKVector3 c1xt1 = control_point.crossCopy(tangent_circle_center_next_1);
                IKVector3 t1xc2 = tangent_circle_center_next_1.crossCopy(next.control_point);
                return input.dot(c1xt1) > 0 && input.dot(t1xc2) > 0;
            } else {
                IKVector3 t2xc1 = tangent_circle_center_next_2.crossCopy(control_point);
                IKVector3 c2xt2 = next.control_point.crossCopy(tangent_circle_center_next_2);
                return input.dot(t2xc1) > 0 && input.dot(c2xt2) > 0;
            }
        }
    }

    public IKVector3 closestCone(IKLimitCone next, IKVector3 input) {
        if (input.dot(control_point) > input.dot(next.control_point))
            return this.control_point.copy();
        else
            return next.control_point.copy();
    }

    public IKVector3 getOnPathSequence(IKLimitCone next, IKVector3 input) {
        IKVector3 c1xc2 = control_point.crossCopy(next.control_point);
        float c1c2fir = input.dot(c1xc2);
        if (c1c2fir < 0.0) {
            IKVector3 c1xt1 = control_point.crossCopy(tangent_circle_center_next_1);
            IKVector3 t1xc2 = tangent_circle_center_next_1.crossCopy(next.control_point);
            if (input.dot(c1xt1) > 0 && input.dot(t1xc2) > 0) {
                IKRay3D tan1ToInput = new IKRay3D(tangent_circle_center_next_1, input);
                IKVector3 result = new IKVector3();
                tan1ToInput.intersectsPlane(new IKVector3(0, 0, 0), control_point, next.control_point, result);
                return result.normalize();
            } else {
                return null;
            }
        } else {
            IKVector3 t2xc1 = tangent_circle_center_next_2.crossCopy(control_point);
            IKVector3 c2xt2 = next.control_point.crossCopy(tangent_circle_center_next_2);
            if (input.dot(t2xc1) > 0 && input.dot(c2xt2) > 0) {
                IKRay3D tan2ToInput = new IKRay3D(tangent_circle_center_next_2, input);
                IKVector3 result = new IKVector3();
                tan2ToInput.intersectsPlane(new IKVector3(0, 0, 0), control_point, next.control_point, result);
                return result.normalize();
            } else {
                return null;
            }
        }

    }

    public IKVector3 getOnGreatTangentTriangle(IKLimitCone next,
                                               IKVector3 input) {
        IKVector3 c1xc2 = control_point.crossCopy(next.control_point);
        float c1c2fir = input.dot(c1xc2);
        if (c1c2fir < 0.0) {
            IKVector3 c1xt1 = control_point.crossCopy(tangent_circle_center_next_1);
            IKVector3 t1xc2 = tangent_circle_center_next_1.crossCopy(next.control_point);
            if (input.dot(c1xt1) > 0 && input.dot(t1xc2) > 0) {
                if (input.dot(tangent_circle_center_next_1) > tangent_circle_radius_next_cos) {
                    IKVector3 planeNormal = tangent_circle_center_next_1.crossCopy(input);
                    IKQuaternion rotateAboutBy = new IKQuaternion(planeNormal, tangent_circle_radius_next);
                    return rotateAboutBy.applyToCopy(tangent_circle_center_next_1);
                } else {
                    return input.copy();
                }
            } else {
                return null;
            }
        } else {
            IKVector3 t2xc1 = tangent_circle_center_next_2.crossCopy(control_point);
            IKVector3 c2xt2 = next.control_point.crossCopy(tangent_circle_center_next_2);
            if (input.dot(t2xc1) > 0 && input.dot(c2xt2) > 0) {
                if (input.dot(tangent_circle_center_next_2) > tangent_circle_radius_next_cos) {
                    IKVector3 planeNormal = tangent_circle_center_next_2.crossCopy(input);
                    IKQuaternion rotateAboutBy = new IKQuaternion(planeNormal, tangent_circle_radius_next);
                    return rotateAboutBy.applyToCopy(tangent_circle_center_next_2);
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
        float controlPointDotProduct = input.dot(this.getControl_point());
        float radiusCosine = this.getRadius_cosine();
        if (controlPointDotProduct > radiusCosine) {
            inBounds[0] = true;
            return null;
        } else {
            IKVector3 axis = this.getControl_point().crossCopy(input);
            IKQuaternion rotTo = new IKQuaternion(axis, this.getRadius());
            IKVector3 result = rotTo.applyToCopy(this.getControl_point());
            inBounds[0] = false;
            return result;
        }
    }

    public void updateTangentHandles(IKLimitCone next) {
        this.control_point.normalize();
        if (next != null) {
            float radA = this.getRadius();
            float radB = next.getRadius();

            IKVector3 A = this.getControl_point().copy();
            IKVector3 B = next.getControl_point().copy();

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

            this.tangent_circle_center_next_1 = sphereIntersect1;
            this.tangent_circle_center_next_2 = sphereIntersect2;
            this.tangent_circle_radius_next = tRadius;

            next.tangent_circle_center_previous_1 = sphereIntersect1;
            next.tangent_circle_center_previous_2 = sphereIntersect2;
            next.tangent_circle_radius_previous = tRadius;
        }

        this.tangent_circle_radius_next_cos = IKMathUtils.cos(tangent_circle_radius_next);
        this.tangent_circle_radius_previous_cos = IKMathUtils.cos(tangent_circle_radius_previous);

        if (tangent_circle_center_next_1 == null)
            tangent_circle_center_next_1 = control_point.getOrthogonal().normalize();
        if (tangent_circle_center_next_2 == null)
            tangent_circle_center_next_2 = IKVector3.multiply(tangent_circle_center_next_1, -1).normalize();
        if (next != null)
            computeTriangles(next);
    }

    private void computeTriangles(IKLimitCone next) {
        first_triangle_next[1] = this.tangent_circle_center_next_1.normalize();
        first_triangle_next[0] = this.getControl_point().normalize();
        first_triangle_next[2] = next.getControl_point().normalize();

        second_triangle_next[1] = this.tangent_circle_center_next_2.normalize();
        second_triangle_next[0] = this.getControl_point().normalize();
        second_triangle_next[2] = next.getControl_point().normalize();
    }

    public IKVector3 getControl_point() {
        return control_point;
    }

    public void setControl_point(IKVector3 control_point) {
        this.control_point = control_point.copy();
        this.control_point.normalize();
        if (this.parent_kusudama != null)
            this.parent_kusudama.constraintUpdateNotification();
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = IKMathUtils.max(Float.MIN_VALUE, radius);
        this.radius_cosine = IKMathUtils.cos(this.radius);
        if (this.parent_kusudama != null)
            this.parent_kusudama.constraintUpdateNotification();
    }

    public float getRadius_cosine() {
        return this.radius_cosine;
    }

    public IKKusudama getParent_kusudama() {
        return parent_kusudama;
    }

}
