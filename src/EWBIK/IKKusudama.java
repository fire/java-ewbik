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

import java.util.ArrayList;

/**
 * Note, this class is a concrete implementation of the abstract class
 * Kusudama. Please refer to the {@link IKKusudama
 * Kusudama docs.}
 */
public class IKKusudama {

    public static final float TAU = IKMathUtils.PI * 2;
    public static final float PI = IKMathUtils.PI;
    protected IKNode3D limiting_node_3d;
    protected float painfullness;
    /**
     * An array containing all of the Kusudama's LimitCones. The kusudama is
     * built
     * up
     * with the expectation that any LimitCone in the array is connected to the
     * cone
     * at the previous element in the array,
     * and the cone at the next element in the array.
     */
    protected ArrayList<IKLimitCone> limitCones = new ArrayList<IKLimitCone>();
    /**
     * Defined as some Angle in radians about the limitingAxes Y axis, 0 being
     * equivalent to the
     * limitingAxes Z axis.
     */
    protected float minimal_axial_angle = IKMathUtils.PI;
    /**
     * Defined as some Angle in radians about the limitingAxes Y axis, 0 being
     * equivalent to the
     * minAxialAngle
     */
    protected float range = IKMathUtils.PI * 3;
    protected boolean constrain_orientation = false;
    protected boolean constrain_axis = false;
    // for IK solvers. Defines the weight ratio between the unconstrained IK solved
    // orientation and the constrained orientation for this bone
    // per iteration. This should help stabilize solutions somewhat by allowing for
    // soft constraint violations.
    protected Float strength = 1f;
    protected IKBone3D attached_to;

    float[] cone_sequence;
    int cone_count;
    IKRay3D bone_ray = new IKRay3D(new IKVector3(), new IKVector3());
    IKRay3D constrained_ray = new IKRay3D(new IKVector3(), new IKVector3());
    float rotational_freedom = 1f;

    /**
     * Kusudamas are a sequential collection of reach cones, forming a path by their
     * tangents. <br>
     * <br>
     * <p>
     * A reach cone is essentially a cone bounding the rotation of a ball-and-socket
     * joint.
     * A reach cone is defined as a vector pointing in the direction which the cone
     * is opening,
     * and a radius (in radians) representing how much the cone is opening up.
     * <br>
     * <br>
     * You can think of a Kusudama (taken from the Japanese word for "ball with a
     * bunch of cones sticking out of it") as a ball with
     * with a bunch of reach-cones sticking out of it. Except that these reach cones
     * are arranged sequentially, and a smooth path is
     * automatically inferred leading from one cone to the next.
     *
     * @param forBone the bone this kusudama will be attached to.
     */
    public IKKusudama(IKBone3D forBone) {
        this.attached_to = forBone;
        this.limiting_node_3d = forBone.getMajorRotationAxes();
        this.attached_to.addConstraint(this);
        this.enable();
    }

    /**
     * {@inheritDoc}
     **/
    public IKLimitCone createLimitConeForIndex(int insertAt, IKVector3 newPoint,
                                               float radius) {
        return new IKLimitCone(newPoint, radius, this);
    }

    /*
    public void drawMe(PGraphics p, int boneCol, float pinSize) {

        updateShaderTexture();

        PMatrix localMat = limitingAxes().getLocalPMatrix();
        p.applyMatrix(localMat);
        float circumference = attachedTo().getBoneHeight() / 2.5f;
        InverseKinematics.Vector3 min = new InverseKinematics.Vector3(0f, 0f, circumference);
        InverseKinematics.Vector3 current = new InverseKinematics.Vector3(0f, 0f, circumference);
        Quaternion minRot = new Quaternion(new InverseKinematics.Vector3(0, 1, 0), minAxialAngle());
        float absAngle = minAxialAngle + range;
        Quaternion maxRot = new Quaternion(new InverseKinematics.Vector3(0, 1, 0), absAngle);

        float pieces = 20f;
        float granularity = 1f / pieces;
        p.beginShape(PConstants.TRIANGLE_FAN);
        p.noStroke();
        p.fill(0, 150, 0, 120);
        p.vertex(0, 0, 0);
        for (float i = 0; i <= pieces + (3 * granularity); i++) {
            InverseKinematics.Quaternion interp = new InverseKinematics.Quaternion(i * granularity, minRot, minRot);
            current = interp.applyTo(min);
            p.vertex(current.x, current.y, current.z);
        }
        p.endShape();
        float r = p.red(System.identityHashCode(this));
        float g = p.green(System.identityHashCode(this));
        float b = p.blue(System.identityHashCode(this));
        p.fill(p.color(r, g, b));
        p.textureMode(PConstants.NORMAL);
        p.shader(currentShader);
        p.fill(p.color(200, 0, 200, 255));

        currentShader.set("modelViewInv", ((PGraphicsOpenGL) p).modelviewInv);
        currentShader.set("coneSequence", coneSequence, 4);
        currentShader.set("coneCount", coneCount);
        p.sphereDetail(30);
        p.sphere(attachedTo().getBoneHeight() / 3.5f);
        p.resetShader();
        Quaternion alignRot = limitingNode3D.getGlobalMBasis().getInverseRotation()
                .applyTo(attachedTo().localAxes().getGlobalMBasis().rotation);

        Quaternion[] decomposition = alignRot.getSwingTwist(new InverseKinematics.Vector3(0, 1, 0));
        float angle = decomposition[1].getAngle() * decomposition[1].getAxis().y;
        Quaternion zRot = new Quaternion(new InverseKinematics.Vector3(0, 1, 0), angle);
        InverseKinematics.Vector3 yaw = new InverseKinematics.Vector3(0, 0, circumference);
        yaw = zRot.applyToCopy(yaw);
        p.stroke(25, 25, 195);
        p.strokeWeight(4);
        p.line(0f, 0f, 0f, yaw.x, yaw.y, yaw.z);

    }*/

    protected void updateShaderTexture() {

        if (cone_sequence == null || cone_sequence.length != getLimitCones().size() * 12
                || cone_count != getLimitCones().size()) {
            cone_sequence = new float[getLimitCones().size() * 12];
            cone_count = getLimitCones().size();
        }

        int idx = 0;
        for (IKLimitCone lc : getLimitCones()) {
            IKVector3 controlPoint = lc.getControl_point();
            IKVector3 leftTangent = lc.tangent_circle_center_next_1;
            IKVector3 rightTangent = lc.tangent_circle_center_next_2;
            leftTangent = leftTangent.normalize();
            controlPoint = controlPoint.normalize();
            rightTangent = rightTangent.normalize();
            float tanRan = (float) lc.tangent_circle_radius_next;
            float controlRan = (float) lc.getRadius();
            cone_sequence[idx] = controlPoint.x;
            cone_sequence[idx + 1] = controlPoint.y;
            cone_sequence[idx + 2] = controlPoint.z;
            cone_sequence[idx + 3] = controlRan;
            idx += 4;
            cone_sequence[idx] = leftTangent.x;
            cone_sequence[idx + 1] = leftTangent.y;
            cone_sequence[idx + 2] = leftTangent.z;
            cone_sequence[idx + 3] = tanRan;
            idx += 4;
            cone_sequence[idx] = rightTangent.x;
            cone_sequence[idx + 1] = rightTangent.y;
            cone_sequence[idx + 2] = rightTangent.z;
            cone_sequence[idx + 3] = tanRan;
            idx += 4;
        }

        // currentShader = kusudamaShader;
    }

    /**
     * @return the limitingAxes of this Kusudama (these are just its parentBone's
     * majorRotationAxes)
     */
    @SuppressWarnings("unchecked")
    public IKNode3D limitingAxes() {
        return limiting_node_3d;
    }

    public void updateTangentRadii() {

        for (int i = 0; i < limitCones.size(); i++) {
            IKLimitCone next = i < limitCones.size() - 1
                    ? limitCones.get(i + 1)
                    : null;
            limitCones.get(i).updateTangentHandles(next);
        }
        updateShaderTexture();
    }

    @SuppressWarnings("unchecked")
    public ArrayList<IKLimitCone> getLimitCones() {
        return (ArrayList<IKLimitCone>) this.limitCones;
    }

    public void constraintUpdateNotification() {
        this.updateTangentRadii();
        this.updateRotationalFreedom();
    }

    /**
     * This function should be called after you've set all of the Limiting Cones
     * for this Kusudama. It will orient the axes relative to which constrained
     * rotations are computed
     * so as to minimize the potential for undesirable twist rotations due to
     * antipodal singularities.
     * <p>
     * In general, auto-optimization attempts to point the y-component of the
     * constraint
     * axes in the direction that places it within an oreintation allowed by the
     * constraint,
     * and roughly as far as possible from any orientations not allowed by the
     * constraint.
     */
    public void optimizeLimitingAxes() {
        IKNode3D originalLimitingNode3D = limiting_node_3d.getGlobalCopy();

        ArrayList<IKVector3> directions = new ArrayList<>();
        if (getLimitCones().size() == 1) {
            directions.add((limitCones.get(0).getControl_point()).copy());
        } else {
            for (int i = 0; i < getLimitCones().size() - 1; i++) {
                IKVector3 thisC = getLimitCones().get(i).getControl_point().copy();
                IKVector3 nextC = getLimitCones().get(i + 1).getControl_point().copy();
                IKQuaternion thisToNext = new IKQuaternion(thisC, nextC);
                IKQuaternion halfThisToNext = new IKQuaternion(thisToNext.getAxis(), thisToNext.getAngle() / 2f);

                IKVector3 halfAngle = halfThisToNext.applyToCopy(thisC);
                halfAngle.normalize();
                halfAngle.multiply(thisToNext.getAngle());
                directions.add(halfAngle);
            }
        }

        IKVector3 newY = new IKVector3();
        for (IKVector3 dv : directions) {
            newY.add(dv);
        }

        newY.divide(directions.size());
        if (newY.mag() != 0 && !Float.isNaN(newY.y)) {
            newY.normalize();
        } else {
            newY = new IKVector3(0, 1f, 0);
        }

        IKRay3D newYRay = new IKRay3D(new IKVector3(0, 0, 0), newY);

        IKQuaternion oldYtoNewY = new IKQuaternion(limiting_node_3d.calculateY().heading(),
                originalLimitingNode3D.getGlobalOf(newYRay).heading());
        limiting_node_3d.rotateBy(oldYtoNewY);

        for (IKLimitCone lc : getLimitCones()) {
            originalLimitingNode3D.setToGlobalOf(lc.getControl_point(), lc.getControl_point());
            limiting_node_3d.setToLocalOf(lc.getControl_point(), lc.getControl_point());
            lc.getControl_point().normalize();
        }

        this.updateTangentRadii();
    }

    /**
     * Snaps the bone this Kusudama is constraining to be within the Kusudama's
     * orientational and axial limits.
     */
    public void snapToLimits() {
        if (constrain_orientation) {
            setAxesToOrientationSnap(attachedTo().localAxes(), limiting_node_3d, 0);
        }
        if (constrain_axis) {
            snapToTwistLimits(attachedTo().localAxes(), limiting_node_3d);
        }
    }

    /**
     * Presumes the input axes are the bone's localAxes, and rotates
     * them to satisfy the snap limits.
     *
     * @param toSet
     */
    public void setAxesToSnapped(IKNode3D toSet,
                                 IKNode3D limitingNode3D, float cosHalfAngleDampen) {
        if (limitingNode3D != null) {
            if (constrain_orientation) {
                setAxesToOrientationSnap(toSet, limitingNode3D, cosHalfAngleDampen);
            }
            if (constrain_axis) {
                snapToTwistLimits(toSet, limitingNode3D);
            }
        }
    }

    public void setAxesToReturnfulled(IKNode3D toSet,
                                      IKNode3D limitingNode3D, float cosHalfReturnfullness,
                                      float angleReturnfullness) {
        if (limitingNode3D != null && painfullness > 0f) {
            if (constrain_orientation) {
                IKVector3 origin = toSet.calculatePosition();
                IKVector3 inPoint = toSet.calculateY().p2().copy();
                IKVector3 pathPoint = pointOnPathSequence(inPoint, limitingNode3D);
                inPoint.sub(origin);
                pathPoint.sub(origin);
                IKQuaternion toClamp = new IKQuaternion(inPoint, pathPoint);
                toClamp.clampToQuadranceAngle(cosHalfReturnfullness);
                toSet.rotateBy(toClamp);
            }
            if (constrain_axis) {
                float angleToTwistMid = angleToTwistCenter(toSet, limitingNode3D);
                float clampedAngle = IKMathUtils.clamp(angleToTwistMid, -angleReturnfullness, angleReturnfullness);
                toSet.rotateAboutY(clampedAngle, false);
            }
        }
    }

    /**
     * @return A value between (ideally between 0 and 1) dictating
     * how much the bone to which this kusudama belongs
     * prefers to be away from the edges of the kusudama
     * if it can.
     */
    public float getPainfullness() {
        return painfullness;
    }

    /**
     * A value between (ideally between 0 and 1) dictating
     * how much the bone to which this kusudama belongs
     * prefers to be away from the edges of the kusudama
     * if it can. This is useful for avoiding unnatural poses,
     * as the kusudama will push bones back into their more
     * "comfortable" regions. Leave this value at its default of
     * 0 unless you empircal observations show you need it.
     * Setting this value to anything higher than 0.4 is probably overkill
     * in most situations.
     *
     * @param amt
     */
    public void setPainfullness(float amt) {
        painfullness = amt;
        if (attachedTo() != null && attachedTo().parent_armature != null) {
            IKShadowNode s = attachedTo().parent_armature.boneSegmentMap.get(this.attachedTo());
            if (s != null) {
                IKShadowBone wb = s.simulatedBones.get(this.attachedTo());
                if (wb != null) {
                    wb.updateCosDampening();
                }
            }
        }
    }

    public boolean isInLimits_(IKVector3 globalPoint) {
        float[] inBounds = {1f};
        IKVector3 inLimits = this.pointInLimits(limiting_node_3d.getLocalOf(globalPoint), inBounds);
        return inBounds[0] > 0f;
    }

    /**
     * Presumes the input axes are the bone's localAxes, and rotates
     * them to satisfy the snap limits.
     *
     * @param toSet
     */
    public void setAxesToOrientationSnap(IKNode3D toSet,
                                         IKNode3D limitingNode3D, float cosHalfAngleDampen) {
        float[] inBounds = {1f};
        limitingNode3D.updateGlobal();
        bone_ray.p1().set(limitingNode3D.calculatePosition());
        bone_ray.p2().set(toSet.calculateY().p2());
        IKVector3 bonetip = limitingNode3D.getLocalOf(toSet.calculateY().p2());
        IKVector3 inLimits = this.pointInLimits(bonetip, inBounds);

        if (inBounds[0] == -1 && inLimits != null) {
            constrained_ray.p1().set(bone_ray.p1());
            constrained_ray.p2().set(limitingNode3D.getGlobalOf(inLimits));
            IKQuaternion rectifiedRot = new IKQuaternion(bone_ray.heading(), constrained_ray.heading());
            toSet.rotateBy(rectifiedRot);
            toSet.updateGlobal();
        }
    }

    public boolean isInOrientationLimits(IKNode3D globalNode3D,
                                         IKNode3D limitingNode3D) {
        float[] inBounds = {1f};
        IKVector3 inLimits = this.pointInLimits(limitingNode3D.getLocalOf(globalNode3D.calculateY().p2()), inBounds);
        return inBounds[0] != -1l;
    }

    /**
     * Kusudama constraints decompose the bone orientation into a swing component,
     * and a twist component.
     * The "Swing" component is the final direction of the bone. The "Twist"
     * component represents how much
     * the bone is rotated about its own final direction. Where limit cones allow
     * you to constrain the "Swing"
     * component, this method lets you constrain the "twist" component.
     *
     * @param minAnlge some angle in radians about the major rotation frame's y-axis
     *                 to serve as the first angle within the range that the bone is
     *                 allowed to twist.
     * @param inRange  some angle in radians added to the minAngle. if the bone's
     *                 local Z goes maxAngle radians beyond the minAngle, it is
     *                 considered past the limit.
     *                 This value is always interpreted as being in the positive
     *                 direction. For example, if this value is -PI/2, the entire
     *                 range from minAngle to minAngle + 3PI/4 is
     *                 considered valid.
     */
    public void setAxialLimits(float minAngle, float inRange) {
        minimal_axial_angle = minAngle;
        range = toTau(inRange);
        constraintUpdateNotification();
    }

    /**
     * @param toSet
     * @param limitingNode3D
     * @return radians of twist required to snap bone into twist limits (0 if bone
     * is already in twist limits)
     */
    public float snapToTwistLimits(IKNode3D toSet,
                                   IKNode3D limitingNode3D) {

        if (!constrain_axis)
            return 0f;

        IKQuaternion alignRot = limitingNode3D.getGlobalMBasis().getInverseRotation()
                .applyTo(toSet.getGlobalMBasis().rotation);
        IKQuaternion[] decomposition = alignRot.getSwingTwist(new IKVector3(0, 1, 0));
        float angleDelta2 = decomposition[1].getAngle() * decomposition[1].getAxis().y * -1f;
        angleDelta2 = toTau(angleDelta2);
        float fromMinToAngleDelta = toTau(signedAngleDifference(angleDelta2, TAU - this.minAxialAngle()));

        if (fromMinToAngleDelta < TAU - range) {
            float distToMin = IKMathUtils.abs(signedAngleDifference(angleDelta2, TAU - this.minAxialAngle()));
            float distToMax = IKMathUtils.abs(signedAngleDifference(angleDelta2, TAU - (this.minAxialAngle() + range)));
            float turnDiff = 1f;
            if (distToMin < distToMax) {
                turnDiff = turnDiff * (fromMinToAngleDelta);
                toSet.rotateAboutY(turnDiff, true);
            } else {
                turnDiff = turnDiff * (range - (TAU - fromMinToAngleDelta));
                toSet.rotateAboutY(turnDiff, true);
            }
            return turnDiff < 0 ? turnDiff * -1 : turnDiff;
        } else {
            return 0;
        }
    }

    public float angleToTwistCenter(IKNode3D toSet,
                                    IKNode3D limitingNode3D) {

        if (!constrain_axis)
            return 0f;

        IKQuaternion alignRot = limitingNode3D.getGlobalMBasis().getInverseRotation()
                .applyTo(toSet.getGlobalMBasis().rotation);
        IKQuaternion[] decomposition = alignRot.getSwingTwist(new IKVector3(0, 1, 0));
        float angleDelta2 = decomposition[1].getAngle() * decomposition[1].getAxis().y * -1f;
        angleDelta2 = toTau(angleDelta2);

        float distToMid = signedAngleDifference(angleDelta2, TAU - (this.minAxialAngle() + (range / 2f)));
        return distToMid;

    }

    public boolean inTwistLimits(IKNode3D boneNode3D,
                                 IKNode3D limitingNode3D) {

        limitingNode3D.updateGlobal();
        IKQuaternion alignRot = limitingNode3D.getGlobalMBasis().getInverseRotation()
                .applyTo(boneNode3D.globalMBasis.rotation);
        IKQuaternion[] decomposition = alignRot.getSwingTwist(new IKVector3(0, 1, 0));

        float angleDelta = decomposition[1].getAngle() * decomposition[1].getAxis().y * -1;

        angleDelta = toTau(angleDelta);
        float fromMinToAngleDelta = toTau(signedAngleDifference(angleDelta, TAU - this.minAxialAngle()));

        if (fromMinToAngleDelta < TAU - range) {
            float distToMin = IKMathUtils.abs(signedAngleDifference(angleDelta, TAU - this.minAxialAngle()));
            float distToMax = IKMathUtils.abs(signedAngleDifference(angleDelta, TAU - (this.minAxialAngle() + range)));
            if (distToMin < distToMax) {
                return false;
            } else {
                return false;
            }
        }
        return true;
    }

    public float signedAngleDifference(float minAngle, float base) {
        float d = IKMathUtils.abs(minAngle - base) % TAU;
        float r = d > PI ? TAU - d : d;

        float sign = (minAngle - base >= 0 && minAngle - base <= PI)
                || (minAngle - base <= -PI && minAngle - base >= -TAU) ? 1f : -1f;
        r *= sign;
        return r;
    }

    /**
     * Given a point (in global coordinates), checks to see if a ray can be extended
     * from the Kusudama's
     * origin to that point, such that the ray in the Kusudama's reference frame is
     * within the range allowed by the Kusudama's
     * coneLimits.
     * If such a ray exists, the original point is returned (the point is within the
     * limits).
     * If it cannot exist, the tip of the ray within the kusudama's limits that
     * would require the least rotation
     * to arrive at the input point is returned.
     *
     * @param inPoint the point to test.
     * @param returns a number from -1 to 1 representing the point's distance from
     *                the boundary, 0 means the point is right on
     *                the boundary, 1 means the point is within the boundary and on
     *                the path furthest from the boundary. any negative number means
     *                the point is outside of the boundary, but does not signify
     *                anything about how far from the boundary the point is.
     * @return the original point, if it's in limits, or the closest point which is
     * in limits.
     */
    public IKVector3 pointInLimits(IKVector3 inPoint, float[] inBounds) {

        IKVector3 point = inPoint.copy();
        point.normalize();

        inBounds[0] = -1;

        IKVector3 closestCollisionPoint = null;
        float closestCos = -2f;
        if (limitCones.size() > 1 && this.constrain_orientation) {
            for (int i = 0; i < limitCones.size() - 1; i++) {
                IKVector3 collisionPoint = inPoint.copy();
                collisionPoint.set(0, 0, 0);
                IKLimitCone nextCone = limitCones.get(i + 1);
                boolean inSegBounds = limitCones.get(i).inBoundsFromThisToNext(nextCone, point, collisionPoint);
                if (inSegBounds == true) {
                    inBounds[0] = 1;
                } else {
                    float thisCos = collisionPoint.dot(point);
                    if (closestCollisionPoint == null || thisCos > closestCos) {
                        closestCollisionPoint = collisionPoint.copy();
                        closestCos = thisCos;
                    }
                }
            }
            if (inBounds[0] == -1) {
                return closestCollisionPoint;
            } else {
                return point;
            }
        } else if (constrain_orientation) {
            float pointdot = point.dot(limitCones.get(0).getControl_point());
            float radcos = limitCones.get(0).getRadius_cosine();
            if (pointdot > radcos) {
                inBounds[0] = 1;
                return inPoint;
            } else {
                IKVector3 axis = limitCones.get(0).getControl_point().crossCopy(point);
                IKQuaternion toLimit = new IKQuaternion(axis, limitCones.get(0).getRadius());
                IKVector3 newPoint = toLimit.applyToCopy(limitCones.get(0).getControl_point());
                return newPoint;
            }
        } else {
            inBounds[0] = 1;
            return inPoint;
        }
    }

    public IKVector3 pointOnPathSequence(IKVector3 inPoint,
                                         IKNode3D limitingNode3D) {
        float closestPointDot = 0f;
        IKVector3 point = limitingNode3D.getLocalOf(inPoint);
        point.normalize();
        IKVector3 result = (IKVector3) point.copy();

        if (limitCones.size() == 1) {
            result.set(limitCones.get(0).getControl_point());
        } else {
            for (int i = 0; i < limitCones.size() - 1; i++) {
                IKLimitCone nextCone = limitCones.get(i + 1);
                IKVector3 closestPathPoint = limitCones.get(i).getClosestPathPoint(nextCone, point);
                float closeDot = closestPathPoint.dot(point);
                if (closeDot > closestPointDot) {
                    result.set(closestPathPoint);
                    closestPointDot = closeDot;
                }
            }
        }

        return limitingNode3D.getGlobalOf(result);
    }

    public IKBone3D attachedTo() {
        return this.attached_to;
    }

    /**
     * Add a LimitCone to the Kusudama.
     *
     * @param newPoint where on the Kusudama to add the LimitCone (in Kusudama's
     *                 local coordinate frame defined by its bone's
     *                 majorRotationAxes))
     * @param radius   the radius of the LimitCone
     * @param previous the LimitCone adjacent to this one (may be null if
     *                 LimitCone
     *                 is not supposed to be between two existing LimitCones)
     * @param next     the other LimitCone adjacent to this one (may be null if
     *                 LimitCone is not supposed to be between two existing
     *                 LimitCones)
     */
    public void addLimitCone(IKVector3 newPoint, float radius,
                             IKLimitCone previous,
                             IKLimitCone next) {
        int insertAt = 0;

        if (next == null || limitCones.size() == 0) {
            addLimitConeAtIndex(-1, newPoint, radius);
        } else if (previous != null) {
            insertAt = limitCones.indexOf(previous) + 1;
        } else {
            insertAt = (int) IKMathUtils.max(0, limitCones.indexOf(next));
        }
        addLimitConeAtIndex(insertAt, newPoint, radius);
    }

    public void removeLimitCone(IKLimitCone limitCone) {
        this.limitCones.remove(limitCone);
        this.updateTangentRadii();
        this.updateRotationalFreedom();
    }

    /**
     * Adds a LimitCone to the Kusudama. LimitCones are reach cones which
     * can be
     * arranged sequentially. The Kusudama will infer
     * a smooth path leading from one LimitCone to the next.
     * <p>
     * Using a single LimitCone is functionally equivalent to a classic
     * reachCone
     * constraint.
     *
     * @param insertAt the intended index for this LimitCone in the sequence of
     *                 LimitCones from which the Kusudama will infer a
     *                 path. @see
     *                 ewbik.ik.Kusudama.LimitCones LimitCones array.
     * @param newPoint where on the Kusudama to add the LimitCone (in Kusudama's
     *                 local coordinate frame defined by its bone's
     *                 majorRotationAxes))
     * @param radius   the radius of the LimitCone
     */
    public void addLimitConeAtIndex(int insertAt, IKVector3 newPoint, float radius) {
        IKLimitCone newCone = createLimitConeForIndex(insertAt, newPoint,
                radius);
        if (insertAt == -1) {
            limitCones.add(newCone);
        } else {
            limitCones.add(insertAt, newCone);
        }
        this.updateTangentRadii();
        this.updateRotationalFreedom();

    }

    public float toTau(float angle) {
        float result = angle;
        if (angle < 0) {
            result = (2 * IKMathUtils.PI) + angle;
        }
        result = result % (IKMathUtils.PI * 2);
        return result;
    }

    public float mod(float x, float y) {
        if (y != 0 && x != 0) {
            float result = x % y;
            if (result < 0)
                result += y;
            return result;
        } else
            return 0;
    }

    /**
     * @return the lower bound on the axial constraint
     */
    public float minAxialAngle() {
        return minimal_axial_angle;
    }

    public float maxAxialAngle() {
        return range;
    }

    /**
     * the upper bound on the axial constraint in absolute terms
     *
     * @return
     */
    public float absoluteMaxAxialAngle() {
        return signedAngleDifference(range + minimal_axial_angle, IKMathUtils.PI * 2f);
    }

    public boolean isConstrain_axis() {
        return constrain_axis;
    }

    public boolean isConstrain_orientation() {
        return constrain_orientation;
    }

    public void disableOrientationalLimits() {
        this.constrain_orientation = false;
    }

    public void enableOrientationalLimits() {
        this.constrain_orientation = true;
    }

    public void toggleOrientationalLimits() {
        this.constrain_orientation = !this.constrain_orientation;
    }

    public void disableAxialLimits() {
        this.constrain_axis = false;
    }

    public void enableAxialLimits() {
        this.constrain_axis = true;
    }

    public void toggleAxialLimits() {
        constrain_axis = !constrain_axis;
    }

    public boolean isEnabled() {
        return constrain_axis || constrain_orientation;
    }

    public void disable() {
        this.constrain_axis = false;
        this.constrain_orientation = false;
    }

    public void enable() {
        this.constrain_axis = true;
        this.constrain_orientation = true;
    }

    /**
     * @return a measure of the rotational freedom afforded by this constraint.
     * with 0 meaning no rotational freedom (the bone is essentially
     * stationary in relation to its parent)
     * and 1 meaning full rotational freedom (the bone is completely
     * unconstrained).
     * <p>
     * This should be computed as ratio between orientations a bone can be
     * in and orientations
     * a bone cannot be in as defined by its representation as a point on
     * the surface of a hypersphere.
     */
    public float getRotational_freedom() {

        // computation cached from updateRotationalFreedom
        // feel free to override that method if you want your own more correct result.
        // please contribute back a better solution if you write one.
        return rotational_freedom;
    }

    protected void updateRotationalFreedom() {
        float axialConstrainedHyperArea = isConstrain_axis() ? (range / TAU) : 1f;
        float totalLimitConeSurfaceAreaRatio = 0f;
        for (IKLimitCone l : limitCones) {
            totalLimitConeSurfaceAreaRatio += (l.getRadius() * 2f) / TAU;
        }
        rotational_freedom = axialConstrainedHyperArea
                * (isConstrain_orientation() ? IKMathUtils.min(totalLimitConeSurfaceAreaRatio, 1f) : 1f);
    }

    /**
     * attaches the Kusudama to the BoneExample. If the
     * kusudama has its own limiting axes specified,
     * replaces the bone's major rotation
     * axes with the Kusudamas limiting axes.
     * <p>
     * otherwise, this function will set the kusudama's
     * limiting axes to the major rotation axes specified by the bone.
     *
     * @param forBone the bone to which to attach this Kusudama.
     */
    public void attachTo(IKBone3D forBone) {
        this.attached_to = forBone;
        if (this.limiting_node_3d == null)
            this.limiting_node_3d = forBone.getMajorRotationAxes();
        else {
            forBone.setFrameofRotation(this.limiting_node_3d);
            this.limiting_node_3d = forBone.getMajorRotationAxes();
        }
    }

    /**
     * for IK solvers. Defines the weight ratio between the unconstrained IK solved
     * orientation and the constrained orientation for this bone
     * per iteration. This should help stabilize solutions somewhat by allowing for
     * soft constraint violations.
     **/
    public float getStrength() {
        return this.strength;
    }

    /**
     * for IK solvers. Defines the weight ratio between the unconstrained IK solved
     * orientation and the constrained orientation for this bone
     * per iteration. This should help stabilize solutions somewhat by allowing for
     * soft constraint violations.
     *
     * @param strength a value between 0 and 1. Any other value will be clamped to
     *                 this range.
     **/
    public void setStrength(float newStrength) {
        this.strength = IKMathUtils.max(0f, IKMathUtils.min(1f, newStrength));
    }
}
