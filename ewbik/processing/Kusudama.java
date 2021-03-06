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

package ewbik.processing.singlePrecision;

import ewbik.asj.LoadManager;
import ewbik.asj.SaveManager;
import ewbik.asj.Saveable;
import ewbik.ik.ShadowNode3D;
import ewbik.math.*;
import ik.Bone;
import processing.Node3D;
import processing.core.PConstants;
import processing.core.PGraphics;
import processing.core.PMatrix;
import processing.core.PVector;
import processing.opengl.PGraphicsOpenGL;
import processing.opengl.PShader;

import java.util.ArrayList;

/**
 * Note, this class is a concrete implementation of the abstract class
 * Kusudama. Please refer to the {@link Kusudama
 * Kusudama docs.}
 */
public class Kusudama implements Saveable {

    public static final float TAU = MathUtils.PI * 2;
    public static final float PI = MathUtils.PI;
    public static PShader kusudamaShader;
    public static PShader currentShader;
    protected Node3D limitingNode3D;
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
    protected ArrayList<ewbik.processing.singlePrecision.LimitCone> limitCones = new ArrayList<ewbik.processing.singlePrecision.LimitCone>();
    /**
     * Defined as some Angle in radians about the limitingAxes Y axis, 0 being
     * equivalent to the
     * limitingAxes Z axis.
     */
    protected float minAxialAngle = MathUtils.PI;
    /**
     * Defined as some Angle in radians about the limitingAxes Y axis, 0 being
     * equivalent to the
     * minAxialAngle
     */
    protected float range = MathUtils.PI * 3;
    protected boolean orientationallyConstrained = false;
    protected boolean axiallyConstrained = false;
    // for IK solvers. Defines the weight ratio between the unconstrained IK solved
    // orientation and the constrained orientation for this bone
    // per iteration. This should help stabilize solutions somewhat by allowing for
    // soft constraint violations.
    protected Float strength = 1f;
    protected Bone attachedTo;

    float[] coneSequence;
    int coneCount;
    Ray3D boneRay = new Ray3D(new Vector3(), new Vector3());
    Ray3D constrainedRay = new Ray3D(new Vector3(), new Vector3());
    float unitHyperArea = 2 * MathUtils.pow(MathUtils.PI, 2);
    float unitArea = 4 * MathUtils.PI;
    float rotationalFreedom = 1f;

    public Kusudama() {
    }

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
    public Kusudama(Bone forBone) {
        this.attachedTo = forBone;
        this.limitingNode3D = forBone.getMajorRotationAxes();
        this.attachedTo.addConstraint(this);
        this.enable();
    }

    /**
     * {@inheritDoc}
     **/
    public ewbik.processing.singlePrecision.LimitCone createLimitConeForIndex(int insertAt, Vector3 newPoint,
            float radius) {
        return new LimitCone(Node3D.toPVector(newPoint), radius, this);
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
    public void addLimitConeAtIndex(int insertAt, PVector newPoint, float radius) {
        addLimitConeAtIndex(insertAt, Node3D.toVec3f(newPoint), radius);
    }

    public boolean isInLimits(PVector inPoint) {
        return isInLimits_(
                Node3D.toVec3f(inPoint));
    }

    public void drawMe(PGraphics p, int boneCol, float pinSize) {

        updateShaderTexture();

        PMatrix localMat = limitingAxes().getLocalPMatrix();
        p.applyMatrix(localMat);
        float circumference = attachedTo().getBoneHeight() / 2.5f;
        ewbik.math.Vector3 min = new ewbik.math.Vector3(0f, 0f, circumference);
        ewbik.math.Vector3 current = new ewbik.math.Vector3(0f, 0f, circumference);
        Quaternion minRot = new Quaternion(new ewbik.math.Vector3(0, 1, 0), minAxialAngle());
        float absAngle = minAxialAngle + range;
        Quaternion maxRot = new Quaternion(new ewbik.math.Vector3(0, 1, 0), absAngle);

        float pieces = 20f;
        float granularity = 1f / pieces;
        p.beginShape(PConstants.TRIANGLE_FAN);
        p.noStroke();
        p.fill(0, 150, 0, 120);
        p.vertex(0, 0, 0);
        for (float i = 0; i <= pieces + (3 * granularity); i++) {
            ewbik.math.Quaternion interp = new ewbik.math.Quaternion(i * granularity, minRot, minRot);
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

        Quaternion[] decomposition = alignRot.getSwingTwist(new ewbik.math.Vector3(0, 1, 0));
        float angle = decomposition[1].getAngle() * decomposition[1].getAxis().y;
        Quaternion zRot = new Quaternion(new ewbik.math.Vector3(0, 1, 0), angle);
        ewbik.math.Vector3 yaw = new ewbik.math.Vector3(0, 0, circumference);
        yaw = zRot.applyToCopy(yaw);
        p.stroke(25, 25, 195);
        p.strokeWeight(4);
        p.line(0f, 0f, 0f, yaw.x, yaw.y, yaw.z);

    }

    protected void updateShaderTexture() {

        if (coneSequence == null || coneSequence.length != getLimitCones().size() * 12
                || coneCount != getLimitCones().size()) {
            coneSequence = new float[getLimitCones().size() * 12];
            coneCount = getLimitCones().size();
        }

        int idx = 0;
        for (LimitCone lc : getLimitCones()) {
            PVector controlPoint = Node3D.toPVector(lc.getControlPoint());
            PVector leftTangent = Node3D.toPVector(lc.tangentCircleCenterNext1);
            PVector rightTangent = Node3D.toPVector(lc.tangentCircleCenterNext2);
            leftTangent = leftTangent.normalize();
            controlPoint = controlPoint.normalize();
            rightTangent = rightTangent.normalize();
            float tanRan = (float) lc.tangentCircleRadiusNext;
            float controlRan = (float) lc.getRadius();
            coneSequence[idx] = controlPoint.x;
            coneSequence[idx + 1] = controlPoint.y;
            coneSequence[idx + 2] = controlPoint.z;
            coneSequence[idx + 3] = controlRan;
            idx += 4;
            coneSequence[idx] = leftTangent.x;
            coneSequence[idx + 1] = leftTangent.y;
            coneSequence[idx + 2] = leftTangent.z;
            coneSequence[idx + 3] = tanRan;
            idx += 4;
            coneSequence[idx] = rightTangent.x;
            coneSequence[idx + 1] = rightTangent.y;
            coneSequence[idx + 2] = rightTangent.z;
            coneSequence[idx + 3] = tanRan;
            idx += 4;
        }

        currentShader = kusudamaShader;
    }

    /**
     * @return the limitingAxes of this Kusudama (these are just its parentBone's
     *         majorRotationAxes)
     */
    @SuppressWarnings("unchecked")
    public Node3D limitingAxes() {
        return limitingNode3D;
    }

    public void updateTangentRadii() {

        for (int i = 0; i < limitCones.size(); i++) {
            ewbik.processing.singlePrecision.LimitCone next = i < limitCones.size() - 1
                    ? limitCones.get(i + 1)
                    : null;
            limitCones.get(i).updateTangentHandles(next);
        }
        updateShaderTexture();
    }

    @SuppressWarnings("unchecked")
    public ArrayList<LimitCone> getLimitCones() {
        return (ArrayList<LimitCone>) this.limitCones;
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
        Node3D originalLimitingNode3D = limitingNode3D.getGlobalCopy();

        ArrayList<Vector3> directions = new ArrayList<>();
        if (getLimitCones().size() == 1) {
            directions.add((limitCones.get(0).getControlPoint()).copy());
        } else {
            for (int i = 0; i < getLimitCones().size() - 1; i++) {
                Vector3 thisC = getLimitCones().get(i).getControlPoint().copy();
                Vector3 nextC = getLimitCones().get(i + 1).getControlPoint().copy();
                Quaternion thisToNext = new Quaternion(thisC, nextC);
                Quaternion halfThisToNext = new Quaternion(thisToNext.getAxis(), thisToNext.getAngle() / 2f);

                Vector3 halfAngle = halfThisToNext.applyToCopy(thisC);
                halfAngle.normalize();
                halfAngle.multiply(thisToNext.getAngle());
                directions.add(halfAngle);
            }
        }

        Vector3 newY = new Vector3();
        for (Vector3 dv : directions) {
            newY.add(dv);
        }

        newY.divide(directions.size());
        if (newY.mag() != 0 && !Float.isNaN(newY.y)) {
            newY.normalize();
        } else {
            newY = new Vector3(0, 1f, 0);
        }

        Ray3D newYRay = new Ray3D(new Vector3(0, 0, 0), newY);

        Quaternion oldYtoNewY = new Quaternion(limitingNode3D.calculateY().heading(),
                originalLimitingNode3D.getGlobalOf(newYRay).heading());
        limitingNode3D.rotateBy(oldYtoNewY);

        for (ewbik.processing.singlePrecision.LimitCone lc : getLimitCones()) {
            originalLimitingNode3D.setToGlobalOf(lc.getControlPoint(), lc.getControlPoint());
            limitingNode3D.setToLocalOf(lc.getControlPoint(), lc.getControlPoint());
            lc.getControlPoint().normalize();
        }

        this.updateTangentRadii();
    }

    /**
     * Snaps the bone this Kusudama is constraining to be within the Kusudama's
     * orientational and axial limits.
     */
    public void snapToLimits() {
        if (orientationallyConstrained) {
            setAxesToOrientationSnap(attachedTo().localAxes(), limitingNode3D, 0);
        }
        if (axiallyConstrained) {
            snapToTwistLimits(attachedTo().localAxes(), limitingNode3D);
        }
    }

    /**
     * Presumes the input axes are the bone's localAxes, and rotates
     * them to satisfy the snap limits.
     *
     * @param toSet
     */
    public void setAxesToSnapped(Node3D toSet,
            Node3D limitingNode3D, float cosHalfAngleDampen) {
        if (limitingNode3D != null) {
            if (orientationallyConstrained) {
                setAxesToOrientationSnap(toSet, limitingNode3D, cosHalfAngleDampen);
            }
            if (axiallyConstrained) {
                snapToTwistLimits(toSet, limitingNode3D);
            }
        }
    }

    public void setAxesToReturnfulled(Node3D toSet,
            Node3D limitingNode3D, float cosHalfReturnfullness,
            float angleReturnfullness) {
        if (limitingNode3D != null && painfullness > 0f) {
            if (orientationallyConstrained) {
                Vector3 origin = toSet.calculatePosition();
                Vector3 inPoint = toSet.calculateY().p2().copy();
                Vector3 pathPoint = pointOnPathSequence(inPoint, limitingNode3D);
                inPoint.sub(origin);
                pathPoint.sub(origin);
                Quaternion toClamp = new Quaternion(inPoint, pathPoint);
                toClamp.clampToQuadranceAngle(cosHalfReturnfullness);
                toSet.rotateBy(toClamp);
            }
            if (axiallyConstrained) {
                float angleToTwistMid = angleToTwistCenter(toSet, limitingNode3D);
                float clampedAngle = MathUtils.clamp(angleToTwistMid, -angleReturnfullness, angleReturnfullness);
                toSet.rotateAboutY(clampedAngle, false);
            }
        }
    }

    /**
     * @return A value between (ideally between 0 and 1) dictating
     *         how much the bone to which this kusudama belongs
     *         prefers to be away from the edges of the kusudama
     *         if it can.
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
        if (attachedTo() != null && attachedTo().parentArmature != null) {
            ewbik.ik.ShadowNode3D s = attachedTo().parentArmature.boneSegmentMap.get(this.attachedTo());
            if (s != null) {
                ShadowNode3D.ShadowBone wb = s.simulatedBones.get(this.attachedTo());
                if (wb != null) {
                    wb.updateCosDampening();
                }
            }
        }
    }

    public boolean isInLimits_(Vector3 globalPoint) {
        float[] inBounds = { 1f };
        Vector3 inLimits = this.pointInLimits(limitingNode3D.getLocalOf(globalPoint), inBounds);
        return inBounds[0] > 0f;
    }

    /**
     * Presumes the input axes are the bone's localAxes, and rotates
     * them to satisfy the snap limits.
     *
     * @param toSet
     */
    public void setAxesToOrientationSnap(Node3D toSet,
            Node3D limitingNode3D, float cosHalfAngleDampen) {
        float[] inBounds = { 1f };
        limitingNode3D.updateGlobal();
        boneRay.p1().set(limitingNode3D.calculatePosition());
        boneRay.p2().set(toSet.calculateY().p2());
        Vector3 bonetip = limitingNode3D.getLocalOf(toSet.calculateY().p2());
        Vector3 inLimits = this.pointInLimits(bonetip, inBounds);

        if (inBounds[0] == -1 && inLimits != null) {
            constrainedRay.p1().set(boneRay.p1());
            constrainedRay.p2().set(limitingNode3D.getGlobalOf(inLimits));
            Quaternion rectifiedRot = new Quaternion(boneRay.heading(), constrainedRay.heading());
            toSet.rotateBy(rectifiedRot);
            toSet.updateGlobal();
        }
    }

    public boolean isInOrientationLimits(Node3D globalNode3D,
            Node3D limitingNode3D) {
        float[] inBounds = { 1f };
        Vector3 inLimits = this.pointInLimits(limitingNode3D.getLocalOf(globalNode3D.calculateY().p2()), inBounds);
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
        minAxialAngle = minAngle;
        range = toTau(inRange);
        constraintUpdateNotification();
    }

    /**
     * @param toSet
     * @param limitingNode3D
     * @return radians of twist required to snap bone into twist limits (0 if bone
     *         is already in twist limits)
     */
    public float snapToTwistLimits(Node3D toSet,
            Node3D limitingNode3D) {

        if (!axiallyConstrained)
            return 0f;

        Quaternion alignRot = limitingNode3D.getGlobalMBasis().getInverseRotation()
                .applyTo(toSet.getGlobalMBasis().rotation);
        Quaternion[] decomposition = alignRot.getSwingTwist(new Vector3(0, 1, 0));
        float angleDelta2 = decomposition[1].getAngle() * decomposition[1].getAxis().y * -1f;
        angleDelta2 = toTau(angleDelta2);
        float fromMinToAngleDelta = toTau(signedAngleDifference(angleDelta2, TAU - this.minAxialAngle()));

        if (fromMinToAngleDelta < TAU - range) {
            float distToMin = MathUtils.abs(signedAngleDifference(angleDelta2, TAU - this.minAxialAngle()));
            float distToMax = MathUtils.abs(signedAngleDifference(angleDelta2, TAU - (this.minAxialAngle() + range)));
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

    public float angleToTwistCenter(Node3D toSet,
            Node3D limitingNode3D) {

        if (!axiallyConstrained)
            return 0f;

        Quaternion alignRot = limitingNode3D.getGlobalMBasis().getInverseRotation()
                .applyTo(toSet.getGlobalMBasis().rotation);
        Quaternion[] decomposition = alignRot.getSwingTwist(new Vector3(0, 1, 0));
        float angleDelta2 = decomposition[1].getAngle() * decomposition[1].getAxis().y * -1f;
        angleDelta2 = toTau(angleDelta2);

        float distToMid = signedAngleDifference(angleDelta2, TAU - (this.minAxialAngle() + (range / 2f)));
        return distToMid;

    }

    public boolean inTwistLimits(Node3D boneNode3D,
            Node3D limitingNode3D) {

        limitingNode3D.updateGlobal();
        Quaternion alignRot = limitingNode3D.getGlobalMBasis().getInverseRotation()
                .applyTo(boneNode3D.globalMBasis.rotation);
        Quaternion[] decomposition = alignRot.getSwingTwist(new Vector3(0, 1, 0));

        float angleDelta = decomposition[1].getAngle() * decomposition[1].getAxis().y * -1;

        angleDelta = toTau(angleDelta);
        float fromMinToAngleDelta = toTau(signedAngleDifference(angleDelta, TAU - this.minAxialAngle()));

        if (fromMinToAngleDelta < TAU - range) {
            float distToMin = MathUtils.abs(signedAngleDifference(angleDelta, TAU - this.minAxialAngle()));
            float distToMax = MathUtils.abs(signedAngleDifference(angleDelta, TAU - (this.minAxialAngle() + range)));
            if (distToMin < distToMax) {
                return false;
            } else {
                return false;
            }
        }
        return true;
    }

    public float signedAngleDifference(float minAngle, float base) {
        float d = MathUtils.abs(minAngle - base) % TAU;
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
     *         in limits.
     */
    public Vector3 pointInLimits(Vector3 inPoint, float[] inBounds) {

        Vector3 point = inPoint.copy();
        point.normalize();

        inBounds[0] = -1;

        Vector3 closestCollisionPoint = null;
        float closestCos = -2f;
        if (limitCones.size() > 1 && this.orientationallyConstrained) {
            for (int i = 0; i < limitCones.size() - 1; i++) {
                Vector3 collisionPoint = inPoint.copy();
                collisionPoint.set(0, 0, 0);
                ewbik.processing.singlePrecision.LimitCone nextCone = limitCones.get(i + 1);
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
        } else if (orientationallyConstrained) {
            float pointdot = point.dot(limitCones.get(0).getControlPoint());
            float radcos = limitCones.get(0).getRadiusCosine();
            if (pointdot > radcos) {
                inBounds[0] = 1;
                return inPoint;
            } else {
                Vector3 axis = limitCones.get(0).getControlPoint().crossCopy(point);
                Quaternion toLimit = new Quaternion(axis, limitCones.get(0).getRadius());
                Vector3 newPoint = toLimit.applyToCopy(limitCones.get(0).getControlPoint());
                return newPoint;
            }
        } else {
            inBounds[0] = 1;
            return inPoint;
        }
    }

    public Vector3 pointOnPathSequence(Vector3 inPoint,
            Node3D limitingNode3D) {
        float closestPointDot = 0f;
        Vector3 point = limitingNode3D.getLocalOf(inPoint);
        point.normalize();
        Vector3 result = (Vector3) point.copy();

        if (limitCones.size() == 1) {
            result.set(limitCones.get(0).getControlPoint());
        } else {
            for (int i = 0; i < limitCones.size() - 1; i++) {
                ewbik.processing.singlePrecision.LimitCone nextCone = limitCones.get(i + 1);
                Vector3 closestPathPoint = limitCones.get(i).getClosestPathPoint(nextCone, point);
                float closeDot = closestPathPoint.dot(point);
                if (closeDot > closestPointDot) {
                    result.set(closestPathPoint);
                    closestPointDot = closeDot;
                }
            }
        }

        return limitingNode3D.getGlobalOf(result);
    }

    public Bone attachedTo() {
        return this.attachedTo;
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
    public void addLimitCone(Vector3 newPoint, float radius,
            ewbik.processing.singlePrecision.LimitCone previous,
            ewbik.processing.singlePrecision.LimitCone next) {
        int insertAt = 0;

        if (next == null || limitCones.size() == 0) {
            addLimitConeAtIndex(-1, newPoint, radius);
        } else if (previous != null) {
            insertAt = limitCones.indexOf(previous) + 1;
        } else {
            insertAt = (int) MathUtils.max(0, limitCones.indexOf(next));
        }
        addLimitConeAtIndex(insertAt, newPoint, radius);
    }

    public void removeLimitCone(ewbik.processing.singlePrecision.LimitCone limitCone) {
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
    public void addLimitConeAtIndex(int insertAt, Vector3 newPoint, float radius) {
        ewbik.processing.singlePrecision.LimitCone newCone = createLimitConeForIndex(insertAt, newPoint,
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
            result = (2 * MathUtils.PI) + angle;
        }
        result = result % (MathUtils.PI * 2);
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
        return minAxialAngle;
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
        return signedAngleDifference(range + minAxialAngle, MathUtils.PI * 2f);
    }

    public boolean isAxiallyConstrained() {
        return axiallyConstrained;
    }

    public boolean isOrientationallyConstrained() {
        return orientationallyConstrained;
    }

    public void disableOrientationalLimits() {
        this.orientationallyConstrained = false;
    }

    public void enableOrientationalLimits() {
        this.orientationallyConstrained = true;
    }

    public void toggleOrientationalLimits() {
        this.orientationallyConstrained = !this.orientationallyConstrained;
    }

    public void disableAxialLimits() {
        this.axiallyConstrained = false;
    }

    public void enableAxialLimits() {
        this.axiallyConstrained = true;
    }

    public void toggleAxialLimits() {
        axiallyConstrained = !axiallyConstrained;
    }

    public boolean isEnabled() {
        return axiallyConstrained || orientationallyConstrained;
    }

    public void disable() {
        this.axiallyConstrained = false;
        this.orientationallyConstrained = false;
    }

    public void enable() {
        this.axiallyConstrained = true;
        this.orientationallyConstrained = true;
    }

    /**
     * @return a measure of the rotational freedom afforded by this constraint.
     *         with 0 meaning no rotational freedom (the bone is essentially
     *         stationary in relation to its parent)
     *         and 1 meaning full rotational freedom (the bone is completely
     *         unconstrained).
     *         <p>
     *         This should be computed as ratio between orientations a bone can be
     *         in and orientations
     *         a bone cannot be in as defined by its representation as a point on
     *         the surface of a hypersphere.
     */
    public float getRotationalFreedom() {

        // computation cached from updateRotationalFreedom
        // feel free to override that method if you want your own more correct result.
        // please contribute back a better solution if you write one.
        return rotationalFreedom;
    }

    protected void updateRotationalFreedom() {
        float axialConstrainedHyperArea = isAxiallyConstrained() ? (range / TAU) : 1f;
        float totalLimitConeSurfaceAreaRatio = 0f;
        for (ewbik.processing.singlePrecision.LimitCone l : limitCones) {
            totalLimitConeSurfaceAreaRatio += (l.getRadius() * 2f) / TAU;
        }
        rotationalFreedom = axialConstrainedHyperArea
                * (isOrientationallyConstrained() ? MathUtils.min(totalLimitConeSurfaceAreaRatio, 1f) : 1f);
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
    public void attachTo(Bone forBone) {
        this.attachedTo = forBone;
        if (this.limitingNode3D == null)
            this.limitingNode3D = forBone.getMajorRotationAxes();
        else {
            forBone.setFrameofRotation(this.limitingNode3D);
            this.limitingNode3D = forBone.getMajorRotationAxes();
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
        this.strength = MathUtils.max(0f, MathUtils.min(1f, newStrength));
    }

    @Override
    public void makeSaveable(SaveManager saveManager) {
        saveManager.addToSaveState(this);
        for (ewbik.processing.singlePrecision.LimitCone lc : limitCones) {
            lc.makeSaveable(saveManager);
        }
    }

    @Override
    public ewbik.asj.data.JSONObject getSaveJSON(SaveManager saveManager) {
        ewbik.asj.data.JSONObject saveJSON = new ewbik.asj.data.JSONObject();
        saveJSON.setString("identityHash", this.getIdentityHash());
        saveJSON.setString("limitAxes", limitingAxes().getIdentityHash());
        saveJSON.setString("attachedTo", attachedTo().getIdentityHash());
        saveJSON.setJSONArray("limitCones", saveManager.arrayListToJSONArray(limitCones));
        saveJSON.setFloat("minAxialAngle", minAxialAngle);
        saveJSON.setFloat("axialRange", range);
        saveJSON.setBoolean("axiallyConstrained", this.axiallyConstrained);
        saveJSON.setBoolean("orientationallyConstrained", this.orientationallyConstrained);
        saveJSON.setFloat("painfulness", this.painfullness);
        return saveJSON;
    }

    public void loadFromJSONObject(ewbik.asj.data.JSONObject j, LoadManager l) {
        this.attachedTo = l.getObjectFor(Bone.class, j, "attachedTo");
        this.limitingNode3D = l.getObjectFor(Node3D.class, j, "limitAxes");
        limitCones = new ArrayList<>();
        l.arrayListFromJSONArray(j.getJSONArray("limitCones"), limitCones,
                ewbik.processing.singlePrecision.LimitCone.class);
        this.minAxialAngle = j.getFloat("minAxialAngle");
        this.range = j.getFloat("axialRange");
        this.axiallyConstrained = j.getBoolean("axiallyConstrained");
        this.orientationallyConstrained = j.getBoolean("orientationallyConstrained");
        this.painfullness = j.getFloat("painfulness");
    }

    @Override
    public void notifyOfSaveIntent(SaveManager saveManager) {
    }

    @Override
    public void notifyOfSaveCompletion(SaveManager saveManager) {
    }

    @Override
    public void notifyOfLoadCompletion() {
        this.constraintUpdateNotification();
        this.optimizeLimitingAxes();
    }

    @Override
    public boolean isLoading() {
        return false;
    }

    @Override
    public void setLoading(boolean loading) {
    }
}
