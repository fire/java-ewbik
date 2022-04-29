/*

Copyright (c) 2016 Eron Gjoni

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 */

package EWBIK;

public class IKQuaternion {
    public IKBasis rotation = IKBasis.IDENTITY;
    private final float[] workingInput = new float[3];
    private final float[] workingOutput = new float[3];

    public IKQuaternion() {
        this.rotation = new IKBasis(
                IKBasis.IDENTITY.getX(), IKBasis.IDENTITY.getY(), IKBasis.IDENTITY.getZ(), IKBasis.IDENTITY.getW(),
                false);
    }

    public IKQuaternion(IKBasis r) {
        this.rotation = new IKBasis(r.getX(), r.getY(), r.getZ(), r.getW());
    }

    public IKQuaternion(IKRotationOrder order,
                        float alpha1, float alpha2, float alpha3) {
        rotation = new IKBasis(order, alpha1, alpha2, alpha3);
    }

    public IKQuaternion(IKVector3 v1, IKVector3 v2, IKVector3 u1, IKVector3 u2) {
        rotation = new IKBasis(v1, v2, u1, u2);
    }

    public IKQuaternion(IKVector3 axis, float angle) {
        rotation = new IKBasis(axis, angle);
    }

    public IKQuaternion(float w, float x, float y, float z, boolean needsNormalization) {
        this.rotation = new IKBasis(x, y, z, w, needsNormalization);
    }

    public IKQuaternion(IKVector3 begin, IKVector3 end) {
        rotation = new IKBasis(begin, end);
    }

    /*
     * interpolate between two rotations (SLERP)
     *
     */
    public IKQuaternion(float amount, IKQuaternion v1, IKQuaternion v2) {
        rotation = slerp(amount, v1.rotation, v2.rotation);
    }

    public static IKBasis slerp(float amount, IKBasis value1, IKBasis value2) {

        if (Float.isNaN(amount)) {
            return new IKBasis(value1.getX(), value1.getY(), value1.getZ(), value1.getW());
        }
        if (amount < 0.0f)
            return value1;
        else if (amount > 1.0f)
            return value2;

        float dot = value1.dotProduct(value2);
        float x2, y2, z2, w2;
        x2 = value2.getX();
        y2 = value2.getY();
        z2 = value2.getZ();
        w2 = value2.getW();

        float t1, t2;

        final float EPSILON = 0.0001f;
        if ((1.0f - dot) > EPSILON) // The standard case (slerp).
        {
            float angle = IKMathUtils.acos(dot);
            float sinAngle = IKMathUtils.sin(angle);
            t1 = IKMathUtils.sin((1.0f - amount) * angle) / sinAngle;
            t2 = IKMathUtils.sin(amount * angle) / sinAngle;
        } else // lerp
        {
            t1 = 1.0f - amount;
            t2 = amount;
        }

        return new IKBasis(
                (value1.getX() * t1) + (x2 * t2), (value1.getY() * t1) + (y2 * t2), (value1.getZ() * t1) + (z2 * t2), (value1.getW() * t1) + (w2 * t2)
        );
    }

    public static IKQuaternion nlerp(IKQuaternion[] rotations, float[] weights) {

        if (weights == null) {
            return nlerp(rotations);
        } else {
            float q0 = 0f;
            float q1 = 0f;
            float q2 = 0f;
            float q3 = 0f;
            float total = 0f;

            for (int i = 0; i < rotations.length; i++) {
                IKBasis r = rotations[i].rotation;
                float weight = weights[i];
                q0 += r.getW() * weight;
                q1 += r.getX() * weight;
                q2 += r.getY() * weight;
                q3 += r.getZ() * weight;
                total += weight;
            }

            q0 /= total;
            q1 /= total;
            q2 /= total;
            q3 /= total;

            return new IKQuaternion(q0, q1, q2, q3, true);
        }
    }

    public static IKQuaternion nlerp(IKQuaternion[] rotations) {
        float q0 = 0f;
        float q1 = 0f;
        float q2 = 0f;
        float q3 = 0f;
        float total = rotations.length;

        for (int i = 0; i < rotations.length; i++) {
            IKBasis r = rotations[i].rotation;
            q0 += r.getW();
            q1 += r.getX();
            q2 += r.getY();
            q3 += r.getZ();
        }
        q0 /= total;
        q1 /= total;
        q2 /= total;
        q3 /= total;

        return new IKQuaternion(q0, q1, q2, q3, true);
    }

    /**
     * finds the instantaneous (optionally weighted) average of the given rotations
     * in an order-agnostic manner. Conceptually, this is equivalent to breaking
     * down
     * each rotation into an infinitesimal sequence of rotations, and then applying
     * the rotations
     * in alternation.
     *
     * @param rots
     * @param weights if this parameter receives null, equal weights are assumed for
     *                all rotations. Otherwise,
     *                every element in this array is treated as a weight on the
     *                corresponding element of the rots array.
     * @return the weighted average Rotation. If the total weights are 0, then
     *         returns null.
     */
    public static IKQuaternion instantaneousAvg(IKQuaternion[] rots, float[] weights) {
        IKVector3 accumulatedAxisAngle = new IKVector3();
        float totalWeight = rots.length;
        if (weights != null) {
            totalWeight = 0f;
            for (int i = 0; i < weights.length; i++) {
                totalWeight += weights[i];
            }
        }

        if (totalWeight == 0) {
            return null;
        }

        for (int i = 0; i < rots.length; i++) {
            IKVector3 axis = rots[i].getAxis();
            float angle = rots[i].getAngle();
            angle /= totalWeight;
            if (weights != null) {
                angle *= weights[i];
            }
            axis.multiply(angle);
            accumulatedAxisAngle.add(axis);
        }
        float extractAngle = accumulatedAxisAngle.mag();
        accumulatedAxisAngle.divide(extractAngle);
        return new IKQuaternion(accumulatedAxisAngle, extractAngle);
    }

    public IKQuaternion copy() {
        return new IKQuaternion(
                new IKBasis(rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getW(), false));
    }

    /**
     * sets the value of this rotation to r
     *
     * @param r a rotation to make this rotation equivalent to
     */
    public void set(IKBasis r) {
        if (r != null)
            this.rotation.set(r.getW(), r.getX(), r.getY(), r.getZ(), false);
        else
            this.set(IKBasis.IDENTITY);
    }

    /**
     * sets the value of this rotation to r
     *
     * @param r a rotation to make this rotation equivalent to
     */
    public void set(IKQuaternion r) {
        if (r != null)
            this.set(r.rotation);
        else
            this.set(IKBasis.IDENTITY);
    }

    /**
     * sets the value of this rotation to what is represented
     * by the input axis angle parameters
     *
     * @param axis
     * @param angle
     */
    public void set(IKVector3 axis, float angle) {
        this.rotation.set(axis, angle);
    }

    /**
     * sets the value of this rotation to what is represented
     * by the input startVector targetVector parameters
     *
     * @param axis
     * @param angle
     */
    public void set(IKVector3 startVec, IKVector3 targetVec) {
        this.rotation.set(startVec, targetVec);
    }

    public void applyTo(IKVector3 v, IKVector3 output) {
        workingInput[0] = v.x;
        workingInput[1] = v.y;
        workingInput[2] = v.z;
        rotation.applyTo(workingInput, workingOutput);
        output.set(workingOutput);
    }

    public void applyInverseTo(IKVector3 v, IKVector3 output) {
        workingInput[0] = v.x;
        workingInput[1] = v.y;
        workingInput[2] = v.z;
        rotation.applyInverseTo(workingInput, workingOutput);
        output.set(workingOutput);
    }

    /**
     * applies the rotation to a copy of the input vector
     *
     * @param v
     * @return
     */

    public IKVector3 applyToCopy(IKVector3 v) {
        workingInput[0] = v.x;
        workingInput[1] = v.y;
        workingInput[2] = v.z;
        rotation.applyTo(workingInput, workingOutput);
        IKVector3 copy = (IKVector3) v.copy();
        return (IKVector3) copy.set(workingOutput[0], workingOutput[1], workingOutput[2]);
    }

    public IKVector3 applyInverseToCopy(IKVector3 v) {
        workingInput[0] = v.x;
        workingInput[1] = v.y;
        workingInput[2] = v.z;
        rotation.applyInverseTo(workingInput, workingOutput);
        IKVector3 copy = (IKVector3) v.copy();
        return (IKVector3) copy.set(workingOutput[0], workingOutput[1], workingOutput[2]);
    }

    /**
     * Given a rotation r, this function returns a rotation L such that
     * this.applyTo(L) = r.
     *
     * @param r
     * @return public Quaternion getLocalOfRotation(Quaternion r) {
     *         Rotation composedRot = this.rotation.composeInverse(r.rotation,
     *         RotationConvention.VECTOR_OPERATOR);
     *         <p>
     *         <p>
     *         return new Quaternion(composedRot);
     *         }
     */

    private IKQuaternion getNormalized(IKBasis r) {
        return new IKQuaternion(new IKBasis(r.getX(), r.getY(), r.getZ(), r.getW(), true));
    }

    public IKRay3D applyToCopy(IKRay3D rIn) {
        workingInput[0] = rIn.p2().x - rIn.p1().x;
        workingInput[1] = rIn.p2().y - rIn.p1().y;
        workingInput[2] = rIn.p2().z - rIn.p1().z;

        this.rotation.applyTo(workingInput, workingOutput);
        IKRay3D result = rIn.copy();
        result.heading(workingOutput);
        return result;
    }

    public IKRay3D applyInverseTo(IKRay3D rIn) {
        workingInput[0] = rIn.p2().x - rIn.p1().x;
        workingInput[1] = rIn.p2().y - rIn.p1().y;
        workingInput[2] = rIn.p2().z - rIn.p1().z;

        this.rotation.applyInverseTo(workingInput, workingOutput);
        IKRay3D result = rIn.copy();
        result.p2().add(workingOutput);
        return result;
    }

    public void applyTo(IKQuaternion rot, IKQuaternion storeIn) {
        IKBasis r = rot.rotation;
        IKBasis tr = this.rotation;
        storeIn.rotation.set(
                r.getW() * tr.getW() - (r.getX() * tr.getX() + r.getY() * tr.getY() + r.getZ() * tr.getZ()),
                r.getX() * tr.getW() + r.getW() * tr.getX() + (r.getY() * tr.getZ() - r.getZ() * tr.getY()),
                r.getY() * tr.getW() + r.getW() * tr.getY() + (r.getZ() * tr.getX() - r.getX() * tr.getZ()),
                r.getZ() * tr.getW() + r.getW() * tr.getZ() + (r.getX() * tr.getY() - r.getY() * tr.getX()),
                true);
    }

    public void applyInverseTo(IKQuaternion rot, IKQuaternion storeIn) {
        IKBasis r = rot.rotation;
        IKBasis tr = this.rotation;
        storeIn.rotation.set(
                -r.getW() * tr.getW() - (r.getX() * tr.getX() + r.getY() * tr.getY() + r.getZ() * tr.getZ()),
                -r.getX() * tr.getW() + r.getW() * tr.getX() + (r.getY() * tr.getZ() - r.getZ() * tr.getY()),
                -r.getY() * tr.getW() + r.getW() * tr.getY() + (r.getZ() * tr.getX() - r.getX() * tr.getZ()),
                -r.getZ() * tr.getW() + r.getW() * tr.getZ() + (r.getX() * tr.getY() - r.getY() * tr.getX()),
                true);
    }

    public IKQuaternion applyTo(IKQuaternion rot) {
        IKBasis r = rot.rotation;
        IKBasis tr = this.rotation;
        IKBasis result = new IKBasis(
                r.getX() * tr.getW() + r.getW() * tr.getX() + (r.getY() * tr.getZ() - r.getZ() * tr.getY()), r.getY() * tr.getW() + r.getW() * tr.getY() + (r.getZ() * tr.getX() - r.getX() * tr.getZ()), r.getZ() * tr.getW() + r.getW() * tr.getZ() + (r.getX() * tr.getY() - r.getY() * tr.getX()), r.getW() * tr.getW() - (r.getX() * tr.getX() + r.getY() * tr.getY() + r.getZ() * tr.getZ()),
                true);
        return new IKQuaternion(result);
    }

    public IKQuaternion applyInverseTo(IKQuaternion rot) {
        IKBasis r = rot.rotation;
        IKBasis tr = this.rotation;
        IKBasis result = new IKBasis(
                -r.getX() * tr.getW() + r.getW() * tr.getX() + (r.getY() * tr.getZ() - r.getZ() * tr.getY()), -r.getY() * tr.getW() + r.getW() * tr.getY() + (r.getZ() * tr.getX() - r.getX() * tr.getZ()), -r.getZ() * tr.getW() + r.getW() * tr.getZ() + (r.getX() * tr.getY() - r.getY() * tr.getX()), -r.getW() * tr.getW() - (r.getX() * tr.getX() + r.getY() * tr.getY() + r.getZ() * tr.getZ()),
                true);
        return new IKQuaternion(result);
    }

    public float getAngle() {
        return rotation.getAngle();
    }

    public IKVector3 getAxis() {
        IKVector3 result = new IKVector3();
        getAxis(result);
        return result;
    }

    public void getAxis(IKVector3 output) {
        rotation.setToAxis(output);
    }

    public IKQuaternion revert() {
        return new IKQuaternion(this.rotation.revert());
    }

    /**
     * sets the values of the given rotation equal to the inverse of this rotation
     *
     * @param storeIN
     */
    public void setToReversion(IKQuaternion r) {
        rotation.revert(r.rotation);
    }

    /**
     * Get the swing rotation and twist rotation for the specified axis. The twist
     * rotation represents the rotation around the
     * specified axis. The swing rotation represents the rotation of the specified
     * axis itself, which is the rotation around an
     * axis perpendicular to the specified axis. The swing and twist rotation can be
     * used to reconstruct the original
     * quaternion: this = swing * twist
     *
     * @param axisX the X component of the normalized axis for which to get the
     *              swing and twist rotation
     * @param axisY the Y component of the normalized axis for which to get the
     *              swing and twist rotation
     * @param axisZ the Z component of the normalized axis for which to get the
     *              swing and twist rotation
     * @return an Array of Quaternion objects. With the first element representing
     *         the
     *         swing, and the second representing the twist
     * @see <a href=
     *      "http://www.euclideanspace.com/maths/geometry/rotations/for/decomposition">calculation</a>
     */
    public IKQuaternion[] getSwingTwist(IKVector3 axis) {
        IKQuaternion twistRot = new IKQuaternion(
                new IKBasis(rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getW()));
        final float d = IKVector3.dot(twistRot.rotation.getX(), twistRot.rotation.getY(), twistRot.rotation.getZ(),
                axis.x, axis.y, axis.z);
        twistRot.rotation.set(rotation.getW(), axis.x * d, axis.y * d, axis.z * d, true);
        if (d < 0)
            twistRot.rotation.multiply(-1f);

        IKQuaternion swing = new IKQuaternion(twistRot.rotation);
        swing.rotation.setToConjugate();
        swing.rotation = IKBasis.multiply(twistRot.rotation, swing.rotation);

        IKQuaternion[] result = new IKQuaternion[2];
        result[0] = swing;
        result[1] = twistRot;
        return result;
    }

    public String toString() {
        return rotation.toString();
    }

    public boolean equalTo(IKQuaternion m) {
        return IKBasis.distance(this.rotation, m.rotation) < IKMathUtils.DOUBLE_ROUNDING_ERROR;
    }

    public float[] getAngles(IKRotationOrder order) {
        return rotation.getAngles(order);
    }

    public IKVector3 applyTo(IKVector3 u) {
        return rotation.applyTo(u);
    }

    public void clampToAngle(float angle) {
        rotation.clampToAngle(angle);
    }

    public void clampToQuadranceAngle(float cosHalfAngle) {
        rotation.clampToQuadranceAngle(cosHalfAngle);
    }

}
