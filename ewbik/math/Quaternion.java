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

package ewbik.math;

import ewbik.asj.data.JSONArray;

public class Quaternion {
    public MRotation rotation;
    private float[] workingInput = new float[3];
    private float[] workingOutput = new float[3];

    public Quaternion() {
        this.rotation = new MRotation(
                MRotation.IDENTITY.getQ0(),
                MRotation.IDENTITY.getQ1(),
                MRotation.IDENTITY.getQ2(),
                MRotation.IDENTITY.getQ3(), false);
    }

    ;

    public Quaternion(MRotation r) {
        this.rotation = new MRotation(r.getQ0(), r.getQ1(), r.getQ2(), r.getQ3());
    }

    public <V extends Vec3f<?>> Quaternion(V v1, V v2, V u1, V u2) {
        // try {
        rotation = new MRotation(v1, v2, u1, u2);
        // } catch(Exception e) {
        // rotation = new MRotation(v1, 0f);
        // }
    }

    public <V extends Vec3f<?>> Quaternion(V axis, float angle) {
        // try {
        rotation = new MRotation(axis, angle);
        // } catch(Exception e) {
        // rotation = new MRotation(RotationOrder.X, 0f);
        // }
    }

    public Quaternion(float w, float x, float y, float z, boolean needsNormalization) {
        this.rotation = new MRotation(w, x, y, z, needsNormalization);
    }

    public <V extends Vec3f<?>> Quaternion(V begin, V end) {
        // try{
        rotation = new MRotation(begin, end);
        // } catch(Exception e) {
        // rotation = new MRotation(RotationOrder.X, 0f);
        // }
    }

    /*
     * interpolate between two rotations (SLERP)
     *
     */
    public Quaternion(float amount, Quaternion v1, Quaternion v2) {
        rotation = slerp(amount, v1.rotation, v2.rotation);
    }

    /**
     * loads a rotation from a JSON array of quaternion values.
     * <p>
     * where
     * jarray[0] = q0 = w;
     * jarray[1] = q1 = x;
     * jarray[2] = q2 = y;
     * jarray[3] = q3 = z;
     *
     * @param jarray
     */
    public Quaternion(JSONArray jarray) {
        rotation = new MRotation(jarray.getFloat(0), jarray.getFloat(1), jarray.getFloat(2), jarray.getFloat(3), true);
    }

    public static MRotation slerp(float amount, MRotation value1, MRotation value2) {

        if (Float.isNaN(amount)) {
            return new MRotation(value1.getQ0(), value1.getQ1(), value1.getQ2(), value1.getQ3());
        }
        if (amount < 0.0f)
            return value1;
        else if (amount > 1.0f)
            return value2;

        float dot = value1.dotProduct(value2);
        float x2, y2, z2, w2;
        /*
         * if (dot < 0.0f)
         * {
         * dot = 0.0f - dot;
         * x2 = 0.0f - value2.getQ1();
         * y2 = 0.0f - value2.getQ2();
         * z2 = 0.0f - value2.getQ3();
         * w2 = 0.0f - value2.getQ0();
         * }
         * else
         * {
         */
        x2 = value2.getQ1();
        y2 = value2.getQ2();
        z2 = value2.getQ3();
        w2 = value2.getQ0();
        // }

        float t1, t2;

        final float EPSILON = 0.0001f;
        if ((1.0f - dot) > EPSILON) // standard case (slerp)
        {
            float angle = MathUtils.acos(dot);
            float sinAngle = MathUtils.sin(angle);
            t1 = MathUtils.sin((1.0f - amount) * angle) / sinAngle;
            t2 = MathUtils.sin(amount * angle) / sinAngle;
        } else // just lerp
        {
            t1 = 1.0f - amount;
            t2 = amount;
        }

        return new MRotation(
                (value1.getQ0() * t1) + (w2 * t2),
                (value1.getQ1() * t1) + (x2 * t2),
                (value1.getQ2() * t1) + (y2 * t2),
                (value1.getQ3() * t1) + (z2 * t2));
    }

    public static Quaternion nlerp(Quaternion[] rotations, float[] weights) {

        if (weights == null) {
            return nlerp(rotations);
        } else {
            float q0 = 0f;
            float q1 = 0f;
            float q2 = 0f;
            float q3 = 0f;
            float total = 0f;

            for (int i = 0; i < rotations.length; i++) {
                MRotation r = rotations[i].rotation;
                float weight = weights[i];
                q0 += r.getQ0() * weight;
                q1 += r.getQ1() * weight;
                q2 += r.getQ2() * weight;
                q3 += r.getQ3() * weight;
                total += weight;
            }

            q0 /= total;
            q1 /= total;
            q2 /= total;
            q3 /= total;

            return new Quaternion(q0, q1, q2, q3, true);
        }
    }

    public static Quaternion nlerp(Quaternion[] rotations) {
        float q0 = 0f;
        float q1 = 0f;
        float q2 = 0f;
        float q3 = 0f;
        float total = rotations.length;

        for (int i = 0; i < rotations.length; i++) {
            MRotation r = rotations[i].rotation;
            q0 += r.getQ0();
            q1 += r.getQ1();
            q2 += r.getQ2();
            q3 += r.getQ3();
        }
        q0 /= total;
        q1 /= total;
        q2 /= total;
        q3 /= total;

        return new Quaternion(q0, q1, q2, q3, true);
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
     * returns null.
     */
    public static Quaternion instantaneousAvg(Quaternion[] rots, float[] weights) {
        Vector3 accumulatedAxisAngle = new Vector3();
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
            Vector3 axis = rots[i].getAxis();
            float angle = rots[i].getAngle();
            angle /= totalWeight;
            if (weights != null) {
                angle *= weights[i];
            }
            axis.mult(angle);
            accumulatedAxisAngle.add(axis);
        }
        float extractAngle = accumulatedAxisAngle.mag();
        accumulatedAxisAngle.div(extractAngle);
        return new Quaternion(accumulatedAxisAngle, extractAngle);
    }

    public Quaternion copy() {
        return new Quaternion(new MRotation(rotation.getQ0(), rotation.getQ1(), rotation.getQ2(), rotation.getQ3(), false));
    }

    /**
     * sets the value of this rotation to r
     *
     * @param r a rotation to make this rotation equivalent to
     */
    public void set(MRotation r) {
        if (r != null)
            this.rotation.set(r.getQ0(), r.getQ1(), r.getQ2(), r.getQ3(), false);
        else
            this.set(MRotation.IDENTITY);
    }

    /**
     * sets the value of this rotation to r
     *
     * @param r a rotation to make this rotation equivalent to
     */
    public void set(Quaternion r) {
        if (r != null)
            this.set(r.rotation);
        else
            this.set(MRotation.IDENTITY);
    }

    /**
     * sets the value of this rotation to what is represented
     * by the input axis angle parameters
     *
     * @param axis
     * @param angle
     */
    public <V extends Vec3f<?>> void set(V axis, float angle) {
        this.rotation.set(axis, angle);
    }

    /**
     * sets the value of this rotation to what is represented
     * by the input startVector targetVector parameters
     *
     * @param axis
     * @param angle
     */
    public <V extends Vec3f<?>> void set(V startVec, V targetVec) {
        this.rotation.set(startVec, targetVec);
    }

    public <V extends Vec3f<?>> void applyTo(V v, V output) {
        workingInput[0] = v.x;
        workingInput[1] = v.y;
        workingInput[2] = v.z;
        rotation.applyTo(workingInput, workingOutput);
        output.set(workingOutput);
    }

    public <V extends Vec3f<?>> void applyInverseTo(V v, V output) {
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

    public <T extends Vec3f<?>> T applyToCopy(T v) {
        workingInput[0] = v.x;
        workingInput[1] = v.y;
        workingInput[2] = v.z;
        rotation.applyTo(workingInput, workingOutput);
        T copy = (T) v.copy();
        return (T) copy.set(workingOutput[0], workingOutput[1], workingOutput[2]);
    }

    public <T extends Vec3f<?>> T applyInverseToCopy(T v) {
        workingInput[0] = v.x;
        workingInput[1] = v.y;
        workingInput[2] = v.z;
        rotation.applyInverseTo(workingInput, workingOutput);
        T copy = (T) v.copy();
        return (T) copy.set(workingOutput[0], workingOutput[1], workingOutput[2]);
    }

    /**
     * Given a rotation r, this function returns a rotation L such that
     * this.applyTo(L) = r.
     *
     * @param r
     * @return public Quaternion getLocalOfRotation(Quaternion r) {
     * Rotation composedRot = this.rotation.composeInverse(r.rotation,
     * RotationConvention.VECTOR_OPERATOR);
     * <p>
     * <p>
     * return new Quaternion(composedRot);
     * }
     */

    private Quaternion getNormalized(MRotation r) {
        return new Quaternion(new MRotation(r.getQ0(), r.getQ1(), r.getQ2(), r.getQ3(), true));
    }

    public Ray3 applyToCopy(Ray3 rIn) {
        workingInput[0] = rIn.p2().x - rIn.p1().x;
        workingInput[1] = rIn.p2().y - rIn.p1().y;
        workingInput[2] = rIn.p2().z - rIn.p1().z;

        this.rotation.applyTo(workingInput, workingOutput);
        Ray3 result = rIn.copy();
        result.heading(workingOutput);
        return result;
    }

    public Ray3 applyInverseTo(Ray3 rIn) {
        workingInput[0] = rIn.p2().x - rIn.p1().x;
        workingInput[1] = rIn.p2().y - rIn.p1().y;
        workingInput[2] = rIn.p2().z - rIn.p1().z;

        this.rotation.applyInverseTo(workingInput, workingOutput);
        Ray3 result = rIn.copy();
        result.p2().add(workingOutput);
        return result;
    }

    public void applyTo(Quaternion rot, Quaternion storeIn) {
        MRotation r = rot.rotation;
        MRotation tr = this.rotation;
        storeIn.rotation.set(
                r.getQ0() * tr.getQ0() - (r.getQ1() * tr.getQ1() + r.getQ2() * tr.getQ2() + r.getQ3() * tr.getQ3()),
                r.getQ1() * tr.getQ0() + r.getQ0() * tr.getQ1() + (r.getQ2() * tr.getQ3() - r.getQ3() * tr.getQ2()),
                r.getQ2() * tr.getQ0() + r.getQ0() * tr.getQ2() + (r.getQ3() * tr.getQ1() - r.getQ1() * tr.getQ3()),
                r.getQ3() * tr.getQ0() + r.getQ0() * tr.getQ3() + (r.getQ1() * tr.getQ2() - r.getQ2() * tr.getQ1()),
                true);
    }

    public void applyInverseTo(Quaternion rot, Quaternion storeIn) {
        MRotation r = rot.rotation;
        MRotation tr = this.rotation;
        storeIn.rotation.set(
                -r.getQ0() * tr.getQ0() - (r.getQ1() * tr.getQ1() + r.getQ2() * tr.getQ2() + r.getQ3() * tr.getQ3()),
                -r.getQ1() * tr.getQ0() + r.getQ0() * tr.getQ1() + (r.getQ2() * tr.getQ3() - r.getQ3() * tr.getQ2()),
                -r.getQ2() * tr.getQ0() + r.getQ0() * tr.getQ2() + (r.getQ3() * tr.getQ1() - r.getQ1() * tr.getQ3()),
                -r.getQ3() * tr.getQ0() + r.getQ0() * tr.getQ3() + (r.getQ1() * tr.getQ2() - r.getQ2() * tr.getQ1()),
                true);
    }

    public Quaternion applyTo(Quaternion rot) {
        MRotation r = rot.rotation;
        MRotation tr = this.rotation;
        MRotation result = new MRotation(
                r.getQ0() * tr.getQ0() - (r.getQ1() * tr.getQ1() + r.getQ2() * tr.getQ2() + r.getQ3() * tr.getQ3()),
                r.getQ1() * tr.getQ0() + r.getQ0() * tr.getQ1() + (r.getQ2() * tr.getQ3() - r.getQ3() * tr.getQ2()),
                r.getQ2() * tr.getQ0() + r.getQ0() * tr.getQ2() + (r.getQ3() * tr.getQ1() - r.getQ1() * tr.getQ3()),
                r.getQ3() * tr.getQ0() + r.getQ0() * tr.getQ3() + (r.getQ1() * tr.getQ2() - r.getQ2() * tr.getQ1()),
                true);
        return new Quaternion(result);
    }

    public Quaternion applyInverseTo(Quaternion rot) {
        MRotation r = rot.rotation;
        MRotation tr = this.rotation;
        MRotation result = new MRotation(
                -r.getQ0() * tr.getQ0() - (r.getQ1() * tr.getQ1() + r.getQ2() * tr.getQ2() + r.getQ3() * tr.getQ3()),
                -r.getQ1() * tr.getQ0() + r.getQ0() * tr.getQ1() + (r.getQ2() * tr.getQ3() - r.getQ3() * tr.getQ2()),
                -r.getQ2() * tr.getQ0() + r.getQ0() * tr.getQ2() + (r.getQ3() * tr.getQ1() - r.getQ1() * tr.getQ3()),
                -r.getQ3() * tr.getQ0() + r.getQ0() * tr.getQ3() + (r.getQ1() * tr.getQ2() - r.getQ2() * tr.getQ1()),
                true);
        return new Quaternion(result);
    }

    public float getAngle() {
        return (float) rotation.getAngle();
    }

    public Vector3 getAxis() {
        Vector3 result = new Vector3();
        getAxis(result);
        return result;
    }

    public <T extends Vec3f<?>> void getAxis(T output) {
        rotation.setToAxis(output);
    }

    public Quaternion revert() {
        return new Quaternion(this.rotation.revert());
    }

    /**
     * sets the values of the given rotation equal to the inverse of this rotation
     *
     * @param storeIN
     */
    public void setToReversion(Quaternion r) {
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
     * @return an Array of Quaternion objects. With the first element representing the
     * swing, and the second representing the twist
     * @see <a href=
     * "http://www.euclideanspace.com/maths/geometry/rotations/for/decomposition">calculation</a>
     */
    public Quaternion[] getSwingTwist(Vector3 axis) {
        Quaternion twistRot = new Quaternion(new MRotation(rotation.getQ0(), rotation.getQ1(), rotation.getQ2(), rotation.getQ3()));
        final float d = Vector3.dot(twistRot.rotation.getQ1(), twistRot.rotation.getQ2(), twistRot.rotation.getQ3(),
                axis.x, axis.y, axis.z);
        twistRot.rotation.set(rotation.getQ0(), axis.x * d, axis.y * d, axis.z * d, true);
        if (d < 0)
            twistRot.rotation.multiply(-1f);

        Quaternion swing = new Quaternion(twistRot.rotation);
        swing.rotation.setToConjugate();
        swing.rotation = MRotation.multiply(twistRot.rotation, swing.rotation);

        Quaternion[] result = new Quaternion[2];
        result[0] = swing;
        result[1] = twistRot;
        return result;
    }

    public String toString() {
        return rotation.toString();// "\n axis: "+ this.getAxis().toVec3f() +", \n angle:
        // "+((float)MathUtils.toDegrees(this.getAngle()));
    }

    public boolean equalTo(Quaternion m) {
        return MRotation.distance(this.rotation, m.rotation) < MathUtils.DOUBLE_ROUNDING_ERROR;
    }

    public JSONArray toJsonArray() {
        JSONArray result = new JSONArray();
        result.append(this.rotation.getQ0());
        result.append(this.rotation.getQ1());
        result.append(this.rotation.getQ2());
        result.append(this.rotation.getQ3());
        return result;
    }

}
