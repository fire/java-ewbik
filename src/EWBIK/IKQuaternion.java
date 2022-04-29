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
    private final float[] workingInput = new float[3];
    private final float[] workingOutput = new float[3];
    public IKBasis rotation;

    public IKQuaternion() {
        this.rotation = new IKBasis(
                IKBasis.IDENTITY.getX(), IKBasis.IDENTITY.getY(), IKBasis.IDENTITY.getZ(), IKBasis.IDENTITY.getW(),
                false);
    }

    public IKQuaternion(IKBasis r) {
        this.rotation = new IKBasis(r.getX(), r.getY(), r.getZ(), r.getW());
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

    public IKQuaternion applyTo(IKQuaternion rot) {
        IKBasis r = rot.rotation;
        IKBasis tr = this.rotation;
        IKBasis result = new IKBasis(
                r.getX() * tr.getW() + r.getW() * tr.getX() + (r.getY() * tr.getZ() - r.getZ() * tr.getY()), r.getY() * tr.getW() + r.getW() * tr.getY() + (r.getZ() * tr.getX() - r.getX() * tr.getZ()), r.getZ() * tr.getW() + r.getW() * tr.getZ() + (r.getX() * tr.getY() - r.getY() * tr.getX()), r.getW() * tr.getW() - (r.getX() * tr.getX() + r.getY() * tr.getY() + r.getZ() * tr.getZ()),
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
     * the
     * swing, and the second representing the twist
     * @see <a href=
     * "http://www.euclideanspace.com/maths/geometry/rotations/for/decomposition">calculation</a>
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

    public void clampToAngle(float angle) {
        rotation.clampToAngle(angle);
    }

    public void clampToQuadranceAngle(float cosHalfAngle) {
        rotation.clampToQuadranceAngle(cosHalfAngle);
    }

}
