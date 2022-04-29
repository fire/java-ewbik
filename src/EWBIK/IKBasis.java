package EWBIK;

public class IKBasis {
    public static final IKBasis IDENTITY = new IKBasis(0.0f, 0.0f, 0.0f, 1.0f, false);

    /**
     * Scalar coordinate of the quaternion.
     */

    private float w;

    /**
     * First coordinate of the vectorial part of the quaternion.
     */
    private float x;

    /**
     * Second coordinate of the vectorial part of the quaternion.
     */
    private float y;

    /**
     * Third coordinate of the vectorial part of the quaternion.
     */
    private float z;

    /**
     * Build a rotation from the quaternion coordinates.
     * <p>
     * A rotation can be built from a <em>normalized</em> quaternion,
     * i.e. a quaternion for which q<sub>0</sub><sup>2</sup> +
     * q<sub>1</sub><sup>2</sup> + q<sub>2</sub><sup>2</sup> +
     * q<sub>3</sub><sup>2</sup> = 1. If the quaternion is not normalized,
     * the constructor can normalize it in a preprocessing step.
     * </p>
     * <p>
     * Note that some conventions put the scalar part of the quaternion
     * as the 4<sup>th</sup> component and the vector part as the first three
     * components. This is <em>not</em> our convention. We put the scalar part
     * as the first component.
     * </p>
     *
     * @param x                  first coordinate of the vectorial part of the
     *                           quaternion
     * @param y                  second coordinate of the vectorial part of the
     *                           quaternion
     * @param z                  third coordinate of the vectorial part of the
     *                           quaternion
     * @param w                  scalar part of the quaternion
     * @param needsNormalization if true, the coordinates are considered
     *                           not to be normalized, a normalization preprocessing
     *                           step is performed
     */
    public IKBasis(float x, float y, float z, float w,
                   boolean needsNormalization) {

        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;

        if (needsNormalization)
            setToNormalized();

    }

    /**
     * assumes no noralization required
     **/
    public IKBasis(float x, float y, float z, float w) {
        this(x, y, z, w, false);
    }

    /**
     * Build a rotation from an axis and an angle.
     * <p>
     * We use the convention that angles are oriented according to
     * the effect of the rotation on vectors around the axis. That means
     * that if (i, j, k) is a direct frame and if we first provide +k as
     * the axis and &pi;/2 as the angle to this constructor, and then
     * {@link #applyTo(T) apply} the instance to +i, we will get
     * +j.
     * </p>
     * <p>
     * Another way to represent our convention is to say that a rotation
     * of angle &theta; about the unit vector (x, y, z) is the same as the
     * rotation build from quaternion components { cos(-&theta;/2),
     * x * sin(-&theta;/2), y * sin(-&theta;/2), z * sin(-&theta;/2) }.
     * Note the minus sign on the angle!
     * </p>
     * <p>
     * On the one hand this convention is consistent with a vectorial
     * perspective (moving vectors in fixed frames), on the other hand it
     * is different from conventions with a frame perspective (fixed vectors
     * viewed from different frames) like the ones used for example in spacecraft
     * attitude community or in the graphics community.
     * </p>
     *
     * @param axis  axis around which to rotate
     * @param angle rotation angle.
     * @throws IKMathUtils.MathIllegalArgumentException if the axis norm is zero
     */
    public IKBasis(IKVector3 axis, float angle) {

        float norm = axis.mag();
        if (norm == 0) {
            try {
                throw new Exception("Zero Norm for Rotation defining vector");
            } catch (Exception e) {
                e.printStackTrace(System.out);
            }
        }

        float halfAngle = -0.5f * angle;
        float coeff = IKMathUtils.sin(halfAngle) / norm;

        w = IKMathUtils.cos(halfAngle);
        x = coeff * axis.x;
        y = coeff * axis.y;
        z = coeff * axis.z;
    }

    /**
     * Build one of the rotations that transform one vector into another one.
     * <p>
     * Except for a possible scale factor, if the instance were
     * applied to the vector u it will produce the vector v. There is an
     * infinite number of such rotations, this constructor choose the
     * one with the smallest associated angle (i.e. the one whose axis
     * is orthogonal to the (u, v) plane). If u and v are colinear, an
     * arbitrary rotation axis is chosen.
     * </p>
     *
     * @param u origin vector
     * @param v desired image of u by the rotation
     * @throws IKMathUtils.MathArithmeticException if the norm of one of the vectors
     *                                             is zero
     */
    public IKBasis(IKVector3 u, IKVector3 v) {

        float normProduct = u.mag() * v.mag();
        if (normProduct == 0) {
            // throw new
            // MathArithmeticException(LocalizedFormats.ZERO_NORM_FOR_ROTATION_DEFINING_VECTOR);
            this.w = 1f;
            this.x = 0f;
            this.y = 0f;
            this.z = 0f;
            return;
        }

        float dot = u.dot(v);

        if (dot < ((2.0e-15 - 1.0f) * normProduct)) {
            // special case u = -v: we select a PI angle rotation around
            // an arbitrary vector orthogonal to u
            IKVector3 w = (IKVector3) u.getOrthogonal();
            this.w = 0.0f;
            x = -w.x;
            y = -w.y;
            z = -w.z;
        } else {
            // general case: (u, v) defines a plane, we select
            // the shortest possible rotation: axis orthogonal to this plane
            w = IKMathUtils.sqrt(0.5f * (1.0f + dot / normProduct));
            float coeff = 1.0f / (2.0f * w * normProduct);
            IKVector3 q = (IKVector3) v.crossCopy(u);
            x = coeff * q.x;
            y = coeff * q.y;
            z = coeff * q.z;
        }

        if (Float.isNaN(w) || Float.isNaN(x) || Float.isNaN(y) || Float.isNaN(z)
                || !(Float.isFinite(w) && Float.isFinite(x) && Float.isFinite(y) && Float.isFinite(z))) {
            System.out.println("errror");
        }
    }

    public static IKBasis multiply(final IKBasis q1, final IKBasis q2) {
        // Components of the first quaternion.
        final float q1a = q1.getW();
        final float q1b = q1.getX();
        final float q1c = q1.getY();
        final float q1f = q1.getZ();

        // Components of the second quaternion.
        final float q2a = q2.getW();
        final float q2b = q2.getX();
        final float q2c = q2.getY();
        final float q2f = q2.getZ();

        // Components of the product.
        final float w = q1a * q2a - q1b * q2b - q1c * q2c - q1f * q2f;
        final float x = q1a * q2b + q1b * q2a + q1c * q2f - q1f * q2c;
        final float y = q1a * q2c - q1b * q2f + q1c * q2a + q1f * q2b;
        final float z = q1a * q2f + q1b * q2c - q1c * q2b + q1f * q2a;

        return new IKBasis(x, y, z, w);
    }

    public void clampToAngle(float angle) {
        float cosHalfAngle = IKMathUtils.cos(0.5f * angle);
        clampToQuadranceAngle(cosHalfAngle);
    }

    public void clampToQuadranceAngle(float cosHalfAngle) {
        float newCoeff = 1f - (cosHalfAngle * cosHalfAngle);
        float currentCoeff = x * x + y * y + z * z;
        if (newCoeff > currentCoeff)
            return;
        else {
            w = w < 0 ? -cosHalfAngle : cosHalfAngle;
            float compositeCoeff = IKMathUtils.sqrt(newCoeff / currentCoeff);
            x *= compositeCoeff;
            y *= compositeCoeff;
            z *= compositeCoeff;
        }
    }

    /**
     * @return a copy of this MRotation
     */
    public IKBasis copy() {
        return new IKBasis(getX(), getY(), getZ(), getW());
    }

    /**
     * sets the values of the given rotation equal to the inverse of this rotation
     *
     * @param storeIN
     */
    public void revert(IKBasis storeIn) {
        storeIn.set(-w, x, y, z, true);
    }

    /**
     * Get the scalar coordinate of the quaternion.
     *
     * @return scalar coordinate of the quaternion
     */
    public float getW() {
        return w;
    }

    /**
     * Get the first coordinate of the vectorial part of the quaternion.
     *
     * @return first coordinate of the vectorial part of the quaternion
     */
    public float getX() {
        return x;
    }

    /**
     * Get the second coordinate of the vectorial part of the quaternion.
     *
     * @return second coordinate of the vectorial part of the quaternion
     */
    public float getY() {
        return y;
    }

    /**
     * Get the third coordinate of the vectorial part of the quaternion.
     *
     * @return third coordinate of the vectorial part of the quaternion
     */
    public float getZ() {
        return z;
    }

    /**
     * Get the normalized axis of the rotation.
     *
     * @return normalized axis of the rotation
     * @see #Rotation(T, float)
     */
    public IKVector3 getAxis() {
        float squaredSine = x * x + y * y + z * z;
        if (squaredSine == 0) {
            return new IKVector3(1, 0, 0);
        } else if (w < 0) {
            float inverse = 1 / IKMathUtils.sqrt(squaredSine);
            return new IKVector3(x * inverse, y * inverse, z * inverse);
        }
        float inverse = -1 / IKMathUtils.sqrt(squaredSine);
        return new IKVector3(x * inverse, y * inverse, z * inverse);
    }

    /**
     * Get the normalized axis of the rotation.
     *
     * @return normalized axis of the rotation
     * @see #Rotation(T, float)
     */
    public void setToAxis(IKVector3 v) {
        float squaredSine = x * x + y * y + z * z;
        if (squaredSine == 0) {
            v.set(1, 0, 0);
            return;
        } else if (w < 0) {
            float inverse = 1 / IKMathUtils.sqrt(squaredSine);
            v.set(x * inverse, y * inverse, z * inverse);
            return;
        }
        float inverse = -1 / IKMathUtils.sqrt(squaredSine);
        v.set(x * inverse, y * inverse, z * inverse);
    }

    /**
     * Get the angle of the rotation.
     *
     * @return angle of the rotation (between 0 and &pi;)
     * @see #Rotation(T, float)
     */
    public float getAngle() {
        if ((w < -0.1) || (w > 0.1)) {
            return 2 * IKMathUtils.asin(IKMathUtils.sqrt(x * x + y * y + z * z));
        } else if (w < 0) {
            return 2 * IKMathUtils.acos(-w);
        }
        return 2 * IKMathUtils.acos(w);
    }

    /**
     * modify this rotation to have the specified angle,
     * without changing the axis.
     *
     * @param angle
     */
    public void setAngle(float newAngle) {
        float squaredSine = x * x + y * y + z * z;
        if (squaredSine != 0) {
            float halfAngle = -0.5f * newAngle;
            float cosHalfAngle = IKMathUtils.cos(halfAngle);

            float inverseCoeff = IKMathUtils.sqrt(((1f - (cosHalfAngle * cosHalfAngle)) / squaredSine));
            inverseCoeff = newAngle < 0 ? -inverseCoeff : inverseCoeff;

            w = w < 0 ? -cosHalfAngle : cosHalfAngle;
            x = inverseCoeff * x;
            y = inverseCoeff * y;
            z = inverseCoeff * z;
        }
    }

    /**
     * Multiplies the instance by a scalar.
     *
     * @param alpha Scalar factor.
     * @return a scaled quaternion.
     */
    public IKBasis multiply(final float alpha) {
        return new IKBasis(alpha * x, alpha * y, alpha * z, alpha * w
        );
    }

    /**
     * Apply the rotation to a vector stored in an array.
     *
     * @param in  an array with three items which stores vector to rotate
     * @param out an array with three items to put result to (it can be the same
     *            array as in)
     */
    public void applyTo(final float[] in, final float[] out) {

        final float x = in[0];
        final float y = in[1];
        final float z = in[2];

        final float s = this.x * x + this.y * y + this.z * z;

        out[0] = 2 * (w * (x * w - (this.y * z - this.z * y)) + s * this.x) - x;
        out[1] = 2 * (w * (y * w - (this.z * x - this.x * z)) + s * this.y) - y;
        out[2] = 2 * (w * (z * w - (this.x * y - this.y * x)) + s * this.z) - z;

    }

    public IKBasis setToConjugate() {
        x = -x;
        y = -y;
        z = -z;
        return this;
    }

    public void set(float q0, float q1, float q2, float q3,
                    boolean needsNormalization) {

        this.w = q0;
        this.x = q1;
        this.y = q2;
        this.z = q3;

        if (needsNormalization)
            setToNormalized();
    }

    public void setToNormalized() {
        // normalization preprocessing
        float inv = 1.0f / IKMathUtils.sqrt(w * w + x * x + y * y + z * z);
        w *= inv;
        x *= inv;
        y *= inv;
        z *= inv;
    }

    public void set(IKVector3 u, IKVector3 v) {

        float normProduct = u.mag() * v.mag();
        if (normProduct == 0) {
            try {
                throw new Exception("Zero Norm for Rotation defining vector");
            } catch (Exception e) {
                e.printStackTrace(System.out);
            }
        }

        float dot = u.dot(v);

        if (dot < ((2.0e-15 - 1.0f) * normProduct)) {
            // special case u = -v: we select a PI angle rotation around
            // an arbitrary vector orthogonal to u
            IKVector3 w = (IKVector3) u.getOrthogonal();
            this.w = 0.0f;
            x = -w.x;
            y = -w.y;
            z = -w.z;
        } else {
            // general case: (u, v) defines a plane, we select
            // the shortest possible rotation: axis orthogonal to this plane
            w = IKMathUtils.sqrt(0.5f * (1.0f + dot / normProduct));
            float coeff = 1.0f / (2.0f * w * normProduct);
            IKVector3 q = (IKVector3) v.crossCopy(u);
            x = coeff * q.x;
            y = coeff * q.y;
            z = coeff * q.z;
        }

    }

    public void set(IKVector3 axis, float angle) {

        float norm = axis.mag();
        if (norm == 0) {
            try {
                throw new Exception("Zero Norm for Rotation defining vector");
            } catch (Exception e) {
                e.printStackTrace(System.out);
            }
        }

        float halfAngle = -0.5f * angle;
        float coeff = IKMathUtils.sin(halfAngle) / norm;

        w = IKMathUtils.cos(halfAngle);
        x = coeff * axis.x;
        y = coeff * axis.y;
        z = coeff * axis.z;
    }

    public String toString() {
        String result = "axis: " + getAxis().toVec3f().toString();
        result += "\n angle : " + IKMathUtils.toDegrees(getAngle()) + " degrees ";
        result += "\n angle : " + getAngle() + " radians ";
        return result;
    }
}
