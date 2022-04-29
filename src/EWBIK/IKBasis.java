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
     * Build the rotation that transforms a pair of vector into another pair.
     * <p>
     * Except for possible scale factors, if the instance were applied to
     * the pair (u<sub>1</sub>, u<sub>2</sub>) it will produce the pair
     * (v<sub>1</sub>, v<sub>2</sub>).
     * </p>
     * <p>
     * If the angular separation between u<sub>1</sub> and u<sub>2</sub> is
     * not the same as the angular separation between v<sub>1</sub> and
     * v<sub>2</sub>, then a corrected v'<sub>2</sub> will be used rather than
     * v<sub>2</sub>, the corrected vector will be in the (v<sub>1</sub>,
     * v<sub>2</sub>) plane.
     * </p>
     *
     * @param u1 first vector of the origin pair
     * @param u2 second vector of the origin pair
     * @param v1 desired image of u1 by the rotation
     * @param v2 desired image of u2 by the rotation
     * @throws IKMathUtils.MathArithmeticException if the norm of one of the vectors
     *                                             is zero,
     *                                             or if one of the pair is
     *                                             degenerated (i.e.
     *                                             the vectors of the pair are
     *                                             colinear)
     */
    public IKBasis(IKVector3 u1, IKVector3 u2, IKVector3 v1, IKVector3 v2) {

        // norms computation
        float u1u1 = u1.dot(u1);
        float u2u2 = u2.dot(u2);
        float v1v1 = v1.dot(v1);
        float v2v2 = v2.dot(v2);
        if ((u1u1 == 0) || (u2u2 == 0) || (v1v1 == 0) || (v2v2 == 0)) {
            throw new IllegalArgumentException("zero norm for rotation defining vector");
        }

        float u1x = u1.x;
        float u1y = u1.y;
        float u1z = u1.z;

        float u2x = u2.x;
        float u2y = u2.y;
        float u2z = u2.z;

        // normalize v1 in order to have (v1'|v1') = (u1|u1)
        float coeff = (float) IKMathUtils.sqrt(u1u1 / v1v1);
        float v1x = coeff * v1.x;
        float v1y = coeff * v1.y;
        float v1z = coeff * v1.z;
        IKVector3 va1 = new IKVector3(v1x, v1y, v1z);

        // adjust v2 in order to have (u1|u2) = (v1|v2) and (v2'|v2') = (u2|u2)
        float u1u2 = u1.dot(u2);
        float va1v2 = va1.dot(v2);
        float coeffU = u1u2 / u1u1;
        float coeffV = va1v2 / u1u1;
        float beta = (float) IKMathUtils.sqrt((u2u2 - u1u2 * coeffU) / (v2v2 - va1v2 * coeffV));
        float alpha = coeffU - beta * coeffV;
        float v2x = alpha * v1x + beta * v2.x;
        float v2y = alpha * v1y + beta * v2.y;
        float v2z = alpha * v1z + beta * v2.z;
        IKVector3 va2 = (IKVector3) v2.copy();
        va2.set(v2x, v2y, v2z);

        // preliminary computation (we use explicit formulation instead
        // of relying on the Vector3 class in order to avoid building lots
        // of temporary objects)
        IKVector3 uRef = u1;
        IKVector3 vRef = (IKVector3) va1;
        float dx1 = v1x - u1.x;
        float dy1 = v1y - u1.y;
        float dz1 = v1z - u1.z;
        float dx2 = v2x - u2.x;
        float dy2 = v2y - u2.y;
        float dz2 = v2z - u2.z;
        IKVector3 k = new IKVector3(dy1 * dz2 - dz1 * dy2,
                dz1 * dx2 - dx1 * dz2,
                dx1 * dy2 - dy1 * dx2);
        float c = k.x * (u1y * u2z - u1z * u2y) +
                k.y * (u1z * u2x - u1x * u2z) +
                k.z * (u1x * u2y - u1y * u2x);

        if (IKMathUtils.abs(c) <= IKMathUtils.DOUBLE_ROUNDING_ERROR) {
            // the (q1, q2, q3) vector is in the (u1, u2) plane
            // we try other vectors
            IKVector3 u3 = (IKVector3) u1.crossCopy(u2);
            IKVector3 v3 = va1.crossCopy(va2);
            float u3x = u3.x;
            float u3y = u3.y;
            float u3z = u3.z;
            float v3x = v3.x;
            float v3y = v3.y;
            float v3z = v3.z;

            float dx3 = v3x - u3x;
            float dy3 = v3y - u3y;
            float dz3 = v3z - u3z;
            k = new IKVector3(dy1 * dz3 - dz1 * dy3,
                    dz1 * dx3 - dx1 * dz3,
                    dx1 * dy3 - dy1 * dx3);
            c = k.x * (u1y * u3z - u1z * u3y) +
                    k.y * (u1z * u3x - u1x * u3z) +
                    k.z * (u1x * u3y - u1y * u3x);

            if (IKMathUtils.abs(c) <= IKMathUtils.DOUBLE_ROUNDING_ERROR) {
                // the (q1, q2, q3) vector is aligned with u1:
                // we try (u2, u3) and (v2, v3)
                k = new IKVector3(dy2 * dz3 - dz2 * dy3,
                        dz2 * dx3 - dx2 * dz3,
                        dx2 * dy3 - dy2 * dx3);
                c = k.x * (u2y * u3z - u2z * u3y) +
                        k.y * (u2z * u3x - u2x * u3z) +
                        k.z * (u2x * u3y - u2y * u3x);

                if (IKMathUtils.abs(c) <= IKMathUtils.DOUBLE_ROUNDING_ERROR) {
                    // the (q1, q2, q3) vector is aligned with everything
                    // this is really the identity rotation
                    w = 1.0f;
                    x = 0.0f;
                    y = 0.0f;
                    z = 0.0f;
                    return;
                }

                // we will have to use u2 and v2 to compute the scalar part
                uRef = u2;
                vRef = va2;

            }

        }

        // compute the vectorial part
        c = (float) IKMathUtils.sqrt(c);
        float inv = 1.0f / (c + c);
        x = inv * k.x;
        y = inv * k.y;
        z = inv * k.z;

        // compute the scalar part
        k = new IKVector3(uRef.y * z - uRef.z * y,
                uRef.z * x - uRef.x * z,
                uRef.x * y - uRef.y * x);
        c = k.dot(k);
        w = vRef.dot(k) / (c + c);

        /*
         * // build orthonormalized base from u1, u2
         * // this fails when vectors are null or colinear, which is forbidden to define
         * a rotation
         * final Vector3 u3 = u1.crossCopy(u2).normalize();
         * u2 = u3.crossCopy(u1).normalize();
         * u1 = u1.normalize();
         *
         * // build an orthonormalized base from v1, v2
         * // this fails when vectors are null or colinear, which is forbidden to define
         * a rotation
         * final Vector3 v3 = v1.crossCopy(v2).normalize();
         * v2 = v3.crossCopy(v1).normalize();
         * v1 = v1.normalize();
         *
         * // buid a matrix transforming the first base into the second one
         * final float[][] m = new float[][] {
         * {
         * MathArrays.linearCombination(u1.x, v1.x, u2.x, v2.x, u3.x, v3.x),
         * MathArrays.linearCombination(u1.y, v1.x, u2.y, v2.x, u3.y, v3.x),
         * MathArrays.linearCombination(u1.z, v1.x, u2.z, v2.x, u3.z, v3.x)
         * },
         * {
         * MathArrays.linearCombination(u1.x, v1.y, u2.x, v2.y, u3.x, v3.y),
         * MathArrays.linearCombination(u1.y, v1.y, u2.y, v2.y, u3.y, v3.y),
         * MathArrays.linearCombination(u1.z, v1.y, u2.z, v2.y, u3.z, v3.y)
         * },
         * {
         * MathArrays.linearCombination(u1.x, v1.z, u2.x, v2.z, u3.x, v3.z),
         * MathArrays.linearCombination(u1.y, v1.z, u2.y, v2.z, u3.y, v3.z),
         * MathArrays.linearCombination(u1.z, v1.z, u2.z, v2.z, u3.z, v3.z)
         * }
         * };
         *
         * float[] quat = mat2quat(m);
         * q0 = quat[0];
         * q1 = quat[1];
         * q2 = quat[2];
         * q3 = quat[3];
         */

        if (Float.isNaN(w) || Float.isNaN(x) || Float.isNaN(y) || Float.isNaN(z)
                || !(Float.isFinite(w) && Float.isFinite(x) && Float.isFinite(y) && Float.isFinite(z))) {
            System.out.println("errror");
        }

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

    /**
     * Build a rotation from three Cartesian or Euler elementary rotations.
     * <p>
     * Cartesian rotations are three successive rotations around the
     * canonical axes X, Y and Z, each axis being used once. There are
     * 6 such sets of rotations (XYZ, XZY, YXZ, YZX, ZXY and ZYX). Euler
     * rotations are three successive rotations around the canonical
     * axes X, Y and Z, the first and last rotations being around the
     * same axis. There are 6 such sets of rotations (XYX, XZX, YXY,
     * YZY, ZXZ and ZYZ), the most popular one being ZXZ.
     * </p>
     * <p>
     * Beware that many people routinely use the term Euler angles even
     * for what really are Cartesian angles (this confusion is especially
     * widespread in the aerospace business where Roll, Pitch and Yaw angles
     * are often wrongly tagged as Euler angles).
     * </p>
     *
     * @param order  order of rotations to use
     * @param alpha1 angle of the first elementary rotation
     * @param alpha2 angle of the second elementary rotation
     * @param alpha3 angle of the third elementary rotation
     */
    public IKBasis(IKRotationOrder order,
                   float alpha1, float alpha2, float alpha3) {
        IKBasis r1 = new IKBasis(order.getA1(), alpha1);
        IKBasis r2 = new IKBasis(order.getA2(), alpha2);
        IKBasis r3 = new IKBasis(order.getA3(), alpha3);
        IKBasis composed = r1.applyTo(r2.applyTo(r3));
        w = composed.w;
        x = composed.x;
        y = composed.y;
        z = composed.z;

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
     * Get the Cartesian or Euler angles corresponding to the instance.
     * <p>
     * The equations show that each rotation can be defined by two
     * different values of the Cartesian or Euler angles set. For example
     * if Cartesian angles are used, the rotation defined by the angles
     * a<sub>1</sub>, a<sub>2</sub> and a<sub>3</sub> is the same as
     * the rotation defined by the angles &pi; + a<sub>1</sub>, &pi;
     * - a<sub>2</sub> and &pi; + a<sub>3</sub>. This method implements
     * the following arbitrary choices:
     * </p>
     * <ul>
     * <li>for Cartesian angles, the chosen set is the one for which the
     * second angle is between -&pi;/2 and &pi;/2 (i.e its cosine is
     * positive),</li>
     * <li>for Euler angles, the chosen set is the one for which the
     * second angle is between 0 and &pi; (i.e its sine is positive).</li>
     * </ul>
     * <p>
     * Cartesian and Euler angle have a very disappointing drawback: all
     * of them have singularities. This means that if the instance is
     * too close to the singularities corresponding to the given
     * rotation order, it will be impossible to retrieve the angles. For
     * Cartesian angles, this is often called gimbal lock. There is
     * <em>nothing</em> to do to prevent this, it is an intrinsic problem
     * with Cartesian and Euler representation (but not a problem with the
     * rotation itself, which is perfectly well defined). For Cartesian
     * angles, singularities occur when the second angle is close to
     * -&pi;/2 or +&pi;/2, for Euler angle singularities occur when the
     * second angle is close to 0 or &pi;, this implies that the identity
     * rotation is always singular for Euler angles!
     * </p>
     *
     * @param order rotation order to use
     * @return an array of three angles, in the order specified by the set
     * @throws IKMathUtils.CardanEulerSingularityException if the rotation is
     *                                                     singular with respect to
     *                                                     the angles
     *                                                     set specified
     */
    public float[] getAngles(IKRotationOrder order) {

        if (order == IKRotationOrder.XYZ) {

            // r (T .plusK) coordinates are :
            // sin (theta), -cos (theta) sin (phi), cos (theta) cos (phi)
            // (-r) (T .plusI) coordinates are :
            // cos (psi) cos (theta), -sin (psi) cos (theta), sin (theta)
            // and we can choose to have theta in the interval [-PI/2 ; +PI/2]
            IKVector3 v1 = applyTo(IKRotationOrder.Z);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.X);
            if ((v2.z < -0.9999999999) || (v2.z > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(-(v1.y), v1.z),
                    IKMathUtils.asin(v2.z),
                    IKMathUtils.atan2(-(v2.y), v2.x)
            };

        } else if (order == IKRotationOrder.XZY) {

            // r (T .plusJ) coordinates are :
            // -sin (psi), cos (psi) cos (phi), cos (psi) sin (phi)
            // (-r) (T .plusI) coordinates are :
            // cos (theta) cos (psi), -sin (psi), sin (theta) cos (psi)
            // and we can choose to have psi in the interval [-PI/2 ; +PI/2]
            IKVector3 v1 = applyTo(IKRotationOrder.X);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.Y);
            if ((v2.y < -0.9999999999) || (v2.y > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(v1.z, v1.y),
                    -IKMathUtils.asin(v2.y),
                    IKMathUtils.atan2(v2.z, v2.x)
            };

        } else if (order == IKRotationOrder.YXZ) {

            // r (T .plusK) coordinates are :
            // cos (phi) sin (theta), -sin (phi), cos (phi) cos (theta)
            // (-r) (T .plusJ) coordinates are :
            // sin (psi) cos (phi), cos (psi) cos (phi), -sin (phi)
            // and we can choose to have phi in the interval [-PI/2 ; +PI/2]
            IKVector3 v1 = applyTo(IKRotationOrder.Z);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.Y);
            if ((v2.z < -0.9999999999) || (v2.z > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(v1.x, v1.z),
                    -IKMathUtils.asin(v2.z),
                    IKMathUtils.atan2(v2.x, v2.y)
            };

        } else if (order == IKRotationOrder.YZX) {

            // r (T .plusI) coordinates are :
            // cos (psi) cos (theta), sin (psi), -cos (psi) sin (theta)
            // (-r) (T .plusJ) coordinates are :
            // sin (psi), cos (phi) cos (psi), -sin (phi) cos (psi)
            // and we can choose to have psi in the interval [-PI/2 ; +PI/2]
            IKVector3 v1 = applyTo(IKRotationOrder.X);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.Y);
            if ((v2.x < -0.9999999999) || (v2.x > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(-(v1.z), v1.x),
                    IKMathUtils.asin(v2.x),
                    IKMathUtils.atan2(-(v2.z), v2.y)
            };

        } else if (order == IKRotationOrder.ZXY) {

            // r (T .plusJ) coordinates are :
            // -cos (phi) sin (psi), cos (phi) cos (psi), sin (phi)
            // (-r) (T .plusK) coordinates are :
            // -sin (theta) cos (phi), sin (phi), cos (theta) cos (phi)
            // and we can choose to have phi in the interval [-PI/2 ; +PI/2]
            IKVector3 v1 = applyTo(IKRotationOrder.Y);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.Z);
            if ((v2.y < -0.9999999999) || (v2.y > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(-(v1.x), v1.y),
                    IKMathUtils.asin(v2.y),
                    IKMathUtils.atan2(-(v2.x), v2.z)
            };

        } else if (order == IKRotationOrder.ZYX) {

            // r (T .plusI) coordinates are :
            // cos (theta) cos (psi), cos (theta) sin (psi), -sin (theta)
            // (-r) (T .plusK) coordinates are :
            // -sin (theta), sin (phi) cos (theta), cos (phi) cos (theta)
            // and we can choose to have theta in the interval [-PI/2 ; +PI/2]
            IKVector3 v1 = applyTo(IKRotationOrder.X);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.Z);
            if ((v2.x < -0.9999999999) || (v2.x > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(v1.y, v1.x),
                    -IKMathUtils.asin(v2.x),
                    IKMathUtils.atan2(v2.y, v2.z)
            };

        } else if (order == IKRotationOrder.XYX) {

            // r (T .plusI) coordinates are :
            // cos (theta), sin (phi1) sin (theta), -cos (phi1) sin (theta)
            // (-r) (T .plusI) coordinates are :
            // cos (theta), sin (theta) sin (phi2), sin (theta) cos (phi2)
            // and we can choose to have theta in the interval [0 ; PI]
            IKVector3 v1 = applyTo(IKRotationOrder.X);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.X);
            if ((v2.x < -0.9999999999) || (v2.x > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(v1.y, -v1.z),
                    IKMathUtils.acos(v2.x),
                    IKMathUtils.atan2(v2.y, v2.z)
            };

        } else if (order == IKRotationOrder.XZX) {

            // r (T .plusI) coordinates are :
            // cos (psi), cos (phi1) sin (psi), sin (phi1) sin (psi)
            // (-r) (T .plusI) coordinates are :
            // cos (psi), -sin (psi) cos (phi2), sin (psi) sin (phi2)
            // and we can choose to have psi in the interval [0 ; PI]
            IKVector3 v1 = applyTo(IKRotationOrder.X);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.X);
            if ((v2.x < -0.9999999999) || (v2.x > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(v1.z, v1.y),
                    IKMathUtils.acos(v2.x),
                    IKMathUtils.atan2(v2.z, -v2.y)
            };

        } else if (order == IKRotationOrder.YXY) {

            // r (T .plusJ) coordinates are :
            // sin (theta1) sin (phi), cos (phi), cos (theta1) sin (phi)
            // (-r) (T .plusJ) coordinates are :
            // sin (phi) sin (theta2), cos (phi), -sin (phi) cos (theta2)
            // and we can choose to have phi in the interval [0 ; PI]
            IKVector3 v1 = applyTo(IKRotationOrder.Y);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.Y);
            if ((v2.y < -0.9999999999) || (v2.y > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(v1.x, v1.z),
                    IKMathUtils.acos(v2.y),
                    IKMathUtils.atan2(v2.x, -v2.z)
            };

        } else if (order == IKRotationOrder.YZY) {

            // r (T .plusJ) coordinates are :
            // -cos (theta1) sin (psi), cos (psi), sin (theta1) sin (psi)
            // (-r) (T .plusJ) coordinates are :
            // sin (psi) cos (theta2), cos (psi), sin (psi) sin (theta2)
            // and we can choose to have psi in the interval [0 ; PI]
            IKVector3 v1 = applyTo(IKRotationOrder.Y);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.Y);
            if ((v2.y < -0.9999999999) || (v2.y > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(v1.z, -v1.x),
                    IKMathUtils.acos(v2.y),
                    IKMathUtils.atan2(v2.z, v2.x)
            };

        } else if (order == IKRotationOrder.ZXZ) {

            // r (T .plusK) coordinates are :
            // sin (psi1) sin (phi), -cos (psi1) sin (phi), cos (phi)
            // (-r) (T .plusK) coordinates are :
            // sin (phi) sin (psi2), sin (phi) cos (psi2), cos (phi)
            // and we can choose to have phi in the interval [0 ; PI]
            IKVector3 v1 = applyTo(IKRotationOrder.Z);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.Z);
            if ((v2.z < -0.9999999999) || (v2.z > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(v1.x, -v1.y),
                    IKMathUtils.acos(v2.z),
                    IKMathUtils.atan2(v2.x, v2.y)
            };

        } else { // last possibility is ZYZ

            // r (T .plusK) coordinates are :
            // cos (psi1) sin (theta), sin (psi1) sin (theta), cos (theta)
            // (-r) (T .plusK) coordinates are :
            // -sin (theta) cos (psi2), sin (theta) sin (psi2), cos (theta)
            // and we can choose to have theta in the interval [0 ; PI]
            IKVector3 v1 = applyTo(IKRotationOrder.Z);
            IKVector3 v2 = applyInverseTo(IKRotationOrder.Z);
            if ((v2.z < -0.9999999999) || (v2.z > 0.9999999999)) {
                try {
                    throw new IKMathUtils.CardanEulerSingularityException(true);
                } catch (IKMathUtils.CardanEulerSingularityException e) {
                    e.printStackTrace(System.out);
                }
            }
            return new float[]{
                    IKMathUtils.atan2(v1.y, v1.x),
                    IKMathUtils.acos(v2.z),
                    IKMathUtils.atan2(v2.y, -v2.x)
            };

        }

    }

    /**
     * Apply the rotation to a vector.
     *
     * @param u vector to apply the rotation to
     * @return a new vector which is the image of u by the rotation
     */
    public IKVector3 applyTo(IKVector3 u) {

        float x = u.x;
        float y = u.y;
        float z = u.z;

        float s = this.x * x + this.y * y + this.z * z;
        IKVector3 result = (IKVector3) u.copy();
        result.set(2 * (w * (x * w - (this.y * z - this.z * y)) + s * this.x) - x,
                2 * (w * (y * w - (this.z * x - this.x * z)) + s * this.y) - y,
                2 * (w * (z * w - (this.x * y - this.y * x)) + s * this.z) - z);
        return result;
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

    /**
     * Apply the inverse of the rotation to a vector.
     *
     * @param u vector to apply the inverse of the rotation to
     * @return a new vector which such that u is its image by the rotation
     */
    public IKVector3 applyInverseTo(IKVector3 u) {

        float x = u.x;
        float y = u.y;
        float z = u.z;

        float s = this.x * x + this.y * y + this.z * z;
        float m0 = -w;

        IKVector3 result = (IKVector3) u.copy();
        result.set(2 * (m0 * (x * m0 - (this.y * z - this.z * y)) + s * this.x) - x,
                2 * (m0 * (y * m0 - (this.z * x - this.x * z)) + s * this.y) - y,
                2 * (m0 * (z * m0 - (this.x * y - this.y * x)) + s * this.z) - z);
        return result;

    }

    /**
     * Apply the instance to another rotation.
     * Applying the instance to a rotation is computing the composition
     * in an order compliant with the following rule : let u be any
     * vector and v its image by r (i.e. r.applyTo(u) = v), let w be the image
     * of v by the instance (i.e. applyTo(v) = w), then w = comp.applyTo(u),
     * where comp = applyTo(r).
     *
     * @param r rotation to apply the rotation to
     * @return a new rotation which is the composition of r by the instance
     */
    public IKBasis applyTo(IKBasis r) {
        return new IKBasis(r.x * w + r.w * x + (r.y * z - r.z * y), r.y * w + r.w * y + (r.z * x - r.x * z), r.z * w + r.w * z + (r.x * y - r.y * x), r.w * w - (r.x * x + r.y * y + r.z * z),
                false);
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
