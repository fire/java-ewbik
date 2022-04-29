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
     *  @param x                 first coordinate of the vectorial part of the
     *                           quaternion
     * @param y                 second coordinate of the vectorial part of the
     *                           quaternion
     * @param z                 third coordinate of the vectorial part of the
 *                           quaternion
     * @param w                 scalar part of the quaternion
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
     * creates an identity rotation
     */
    public IKBasis() {
        this(0.0f, 0.0f, 0.0f, 1.0f, false);
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
     * Build a rotation from a 3X3 given as a 1f array with 9 elements,
     * or 4x4 matrix given as a 1f array with 16 elements.
     * This constructor will detect the appropriate case based on the length
     * of the input array.
     * Input array should be in column major order, so, for a 3x3 matrix, the
     * indices correspond as follows: <br/>
     * 0, 3, 6 <br/>
     * 1, 4, 7 <br/>
     * 2, 5, 8 <br/>
     * <p>
     * And for a 4x4 matrix the indices are:
     * <br/>
     * 0, 4, 8, 12 <br/>
     * 1, 5, 9, 13 <br/>
     * 2, 6, 10, 14 <br/>
     * 3, 7, 11, 15 <br/>
     *
     *
     * <p>
     * Rotation matrices are orthogonal matrices, i.e. unit matrices
     * (which are matrices for which m.m<sup>T</sup> = I) with real
     * coefficients. The module of the determinant of unit matrices is
     * 1, among the orthogonal 3X3 matrices, only the ones having a
     * positive determinant (+1) are rotation matrices.
     * </p>
     * <p>
     * When a rotation is defined by a matrix with truncated values
     * (typically when it is extracted from a technical sheet where only
     * four to five significant digits are available), the matrix is not
     * orthogonal anymore. This constructor handles this case
     * transparently by using a copy of the given matrix and applying a
     * correction to the copy in order to perfect its orthogonality. If
     * the Frobenius norm of the correction needed is above the given
     * threshold, then the matrix is considered to be too far from a
     * true rotation matrix and an exception is thrown.
     * <p>
     *
     * @param m         rotation matrix
     * @param is4x4     set to true if passing in a 4x4 matrix.
     * @param threshold convergence threshold for the iterative
     *                  orthogonality correction (convergence is reached when the
     *                  difference between two steps of the Frobenius norm of the
     *                  correction is below this threshold)
     * @throws IKMathUtils.NotARotationMatrixException if the matrix is not a 3X3
     *                                               matrix, or if it cannot be
     *                                               transformed
     *                                               into an orthogonal matrix
     *                                               with the given threshold, or if
     *                                               the
     *                                               determinant of the resulting
     *                                               orthogonal matrix is negative
     */
    public IKBasis(float[] m, float threshold) {
        // dimension check
        if ((m.length != 9 || m.length != 16)) {
            this.w = 1.0f;
            this.x = 0.0f;
            this.y = 0.0f;
            this.z = 0.0f;
            return;
        }

        float[][] im = new float[3][3];
        if (m.length == 9) {
            im[0][0] = m[0];
            im[0][1] = m[0];
            im[0][2] = m[0];
            im[0][0] = m[0];
            im[0][1] = m[0];
            im[0][2] = m[0];
            im[0][0] = m[0];
            im[0][1] = m[0];
            im[0][2] = m[0];
        }

        // compute a "close" orthogonal matrix
        float[][] ort = orthogonalizeMatrix(im, threshold);

        // check the sign of the determinant
        float det = ort[0][0] * (ort[1][1] * ort[2][2] - ort[2][1] * ort[1][2]) -
                ort[1][0] * (ort[0][1] * ort[2][2] - ort[2][1] * ort[0][2]) +
                ort[2][0] * (ort[0][1] * ort[1][2] - ort[1][1] * ort[0][2]);
        if (det < 0.0f) {
            this.w = 1.0f;
            this.x = 0.0f;
            this.y = 0.0f;
            this.z = 0.0f;
            return;
        }

        float[] quat = mat2quat(ort);
        w = quat[0];
        x = quat[1];
        y = quat[2];
        z = quat[3];
    }

    /**
     * Build a rotation from a 3X3 matrix.
     * <p>
     * Rotation matrices are orthogonal matrices, i.e. unit matrices
     * (which are matrices for which m.m<sup>T</sup> = I) with real
     * coefficients. The module of the determinant of unit matrices is
     * 1, among the orthogonal 3X3 matrices, only the ones having a
     * positive determinant (+1) are rotation matrices.
     * </p>
     * <p>
     * When a rotation is defined by a matrix with truncated values
     * (typically when it is extracted from a technical sheet where only
     * four to five significant digits are available), the matrix is not
     * orthogonal anymore. This constructor handles this case
     * transparently by using a copy of the given matrix and applying a
     * correction to the copy in order to perfect its orthogonality. If
     * the Frobenius norm of the correction needed is above the given
     * threshold, then the matrix is considered to be too far from a
     * true rotation matrix and an exception is thrown.
     * <p>
     *
     * @param m         rotation matrix
     * @param threshold convergence threshold for the iterative
     *                  orthogonality correction (convergence is reached when the
     *                  difference between two steps of the Frobenius norm of the
     *                  correction is below this threshold)
     * @throws IKMathUtils.NotARotationMatrixException if the matrix is not a 3X3
     *                                               matrix, or if it cannot be
     *                                               transformed
     *                                               into an orthogonal matrix
     *                                               with the given threshold, or if
     *                                               the
     *                                               determinant of the resulting
     *                                               orthogonal matrix is negative
     */
    public IKBasis(float[][] m, float threshold) {

        // dimension check
        if ((m.length != 3) || (m[0].length != 3) ||
                (m[1].length != 3) || (m[2].length != 3)) {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
            return;
        }

        // compute a "close" orthogonal matrix
        float[][] ort = orthogonalizeMatrix(m, threshold);

        // check the sign of the determinant
        float det = ort[0][0] * (ort[1][1] * ort[2][2] - ort[2][1] * ort[1][2]) -
                ort[1][0] * (ort[0][1] * ort[2][2] - ort[2][1] * ort[0][2]) +
                ort[2][0] * (ort[0][1] * ort[1][2] - ort[1][1] * ort[0][2]);
        if (det < 0.0f) {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
            return;
        }

        float[] quat = mat2quat(ort);
        w = quat[0];
        x = quat[1];
        y = quat[2];
        z = quat[3];
        if (Float.isNaN(w) || Float.isNaN(x) || Float.isNaN(y) || Float.isNaN(z)
                || !(Float.isFinite(w) && Float.isFinite(x) && Float.isFinite(y) && Float.isFinite(z))) {
            System.out.println("errror");
        }
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
     *                                           is zero,
     *                                           or if one of the pair is
     *                                           degenerated (i.e.
     *                                           the vectors of the pair are
     *                                           colinear)
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
     *                                           is zero
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

    /**
     * Convert an orthogonal rotation matrix to a quaternion.
     *
     * @param ort orthogonal rotation matrix
     * @return quaternion corresponding to the matrix
     */
    private static float[] mat2quat(final float[][] ort) {

        final float[] quat = new float[4];

        // There are different ways to compute the quaternions elements
        // from the matrix. They all involve computing one element from
        // the diagonal of the matrix, and computing the three other ones
        // using a formula involving a division by the first element,
        // which unfortunately can be zero. Since the norm of the
        // quaternion is 1, we know at least one element has an absolute
        // value greater or equal to 0.5f, so it is always possible to
        // select the right formula and avoid division by zero and even
        // numerical inaccuracy. Checking the elements in turn and using
        // the first one greater than 0.45 is safe (this leads to a simple
        // test since qi = 0.45 implies 4 qi^2 - 1 = -0.19)
        float s = ort[0][0] + ort[1][1] + ort[2][2];
        if (s > -0.19) {
            // compute q0 and deduce q1, q2 and q3
            quat[0] = 0.5f * IKMathUtils.sqrt(s + 1.0f);
            float inv = 0.25f / quat[0];
            quat[1] = inv * (ort[1][2] - ort[2][1]);
            quat[2] = inv * (ort[2][0] - ort[0][2]);
            quat[3] = inv * (ort[0][1] - ort[1][0]);
        } else {
            s = ort[0][0] - ort[1][1] - ort[2][2];
            if (s > -0.19) {
                // compute q1 and deduce q0, q2 and q3
                quat[1] = 0.5f * IKMathUtils.sqrt(s + 1.0f);
                float inv = 0.25f / quat[1];
                quat[0] = inv * (ort[1][2] - ort[2][1]);
                quat[2] = inv * (ort[0][1] + ort[1][0]);
                quat[3] = inv * (ort[0][2] + ort[2][0]);
            } else {
                s = ort[1][1] - ort[0][0] - ort[2][2];
                if (s > -0.19) {
                    // compute q2 and deduce q0, q1 and q3
                    quat[2] = 0.5f * IKMathUtils.sqrt(s + 1.0f);
                    float inv = 0.25f / quat[2];
                    quat[0] = inv * (ort[2][0] - ort[0][2]);
                    quat[1] = inv * (ort[0][1] + ort[1][0]);
                    quat[3] = inv * (ort[2][1] + ort[1][2]);
                } else {
                    // compute q3 and deduce q0, q1 and q2
                    s = ort[2][2] - ort[0][0] - ort[1][1];
                    quat[3] = 0.5f * IKMathUtils.sqrt(s + 1.0f);
                    float inv = 0.25f / quat[3];
                    quat[0] = inv * (ort[0][1] - ort[1][0]);
                    quat[1] = inv * (ort[0][2] + ort[2][0]);
                    quat[2] = inv * (ort[2][1] + ort[1][2]);
                }
            }
        }

        return quat;

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

    /**
     * Computes the dot-product of two quaternions.
     *
     * @param q1 Quaternionf.
     * @param q2 Quaternionf.
     * @return the dot product of {@code q1} and {@code q2}.
     */
    public static float dotProduct(final IKBasis q1,
            final IKBasis q2) {
        return q1.getW() * q2.getW() +
                q1.getX() * q2.getX() +
                q1.getY() * q2.getY() +
                q1.getZ() * q2.getZ();
    }

    /**
     * Compute the <i>distance</i> between two rotations.
     * <p>
     * The <i>distance</i> is intended here as a way to check if two
     * rotations are almost similar (i.e. they transform vectors the same way)
     * or very different. It is mathematically defined as the angle of
     * the rotation r that prepended to one of the rotations gives the other
     * one:
     * </p>
     *
     * <pre>
     *        r<sub>1</sub>(r) = r<sub>2</sub>
     * </pre>
     * <p>
     * This distance is an angle between 0 and &pi;. Its value is the smallest
     * possible upper bound of the angle in radians between r<sub>1</sub>(v)
     * and r<sub>2</sub>(v) for all possible vectors v. This upper bound is
     * reached for some v. The distance is equal to 0 if and only if the two
     * rotations are identical.
     * </p>
     * <p>
     * Comparing two rotations should always be done using this value rather
     * than for example comparing the components of the quaternions. It is much
     * more stable, and has a geometric meaning. Also comparing quaternions
     * components is error prone since for example quaternions (0.36, 0.48, -0.48,
     * -0.64)
     * and (-0.36, -0.48, 0.48, 0.64) represent exactly the same rotation despite
     * their components are different (they are exact opposites).
     * </p>
     *
     * @param r1 first rotation
     * @param r2 second rotation
     * @return <i>distance</i> between r1 and r2
     */
    public static float distance(IKBasis r1, IKBasis r2) {
        return r1.applyInverseTo(r2).getAngle();
    }

    /**
     * Modify this rotation to have the specified cos(angle/2) representation,
     * without changing the axis.
     *
     * @param angle
     */
    public void setQuadranceAngle(float cosHalfAngle) {
        float squaredSine = x * x + y * y + z * z;
        if (squaredSine != 0) {
            float inverseCoeff = IKMathUtils.sqrt(((1 - (cosHalfAngle * cosHalfAngle)) / squaredSine));
            // inverseCoeff = cosHalfAngle < 0 ? -inverseCoeff : inverseCoeff;
            w = w < 0 ? -cosHalfAngle : cosHalfAngle;
            x = inverseCoeff * x;
            y = inverseCoeff * y;
            z = inverseCoeff * z;
        }
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
     * Revert a rotation.
     * Build a rotation which reverse the effect of another
     * rotation. This means that if r(u) = v, then r.revert(v) = u. The
     * instance is not changed.
     *
     * @return a new rotation whose effect is the reverse of the effect
     *         of the instance
     */
    public IKBasis revert() {
        return new IKBasis(x, y, z, -w, false);
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
     * modify this rotation to have the specified axis,
     * without changing the angle.
     *
     * @param angle
     * @throws Exception
     */
    public void setAxis(IKVector3 newAxis) throws Exception {

        float angle = this.getAngle();
        float norm = newAxis.mag();
        if (norm == 0) {
            try {
                throw new Exception("Zero Norm for Rotation Axis");
            } catch (IKMathUtils.MathIllegalArgumentException e) {
                // TODO Auto-generated catch block
                e.printStackTrace(System.out);
            }
        }

        float halfAngle = -0.5f * angle;
        float coeff = IKMathUtils.sin(halfAngle) / norm;

        w = IKMathUtils.cos(halfAngle);
        x = coeff * newAxis.x;
        y = coeff * newAxis.y;
        z = coeff * newAxis.z;

        if (Float.isNaN(w) || Float.isNaN(x) || Float.isNaN(y) || Float.isNaN(z)
                || !(Float.isFinite(w) && Float.isFinite(x) && Float.isFinite(y) && Float.isFinite(z))) {
            System.out.println("errror");
        }
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

    public IKBasis getInverse() {
        final float squareNorm = w * w + x * x + y * y + z * z;
        if (squareNorm < IKMathUtils.SAFE_MIN_DOUBLE) {
            try {
                throw new Exception("Zero Norm");
            } catch (Exception e) {
                e.printStackTrace(System.out);
            }
        }

        return new IKBasis(-x / squareNorm, -y / squareNorm, -z / squareNorm, w / squareNorm
        );
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
     *                                                   singular with respect to
     *                                                   the angles
     *                                                   set specified
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
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
            return new float[] {
                    IKMathUtils.atan2(v1.y, v1.x),
                    IKMathUtils.acos(v2.z),
                    IKMathUtils.atan2(v2.y, -v2.x)
            };

        }

    }

    /**
     * Get an array representing the 3X3 matrix corresponding to this rotation
     * instance
     * Indices are in column major order. In other words
     * <br/>
     * 0, 3, 6 <br/>
     * 1, 4, 7 <br/>
     * 2, 5, 8 <br/>
     *
     * @return the matrix corresponding to the instance
     */
    public float[] getMatrix3Val() {

        // create the matrix
        float[] values = new float[9];
        setToMatrix3Val(values);
        return values;
    }

    /**
     * set input to the 3X3 matrix corresponding to the instance
     * Indices are in column major order. In other words
     * <br/>
     * 0, 3, 6 <br/>
     * 1, 4, 7 <br/>
     * 2, 5, 8 <br/>
     *
     * @return the matrix corresponding to the instance
     */
    public void setToMatrix3Val(float[] storeIn) {

        // products
        float q0q0 = w * w;
        float q0q1 = w * x;
        float q0q2 = w * y;
        float q0q3 = w * z;
        float q1q1 = x * x;
        float q1q2 = x * y;
        float q1q3 = x * z;
        float q2q2 = y * y;
        float q2q3 = y * z;
        float q3q3 = z * z;

        // create the matrix
        storeIn[0] = 2.0f * (q0q0 + q1q1) - 1.0f;
        storeIn[1] = 2.0f * (q1q2 - q0q3);
        storeIn[2] = 2.0f * (q1q3 + q0q2);

        storeIn[3] = 2.0f * (q1q2 + q0q3);
        storeIn[4] = 2.0f * (q0q0 + q2q2) - 1.0f;
        storeIn[5] = 2.0f * (q2q3 - q0q1);

        storeIn[6] = 2.0f * (q1q3 - q0q2);
        storeIn[7] = 2.0f * (q2q3 + q0q1);
        storeIn[8] = 2.0f * (q0q0 + q3q3) - 1.0f;

    }

    /**
     * Get an array representing the 4X4 matrix corresponding to this rotation
     * instance.
     * Indices are in column major order. In other words
     * <br/>
     * 0, 4, 8, 12 <br/>
     * 1, 5, 9, 13 <br/>
     * 2, 6, 10, 14 <br/>
     * 3, 7, 11, 15 <br/>
     */
    public float[] toMatrix4Val() {
        float[] result = new float[16];
        return toMatrix4Val(result, false);
    }

    /**
     * Get an array representing the 4X4 matrix corresponding to this rotation
     * instance.
     * Indices are in column major order. In other words
     * <br/>
     * 0, 4, 8, 12 <br/>
     * 1, 5, 9, 13 <br/>
     * 2, 6, 10, 14 <br/>
     * 3, 7, 11, 15 <br/>
     *
     * @param storeIn the array to storevalues in.
     * @param zeroOut if true, will zero out any elements in the matrix not
     *                corresponding to this rotation.
     */
    public float[] toMatrix4Val(float[] storeIn, boolean zeroOut) {
        float q0q0 = w * w;
        float q0q1 = w * x;
        float q0q2 = w * y;
        float q0q3 = w * z;
        float q1q1 = x * x;
        float q1q2 = x * y;
        float q1q3 = x * z;
        float q2q2 = y * y;
        float q2q3 = y * z;
        float q3q3 = z * z;

        // create the matrix
        storeIn[0] = 2.0f * (q0q0 + q1q1) - 1.0f;
        storeIn[1] = 2.0f * (q1q2 - q0q3);
        storeIn[2] = 2.0f * (q1q3 + q0q2);

        storeIn[4] = 2.0f * (q1q2 + q0q3);
        storeIn[5] = 2.0f * (q0q0 + q2q2) - 1.0f;
        storeIn[6] = 2.0f * (q2q3 - q0q1);

        storeIn[8] = 2.0f * (q1q3 - q0q2);
        storeIn[9] = 2.0f * (q2q3 + q0q1);
        storeIn[10] = 2.0f * (q0q0 + q3q3) - 1.0f;
        storeIn[15] = 1.0f;

        if (zeroOut) {
            storeIn[3] = 0.0f;
            storeIn[7] = 0.0f;
            storeIn[11] = 0.0f;
            storeIn[12] = 0.0f;
            storeIn[13] = 0.0f;
            storeIn[14] = 0.0f;

        }

        return storeIn;
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
     * Returns the Hamilton product of the instance by a quaternion.
     *
     * @param q Quaternionf.
     * @return the product of this instance with {@code q}, in that order.
     */
    public IKBasis multiply(final IKBasis q) {
        return multiply(this, q);
    }

    /**
     * Computes the dot-product of the instance by a quaternion.
     *
     * @param q Quaternionf.
     * @return the dot product of this instance and {@code q}.
     */
    public float dotProduct(final IKBasis q) {
        return dotProduct(this, q);
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
     * Apply the inverse of the rotation to a vector stored in an array.
     *
     * @param in  an array with three items which stores vector to rotate
     * @param out an array with three items to put result to (it can be the same
     *            array as in)
     */
    public void applyInverseTo(final float[] in, final float[] out) {

        final float x = in[0];
        final float y = in[1];
        final float z = in[2];

        final float s = this.x * x + this.y * y + this.z * z;
        final float m0 = -w;

        out[0] = 2 * (m0 * (x * m0 - (this.y * z - this.z * y)) + s * this.x) - x;
        out[1] = 2 * (m0 * (y * m0 - (this.z * x - this.x * z)) + s * this.y) - y;
        out[2] = 2 * (m0 * (z * m0 - (this.x * y - this.y * x)) + s * this.z) - z;

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

    /**
     * Apply the inverse of the instance to another rotation.
     * Applying the inverse of the instance to a rotation is computing
     * the composition in an order compliant with the following rule :
     * let u be any vector and v its image by r (i.e. r.applyTo(u) = v),
     * let w be the inverse image of v by the instance
     * (i.e. applyInverseTo(v) = w), then w = comp.applyTo(u), where
     * comp = applyInverseTo(r).
     *
     * @param r rotation to apply the rotation to
     * @return a new rotation which is the composition of r by the inverse
     *         of the instance
     */
    public IKBasis applyInverseTo(IKBasis r) {
        return new IKBasis(-r.x * w + r.w * x + (r.y * z - r.z * y), -r.y * w + r.w * y + (r.z * x - r.x * z), -r.z * w + r.w * z + (r.x * y - r.y * x), -r.w * w - (r.x * x + r.y * y + r.z * z),
                false);
    }

    /**
     * Apply the instance to another rotation. Store the result in the specified
     * rotation
     * Applying the instance to a rotation is computing the composition
     * in an order compliant with the following rule : let u be any
     * vector and v its image by r (i.e. r.applyTo(u) = v), let w be the image
     * of v by the instance (i.e. applyTo(v) = w), then w = comp.applyTo(u),
     * where comp = applyTo(r).
     *
     * @param r      rotation to apply the rotation to
     * @param output the rotation to store the result in
     * @return a new rotation which is the composition of r by the instance
     */
    public void applyTo(IKBasis r, IKBasis output) {
        output.set(r.w * w - (r.x * x + r.y * y + r.z * z),
                r.x * w + r.w * x + (r.y * z - r.z * y),
                r.y * w + r.w * y + (r.z * x - r.x * z),
                r.z * w + r.w * z + (r.x * y - r.y * x),
                false);
    }

    /**
     * Apply the inverse of the instance to another rotation. Store the result in
     * the specified rotation
     * Applying the inverse of the instance to a rotation is computing
     * the composition in an order compliant with the following rule :
     * let u be any vector and v its image by r (i.e. r.applyTo(u) = v),
     * let w be the inverse image of v by the instance
     * (i.e. applyInverseTo(v) = w), then w = comp.applyTo(u), where
     * comp = applyInverseTo(r).
     *
     * @param r      rotation to apply the rotation to
     * @param output the rotation to store the result in
     * @return a new rotation which is the composition of r by the inverse
     *         of the instance
     */
    public void applyInverseTo(IKBasis r, IKBasis output) {
        output.set(-r.w * w - (r.x * x + r.y * y + r.z * z),
                -r.x * w + r.w * x + (r.y * z - r.z * y),
                -r.y * w + r.w * y + (r.z * x - r.x * z),
                -r.z * w + r.w * z + (r.x * y - r.y * x),
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

    /**
     * Computes the norm of the quaternion.
     *
     * @return the norm.
     */
    public float len() {
        return IKMathUtils.sqrt(w * w +
                x * x +
                y * y +
                z * z);
    }

    /**
     * Computes the normalized quaternion (the versor of the instance).
     * The norm of the quaternion must not be zero.
     *
     * @return a normalized quaternion.
     * @throws IKMathUtils.ZeroException if the norm of the quaternion is zero.
     */
    public IKBasis normalize() {
        final float norm = len();

        if (norm < IKMathUtils.SAFE_MIN_DOUBLE) {
            try {
                throw new Exception("Zero Norm");
            } catch (Exception e) {
                e.printStackTrace(System.out);
            }
        }

        return new IKBasis(x / norm, y / norm, z / norm, w / norm
        );
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

    /**
     * Perfect orthogonality on a 3X3 matrix.
     *
     * @param m         initial matrix (not exactly orthogonal)
     * @param threshold convergence threshold for the iterative
     *                  orthogonality correction (convergence is reached when the
     *                  difference between two steps of the Frobenius norm of the
     *                  correction is below this threshold)
     * @return an orthogonal matrix close to m
     */
    private float[][] orthogonalizeMatrix(float[][] m, float threshold) {
        float[] m0 = m[0];
        float[] m1 = m[1];
        float[] m2 = m[2];
        float x00 = m0[0];
        float x01 = m0[1];
        float x02 = m0[2];
        float x10 = m1[0];
        float x11 = m1[1];
        float x12 = m1[2];
        float x20 = m2[0];
        float x21 = m2[1];
        float x22 = m2[2];
        float fn = 0;
        float fn1;

        float[][] o = new float[3][3];
        float[] o0 = o[0];
        float[] o1 = o[1];
        float[] o2 = o[2];

        // iterative correction: Xn+1 = Xn - 0.5f * (Xn.Mt.Xn - M)
        int i = 0;
        while (++i < 11) {

            // Mt.Xn
            float mx00 = m0[0] * x00 + m1[0] * x10 + m2[0] * x20;
            float mx10 = m0[1] * x00 + m1[1] * x10 + m2[1] * x20;
            float mx20 = m0[2] * x00 + m1[2] * x10 + m2[2] * x20;
            float mx01 = m0[0] * x01 + m1[0] * x11 + m2[0] * x21;
            float mx11 = m0[1] * x01 + m1[1] * x11 + m2[1] * x21;
            float mx21 = m0[2] * x01 + m1[2] * x11 + m2[2] * x21;
            float mx02 = m0[0] * x02 + m1[0] * x12 + m2[0] * x22;
            float mx12 = m0[1] * x02 + m1[1] * x12 + m2[1] * x22;
            float mx22 = m0[2] * x02 + m1[2] * x12 + m2[2] * x22;

            // Xn+1
            o0[0] = x00 - 0.5f * (x00 * mx00 + x01 * mx10 + x02 * mx20 - m0[0]);
            o0[1] = x01 - 0.5f * (x00 * mx01 + x01 * mx11 + x02 * mx21 - m0[1]);
            o0[2] = x02 - 0.5f * (x00 * mx02 + x01 * mx12 + x02 * mx22 - m0[2]);
            o1[0] = x10 - 0.5f * (x10 * mx00 + x11 * mx10 + x12 * mx20 - m1[0]);
            o1[1] = x11 - 0.5f * (x10 * mx01 + x11 * mx11 + x12 * mx21 - m1[1]);
            o1[2] = x12 - 0.5f * (x10 * mx02 + x11 * mx12 + x12 * mx22 - m1[2]);
            o2[0] = x20 - 0.5f * (x20 * mx00 + x21 * mx10 + x22 * mx20 - m2[0]);
            o2[1] = x21 - 0.5f * (x20 * mx01 + x21 * mx11 + x22 * mx21 - m2[1]);
            o2[2] = x22 - 0.5f * (x20 * mx02 + x21 * mx12 + x22 * mx22 - m2[2]);

            // correction on each elements
            float corr00 = o0[0] - m0[0];
            float corr01 = o0[1] - m0[1];
            float corr02 = o0[2] - m0[2];
            float corr10 = o1[0] - m1[0];
            float corr11 = o1[1] - m1[1];
            float corr12 = o1[2] - m1[2];
            float corr20 = o2[0] - m2[0];
            float corr21 = o2[1] - m2[1];
            float corr22 = o2[2] - m2[2];

            // Frobenius norm of the correction
            fn1 = corr00 * corr00 + corr01 * corr01 + corr02 * corr02 +
                    corr10 * corr10 + corr11 * corr11 + corr12 * corr12 +
                    corr20 * corr20 + corr21 * corr21 + corr22 * corr22;

            // convergence test
            if (IKMathUtils.abs(fn1 - fn) <= threshold) {
                return o;
            }

            // prepare next iteration
            x00 = o0[0];
            x01 = o0[1];
            x02 = o0[2];
            x10 = o1[0];
            x11 = o1[1];
            x12 = o1[2];
            x20 = o2[0];
            x21 = o2[1];
            x22 = o2[2];
            fn = fn1;
        }
        IKBasis returnMatrix = new IKBasis(0.0f, 0.0f, 0.0f, 1.0f, false);
        return new float[][]{returnMatrix.getMatrix3Val()};
    }

    public boolean equalTo(IKBasis m) {
        return distance(this, m) < IKMathUtils.DOUBLE_ROUNDING_ERROR;
    }

    public String toString() {
        String result = "axis: " + getAxis().toVec3f().toString();
        result += "\n angle : " + IKMathUtils.toDegrees(getAngle()) + " degrees ";
        result += "\n angle : " + getAngle() + " radians ";
        return result;
    }
}
