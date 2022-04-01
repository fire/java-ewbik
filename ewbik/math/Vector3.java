package ewbik.math;

import data.agnosticsavior.CanLoad;
import ewbik.asj.data.JSONArray;
import ewbik.asj.data.JSONObject;

public class Vector3 implements CanLoad {

    public final static int X = 0, Y = 1, Z = 2;
    private static final long serialVersionUID = 3840054589595372522L;
    /**
     * the x-component of this vector
     **/
    public float x;
    /**
     * the y-component of this vector
     **/
    public float y;
    /**
     * the z-component of this vector
     **/
    public float z;

    /**
     * Constructs a vector at (0,0,0)
     */
    public Vector3() {
    }

    /**
     * Creates a vector with the given components
     *
     * @param x The x-component
     * @param y The y-component
     * @param z The z-component
     */
    public Vector3(float x, float y, float z) {
        this.set(x, y, z);
    }

    /**
     * Creates a vector from the given vector
     *
     * @param vector The vector
     */
    public Vector3(ewbik.math.Vector3 vector) {
        this.set(vector);
    }

    /**
     * Creates a vector from the given array. The array must have at least 3
     * elements.
     *
     * @param values The array
     */
    public Vector3(final float[] values) {
        this.set(values[0], values[1], values[2]);
    }

    /**
     * @return The euclidean length
     */
    public static float mag(final float x, final float y, final float z) {
        return (float) MathUtils.sqrt(x * x + y * y + z * z);
    }

    public Vector3(JSONObject j) {
        JSONArray components = j.getJSONArray("vec");
        this.x = components.getFloat(0);
        this.y = components.getFloat(1);
        this.z = components.getFloat(2);
    }

    public Vector3(JSONArray j) {
        this.x = j.getFloat(0);
        this.y = j.getFloat(1);
        this.z = j.getFloat(2);
    }

    /**
         * @return a copy of this vector
         */
    public ewbik.math.Vector3 copy() {
        return (ewbik.math.Vector3) new ewbik.math.Vector3(this);
    }

    /**
         * @return a copy of this Vector cast to a single precision analog.
         */
    public ewbik.math.Vector3 toVec3f() {
        return new ewbik.math.Vector3((float) x, (float) y, (float) z);
    }

    public JSONArray toJSONArray() {
        JSONArray vec = new JSONArray();
        vec.append(this.x);
        vec.append(this.y);
        vec.append(this.z);
        return vec;
    }
    /**
     * @return The squared euclidean length
     */
    public static float magSq(final float x, final float y, final float z) {
        return x * x + y * y + z * z;
    }

    /**
     * @return The euclidean distance between the two specified vectors
     */
    public static float dst(final float x1, final float y1, final float z1, final float x2, final float y2,
                            final float z2) {
        final float a = x2 - x1;
        final float b = y2 - y1;
        final float c = z2 - z1;
        return (float) MathUtils.sqrt(a * a + b * b + c * c);
    }

    /**
     * @return the squared distance between the given points
     */
    public static float dst2(final float x1, final float y1, final float z1, final float x2, final float y2,
                             final float z2) {
        final float a = x2 - x1;
        final float b = y2 - y1;
        final float c = z2 - z1;
        return a * a + b * b + c * c;
    }

    /**
     * @return The dot product between the two vectors
     */
    public static float dot(float x1, float y1, float z1, float x2, float y2, float z2) {
        return x1 * x2 + y1 * y2 + z1 * z2;
    }

    public static float dot(ewbik.math.Vector3 u, ewbik.math.Vector3 v) {
        return u.dot(v);
    }

    public static Vector3 add(Vector3 v1, Vector3 v2) {
        return add(v1, v2, null);
    }

    public static Vector3 add(Vector3 v1, Vector3 v2, Vector3 target) {
        if (target == null) {
            target = (Vector3) v1.copy();
            v1.set(
                    v1.x + v2.x,
                    v1.y + v2.y,
                    v1.z + v2.z);
        } else {
            target.set(v1.x + v2.x,
                    v1.y + v2.y,
                    v1.z + v2.z);
        }
        return target;
    }

    /**
     * Subtract one vector from another and store in another vector
     *
     * @param target Vector3 in which to store the result
     */
    static public Vector3 sub(Vector3 v1, Vector3 v2) {
        return sub(v1, v2, (Vector3) null);
    }

    static public Vector3 mult(Vector3 v, float n) {
        return mult(v, n, null);
    }

    static public Vector3 mult(Vector3 v, float n, Vector3 target) {
        if (target == null) {
            target = (Vector3) v.copy();
        }
        target.set(v.x * n, v.y * n, v.z * n);
        return target;
    }

    static public Vector3 div(Vector3 v, float n) {
        return div(v, n, null);
    }

    static public Vector3 div(Vector3 v, float n, Vector3 target) {
        if (target == null) {
            target = (Vector3) v.copy();
        }
        target.set(v.x / n, v.y / n, v.z / n);

        return target;
    }

    /**
     * Subtract v3 from v1 and store in target
     *
     * @param target Vector3 in which to store the result
     * @return
     */
    static public Vector3 sub(Vector3 v1, Vector3 v2, Vector3 target) {
        if (target == null) {
            target = (Vector3) v1.copy();
        }
        target.set(v1.x - v2.x,
                v1.y - v2.y,
                v1.z - v2.z);

        return target;
    }

    /**
     * @param v1     any variable of type Vector3
     * @param v2     any variable of type Vector3
     * @param target Vector3 to store the result
     */
    public static Vector3 cross(Vector3 v1, Vector3 v2, Vector3 target) {
        float crossX = v1.y * v2.z - v2.y * v1.z;
        float crossY = v1.z * v2.x - v2.z * v1.x;
        float crossZ = v1.x * v2.y - v2.x * v1.y;

        if (target == null) {
            target = (Vector3) v1.copy();
        }
        target.set(crossX, crossY, crossZ);

        return target;
    }

    /**
     * Linear interpolate between two vectors (returns a new Vector3 object)
     *
     * @param v1 the vector to start from
     * @param v2 the vector to lerp to
     */
    public static Vector3 lerp(Vector3 v1, Vector3 v2, float amt) {
        Vector3 v = (Vector3) v1.copy();
        v.lerp(v2, amt);
        return v;
    }

    /**
     * ( begin auto-generated from SGVec_3f_angleBetween.xml )
     * <p>
     * Calculates and returns the angle (in radians) between two vectors.
     * <p>
     * ( end auto-generated )
     *
     * @param v1 the x, y, and z components of a Vector3
     * @param v2 the x, y, and z components of a Vector3
     * @webref Vector3:method
     * @usage web_application
     * @brief Calculate and return the angle between two vectors
     */
    static public float angleBetween(ewbik.math.Vector3 v1, ewbik.math.Vector3 v2) {

        // We get NaN if we pass in a zero vector which can cause problems
        // Zero seems like a reasonable angle between a (0,0,0) vector and something
        // else
        if (v1.x == 0 && v1.y == 0 && v1.z == 0)
            return 0.0f;
        if (v2.x == 0 && v2.y == 0 && v2.z == 0)
            return 0.0f;

        float dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
        float v1mag = MathUtils.sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
        float v2mag = MathUtils.sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
        // This should be a number between -1 and 1, since it's "normalized"
        float amt = dot / (v1mag * v2mag);
        // But if it's not due to rounding error, then we need to fix it
        // http://code.google.com/p/processing/issues/detail?id=340
        // Otherwise if outside the range, acos() will return NaN
        // http://www.cppreference.com/wiki/c/math/acos
        if (amt <= -1) {
            return MathUtils.PI;
        } else if (amt >= 1) {
            return 0;
        }
        return (float) MathUtils.acos(amt);
    }

    /**
         * Sets this vector from the given vector
         *
         * @param x
         * @param y
         * @param z
         * @return This vector for chaining
         */
    public ewbik.math.Vector3 set(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return (ewbik.math.Vector3) this;
    }

    public ewbik.math.Vector3 set(final ewbik.math.Vector3 vector) {
        return this.set(vector.getX(), vector.getY(), vector.getZ());
    }

    /**
         * Sets this vector from the given vector
         *
         * @param v The vector
         */
    public ewbik.math.Vector3 set(final float[] values) {
        return this.set(values[0], values[1], values[2]);
    }

    /**
     * Sets the components from the given spherical coordinate
     *
     * @param azimuthalAngle The angle between x-axis in radians [0, 2pi]
     * @param polarAngle     The angle between z-axis in radians [0, pi]
     * @return This vector for chaining
     */
    public ewbik.math.Vector3 setFromSpherical(float azimuthalAngle, float polarAngle) {
        float cosPolar = MathUtils.cos(polarAngle);
        float sinPolar = MathUtils.sin(polarAngle);

        float cosAzim = MathUtils.cos(azimuthalAngle);
        float sinAzim = MathUtils.sin(azimuthalAngle);

        return this.set(cosAzim * sinPolar, sinAzim * sinPolar, cosPolar);
    }

    /**
         * Adds the given vector to this vector
         *
         * @param v The vector
         * @return This vector for chaining
         */
    public Vector3 add(Vector3 vector) {
        return this.add(vector.getX(), vector.getY(), vector.getZ());
    }

    public ewbik.math.Vector3 add(float[] v) {
        return this.add(v[0], v[1], v[2]);
    }

    /**
     * Adds the given vector to this component
     *
     * @param x The x-component of the other vector
     * @param y The y-component of the other vector
     * @param z The z-component of the other vector
     * @return This vector for chaining.
     */
    public ewbik.math.Vector3 add(float x, float y, float z) {
        this.x += x;
        this.y += y;
        this.z += z;
        return (ewbik.math.Vector3) this;
    }

    /**
     * Adds the given value to all three components of the vector.
     *
     * @param values The value
     * @return This vector for chaining
     */
    public ewbik.math.Vector3 add(float values) {
        return set(this.x + values, this.y + values, this.z + values);
    }

    /**
         * Subtracts the given vector from this vector.
         *
         * @param v The vector
         * @return This vector for chaining
         */
    public Vector3 sub(Vector3 a_vec) {
        return sub(a_vec.getX(), a_vec.getY(), a_vec.getZ());
    }

    /**
     * Subtracts the other vector from this vector.
     *
     * @param x The x-component of the other vector
     * @param y The y-component of the other vector
     * @param z The z-component of the other vector
     * @return This vector for chaining
     */
    public ewbik.math.Vector3 sub(float x, float y, float z) {
        return this.set(this.x - x, this.y - y, this.z - z);
    }

    /**
     * Subtracts the given value from all components of this vector
     *
     * @param value The value
     * @return This vector for chaining
     */
    public ewbik.math.Vector3 sub(float value) {
        return this.set(this.x - value, this.y - value, this.z - value);
    }

    /**
         * Scales this vector by another vector
         *
         * @return This vector for chaining
         */
    public Vector3 mult(Vector3 other) {
        return this.set(x * other.getX(), y * other.getY(), z * other.getZ());
    }

    /**
     * Scales this vector by the given values
     *
     * @param vx X value
     * @param vy Y value
     * @param vz Z value
     * @return This vector for chaining
     */
    public ewbik.math.Vector3 mult(float vx, float vy, float vz) {
        return this.set(this.x * vx, this.y * vy, this.z * vz);
    }

    /**
         * ( begin auto-generated from SGVec_3f_div.xml )
         * <p>
         * Divides a vector by a scalar or divides one vector by another.
         * <p>
         * ( end auto-generated )
         *
         * @param n the number by which to divide the vector
         * @webref Vecf:method
         * @usage web_application
         * @brief Divide a vector by a scalar
         */
    public ewbik.math.Vector3 div(float n) {
        x /= n;
        y /= n;
        z /= n;
        return (ewbik.math.Vector3) this;
    }

    /**
         * First scale a supplied vector, then add it to this vector.
         *
         * @param v      addition vector
         * @param scalar for scaling the addition vector
         */
    public Vector3 mulAdd(Vector3 vec, float scalar) {
        this.x += vec.getX() * scalar;
        this.y += vec.getY() * scalar;
        this.z += vec.getZ() * scalar;
        return (ewbik.math.Vector3) this;
    }

    /**
         * First scale a supplied vector, then add it to this vector.
         *
         * @param v      addition vector
         * @param mulVec vector by whose values the addition vector will be scaled
         */
    public Vector3 mulAdd(Vector3 vec, Vector3 mulVec) {
        this.x += vec.getX() * mulVec.getX();
        this.y += vec.getY() * mulVec.getY();
        this.z += vec.getZ() * mulVec.getZ();
        return (ewbik.math.Vector3) this;
    }

    /**
         * @return The euclidean length
         */
    public float mag() {
        return (float) MathUtils.sqrt(x * x + y * y + z * z);
    }

    /**
         * This method is faster than {@link Vector3#mag()} because it avoids calculating a square root. It is useful for comparisons,
         * but not for getting exact lengths, as the return value is the square of the actual length.
         *
         * @return The squared euclidean length
         */
    public float magSq() {
        return x * x + y * y + z * z;
    }

    /**
     * @param vector The other vector
     * @return Whether this and the other vector are equal
     */
    public boolean idt(final ewbik.math.Vector3 vector) {
        return x == vector.x && y == vector.y && z == vector.z;
    }

    /**
         * @param v The other vector
         * @return the distance between this and the other vector
         */
    public float dist(final ewbik.math.Vector3 vector) {
        final float a = vector.x - x;
        final float b = vector.y - y;
        final float c = vector.z - z;
        return (float) MathUtils.sqrt(a * a + b * b + c * c);
    }

    /**
     * @return the distance between this point and the given point
     */
    public float dst(float x, float y, float z) {
        final float a = x - this.x;
        final float b = y - this.y;
        final float c = z - this.z;
        return (float) MathUtils.sqrt(a * a + b * b + c * c);
    }

    /**
         * This method is faster than {@link Vector3#dist(Vector3)} because it avoids calculating a square root. It is useful for
         * comparisons, but not for getting accurate distances, as the return value is the square of the actual distance.
         *
         * @param v The other vector
         * @return the squared distance between this and the other vector
         */
    public float distSq(ewbik.math.Vector3 point) {
        final float a = point.x - x;
        final float b = point.y - y;
        final float c = point.z - z;
        return a * a + b * b + c * c;
    }

    /**
     * Returns the squared distance between this point and the given point
     *
     * @param x The x-component of the other point
     * @param y The y-component of the other point
     * @param z The z-component of the other point
     * @return The squared distance
     */
    public float dst2(float x, float y, float z) {
        final float a = x - this.x;
        final float b = y - this.y;
        final float c = z - this.z;
        return a * a + b * b + c * c;
    }

    /**
         * Normalizes this vector. Does nothing if it is zero.
         *
         * @return This vector for chaining
         */
    public ewbik.math.Vector3 normalize() {
        final float len2 = this.mag();
        if (len2 == 0f || len2 == 1f)
            return (ewbik.math.Vector3) this;
        return this.mult(1f / (float) len2);
    }

    /**
         * @param v The other vector
         * @return The dot product between this and the other vector
         */
    public float dot(Vector3 vector) {
        return x * vector.getX() + y * vector.getY() + z * vector.getZ();
    }

    /**
     * Returns the dot product between this and the given vector.
     *
     * @param x The x-component of the other vector
     * @param y The y-component of the other vector
     * @param z The z-component of the other vector
     * @return The dot product
     */
    public float dot(float x, float y, float z) {
        return this.x * x + this.y * y + this.z * z;
    }

    /**
     * Sets this vector to the cross product between it and the other vector.
     *
     * @param vector Vec3fhe other vector
     * @return This vector for chaining
     */
    public Vector3 crs(final Vector3 vector) {
        return this.set(y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x);
    }

    /**
     * Sets this vector to the cross product between it and the other vector.
     *
     * @param x The x-component of the other vector
     * @param y The y-component of the other vector
     * @param z The z-component of the other vector
     * @return This vector for chaining
     */
    public ewbik.math.Vector3 crs(float x, float y, float z) {
        return this.set(this.y * z - this.z * y, this.z * x - this.x * z, this.x * y - this.y * x);
    }

    /**
     * Left-multiplies the vector by the given 4x3 column major matrix. The matrix
     * should be composed by a 3x3 matrix representing
     * rotation and scale plus a 1x3 matrix representing the translation.
     *
     * @param matrix The matrix
     * @return This vector for chaining
     */
    public ewbik.math.Vector3 mul4x3(float[] matrix) {
        return set(x * matrix[0] + y * matrix[3] + z * matrix[6] + matrix[9],
                x * matrix[1] + y * matrix[4] + z * matrix[7]
                        + matrix[10],
                x * matrix[2] + y * matrix[5] + z * matrix[8] + matrix[11]);
    }

    /**
     * Takes two vectors representing a plane
     * and returns the projection of this vector onto
     * that plane.
     *
     * @param p1 vector representing first edge of plane
     * @param p2 vector representing second edge of plane
     * @return
     */
    public ewbik.math.Vector3 getPlaneProjectionOf(ewbik.math.Vector3 p1, ewbik.math.Vector3 p2) {
        return this.getPlaneProjectionOf((ewbik.math.Vector3) p1.crossCopy(p2));
    }

    /**
     * Takes a vector representing the normal of a plane, and returns
     * the value of this vector projected onto that plane
     *
     * @param norm
     * @return
     */
    public ewbik.math.Vector3 getPlaneProjectionOf(ewbik.math.Vector3 rawNorm) {
        ewbik.math.Vector3 norm = (ewbik.math.Vector3) rawNorm.copy().normalize();
        ewbik.math.Vector3 normProj = (ewbik.math.Vector3) norm.multCopy(this.dot(norm));
        normProj.mult(-1);

        return (ewbik.math.Vector3) normProj.addCopy(this);
    }

    /**
         * @return Whether this vector is a unit length vector
         */
    public boolean isUnit() {
        return isUnit(0.000000001f);
    }

    /**
         * @return Whether this vector is a unit length vector within the given margin.
         */
    public boolean isUnit(final float margin) {
        return MathUtils.abs(magSq() - 1f) < margin;
    }

    /**
         * @return Whether this vector is a zero vector
         */
    public boolean isZero() {
        return x == 0 && y == 0 && z == 0;
    }

    /**
         * @return Whether the length of this vector is smaller than the given margin
         */
    public boolean isZero(final float margin) {
        return magSq() < margin;
    }

    /**
         * @return true if this vector is in line with the other vector (either in the same or the opposite direction)
         */
    public boolean isOnLine(ewbik.math.Vector3 other, float epsilon) {
        return magSq(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x) <= epsilon;
    }

    /**
         * @return true if this vector is in line with the other vector (either in the same or the opposite direction)
         */
    public boolean isOnLine(ewbik.math.Vector3 other) {
        return magSq(y * other.z - z * other.y, z * other.x - x * other.z,
                x * other.y - y * other.x) <= MathUtils.DOUBLE_ROUNDING_ERROR;
    }

    /**
         * @return true if this vector is collinear with the other vector ({@link #isOnLine(Vector3, float)} &&
         * {@link #hasSameDirection(Vector3)}).
         */
    public boolean isCollinear(ewbik.math.Vector3 other, float epsilon) {
        return isOnLine(other, epsilon) && hasSameDirection(other);
    }

    /**
         * @return true if this vector is collinear with the other vector ({@link #isOnLine(Vector3)} &&
         * {@link #hasSameDirection(Vector3)}).
         */
    public boolean isCollinear(ewbik.math.Vector3 other) {
        return isOnLine(other) && hasSameDirection(other);
    }

    /**
         * @return true if this vector is opposite collinear with the other vector ({@link #isOnLine(Vector3, float)} &&
         * {@link #hasOppositeDirection(Vector3)}).
         */
    public boolean isCollinearOpposite(ewbik.math.Vector3 other, float epsilon) {
        return isOnLine(other, epsilon) && hasOppositeDirection(other);
    }

    /**
         * @return true if this vector is opposite collinear with the other vector ({@link #isOnLine(Vector3)} &&
         * {@link #hasOppositeDirection(Vector3)}).
         */
    public boolean isCollinearOpposite(ewbik.math.Vector3 other) {
        return isOnLine(other) && hasOppositeDirection(other);
    }

    /**
         * @return Whether this vector is perpendicular with the other vector. True if the dot product is 0.
         */
    public boolean isPerpendicular(ewbik.math.Vector3 vector) {
        return MathUtils.isZero(dot(vector));
    }

    /**
         * @param epsilon a positive small number close to zero
         * @return Whether this vector is perpendicular with the other vector. True if the dot product is 0.
         */
    public boolean isPerpendicular(ewbik.math.Vector3 vector, float epsilon) {
        return MathUtils.isZero(dot(vector), epsilon);
    }

    /**
         * @return Whether this vector has similar direction compared to the other vector. True if the normalized dot product is > 0.
         */
    public boolean hasSameDirection(ewbik.math.Vector3 vector) {
        return dot(vector) > 0;
    }

    /**
         * @return Whether this vector has opposite direction compared to the other vector. True if the normalized dot product is < 0.
         */
    public boolean hasOppositeDirection(ewbik.math.Vector3 vector) {
        return dot(vector) < 0;
    }

    /**
         * Linearly interpolates between this vector and the target vector by alpha which is in the range [0,1]. The result is stored
         * in this vector.
         *
         * @param target The target vector
         * @param alpha  The interpolation coefficient
         * @return This vector for chaining.
         */
    public ewbik.math.Vector3 lerp(final ewbik.math.Vector3 target, float alpha) {
        x += alpha * (target.x - x);
        y += alpha * (target.y - y);
        z += alpha * (target.z - z);
        return (ewbik.math.Vector3) this;
    }

    /**
     * Spherically interpolates between this vector and the target vector by alpha
     * which is in the range [0,1]. The result is
     * stored in this vector.
     *
     * @param target The target vector
     * @param alpha  The interpolation coefficient
     * @return This vector for chaining.
     */
    public ewbik.math.Vector3 slerp(final ewbik.math.Vector3 target, float alpha) {
        final float dot = dot(target);
        // If the inputs are too close for comfort, simply linearly interpolate.
        if (dot > 0.9995 || dot < -0.9995)
            return lerp(target, alpha);

        // theta0 = angle between input vectors
        final float theta0 = (float) MathUtils.acos(dot);
        // theta = angle between this vector and result
        final float theta = theta0 * alpha;

        final float st = (float) MathUtils.sin(theta);
        final float tx = target.x - x * dot;
        final float ty = target.y - y * dot;
        final float tz = target.z - z * dot;
        final float l2 = tx * tx + ty * ty + tz * tz;
        final float dl = st * ((l2 < 0.0001f) ? 1f : 1f / (float) MathUtils.sqrt(l2));

        return mult((float) MathUtils.cos(theta)).add(tx * dl, ty * dl, tz * dl).normalize();
    }

    /**
     * Converts this {@code Vector3} to a string in the format {@code (x,y,z)}.
     *
     * @return a string representation of this object.
     */
    @Override
    public String toString() {
        return "(" + (float) x + "," + (float) y + "," + (float) z + ")";
    }

    /**
         * Limits the length of this vector, based on the desired maximum length.
         *
         * @param limit desired maximum length for this vector
         * @return this vector for chaining
         */
    public ewbik.math.Vector3 limit(float limit) {
        return limitSq(limit * limit);
    }

    /**
         * Limits the length of this vector, based on the desired maximum length squared.
         * <p/>
         * This method is slightly faster than limit().
         *
         * @param limit2 squared desired maximum length for this vector
         * @return this vector for chaining
         * @see #magSq()
         */
    public ewbik.math.Vector3 limitSq(float limit2) {
        float len2 = magSq();
        if (len2 > limit2) {
            mult((float) MathUtils.sqrt(limit2 / len2));
        }
        return (ewbik.math.Vector3) this;
    }

    /**
         * Sets the length of this vector. Does nothing is this vector is zero.
         *
         * @param len desired length for this vector
         * @return this vector for chaining
         */
    public ewbik.math.Vector3 setMag(float len) {
        return setMagSq(len * len);
    }

    /**
         * Sets the length of this vector, based on the square of the desired length. Does nothing is this vector is zero.
         * <p/>
         * This method is slightly faster than setLength().
         *
         * @param len2 desired square of the length for this vector
         * @return this vector for chaining
         * @see #magSq()
         */
    public ewbik.math.Vector3 setMagSq(float len2) {
        float oldLen2 = magSq();
        return (oldLen2 == 0 || oldLen2 == len2) ? (ewbik.math.Vector3) this : mult((float) MathUtils.sqrt(len2 / oldLen2));
    }

    /**
         * Clamps this vector's length to given min and max values
         *
         * @param min Min length
         * @param max Max length
         * @return This vector for chaining
         */
    public ewbik.math.Vector3 clamp(float min, float max) {
        final float len2 = magSq();
        if (len2 == 0f)
            return (ewbik.math.Vector3) this;
        float max2 = max * max;
        if (len2 > max2)
            return mult((float) MathUtils.sqrt(max2 / len2));
        float min2 = min * min;
        if (len2 < min2)
            return mult((float) MathUtils.sqrt(min2 / len2));
        return (ewbik.math.Vector3) this;
    }

    /**
         * Compares this vector with the other vector, using the supplied epsilon for fuzzy equality testing.
         *
         * @param other
         * @param epsilon
         * @return whether the vectors have fuzzy equality.
         */
    public boolean epsilonEquals(Vector3 other, float epsilon) {
        if (other == null)
            return false;
        if (MathUtils.abs(other.getX() - x) > epsilon)
            return false;
        if (MathUtils.abs(other.getY() - y) > epsilon)
            return false;
        if (MathUtils.abs(other.getZ() - z) > epsilon)
            return false;
        return true;
    }

    /**
     * Compares this vector with the other vector, using the supplied epsilon for
     * fuzzy equality testing.
     *
     * @return whether the vectors are the same.
     */
    public boolean epsilonEquals(float x, float y, float z, float epsilon) {
        if (MathUtils.abs(x - this.x) > epsilon)
            return false;
        if (MathUtils.abs(y - this.y) > epsilon)
            return false;
        if (MathUtils.abs(z - this.z) > epsilon)
            return false;
        return true;
    }

    /**
         * Sets the components of this vector to 0
         *
         * @return This vector for chaining
         */
    public ewbik.math.Vector3 setZero() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        return (ewbik.math.Vector3) this;
    }

    /**
         * @return the X component of this vector.
         */
    public float getX() {
        return this.x;
    }

    /**
         * @return the Y component of this vector.
         */
    public float getY() {
        return this.y;
    }

    /**
         * sets this vector's x component to the input value
         *
         * @param x
         */
    public void setX_(float x) {
        this.x = x;
    }

    /**
         * sets this vector's y component to the input value
         *
         * @param y
         */
    public void setY_(float y) {
        this.y = y;

    }

    /**
     * sets this vector's Z componen to the input value
     *
     * @param Z
     */
    public void setZ_(float z) {
        this.z = z;
    }

    public ewbik.math.Vector3 getOrthogonal() {
        ewbik.math.Vector3 result = this.copy();
        result.set(0, 0, 0);
        float threshold = this.mag() * 0.6f;
        if (threshold > 0) {
            if (MathUtils.abs(x) <= threshold) {
                float inverse = 1 / MathUtils.sqrt(y * y + z * z);
                return result.set(0, inverse * z, -inverse * y);
            } else if (MathUtils.abs(y) <= threshold) {
                float inverse = 1 / MathUtils.sqrt(x * x + z * z);
                return result.set(-inverse * z, 0, inverse * x);
            }
            float inverse = 1 / MathUtils.sqrt(x * x + y * y);
            return result.set(inverse * y, -inverse * x, 0);
        }

        return result;
    }

    /**
         * Scales this vector by a scalar
         *
         * @param scalar The scalar
         * @return This vector for chaining
         */
    public ewbik.math.Vector3 mult(float scalar) {
        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;
        return (ewbik.math.Vector3) this;
    }

    /**
     * should cause this Vector to adopt the xyz values
     * of the input vector. This method has a default implementation
     * that simply calls setX_(), setY_(), setZ() but you should override it if your
     * vector implementation requires more than that.
     *
     * @param v
     */
    public void adoptValuesOf(ewbik.math.Vector3 v) {
        setX_(v.getX());
        setY_(v.getY());
        setZ_(v.getZ());
    }

    /**
         * @return the Y component of this vector.
         */
    public float getZ() {
        return this.z;
    }

    @Override
    public CanLoad populateSelfFromJSON(JSONObject j) {
        JSONArray components = j.getJSONArray("vec");
        this.x = components.getFloat(0);
        this.y = components.getFloat(1);
        this.z = components.getFloat(2);
        return this;
    }

    @Override
    public JSONObject toJSONObject() {
        JSONObject j = new JSONObject();
        JSONArray components = new JSONArray();
        components.append(this.x);
        components.append(this.y);
        components.append(this.z);
        j.setJSONArray("vec", components);
        return j;
    }

    /**
     * makes a copy of this vector and scales it by the given value, then returns that copy.
     *
     * @param v The vector
     * @return the resulting vector
     */
    public Vector3 multCopy(float s) {
        Vector3 cv = this.copy();
        return cv.mult(s);
    }

    /**
     * make a copy of this vector and add the given vector to it, then return that copy.
     *
     * @param v The vector
     * @return The resulting vector
     */
    public Vector3 addCopy(Vector3 v) {
        Vector3 cv = this.copy();
        return cv.add(v);
    }

    /**
     * makes a copy of this vector and sets it to the cross product between it and the input vector,
     * then returns the copy
     *
     * @param vector The other vector
     * @return The copied vector for chaining
     */
    public Vector3 crossCopy(Vector3 vector) {
        Vector3 c = this.copy();
        return c.crs(vector);
    }

    /**
     * make a copy of this vector and subtract the given vector from it, then return that copy.
     *
     * @param v The vector
     * @return the resulting vector
     */
    public Vector3 subCopy(Vector3 v) {
        Vector3 cv = this.copy();
        return cv.sub(v);
    }

    /**
     * makes a copy of this vector and multiplies it componentWise by the given vector, then returns that copy.
     *
     * @param v The vector
     * @return the resulting vector
     */
    public Vector3 multCopy(Vector3 v) {
        Vector3 cv = this.copy();
        return cv.mult(v);
    }
}
