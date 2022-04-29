package EWBIK;

public class IKVector3 {

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
    public IKVector3() {
    }

    /**
     * Creates a vector with the given components
     *
     * @param x The x-component
     * @param y The y-component
     * @param z The z-component
     */
    public IKVector3(float x, float y, float z) {
        this.set(x, y, z);
    }

    /**
     * Creates a vector from the given vector
     *
     * @param vector The vector
     */
    public IKVector3(IKVector3 vector) {
        this.set(vector);
    }

    /**
     * Creates a vector from the given array. The array must have at least 3
     * elements.
     *
     * @param values The array
     */
    public IKVector3(final float[] values) {
        this.set(values[0], values[1], values[2]);
    }

    /**
     * @return a copy of this vector
     */
    public IKVector3 copy() {
        return new IKVector3(this);
    }

    /**
     * @return a copy of this Vector cast to a single precision analog.
     */
    public IKVector3 toVec3f() {
        return new IKVector3(x, y, z);
    }


    /**
     * @return The dot product between the two vectors
     */
    public static float dot(float x1, float y1, float z1, float x2, float y2, float z2) {
        return x1 * x2 + y1 * y2 + z1 * z2;
    }

    /**
     * Subtract one vector from another and store in another vector
     *
     * @param target Vector3 in which to store the result
     */
    static public IKVector3 sub(IKVector3 v1, IKVector3 v2) {
        return sub(v1, v2, null);
    }

    static public IKVector3 multiply(IKVector3 v, float n) {
        return multiply(v, n, null);
    }

    static public IKVector3 multiply(IKVector3 v, float n, IKVector3 target) {
        if (target == null) {
            target = v.copy();
        }
        target.set(v.x * n, v.y * n, v.z * n);
        return target;
    }

    /**
     * Subtract v3 from v1 and store in target
     *
     * @param target Vector3 in which to store the result
     * @return
     */
    static public IKVector3 sub(IKVector3 v1, IKVector3 v2, IKVector3 target) {
        if (target == null) {
            target = v1.copy();
        }
        target.set(v1.x - v2.x,
                v1.y - v2.y,
                v1.z - v2.z);

        return target;
    }

    /**
     * Sets this vector from the given vector
     *
     * @param x
     * @param y
     * @param z
     * @return This vector for chaining
     */
    public IKVector3 set(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    public IKVector3 set(final IKVector3 vector) {
        return this.set(vector.getX(), vector.getY(), vector.getZ());
    }

    /**
     * Sets this vector from the given vector
     *
     * @param v The vector
     */
    public IKVector3 set(final float[] values) {
        return this.set(values[0], values[1], values[2]);
    }

    /**
     * Adds the given vector to this vector
     *
     * @param v The vector
     * @return This vector for chaining
     */
    public IKVector3 add(IKVector3 vector) {
        return this.add(vector.getX(), vector.getY(), vector.getZ());
    }

    /**
     * Adds the given vector to this component
     *
     * @param x The x-component of the other vector
     * @param y The y-component of the other vector
     * @param z The z-component of the other vector
     * @return This vector for chaining.
     */
    public IKVector3 add(float x, float y, float z) {
        this.x += x;
        this.y += y;
        this.z += z;
        return this;
    }

    /**
     * Subtracts the given vector from this vector.
     *
     * @param v The vector
     * @return This vector for chaining
     */
    public IKVector3 sub(IKVector3 a_vec) {
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
    public IKVector3 sub(float x, float y, float z) {
        return this.set(this.x - x, this.y - y, this.z - z);
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
    public IKVector3 divide(float n) {
        x /= n;
        y /= n;
        z /= n;
        return this;
    }

    /**
     * First scale a supplied vector, then add it to this vector.
     *
     * @param v      addition vector
     * @param scalar for scaling the addition vector
     */
    public IKVector3 mulAdd(IKVector3 vec, float scalar) {
        this.x += vec.getX() * scalar;
        this.y += vec.getY() * scalar;
        this.z += vec.getZ() * scalar;
        return this;
    }

    /**
     * @return The euclidean length
     */
    public float mag() {
        return (float) IKMathUtils.sqrt(x * x + y * y + z * z);
    }

    /**
     * This method is faster than {@link IKVector3#mag()} because it avoids
     * calculating a square root. It is useful for comparisons,
     * but not for getting exact lengths, as the return value is the square of the
     * actual length.
     *
     * @return The squared euclidean length
     */
    public float magSq() {
        return x * x + y * y + z * z;
    }

    /**
     * @param v The other vector
     * @return the distance between this and the other vector
     */
    public float dist(final IKVector3 vector) {
        final float a = vector.x - x;
        final float b = vector.y - y;
        final float c = vector.z - z;
        return (float) IKMathUtils.sqrt(a * a + b * b + c * c);
    }

    /**
     * Normalizes this vector. Does nothing if it is zero.
     *
     * @return This vector for chaining
     */
    public IKVector3 normalize() {
        final float len2 = this.mag();
        if (len2 == 0f || len2 == 1f)
            return this;
        return this.multiply(1f / len2);
    }

    /**
     * @param v The other vector
     * @return The dot product between this and the other vector
     */
    public float dot(IKVector3 vector) {
        return x * vector.getX() + y * vector.getY() + z * vector.getZ();
    }

    /**
     * Sets this vector to the cross product between it and the other vector.
     *
     * @param vector Vec3fhe other vector
     * @return This vector for chaining
     */
    public IKVector3 crs(final IKVector3 vector) {
        return this.set(y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x);
    }

    /**
     * Converts this {@code Vector3} to a string in the format {@code (x,y,z)}.
     *
     * @return a string representation of this object.
     */
    @Override
    public String toString() {
        return "(" + x + "," + y + "," + z + ")";
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

    public IKVector3 getOrthogonal() {
        IKVector3 result = this.copy();
        result.set(0, 0, 0);
        float threshold = this.mag() * 0.6f;
        if (threshold > 0) {
            if (IKMathUtils.abs(x) <= threshold) {
                float inverse = 1 / IKMathUtils.sqrt(y * y + z * z);
                return result.set(0, inverse * z, -inverse * y);
            } else if (IKMathUtils.abs(y) <= threshold) {
                float inverse = 1 / IKMathUtils.sqrt(x * x + z * z);
                return result.set(-inverse * z, 0, inverse * x);
            }
            float inverse = 1 / IKMathUtils.sqrt(x * x + y * y);
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
    public IKVector3 multiply(float scalar) {
        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;
        return this;
    }

    /**
     * @return the Y component of this vector.
     */
    public float getZ() {
        return this.z;
    }

    /**
     * makes a copy of this vector and scales it by the given value, then returns
     * that copy.
     *
     * @param v The vector
     * @return the resulting vector
     */
    public IKVector3 multCopy(float s) {
        IKVector3 cv = this.copy();
        return cv.multiply(s);
    }

    /**
     * make a copy of this vector and add the given vector to it, then return that
     * copy.
     *
     * @param v The vector
     * @return The resulting vector
     */
    public IKVector3 addCopy(IKVector3 v) {
        IKVector3 cv = this.copy();
        return cv.add(v);
    }

    /**
     * makes a copy of this vector and sets it to the cross product between it and
     * the input vector,
     * then returns the copy
     *
     * @param vector The other vector
     * @return The copied vector for chaining
     */
    public IKVector3 crossCopy(IKVector3 vector) {
        IKVector3 c = this.copy();
        return c.crs(vector);
    }

    /**
     * make a copy of this vector and subtract the given vector from it, then return
     * that copy.
     *
     * @param v The vector
     * @return the resulting vector
     */
    public IKVector3 subCopy(IKVector3 v) {
        IKVector3 cv = this.copy();
        return cv.sub(v);
    }

}
