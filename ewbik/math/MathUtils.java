/*******************************************************************************
 * Copyright 2011 See AUTHORS file.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

package ewbik.math;

/**
 * Utility and fast math functions.
 * <p>
 * Thanks to Riven on JavaGaming.org for the basis of sin/cos/floor/ceil.
 *
 * @author Nathan Sweet
 */
public final class MathUtils {
    static public final float FLOAT_ROUNDING_ERROR = 0.000001f; // 32 bits, 23 of which may hold the significand for a
                                                                // precision of 6 digits
    static public final double DOUBLE_ROUNDING_ERROR = 0.000000000000001d; // 64, 52 of which represent the significand
                                                                           // for a precision of 15 digits.
    static public final float PI = (float) Math.PI;
    static public final float PI2 = PI * 2f;
    static public final float HALF_PI = (float) (Math.PI / 2d);
    /**
     * multiply by this to convert from radians to degrees
     */
    static public final float radiansToDegrees = 180f / PI;
    /**
     * multiply by this to convert from degrees to radians
     */
    static public final float degreesToRadians = PI / 180f;
    static private final int BIG_ENOUGH_INT = 16 * 1024;
    static private final double BIG_ENOUGH_FLOOR = BIG_ENOUGH_INT;
    static private final double CEIL = 0.9999999;
    static private final double BIG_ENOUGH_ROUND = BIG_ENOUGH_INT + 0.5f;

    /**
     * Returns the sine in radians from a lookup table.
     */
    static public float sin(float radians) {
        return (float) Math.sin(radians);
    }

    /**
     * Returns the cosine in radians from a lookup table.
     */
    static public float cos(float radians) {
        return (float) Math.cos(radians);
    }

    /**
     * Returns atan2 in radians, faster but less accurate than Math.atan2. Average
     * error of 0.00231 radians (0.1323 degrees),
     * largest error of 0.00488 radians (0.2796 degrees).
     */
    static public float atan2(float y, float x) {
        if (x == 0f) {
            if (y > 0f)
                return HALF_PI;
            if (y == 0f)
                return 0f;
            return -HALF_PI;
        }
        final float atan, z = y / x;
        if (Math.abs(z) < 1f) {
            atan = z / (1f + 0.28f * z * z);
            if (x < 0f)
                return atan + (y < 0f ? -PI : PI);
            return atan;
        }
        atan = PI / 2 - z / (z * z + 0.28f);
        return y < 0f ? atan - PI : atan;
    }

    /**
     * Returns the next power of two. Returns the specified value if the value is
     * already a power of two.
     */
    static public int nextPowerOfTwo(int value) {
        if (value == 0)
            return 1;
        value--;
        value |= value >> 1;
        value |= value >> 2;
        value |= value >> 4;
        value |= value >> 8;
        value |= value >> 16;
        return value + 1;
    }

    static public boolean isPowerOfTwo(int value) {
        return value != 0 && (value & value - 1) == 0;
    }

    static public short clamp(short value, short min, short max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    static public int clamp(int value, int min, int max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    static public long clamp(long value, long min, long max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    static public float clamp(float value, float min, float max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    static public double clamp(double value, double min, double max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    /**
     * Linearly interpolates between fromValue to toValue on progress position.
     */
    static public double lerp(double fromValue, double toValue, double progress) {
        return fromValue + (toValue - fromValue) * progress;
    }

    /**
     * Linearly interpolates between two angles in radians. Takes into account that
     * angles wrap at two pi and always takes the
     * direction with the smallest delta angle.
     *
     * @param fromRadians start angle in radians
     * @param toRadians   target angle in radians
     * @param progress    interpolation value in the range [0, 1]
     * @return the interpolated angle in the range [0, PI2[
     */
    public static double lerpAngle(double fromRadians, double toRadians, double progress) {
        double delta = ((toRadians - fromRadians + PI2 + PI) % PI2) - PI;
        return (fromRadians + delta * progress + PI2) % PI2;
    }

    /**
     * Linearly interpolates between two angles in degrees. Takes into account that
     * angles wrap at 360 degrees and always takes
     * the direction with the smallest delta angle.
     *
     * @param fromDegrees start angle in degrees
     * @param toDegrees   target angle in degrees
     * @param progress    interpolation value in the range [0, 1]
     * @return the interpolated angle in the range [0, 360[
     */
    public static double lerpAngleDeg(double fromDegrees, double toDegrees, double progress) {
        double delta = ((toDegrees - fromDegrees + 360 + 180) % 360) - 180;
        return (fromDegrees + delta * progress + 360) % 360;
    }

    /**
     * Returns the largest integer less than or equal to the specified double. This
     * method will only properly floor doubles from
     * -(2^14) to (double.MAX_VALUE - 2^14).
     */
    static public int floor(double value) {
        return (int) (value + BIG_ENOUGH_FLOOR) - BIG_ENOUGH_INT;
    }

    /**
     * Returns the largest integer less than or equal to the specified double. This
     * method will only properly floor doubles that are
     * positive. Note this method simply casts the double to int.
     */
    static public int floorPositive(double value) {
        return (int) value;
    }

    /**
     * Returns the smallest integer greater than or equal to the specified double.
     * This method will only properly ceil doubles from
     * -(2^14) to (double.MAX_VALUE - 2^14).
     */
    static public int ceil(double value) {
        return BIG_ENOUGH_INT - (int) (BIG_ENOUGH_FLOOR - value);
    }

    /**
     * Returns the smallest integer greater than or equal to the specified double.
     * This method will only properly ceil doubles that
     * are positive.
     */
    static public int ceilPositive(double value) {
        return (int) (value + CEIL);
    }

    /**
     * Returns the closest integer to the specified double. This method will only
     * properly round doubles from -(2^14) to
     * (double.MAX_VALUE - 2^14).
     */
    static public int round(double value) {
        return (int) (value + BIG_ENOUGH_ROUND) - BIG_ENOUGH_INT;
    }

    /**
     * Returns the closest integer to the specified double. This method will only
     * properly round doubles that are positive.
     */
    static public int roundPositive(double value) {
        return (int) (value + 0.5f);
    }

    /**
     * Returns true if the value is zero (using the default tolerance as upper
     * bound)
     */
    static public boolean isZero(double value) {
        return Math.abs(value) <= DOUBLE_ROUNDING_ERROR;
    }

    /**
     * Returns true if the value is zero.
     *
     * @param tolerance represent an upper bound below which the value is considered
     *                  zero.
     */
    static public boolean isZero(double value, double tolerance) {
        return Math.abs(value) <= tolerance;
    }

    /**
     * Returns true if a is nearly equal to b. The function uses the default
     * doubleing error tolerance.
     *
     * @param a the first value.
     * @param b the second value.
     */
    static public boolean isEqual(double a, double b) {
        return Math.abs(a - b) <= DOUBLE_ROUNDING_ERROR;
    }

    /**
     * Returns true if a is nearly equal to b.
     *
     * @param a         the first value.
     * @param b         the second value.
     * @param tolerance represent an upper bound below which the two values are
     *                  considered equal.
     */
    static public boolean isEqual(double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }

    /**
     * @return the logarithm of value with base a
     */
    static public double log(double a, double value) {
        return Math.log(value) / Math.log(a);
    }

    /**
     * @return the logarithm of value with base 2
     */
    static public double log2(double value) {
        return log(2, value);
    }

    public static float pow(float val, float power) {
        return (float) Math.pow(val, power);
    }

    public static float abs(float f) {
        return Math.abs(f);
    }

    public static float sqrt(float f) {
        return (float) Math.sqrt(f);
    }

    public static float asin(float sin) {
        return (float) Math.asin(sin);
    }

    public static float acos(float cos) {
        return (float) Math.acos(cos);
    }

    public static float toDegrees(float radians) {
        return radians * radiansToDegrees;
    }

    public static float toRadians(float radians) {
        return radians * degreesToRadians;
    }

    public static float max(float a, float b) {
        return a > b ? a : b;
    }

    public static float min(float a, float b) {
        return a < b ? a : b;
    }

    public static final double SAFE_MIN_DOUBLE;
    public static final double EPSILON_DOUBLE;
    public static final double SAFE_MIN_FLOAT;
    public static final double EPSILON_FLOAT;
    private static final long EXPONENT_OFFSET_DOUBLE = 1023l;
    private static final long EXPONENT_OFFSET_FLOAT = 127;

    static {
        EPSILON_DOUBLE = Double.longBitsToDouble((EXPONENT_OFFSET_DOUBLE - 53l) << 52);
        SAFE_MIN_DOUBLE = Double.longBitsToDouble((EXPONENT_OFFSET_DOUBLE - 1022l) << 52);

        EPSILON_FLOAT = Double.longBitsToDouble((EXPONENT_OFFSET_FLOAT - 24l) << 23);
        SAFE_MIN_FLOAT = Double.longBitsToDouble((EXPONENT_OFFSET_FLOAT - 126) << 23);
    }

    public static class LocalizedFormats {
        public static String ROTATION_MATRIX_DIMENSIONS = "a {0}x{1} matrix cannot be a rotation matrix";
    }


    public static class CardanEulerSingularityException extends Exception {

        public CardanEulerSingularityException(boolean b) {
            // TODO Auto-generated constructor stub
        }

    }

    public class MathIllegalArgumentException extends Exception {
        public MathIllegalArgumentException(String zeroNormForRotationAxis) {

        }

    }

}
