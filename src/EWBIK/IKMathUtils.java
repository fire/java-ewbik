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

package EWBIK;

/**
 * Utility and fast InverseKinematics.math functions.
 * <p>
 * Thanks to Riven on JavaGaming.org for the basis of sin/cos/floor/ceil.
 *
 * @author Nathan Sweet
 */
public final class IKMathUtils {
    static public final float FLOAT_ROUNDING_ERROR = 0.000001f; // 32 bits, 23 of which may hold the significand for a
                                                                // precision of 6 digits
    static public final double DOUBLE_ROUNDING_ERROR = 0.000000000000001d; // 64, 52 of which represent the significand
                                                                           // for a precision of 15 digits.
    static public final float PI = (float) Math.PI;
    static public final float HALF_PI = (float) (Math.PI / 2d);
    /**
     * multiply by this to convert from radians to degrees
     */
    static public final float radiansToDegrees = 180f / PI;
    /**
     * multiply by this to convert from degrees to radians
     */
    static public final float degreesToRadians = PI / 180f;

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

    static public float clamp(float value, float min, float max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
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
