package ewbik.math;

public final class RotationOrder {

        public static final Vector3 X = new Vector3(1, 0, 0);
        public static final Vector3 Y = new Vector3(0, 1, 0);
        public static final Vector3 Z = new Vector3(0, 0, 1);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around X, then around Y, then
         * around Z
         */
        public static final RotationOrder XYZ = new RotationOrder("XYZ", X, Y, Z);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around X, then around Z, then
         * around Y
         */
        public static final RotationOrder XZY = new RotationOrder("XZY", X, Z, Y);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Y, then around X, then
         * around Z
         */
        public static final RotationOrder YXZ = new RotationOrder("YXZ", Y, X, Z);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Y, then around Z, then
         * around X
         */
        public static final RotationOrder YZX = new RotationOrder("YZX", Y, Z, X);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Z, then around X, then
         * around Y
         */
        public static final RotationOrder ZXY = new RotationOrder("ZXY", Z, X, Y);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Z, then around Y, then
         * around X
         */
        public static final RotationOrder ZYX = new RotationOrder("ZYX", Z, Y, X);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around X, then around Y, then
         * around X
         */
        public static final RotationOrder XYX = new RotationOrder("XYX", X, Y, X);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around X, then around Z, then
         * around X
         */
        public static final RotationOrder XZX = new RotationOrder("XZX", X, Z, X);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Y, then around X, then
         * around Y
         */
        public static final RotationOrder YXY = new RotationOrder("YXY", Y, X, Y);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Y, then around Z, then
         * around Y
         */
        public static final RotationOrder YZY = new RotationOrder("YZY", Y, Z, Y);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Z, then around X, then
         * around Z
         */
        public static final RotationOrder ZXZ = new RotationOrder("ZXZ", Z, X, Z);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Z, then around Y, then
         * around Z
         */
        public static final RotationOrder ZYZ = new RotationOrder("ZYZ", Z, Y, Z);

        /**
         * Name of the rotations order.
         */
        private final String name;

        /**
         * Axis of the first rotation.
         */
        private final Vector3 a1;

        /**
         * Axis of the second rotation.
         */
        private final Vector3 a2;

        /**
         * Axis of the third rotation.
         */
        private final Vector3 a3;

        /**
         * Private constructor.
         * This is a utility class that cannot be instantiated by the user,
         * so its only constructor is private.
         *
         * @param name name of the rotation order
         * @param a1   axis of the first rotation
         * @param a2   axis of the second rotation
         * @param a3   axis of the third rotation
         */
        private RotationOrder(final String name,
                        final Vector3 a1, final Vector3 a2, final Vector3 a3) {
                this.name = name;
                this.a1 = a1;
                this.a2 = a2;
                this.a3 = a3;
        }

        /**
         * Get a string representation of the instance.
         *
         * @return a string representation of the instance (in fact, its name)
         */
        @Override
        public String toString() {
                return name;
        }

        /**
         * Get the axis of the first rotation.
         *
         * @return axis of the first rotation
         */
        public Vector3 getA1() {
                return a1;
        }

        /**
         * Get the axis of the second rotation.
         *
         * @return axis of the second rotation
         */
        public Vector3 getA2() {
                return a2;
        }

        /**
         * Get the axis of the second rotation.
         *
         * @return axis of the second rotation
         */
        public Vector3 getA3() {
                return a3;
        }

}