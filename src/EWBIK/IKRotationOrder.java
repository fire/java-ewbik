package EWBIK;

public final class IKRotationOrder {

        public static final IKVector3 X = new IKVector3(1, 0, 0);
        public static final IKVector3 Y = new IKVector3(0, 1, 0);
        public static final IKVector3 Z = new IKVector3(0, 0, 1);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around X, then around Y, then
         * around Z
         */
        public static final IKRotationOrder XYZ = new IKRotationOrder("XYZ", X, Y, Z);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around X, then around Z, then
         * around Y
         */
        public static final IKRotationOrder XZY = new IKRotationOrder("XZY", X, Z, Y);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Y, then around X, then
         * around Z
         */
        public static final IKRotationOrder YXZ = new IKRotationOrder("YXZ", Y, X, Z);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Y, then around Z, then
         * around X
         */
        public static final IKRotationOrder YZX = new IKRotationOrder("YZX", Y, Z, X);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Z, then around X, then
         * around Y
         */
        public static final IKRotationOrder ZXY = new IKRotationOrder("ZXY", Z, X, Y);

        /**
         * Set of Cartesian angles.
         * this ordered set of rotations is around Z, then around Y, then
         * around X
         */
        public static final IKRotationOrder ZYX = new IKRotationOrder("ZYX", Z, Y, X);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around X, then around Y, then
         * around X
         */
        public static final IKRotationOrder XYX = new IKRotationOrder("XYX", X, Y, X);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around X, then around Z, then
         * around X
         */
        public static final IKRotationOrder XZX = new IKRotationOrder("XZX", X, Z, X);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Y, then around X, then
         * around Y
         */
        public static final IKRotationOrder YXY = new IKRotationOrder("YXY", Y, X, Y);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Y, then around Z, then
         * around Y
         */
        public static final IKRotationOrder YZY = new IKRotationOrder("YZY", Y, Z, Y);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Z, then around X, then
         * around Z
         */
        public static final IKRotationOrder ZXZ = new IKRotationOrder("ZXZ", Z, X, Z);

        /**
         * Set of Euler angles.
         * this ordered set of rotations is around Z, then around Y, then
         * around Z
         */
        public static final IKRotationOrder ZYZ = new IKRotationOrder("ZYZ", Z, Y, Z);

        /**
         * Name of the rotations order.
         */
        private final String name;

        /**
         * Axis of the first rotation.
         */
        private final IKVector3 a1;

        /**
         * Axis of the second rotation.
         */
        private final IKVector3 a2;

        /**
         * Axis of the third rotation.
         */
        private final IKVector3 a3;

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
        private IKRotationOrder(final String name,
                                final IKVector3 a1, final IKVector3 a2, final IKVector3 a3) {
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
        public IKVector3 getA1() {
                return a1;
        }

        /**
         * Get the axis of the second rotation.
         *
         * @return axis of the second rotation
         */
        public IKVector3 getA2() {
                return a2;
        }

        /**
         * Get the axis of the second rotation.
         *
         * @return axis of the second rotation
         */
        public IKVector3 getA3() {
                return a3;
        }

}