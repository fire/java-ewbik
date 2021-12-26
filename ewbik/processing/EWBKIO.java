package ewbik.processing;

import ewbik.data.EWBIKLoader;
import ewbik.data.EWBIKSaver;
import ewbik.processing.sceneGraph.Axes;

import java.util.Collection;

public final class EWBKIO {

    /**
     * Return a single precision (float) version of the armature in stored in the
     * specified filepath
     *
     * @param path
     * @return the Armature, or null if the file does not specify an armature
     */
    public static ewbik.processing.Armature LoadArmature_singlePrecision(String path) {
        EWBIKLoader newLoader = new EWBIKLoader();
        @SuppressWarnings("unchecked")
        Collection<ewbik.processing.Armature> ArmatureList = (Collection<ewbik.processing.Armature>) newLoader.importSinglePrecisionArmatures(path,
                Axes.class, ewbik.processing.singlePrecision.Bone.class, ewbik.processing.Armature.class, ewbik.processing.singlePrecision.Kusudama.class, ewbik.processing.singlePrecision.LimitCone.class, ewbik.processing.singlePrecision.IKPin.class);
        for (ewbik.processing.Armature a : ArmatureList) {
            return a;
        }
        return null;
    }

    /**
     * save the given armature into the specified filepath
     *
     * @param path
     * @param loadInto
     */
    public static void SaveArmature(String path, Armature toSave) {
        EWBIKSaver newSaver = new EWBIKSaver();
        newSaver.saveArmature(toSave, path);
    }

}
