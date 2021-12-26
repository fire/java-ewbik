package ewbik.processing;

import ewbik.processing.singlePrecision.*;
import ewbik.processing.singlePrecision.sceneGraph.Axes;

import java.util.Collection;

import data.EWBIKLoader;
import data.EWBIKSaver;

public final class EWBKIO {

	/**
	 * Return a single precision (float) version of the armature in stored in the
	 * specified filepath
	 * 
	 * @param path
	 * @return the Armature, or null if the file does not specify an armature
	 */
	public static Armature LoadArmature_singlePrecision(String path) {
		EWBIKLoader newLoader = new EWBIKLoader();
		@SuppressWarnings("unchecked")
		Collection<Armature> ArmatureList = (Collection<Armature>) newLoader.importSinglePrecisionArmatures(path,
				Axes.class, Bone.class, Armature.class, Kusudama.class, LimitCone.class, IKPin.class);
		for (Armature a : ArmatureList) {
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
