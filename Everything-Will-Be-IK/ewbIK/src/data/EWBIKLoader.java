package data;

import java.beans.Visibility;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import math.floatV.Rot;
import math.floatV.MRotation;

import javax.swing.JTree;
import javax.swing.tree.TreePath;

import asj.LoadManager;
import asj.Saveable;
import asj.TypeIdentifier;
import asj.data.JSONArray;
import asj.data.JSONObject;
import asj.data.StringFuncs;
import math.floatV.SGVec_3f;

public class EWBIKLoader {
	FloatBackedLoader floatBackedLoader = new FloatBackedLoader();

	/**
	 * NOTE: in order to load custom (extended classes), those classes MUST have a
	 * default constructor!
	 * 
	 * @param filePath       location of file to import
	 * @param AxesClass      the class object you've used to extend the AbstractAxes
	 *                       class. If null, AbstractAxes will be used.
	 * @param BoneClass      the class object you've used to extend the AbstractBone
	 *                       class. If null, AbstractBone will be used.
	 * @param ArmatureClass  the class object you've used to extend the
	 *                       AbstractArmature class. If null, AbstractArmature will
	 *                       be used.
	 * @param KusudamaClass  the class object you've used to extend the
	 *                       AbstractKusudama class. If null, AbstractKusudama will
	 *                       be used.
	 * @param LimitConeClass the class object you've used to extend the
	 *                       AbstractLimitCone class. If null, AbstractLimitCone
	 *                       will be used.
	 * @param IKPinClass     the class object you've used to extend the
	 *                       AbstractIKPin class. If null, AbstractIKPin will be
	 *                       used.
	 * 
	 * @return a list of all instantiated armatures specified by the input file.
	 */

	public Collection<? extends IK.AbstractArmature> importSinglePrecisionArmatures(String filepath,
			Class<? extends math.floatV.AbstractAxes> AxesClass,
			Class<? extends IK.AbstractBone> BoneClass,
			Class<? extends IK.AbstractArmature> ArmatureClass,
			Class<? extends IK.Constraint> KusudamaClass,
			Class<? extends IK.AbstractLimitCone> LimitConeClass,
			Class<? extends IK.AbstractIKPin> IKPinClass) {
		File selection = new File(filepath);
		JSONObject loadFile = StringFuncs.loadJSONObject(selection);
		clearCurrentLoadObjects();
		return floatBackedLoader.loadJSON(loadFile,
				AxesClass,
				BoneClass,
				ArmatureClass,
				KusudamaClass,
				LimitConeClass,
				IKPinClass);
	}

	public void updateArmatureSegments() {
		floatBackedLoader.updateArmatureSegments();
	}

	public void clearCurrentLoadObjects() {
		floatBackedLoader.clearCurrentLoadObjects();
	}

	public String getCurrentFilePath() {
		return floatBackedLoader.getCurrentFilePath();
	}

	/**
	 * takes a JSONObject and parses it into the format specified by the
	 * TypeIdentifier.
	 * The Value parameter can be another hashmap, and this
	 * will nest hashmaps from jsonObjects accordingly.
	 * 
	 * @param json
	 * @param result
	 */
	public <T extends Object, V extends Object> HashMap<T, V> hashMapFromJSON(JSONObject json, HashMap<T, V> result,
			TypeIdentifier ti) {
		return floatBackedLoader.hashMapFromJSON(json, result, ti);
	}

	/**
	 * takes a JSONObject and parses it into the format specified by the
	 * TypeIdentifier.
	 * The Value parameter can be another hashmap, and this
	 * will nest hashmaps from jsonObjects accordingly.
	 * 
	 * @param json
	 * @param result
	 */
	public <T extends Object, V extends Object> HashMap<T, V> hashMapFromJSON(JSONObject json, TypeIdentifier ti) {
		return floatBackedLoader.hashMapFromJSON(json, ti);
	}

	public static Object parsePrimitive(Class keyClass, String toParse) {
		if (keyClass == String.class)
			return toParse;
		if (keyClass == Float.class)
			return Float.parseFloat(toParse);
		if (keyClass == Double.class)
			return Double.parseDouble(toParse);
		if (keyClass == Long.class)
			return Long.parseLong(toParse);
		if (keyClass == Boolean.class)
			return Boolean.parseBoolean(toParse);
		if (keyClass == Integer.class)
			return Integer.parseInt(toParse);
		if (keyClass == Byte.class)
			return Byte.parseByte(toParse);
		else
			return null;
	}

	/*
	 * public Object parsePrimitive(Class keyClass, String toParse) {
	 * if(keyClass == String.class) return toParse;
	 * if(keyClass == Float.class) return Float.parseFloat(toParse);
	 * if(keyClass == Double.class) return Double.parseDouble(toParse);
	 * if(keyClass == Long.class) return Long.parseLong(toParse);
	 * if(keyClass == Boolean.class) return Boolean.parseBoolean(toParse);
	 * if(keyClass == Integer.class) return Integer.parseInt(toParse);
	 * if(keyClass == Byte.class) return Byte.parseByte(toParse);
	 * else return null;
	 * }
	 */

	/**
	 * returns the appropriate object from the load hashmaps based on the
	 * identityHash and keyClass.
	 * if the object is not found, returns null
	 * 
	 * @param keyClass
	 * @param identityHash
	 * @return
	 */

	public Saveable getObjectFromClassMaps(Class keyClass, String identityHash) {
		return floatBackedLoader.getObjectFromClassMaps(keyClass, identityHash);
	}

	public static void setTempLoadDirectory(String tempLoadDirectory) {
		tempLoadDirectory = tempLoadDirectory;
	}

	public <T extends Object> void arrayListFromJSONArray(JSONArray jsonArray, ArrayList<T> list, Class c) {
		for (int i = 0; i < jsonArray.size(); i++) {
			Object item = jsonArray.get(i);
			if (c == SGVec_3f.class)
				list.add((T) new SGVec_3f(jsonArray.getJSONArray(i)));
			else if (c == Rot.class)
				list.add((T) new Rot(jsonArray.getJSONArray(i)));
			else if (c == MRotation.class)
				list.add((T) new Rot(jsonArray.getJSONArray(i)).rotation);
			else if (c.getName().startsWith("java.lang"))
				list.add((T) parsePrimitive(c, "" + jsonArray.get(i)));
			else {
				String sitem = Number.class.isAssignableFrom(item.getClass()) ? "" + item : (String) item;
				list.add((T) getObjectFromClassMaps(c, sitem));
			}
		}
	}

}
