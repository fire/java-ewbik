package ewbik.data;

import ewbik.asj.SaveManager;
import ewbik.asj.Saveable;
import ewbik.asj.data.JSONArray;
import ewbik.asj.data.JSONObject;
import ewbik.asj.data.StringFuncs;
import ewbik.ik.*;
import ik.Bone;
import ik.IKPin;
import processing.Node3D;
import processing.Skeleton3D;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.WeakHashMap;

public class FloatBackedSaver extends SaveManager {

    public static String currentFilePath;
    public static String tempDir;
    WeakHashMap<Saveable, Boolean> saveables = new WeakHashMap<Saveable, Boolean>();

    public FloatBackedSaver() {

    }

    public void saveArmature(Skeleton3D toSave, String absolutePath) {
        try {
            File tempFile = File.createTempFile("GiftedApprentice", ".tmp");
            System.out.println(tempFile.getParent());
            System.out.println(tempFile.getParent() + File.separator);
            System.out.println(tempFile.getParent() + File.separator + "GiftedApprentice" + System.currentTimeMillis());
            tempDir = tempFile.getParent() + File.separator + "GiftedApprentice" + System.currentTimeMillis();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addToSaveState(Saveable saveObj) {
        saveables.put(saveObj, true);
    }

    public void removeFromSaveState(Saveable saveObj) {
        saveables.remove(saveObj);
    }

    public void clearSaveState() {
        saveables.clear();
    }

    public void notifyCurrentSaveablesOfSaveCompletion() {
        ArrayList<Saveable> sarr = new ArrayList<>(saveables.keySet());
        for (Saveable s : sarr) {
            s.notifyOfSaveCompletion(this);
        }
    }

    public JSONObject getSaveObject() {

        JSONArray axesJSON = new JSONArray();
        JSONArray armaturesJSON = new JSONArray();
        JSONArray bonesJSON = new JSONArray();
        JSONArray kusudamaJSON = new JSONArray();
        JSONArray limitConeJSON = new JSONArray();
        JSONArray IKPinsJSON = new JSONArray();

        Collection<Saveable> sk = saveables.keySet();

        JSONObject saveObject = new JSONObject();

        for (Saveable s : sk) {
            JSONObject jsonObj = s.getSaveJSON(this);
            if (jsonObj != null) {
                if (Node3D.class.isAssignableFrom(s.getClass()))
                    axesJSON.append(jsonObj);
                if (Skeleton3D.class.isAssignableFrom(s.getClass()))
                    armaturesJSON.append(jsonObj);
                if (Bone.class.isAssignableFrom(s.getClass()))
                    bonesJSON.append(jsonObj);
                if (ewbik.processing.singlePrecision.Kusudama.class.isAssignableFrom(s.getClass()))
                    kusudamaJSON.append(jsonObj);
                if (ewbik.processing.singlePrecision.LimitCone.class.isAssignableFrom(s.getClass()))
                    limitConeJSON.append(jsonObj);
                if (IKPin.class.isAssignableFrom(s.getClass()))
                    IKPinsJSON.append(jsonObj);
            }
        }

        saveObject.setJSONArray("node_3d", axesJSON);
        saveObject.setJSONArray("skeleton_3d", armaturesJSON);
        saveObject.setJSONArray("bones", bonesJSON);
        saveObject.setJSONArray("kusudamas", kusudamaJSON);
        saveObject.setJSONArray("limit_cones", limitConeJSON);
        saveObject.setJSONArray("ik_pins", IKPinsJSON);

        notifyCurrentSaveablesOfSaveCompletion();
        return saveObject;
    }

    public String getSaveString() {
        String resultString = getSaveObject().toString();
        return resultString;
    }

    public void saveAs(String savePath) {
        currentFilePath = savePath;
        save();
    }

    public void save() {
        JSONObject fileContent = getSaveObject();
        StringFuncs.saveJSONObject(fileContent, tempDir + File.separator + "structure");
        try {
            File cFile = new File(currentFilePath);
            if (cFile != null) {
                cFile.getParentFile().mkdirs();
            }
        } catch (Exception e) {
            System.out.println("failed to save");
        }
    }
}
