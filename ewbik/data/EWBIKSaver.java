package ewbik.data;

import ewbik.asj.SaveManager;
import ewbik.asj.Saveable;
import ewbik.asj.data.JSONArray;
import ewbik.asj.data.JSONObject;
import ewbik.asj.data.StringFuncs;
import ewbik.ik.*;
import ewbik.math.AbstractAxes;

import java.util.ArrayList;
import java.util.Collection;
import java.util.WeakHashMap;
import ik.Bone;
import ik.IKPin;

public class EWBIKSaver extends SaveManager {

    WeakHashMap<Saveable, Boolean> saveables = new WeakHashMap<Saveable, Boolean>();

    public static String currentFilePath;
    public static String tempDir;

    public void saveArmature(AbstractSkeleton3D toSave, String path) {
        clearSaveState();
        ((AbstractSkeleton3D) toSave).notifyOfSaveIntent(this);
        saveAs(path);
        notifyCurrentSaveablesOfSaveCompletion();
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
        clearSaveState();
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
                if (AbstractAxes.class.isAssignableFrom(s.getClass()))
                    axesJSON.append(jsonObj);
                if (AbstractSkeleton3D.class.isAssignableFrom(s.getClass()))
                    armaturesJSON.append(jsonObj);
                if (Bone.class.isAssignableFrom(s.getClass()))
                    bonesJSON.append(jsonObj);
                if (AbstractKusudama.class.isAssignableFrom(s.getClass()))
                    kusudamaJSON.append(jsonObj);
                if (AbstractLimitCone.class.isAssignableFrom(s.getClass()))
                    limitConeJSON.append(jsonObj);
                if (IKPin.class.isAssignableFrom(s.getClass()))
                    IKPinsJSON.append(jsonObj);
            }
        }

        saveObject.setJSONArray("axes", axesJSON);
        saveObject.setJSONArray("armatures", armaturesJSON);
        saveObject.setJSONArray("bones", bonesJSON);
        saveObject.setJSONArray("kusudamas", kusudamaJSON);
        saveObject.setJSONArray("limitCones", limitConeJSON);
        saveObject.setJSONArray("IKPins", IKPinsJSON);
        return saveObject;
    }

    public String getSaveString() {
        String resultString = getSaveObject().toString();
        return resultString;
    }

    private void saveAs(String savePath) {
        save(savePath);
    }

    public void save(String savePath) {
        JSONObject fileContent = getSaveObject();
        StringFuncs.saveJSONObject(fileContent, savePath);
    }

}