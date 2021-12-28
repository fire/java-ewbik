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

import ewbik.asj.CanLoad;
import ewbik.asj.data.JSONArray;
import ewbik.asj.data.JSONObject;

//import com.badlogic.gdx.utils.GdxRuntimeException;
//import com.badlogic.gdx.utils.NumberUtils;

/**
 * Encapsulates a 3D vector. Allows chaining operations by returning a reference
 * to itself in all modification methods.
 *
 * @author badlogicgames@gmail.com
 */
public class Vector3 extends Vec3f<ewbik.math.Vector3> implements CanLoad {

    public <V extends Vec3f<?>> Vector3(V v) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
    }

    public Vector3() {
        super();
    }

    public Vector3(float i, float j, float k) {
        super(i, j, k);
    }

    @Override
    public CanLoad populateSelfFromJSON(JSONObject j) {
        JSONArray components = j.getJSONArray("vec");
        this.x = components.getFloat(0);
        this.y = components.getFloat(1);
        this.z = components.getFloat(2);
        return this;
    }

    public Vector3(JSONObject j) {
        JSONArray components = j.getJSONArray("vec");
        this.x = components.getFloat(0);
        this.y = components.getFloat(1);
        this.z = components.getFloat(2);
    }

    public Vector3(JSONArray j) {
        this.x = j.getFloat(0);
        this.y = j.getFloat(1);
        this.z = j.getFloat(2);
    }

    @Override
    public ewbik.math.Vector3 copy() {
        return new ewbik.math.Vector3(this);
    }

    @Override
    public ewbik.math.Vector3 toVec3f() {
        return new ewbik.math.Vector3((float) x, (float) y, (float) z);
    }

    public JSONArray toJSONArray() {
        JSONArray vec = new JSONArray();
        vec.append(this.x);
        vec.append(this.y);
        vec.append(this.z);
        return vec;
    }

    @Override
    public JSONObject toJSONObject() {
        JSONObject j = new JSONObject();
        JSONArray components = new JSONArray();
        components.append(this.x);
        components.append(this.y);
        components.append(this.z);
        j.setJSONArray("vec", components);
        return j;
    }

}
