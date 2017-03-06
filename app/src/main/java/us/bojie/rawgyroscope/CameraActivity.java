/*
 * Copyright 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package us.bojie.rawgyroscope;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;

import us.bojie.rawgyroscope.camera.Camera2RawFragment;

/**
 * Activity displaying a fragment that implements RAW photo captures.
 */
public class CameraActivity extends Activity implements Camera2RawFragment.ResultListener {

    public static final String KEY_RESULT = "result";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);
        if (null == savedInstanceState) {
            getFragmentManager().beginTransaction()
                    .replace(R.id.container, Camera2RawFragment.newInstance())
                    .commit();
        }
    }


    @Override
    public void onResult(String result) {
        Intent intent = new Intent(this, ResultActivity.class);
        intent.putExtra(KEY_RESULT, result);
        startActivity(intent);
    }

}
