package us.bojie.rawgyroscope;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.TextView;

public class ResultActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_result);

        Intent intent = getIntent();
        String result = intent.getStringExtra(CameraActivity.KEY_RESULT);
        TextView resultTextView = (TextView) findViewById(R.id.result);
        resultTextView.setText(result);
    }
}
