package edu.scu.tg3.wireless;

import android.app.Activity;
import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.text.InputFilter;
import android.text.InputType;
import android.text.method.DigitsKeyListener;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.ScrollView;
import android.widget.TextView;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.json.JSONObject;

/** First connection screen. It performs no XR acquisition or robot output. */
public final class PairActivity extends Activity {
    private final ExecutorService pairing = Executors.newSingleThreadExecutor();
    private EditText hostInput;
    private EditText pinInput;
    private TextView status;
    private Button connect;
    private ProgressBar progress;
    private RadioGroup transport;
    private RadioButton usbMode;
    private RadioButton wifiMode;
    private String wifiHost;
    private volatile boolean destroyed;
    private volatile boolean foreground;

    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        ScrollView scroll = new ScrollView(this);
        LinearLayout form = new LinearLayout(this);
        form.setOrientation(LinearLayout.VERTICAL);
        int margin = (int) (24 * getResources().getDisplayMetrics().density);
        form.setPadding(margin, margin, margin, margin);
        scroll.addView(form);
        text(form, "天工 PICO 双臂遥操作", 26);
        text(form, "默认 USB 数据线连接：保持头显连接电脑，在电脑启动器准备 USB 通道后，只需填写 8 位配对码。也可选择 Wi-Fi。", 17);
        transport = new RadioGroup(this);
        transport.setOrientation(LinearLayout.HORIZONTAL);
        usbMode = new RadioButton(this);
        usbMode.setId(View.generateViewId());
        usbMode.setText("USB（默认）");
        wifiMode = new RadioButton(this);
        wifiMode.setId(View.generateViewId());
        wifiMode.setText("Wi-Fi（可选）");
        transport.addView(usbMode);
        transport.addView(wifiMode);
        form.addView(transport);
        text(form, "电脑 IPv4", 18);
        hostInput = new EditText(this);
        hostInput.setSingleLine(true);
        hostInput.setInputType(InputType.TYPE_CLASS_PHONE);
        hostInput.setKeyListener(DigitsKeyListener.getInstance("0123456789."));
        hostInput.setHint("例如 192.168.3.47");
        wifiHost = getSharedPreferences("tg3_connection", MODE_PRIVATE).getString("host", "");
        if ("127.0.0.1".equals(wifiHost)) wifiHost = "";
        form.addView(hostInput);
        transport.setOnCheckedChangeListener((group, checked) -> {
            boolean usb = checked == usbMode.getId();
            if (usb && hostInput.isEnabled()) wifiHost = hostInput.getText().toString().trim();
            hostInput.setEnabled(!usb);
            hostInput.setText(usb ? "127.0.0.1" : wifiHost);
        });
        transport.check(usbMode.getId());
        text(form, "8 位配对码（从电脑启动器读取）", 18);
        pinInput = new EditText(this);
        pinInput.setSingleLine(true);
        pinInput.setInputType(InputType.TYPE_CLASS_NUMBER);
        pinInput.setKeyListener(DigitsKeyListener.getInstance("0123456789"));
        pinInput.setFilters(new InputFilter[] {new InputFilter.LengthFilter(8)});
        pinInput.setSaveEnabled(false); // Never put the PIN into saved view state.
        form.addView(pinInput);
        connect = new Button(this);
        connect.setText("配对并进入透视采集");
        connect.setOnClickListener(view -> pair());
        form.addView(connect);
        progress = new ProgressBar(this);
        progress.setVisibility(View.GONE);
        form.addView(progress);
        status = text(form, "等待配对；当前页面不发送手柄动作。", 17);
        status.setSaveEnabled(false);
        String nativeFailure = getIntent().getStringExtra("native_error");
        if (nativeFailure != null && !nativeFailure.isEmpty()) {
            status.setText("OpenXR 采集未启动：" + nativeFailure + "\n输入已停止；请保留这条具体错误用于诊断。");
            getIntent().removeExtra("native_error");
        }
        text(form, "进入采集后先松开全部按键、扳机和握持键。握持键控制目标跟随，松开保持；扳机闭合并保留握持；左 Y／右 B 张手。走动观察前松开双侧握持键。真实机器人运动仍需电脑端单独授权。", 17);
        text(form, "USB 模式使用 ADB 端口转发，不需要 USB 网络共享。Wi-Fi 模式才填写电脑局域网 IPv4；请勿把接收端口映射到互联网。", 15);
        setContentView(scroll);
    }

    private TextView text(LinearLayout parent, String value, int size) {
        TextView view = new TextView(this);
        view.setText(value);
        view.setTextSize(size);
        view.setPadding(0, 10, 0, 10);
        parent.addView(view);
        return view;
    }

    private void pair() {
        final boolean usb = usbMode.isChecked();
        final String host = usb ? "127.0.0.1" : hostInput.getText().toString().trim();
        final String pin = pinInput.getText().toString();
        if (!WirelessLink.validHost(host) || !pin.matches("[0-9]{8}")) {
            status.setText("请填写有效 IPv4 和完整的 8 位配对码。");
            return;
        }
        connect.setEnabled(false);
        usbMode.setEnabled(false);
        wifiMode.setEnabled(false);
        progress.setVisibility(View.VISIBLE);
        status.setText("正在配对……");
        pairing.execute(() -> {
            try {
                String device = Build.MANUFACTURER + "/" + Build.MODEL;
                if (device.length() > 128) device = device.substring(0, 128);
                JSONObject reply = WirelessLink.pair(host, pin, device);
                final String token = reply.getString("token");
                if (!WirelessLink.PROTOCOL.equals(reply.getString("protocol")) || !WirelessLink.validToken(token)) {
                    throw new IllegalArgumentException("incompatible_pairing_reply");
                }
                runOnUiThread(() -> {
                    if (destroyed) return;
                    pinInput.setText("");
                    progress.setVisibility(View.GONE);
                    connect.setEnabled(true);
                    usbMode.setEnabled(true);
                    wifiMode.setEnabled(true);
                    if (!foreground) {
                        status.setText("配对期间应用已离开前台。请在电脑重新开启配对，再连接。");
                        return;
                    }
                    if (!usb) getSharedPreferences("tg3_connection", MODE_PRIVATE).edit().putString("host", host).apply();
                    Intent intent = new Intent(this, TeleopActivity.class);
                    intent.setAction(Intent.ACTION_MAIN);
                    intent.addCategory("org.khronos.openxr.intent.category.IMMERSIVE_HMD");
                    intent.addCategory("com.picovr.intent.category.VR");
                    intent.putExtra("host", host);
                    intent.putExtra("port", 8877);
                    intent.putExtra("token", token);
                    startActivity(intent);
                    finish();
                });
            } catch (Exception failure) {
                runOnUiThread(() -> {
                    if (destroyed) return;
                    pinInput.setText("");
                    progress.setVisibility(View.GONE);
                    connect.setEnabled(true);
                    usbMode.setEnabled(true);
                    wifiMode.setEnabled(true);
                    status.setText(usb
                        ? "USB 配对未成功。请保持数据线连接、允许 USB 调试，并在电脑点击准备 USB 通道，再重新开启配对。"
                        : "Wi-Fi 配对未成功。请确认电脑地址、局域网互通，并在电脑重新开启配对后重试。");
                });
            }
        });
    }

    @Override protected void onResume() {
        super.onResume();
        foreground = true;
    }

    @Override protected void onPause() {
        foreground = false;
        super.onPause();
    }

    @Override protected void onDestroy() {
        destroyed = true;
        pairing.shutdownNow();
        super.onDestroy();
    }
}
