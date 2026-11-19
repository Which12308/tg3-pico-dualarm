package edu.scu.tg3.wireless;

import android.app.NativeActivity;
import android.content.Intent;
import android.os.Bundle;
import java.util.concurrent.atomic.AtomicBoolean;

/** NativeActivity owns OpenXR; this class only handles the wireless lifecycle. */
public final class TeleopActivity extends NativeActivity {
    private volatile WirelessLink link;
    private final AtomicBoolean pairingRequested = new AtomicBoolean();

    @Override public void onCreate(Bundle savedState) {
        String host = getIntent().getStringExtra("host");
        String token = getIntent().getStringExtra("token");
        int port = getIntent().getIntExtra("port", 8877);
        if (WirelessLink.validHost(host) && WirelessLink.validToken(token) && port == 8877) {
            link = new WirelessLink(host, token);
        }
        super.onCreate(savedState);
        if (link == null) openPairing();
    }

    /** Called by C++ with a single raw OpenXR tracking-frame JSON object. */
    public void onTrackingFrame(String frameJson) {
        WirelessLink current = link;
        if (current != null && !pairingRequested.get()) current.publish(frameJson);
    }

    /** Called by C++ for its HUD. Does not disclose token, PIN, or packet data. */
    public String getLinkStatus() {
        WirelessLink current = link;
        return current == null ? "PAIRING REQUIRED" : current.getStatus();
    }

    /** Called by C++ when the operator requests to return to pairing. */
    public void openPairing() {
        returnToPairing(null);
    }

    /** C++ exception callback: stop input and present its exact bounded reason. */
    public void onNativeError(String message) {
        String reason = message == null ? "Native OpenXR initialization failed" : message;
        if (reason.length() > 400) reason = reason.substring(0, 400);
        returnToPairing(reason);
    }

    private void returnToPairing(String error) {
        if (!pairingRequested.compareAndSet(false, true)) return;
        runOnUiThread(() -> {
            WirelessLink current = link;
            if (current != null) current.close();
            getIntent().removeExtra("token");
            Intent intent = new Intent(this, PairActivity.class);
            if (error != null) intent.putExtra("native_error", error);
            startActivity(intent);
            finish();
        });
    }

    @Override protected void onResume() {
        super.onResume();
        WirelessLink current = link;
        if (current != null) current.resume();
    }

    @Override protected void onPause() {
        WirelessLink current = link;
        if (current != null) current.pause();
        super.onPause();
    }

    @Override public void onWindowFocusChanged(boolean hasFocus) {
        WirelessLink current = link;
        if (current != null) current.windowFocus(hasFocus);
        super.onWindowFocusChanged(hasFocus);
    }

    @Override public void onBackPressed() {
        openPairing();
    }

    @Override protected void onDestroy() {
        WirelessLink current = link;
        link = null;
        if (current != null) current.close();
        getIntent().removeExtra("token");
        super.onDestroy();
    }
}
