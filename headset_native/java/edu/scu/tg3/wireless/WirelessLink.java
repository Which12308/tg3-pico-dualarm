package edu.scu.tg3.wireless;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.UUID;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.json.JSONArray;
import org.json.JSONObject;

/** One worker, one latest sample, no action queue, no robot output interface. */
public final class WirelessLink implements AutoCloseable {
    public static final String PROTOCOL = "tg3-native-1";
    private static final int BODY_LIMIT = 16384;
    private static final long REQUEST_NS = 120_000_000L;
    private final String host;
    private final String token;
    private final Object state = new Object();
    private final AtomicReference<HttpURLConnection> inFlight = new AtomicReference<>();
    private final ScheduledThreadPoolExecutor deadlines;
    private final Thread worker;
    private volatile String status = "WAITING FOR FOCUSED TRACKING";
    private boolean lifecycleActive;
    private boolean windowFocused;
    private boolean sourceFocused;
    private boolean closing;
    private boolean terminated;
    private boolean invalidationPending;
    private Sample latest;
    private String epoch = newEpoch();
    private long ordinal;
    private long lastSentOrdinal = -1;
    private long sent;

    private static final class Sample {
        final String json;
        final String epoch;
        final long capturedNs;
        final long ordinal;
        final boolean invalidation;
        Sample(String json, String epoch, long ordinal, boolean invalidation) {
            this.json = json;
            this.epoch = epoch;
            this.ordinal = ordinal;
            this.capturedNs = System.nanoTime();
            this.invalidation = invalidation;
        }
    }

    public WirelessLink(String host, String token) {
        if (!validHost(host) || !validToken(token)) throw new IllegalArgumentException("invalid_connection");
        this.host = host;
        this.token = token;
        deadlines = new ScheduledThreadPoolExecutor(1, runnable -> {
            Thread thread = new Thread(runnable, "tg3-http-deadline");
            thread.setDaemon(true);
            return thread;
        });
        deadlines.setRemoveOnCancelPolicy(true);
        worker = new Thread(this::run, "tg3-latest-wireless");
        worker.setDaemon(true);
        worker.start();
    }

    public static boolean validHost(String value) {
        // ADB reverse terminates only at this exact loopback address.
        if ("127.0.0.1".equals(value)) return true;
        if (value == null || value.length() > 15) return false;
        String[] parts = value.split("\\.", -1);
        if (parts.length != 4) return false;
        int first = -1;
        for (int i = 0; i < parts.length; i++) {
            String part = parts[i];
            if (!part.matches("[0-9]{1,3}") || (part.length() > 1 && part.charAt(0) == '0')) return false;
            int number = Integer.parseInt(part);
            if (number > 255) return false;
            if (i == 0) first = number;
        }
        return first > 0 && first < 224 && first != 127;
    }

    public static boolean validToken(String value) {
        return value != null && value.matches("[A-Za-z0-9_-]{32,128}");
    }

    public static JSONObject pair(String host, String pin, String device) throws Exception {
        if (!validHost(host) || pin == null || !pin.matches("[0-9]{8}")) {
            throw new IllegalArgumentException("invalid_pairing_request");
        }
        JSONObject request = new JSONObject().put("protocol", PROTOCOL).put("pin", pin).put("device", device);
        HttpURLConnection connection = (HttpURLConnection) new URL("http", host, 8877, "/v1/pair").openConnection();
        connection.setConnectTimeout(2000);
        connection.setReadTimeout(2000);
        try {
            connection.setRequestMethod("POST");
            connection.setRequestProperty("Content-Type", "application/json");
            connection.setUseCaches(false);
            byte[] bytes = request.toString().getBytes(StandardCharsets.UTF_8);
            connection.setDoOutput(true);
            connection.setFixedLengthStreamingMode(bytes.length);
            try (OutputStream output = connection.getOutputStream()) { output.write(bytes); }
            if (connection.getResponseCode() != 200) throw new IOException("pairing_rejected");
            try (InputStream input = connection.getInputStream()) {
                return new JSONObject(readLimited(input));
            }
        } finally {
            connection.disconnect();
        }
    }

    public void resume() {
        synchronized (state) {
            if (closing || terminated) return;
            lifecycleActive = true;
            resetLocked("RELEASE INPUTS TO RESUME");
            state.notifyAll();
        }
        cancelCurrent();
    }

    public void windowFocus(boolean focused) {
        synchronized (state) {
            if (windowFocused == focused || closing || terminated) return;
            windowFocused = focused;
            if (focused) resetLocked("RELEASE INPUTS AFTER FOCUS CHANGE");
            else invalidateLocked("APPLICATION NOT FOCUSED");
            state.notifyAll();
        }
        cancelCurrent();
    }

    public void pause() {
        synchronized (state) {
            if (closing || terminated) return;
            lifecycleActive = false;
            invalidateLocked("PAUSED / INPUT STOPPED");
            state.notifyAll();
        }
        cancelCurrent();
    }

    public String getStatus() { return status; }

    public void publish(String rawFrame) {
        if (rawFrame == null || rawFrame.length() > BODY_LIMIT) return;
        try {
            JSONObject parsed = new JSONObject(rawFrame);
            boolean focused = parsed.getBoolean("focused");
            boolean cancel = false;
            synchronized (state) {
                if (!lifecycleActive || !windowFocused || closing || terminated) return;
                if (!focused) {
                    if (sourceFocused) {
                        invalidateLocked("XR TRACKING NOT FOCUSED");
                        cancel = true;
                    }
                } else {
                    if (!sourceFocused) {
                        resetLocked("RELEASE INPUTS AFTER XR RESUME");
                        sourceFocused = true;
                    }
                    latest = new Sample(rawFrame, epoch, ++ordinal, false);
                }
                state.notifyAll();
            }
            if (cancel) cancelCurrent();
        } catch (Exception malformed) {
            synchronized (state) {
                if (!closing && !terminated) invalidateLocked("INVALID NATIVE FRAME / INPUT STOPPED");
                state.notifyAll();
            }
            cancelCurrent();
        }
    }

    private void resetLocked(String reason) {
        epoch = newEpoch();
        sourceFocused = false;
        latest = null;
        invalidationPending = false;
        lastSentOrdinal = -1;
        status = reason;
    }

    private void invalidateLocked(String reason) {
        resetLocked(reason);
        try {
            JSONObject frame = new JSONObject().put("session", "lifecycle").put("sequence", 0)
                .put("source_time_s", Math.max(0., System.nanoTime() / 1e9)).put("source_age_s", 0.)
                .put("source", "openxr").put("focused", false).put("scene_visible", false)
                .put("head", JSONObject.NULL).put("left", disconnected("left")).put("right", disconnected("right"));
            latest = new Sample(frame.toString(), epoch, ++ordinal, true);
            invalidationPending = true;
        } catch (Exception impossible) {
            latest = null;
            invalidationPending = false;
        }
    }

    private static JSONObject disconnected(String side) throws Exception {
        return new JSONObject().put("pose", JSONObject.NULL).put("connected", false).put("tracked", false)
            .put("grip", 0.).put("trigger", 0.).put("buttons", new JSONArray())
            .put("axes", new JSONArray().put(0.).put(0.)).put("source_id", "lifecycle-" + side);
    }

    private boolean workAvailableLocked() {
        return invalidationPending || (!closing && lifecycleActive && windowFocused && sourceFocused
            && latest != null && latest.ordinal != lastSentOrdinal);
    }

    private void run() {
        try {
            while (true) {
                synchronized (state) {
                    while (!workAvailableLocked() && !closing) {
                        if (lifecycleActive && windowFocused && sourceFocused && latest != null
                            && !latest.invalidation && System.nanoTime() - latest.capturedNs > REQUEST_NS) {
                            invalidateLocked("NATIVE INPUT STALE / RELEASE TO RESUME");
                            break;
                        }
                        state.wait(50);
                    }
                    if (!workAvailableLocked() && closing) return;
                }
                boolean invalidAttempt = false;
                try {
                    JSONObject challenge = request("/v1/challenge", null, null);
                    if (!PROTOCOL.equals(challenge.getString("protocol"))) throw new IOException("protocol_mismatch");
                    Sample sample;
                    synchronized (state) {
                        // Only fetch the sample AFTER challenge receipt.
                        if (!workAvailableLocked()) continue;
                        sample = latest;
                        invalidAttempt = sample.invalidation;
                        if (invalidAttempt) invalidationPending = false; // One best-effort stop packet.
                    }
                    long now = System.nanoTime();
                    double age = (now - sample.capturedNs) / 1e9;
                    if (age < 0. || (!invalidAttempt && age > .12)) {
                        synchronized (state) {
                            if (sample.epoch.equals(epoch)) invalidateLocked("NATIVE INPUT STALE / RELEASE TO RESUME");
                        }
                        continue;
                    }
                    JSONObject frame = new JSONObject(sample.json);
                    if (invalidAttempt) {
                        // This is a fresh confirmed lifecycle event, without
                        // sensor poses or active controls, not renewed old input.
                        frame.put("source_time_s", Math.max(0., now / 1e9));
                        frame.put("source_age_s", 0.);
                        age = 0.;
                    }
                    double sourceAge = frame.getDouble("source_age_s");
                    if (!Double.isFinite(sourceAge) || sourceAge < 0. || sourceAge + age > .12) {
                        throw new IOException("invalid_or_expired_native_timestamp");
                    }
                    frame.put("source_age_s", sourceAge + age);
                    String original = frame.getString("session");
                    if (original.length() > 80) throw new IOException("session_too_long");
                    frame.put("session", sample.epoch + ":" + original);
                    JSONObject envelope = new JSONObject().put("protocol", PROTOCOL)
                        .put("coordinates", "openxr_rh_y_up").put("frame", frame);
                    synchronized (state) {
                        if (!sample.epoch.equals(epoch)) continue;
                        if (!invalidAttempt && (!lifecycleActive || !windowFocused || !sourceFocused || closing)) continue;
                    }
                    JSONObject result = request("/v1/frame", envelope, challenge.getString("nonce"));
                    if (!result.getBoolean("accepted")) throw new IOException("frame_rejected");
                    synchronized (state) {
                        if (sample.epoch.equals(epoch)) {
                            lastSentOrdinal = sample.ordinal;
                            if (!invalidAttempt) status = "WIRELESS INPUT ACTIVE / frames " + (++sent);
                        }
                    }
                    Thread.sleep(5);
                } catch (Exception disconnected) {
                    if (Thread.currentThread().isInterrupted()) return;
                    synchronized (state) {
                        if (closing) return;
                        if (lifecycleActive && windowFocused && !invalidAttempt) {
                            resetLocked("NETWORK LOST / RELEASE TO RECONNECT");
                        } else if (invalidAttempt) {
                            invalidationPending = false;
                        }
                    }
                    Thread.sleep(150);
                }
            }
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
        } finally {
            synchronized (state) { terminated = true; latest = null; }
            cancelCurrent();
            deadlines.shutdownNow();
        }
    }

    private JSONObject request(String path, JSONObject body, String nonce) throws Exception {
        HttpURLConnection connection = (HttpURLConnection) new URL("http", host, 8877, path).openConnection();
        long began = System.nanoTime();
        AtomicBoolean expired = new AtomicBoolean();
        connection.setConnectTimeout(100);
        connection.setReadTimeout(100);
        connection.setUseCaches(false);
        connection.setRequestProperty("Authorization", "Bearer " + token);
        connection.setRequestProperty("Content-Type", "application/json");
        if (nonce != null) connection.setRequestProperty("X-TG3-Nonce", nonce);
        inFlight.set(connection);
        ScheduledFuture<?> deadline = deadlines.schedule(() -> {
            if (inFlight.compareAndSet(connection, null)) {
                expired.set(true);
                connection.disconnect();
            }
        }, REQUEST_NS, TimeUnit.NANOSECONDS);
        boolean succeeded = false;
        try {
            if (body == null) connection.setRequestMethod("GET");
            else {
                connection.setRequestMethod("POST");
                byte[] bytes = body.toString().getBytes(StandardCharsets.UTF_8);
                if (bytes.length > BODY_LIMIT) throw new IOException("oversize_frame");
                connection.setDoOutput(true);
                connection.setFixedLengthStreamingMode(bytes.length);
                try (OutputStream output = connection.getOutputStream()) { output.write(bytes); }
            }
            if (connection.getResponseCode() != 200) throw new IOException("receiver_rejected_request");
            String response;
            try (InputStream input = connection.getInputStream()) { response = readLimited(input); }
            if (expired.get() || System.nanoTime() - began > REQUEST_NS) throw new IOException("request_deadline");
            JSONObject result = new JSONObject(response);
            succeeded = true;
            return result;
        } finally {
            deadline.cancel(false);
            inFlight.compareAndSet(connection, null);
            // On success the consumed response allows Android's HTTP keepalive
            // pool to reuse this LAN connection. Errors always discard it.
            if (!succeeded) connection.disconnect();
        }
    }

    private static String readLimited(InputStream input) throws IOException {
        ByteArrayOutputStream buffer = new ByteArrayOutputStream();
        byte[] chunk = new byte[2048];
        int count;
        while ((count = input.read(chunk)) != -1) {
            if (buffer.size() + count > BODY_LIMIT) throw new IOException("oversize_receiver_reply");
            buffer.write(chunk, 0, count);
        }
        return buffer.toString(StandardCharsets.UTF_8.name());
    }

    private static String newEpoch() { return UUID.randomUUID().toString().replace("-", ""); }

    private void cancelCurrent() {
        HttpURLConnection connection = inFlight.getAndSet(null);
        if (connection != null) connection.disconnect();
    }

    @Override public void close() {
        synchronized (state) {
            if (closing || terminated) return;
            lifecycleActive = false;
            invalidateLocked("CLOSED / INPUT STOPPED");
            closing = true;
            state.notifyAll();
        }
        cancelCurrent();
        // The daemon worker gets one bounded best-effort invalidation attempt,
        // then exits; UI/lifecycle callbacks never wait on a network join.
    }
}
