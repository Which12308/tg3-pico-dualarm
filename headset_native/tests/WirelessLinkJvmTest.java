package edu.scu.tg3.wireless;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.json.JSONArray;
import org.json.JSONObject;

/** Real LAN-stack fixture on this computer. No Android, SSH, or robot calls. */
public final class WirelessLinkJvmTest {
    private static final String TOKEN = "abcdefghijklmnopqrstuvwxyz0123456789TESTTOKEN";
    private final HttpServer server;
    private final ExecutorService handlers = Executors.newCachedThreadPool(r -> {
        Thread thread = new Thread(r, "fixture-http"); thread.setDaemon(true); return thread;
    });
    private final Map<String, Long> challenges = new ConcurrentHashMap<>();
    private final List<JSONObject> received = new CopyOnWriteArrayList<>();
    private final AtomicInteger challengeCount = new AtomicInteger();
    private volatile boolean stall;
    private volatile CountDownLatch holdChallenge;
    private volatile CountDownLatch challengeEntered;
    private long seq;

    private WirelessLinkJvmTest(String host) throws Exception {
        server = HttpServer.create(new InetSocketAddress(host, 8877), 0);
        server.setExecutor(handlers);
        server.createContext("/v1/", this::handle);
        server.start();
    }

    private void handle(HttpExchange exchange) {
        try {
            String path = exchange.getRequestURI().getPath();
            if (path.equals("/v1/pair")) {
                JSONObject body = new JSONObject(read(exchange.getRequestBody()));
                check(body.getString("pin").equals("12345678"), "pair PIN");
                respond(exchange, 200, new JSONObject().put("protocol", WirelessLink.PROTOCOL).put("token", TOKEN));
                return;
            }
            if (!("Bearer " + TOKEN).equals(exchange.getRequestHeaders().getFirst("Authorization"))) {
                respond(exchange, 403, new JSONObject().put("error", "token")); return;
            }
            if (path.equals("/v1/challenge")) {
                challengeCount.incrementAndGet();
                if (stall) Thread.sleep(250);
                CountDownLatch held = holdChallenge;
                if (held != null) {
                    holdChallenge = null;
                    challengeEntered.countDown();
                    held.await(2, TimeUnit.SECONDS);
                }
                String nonce = UUID.randomUUID().toString();
                challenges.put(nonce, System.nanoTime());
                respond(exchange, 200, new JSONObject().put("protocol", WirelessLink.PROTOCOL).put("nonce", nonce));
            } else if (path.equals("/v1/frame")) {
                String nonce = exchange.getRequestHeaders().getFirst("X-TG3-Nonce");
                Long issued = challenges.remove(nonce);
                check(issued != null && System.nanoTime() - issued <= 120_000_000L, "expired/reused challenge");
                JSONObject body = new JSONObject(read(exchange.getRequestBody()));
                check(body.getString("protocol").equals(WirelessLink.PROTOCOL), "wire protocol");
                check(body.getString("coordinates").equals("openxr_rh_y_up"), "OpenXR handedness declaration");
                JSONObject frame = body.getJSONObject("frame");
                check(frame.getString("source").equals("openxr"), "tracking source");
                check(frame.getDouble("source_age_s") >= 0. && frame.getDouble("source_age_s") <= .12, "wire age");
                check(frame.getDouble("source_age_s")+(System.nanoTime()-issued)/1e9 <= .12, "cumulative source and transport age");
                received.add(frame);
                respond(exchange, 200, new JSONObject().put("accepted", true));
            } else respond(exchange, 404, new JSONObject().put("error", "path"));
        } catch (Exception failure) {
            try { respond(exchange, 400, new JSONObject().put("error", "fixture rejection")); }
            catch (Exception ignored) { }
        } finally { exchange.close(); }
    }

    private static void respond(HttpExchange exchange, int code, JSONObject data) throws Exception {
        byte[] bytes = data.toString().getBytes(StandardCharsets.UTF_8);
        exchange.getResponseHeaders().set("Content-Type", "application/json");
        exchange.sendResponseHeaders(code, bytes.length);
        exchange.getResponseBody().write(bytes);
    }

    private static String read(InputStream stream) throws Exception {
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        byte[] bytes = new byte[2048]; int n;
        while ((n = stream.read(bytes)) != -1) output.write(bytes, 0, n);
        return output.toString("UTF-8");
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    private String frame(boolean focused) throws Exception {
        JSONObject pose = new JSONObject().put("position", new JSONArray().put(.2).put(1.).put(-.3))
            .put("orientation_xyzw", new JSONArray().put(0.).put(0.).put(0.).put(1.));
        JSONObject hand = new JSONObject().put("pose", pose).put("connected", true).put("tracked", true)
            .put("grip", 1.).put("trigger", .4).put("buttons", new JSONArray())
            .put("axes", new JSONArray().put(0.).put(0.)).put("source_id", "test-controller");
        return new JSONObject().put("session", "NATIVE_FIXED").put("sequence", seq++)
            .put("source_time_s", 100. + seq*.02).put("source_age_s", 0.).put("source", "openxr")
            .put("focused", focused).put("scene_visible", true).put("head", pose)
            .put("left", hand).put("right", hand).toString();
    }

    private void feed(WirelessLink link, int millis) throws Exception {
        long until = System.nanoTime() + millis*1_000_000L;
        while (System.nanoTime() < until) { link.publish(frame(true)); Thread.sleep(15); }
    }

    private int activeCount() { int count=0; for(JSONObject p:received) if(p.optBoolean("focused"))count++; return count; }
    private int inactiveCount() { return received.size()-activeCount(); }
    private JSONObject latestActive() { for(int i=received.size()-1;i>=0;i--)if(received.get(i).optBoolean("focused"))return received.get(i); throw new AssertionError("no active frame"); }

    private JSONObject run(String host) throws Exception {
        JSONObject paired = WirelessLink.pair(host, "12345678", "PICO host-test");
        check(paired.getString("token").equals(TOKEN), "real pairing POST");
        WirelessLink link = new WirelessLink(host, TOKEN);
        try {
            link.resume(); link.windowFocus(true);
            feed(link, 650);
            check(activeCount() >= 3, "initial continuous samples");
            for (int i=1;i<received.size();i++) {
                JSONObject a=received.get(i-1),b=received.get(i);
                if(a.getString("session").equals(b.getString("session")))
                    check(b.getLong("sequence")>a.getLong("sequence"), "strict single-frame sequence ordering");
            }
            // A GET deliberately held by the fixture; publish another sample
            // while GET is in flight and prove the newer sample is selected.
            Thread.sleep(30);
            holdChallenge=new CountDownLatch(1); challengeEntered=new CountDownLatch(1);
            CountDownLatch held=holdChallenge;
            link.publish(frame(true));
            check(challengeEntered.await(1,TimeUnit.SECONDS), "challenge reached fixture");
            String newest=frame(true); long newestSeq=new JSONObject(newest).getLong("sequence");
            link.publish(newest); held.countDown();
            long deadline=System.nanoTime()+500_000_000L;
            while(System.nanoTime()<deadline && latestActive().getLong("sequence")<newestSeq)Thread.sleep(5);
            check(latestActive().getLong("sequence")==newestSeq, "sample fetched after challenge");

            // A halted native producer must not renew its last active pose.
            Thread.sleep(100); int activeBefore=activeCount();
            Thread.sleep(260);
            check(activeCount()==activeBefore, "old active pose not renewed");
            check(inactiveCount()>=1, "stale native producer sends one invalidation");

            feed(link,250); String beforePause=latestActive().getString("session");
            int invalidBefore=inactiveCount(); link.pause(); Thread.sleep(260);
            check(inactiveCount()>invalidBefore, "pause invalidation packet");
            int pausedCount=activeCount(); link.publish(frame(true)); Thread.sleep(150);
            check(activeCount()==pausedCount, "paused producer cannot send active input");
            link.resume(); link.windowFocus(true); feed(link,300);
            String afterPause=latestActive().getString("session");
            check(!beforePause.equals(afterPause), "resume changes wire session");

            int lostBefore=inactiveCount(); link.publish(frame(false)); Thread.sleep(180);
            check(inactiveCount()>lostBefore, "SDK loss invalidation");
            int quiet=received.size();
            for(int i=0;i<5;i++)link.publish(frame(false));
            Thread.sleep(160);check(received.size()==quiet, "repeated false focus does not queue stale input");
            feed(link,250); String beforeNetwork=latestActive().getString("session");
            stall=true;feed(link,400);stall=false;feed(link,650);
            check(!beforeNetwork.equals(latestActive().getString("session")), "network deadline changes wire session");

            int beforeClose=inactiveCount();link.close();Thread.sleep(260);
            check(inactiveCount()>beforeClose, "close invalidation packet");
            int stopped=received.size();link.publish(frame(true));Thread.sleep(140);
            check(received.size()==stopped,"closed link has no further frames");
            return new JSONObject().put("passed",true).put("real_http",true)
                .put("active_frames",activeCount()).put("invalidation_frames",inactiveCount())
                .put("challenge_after_latest",true).put("strict_sequence_order",true)
                .put("old_pose_not_renewed",true).put("pause_and_resume_session",true)
                .put("network_reconnect_session",true).put("repeated_focus_loss_quiet",true)
                .put("usb_loopback_transport", "127.0.0.1".equals(host))
                .put("other_loopback_addresses_rejected",true)
                .put("headset_tested",false).put("robot_commands_sent",false);
        } finally { link.close(); }
    }

    public static void main(String[] args) throws Exception {
        String host=System.getProperty("tg3.test.host");
        check(WirelessLink.validHost("127.0.0.1"),"exact ADB reverse loopback permitted");
        for (String rejected : new String[] {"127.0.0.2", "127.1.2.3", "127.255.255.255", "127.000.0.1", "localhost"})
            check(!WirelessLink.validHost(rejected),"only exact ADB reverse loopback permitted: " + rejected);
        check(WirelessLink.validHost("192.0.2.10"),"Wi-Fi IPv4 remains supported");
        check(WirelessLink.validHost(host),"test requires permitted IPv4");
        WirelessLinkJvmTest test=new WirelessLinkJvmTest(host);
        try {System.out.println(test.run(host).toString());}
        finally {test.server.stop(0);test.handlers.shutdownNow();}
    }
}
