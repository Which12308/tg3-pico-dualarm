package edu.scu.tg3.wireless;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import org.json.JSONArray;
import org.json.JSONObject;

/** Actual production Java sender against Python gateway. Synthetic poses, no Android or robot. */
public final class PythonGatewayJvmTest {
    public static void main(String[] args) throws Exception {
        String pin=new BufferedReader(new InputStreamReader(System.in,"UTF-8")).readLine();
        String token=WirelessLink.pair("127.0.0.1",pin,"SYNTHETIC_JVM_NO_HEADSET").getString("token");
        WirelessLink link=new WirelessLink("127.0.0.1",token);
        try {
            link.resume();link.windowFocus(true);
            for(int i=0;i<120;i++) {
                JSONObject pose=new JSONObject().put("position",new JSONArray().put(.2).put(1.).put(-.3))
                    .put("orientation_xyzw",new JSONArray().put(0.).put(0.).put(0.).put(1.));
                JSONObject hand=new JSONObject().put("pose",pose).put("connected",true).put("tracked",true)
                    .put("grip",i<30?0.:1.).put("trigger",0.).put("buttons",new JSONArray())
                    .put("axes",new JSONArray().put(0.).put(0.)).put("source_id","synthetic");
                JSONObject frame=new JSONObject().put("session","JVM_PYTHON_DIRECT").put("sequence",i)
                    .put("source_time_s",100.+i*.02).put("source_age_s",0.).put("source","openxr")
                    .put("focused",true).put("scene_visible",true).put("head",pose).put("left",hand).put("right",hand);
                link.publish(frame.toString());Thread.sleep(15);
            }
        } finally {link.close();Thread.sleep(260);}
        System.out.println("{\"java_sender_executed\":true,\"headset_tested\":false,\"robot_commands_sent\":false}");
    }
}
