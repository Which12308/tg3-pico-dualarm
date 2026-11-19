// Native OpenXR client: lifecycle follows Khronos hello_xr (Apache-2.0).
// Loader, pose flags and PICO controller profiles are the vendor's actual APIs.
#define XR_USE_PLATFORM_ANDROID
#define XR_USE_GRAPHICS_API_OPENGL_ES
#include <jni.h>
#include <android/log.h>
#include <android_native_app_glue.h>
#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "tracking_math.h"
#include "stb_easy_font.h"
static_assert(tg3::kRequiredLocationFlags==(XR_SPACE_LOCATION_POSITION_VALID_BIT|XR_SPACE_LOCATION_ORIENTATION_VALID_BIT|
    XR_SPACE_LOCATION_POSITION_TRACKED_BIT|XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT),"OpenXR location flag policy mismatch");

namespace {
void Log(const std::string& text) { __android_log_print(ANDROID_LOG_INFO,"TG3OpenXR","%s",text.c_str()); }
void Check(XrResult result,const char* operation) {
    if(XR_FAILED(result))throw std::runtime_error(std::string(operation)+" failed: "+std::to_string(result));
}
#define XR(call) Check((call),#call)
double Clock() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::string Epoch() {
    static std::mt19937_64 random(std::random_device{}());
    std::ostringstream out;out<<std::hex<<random()<<random();return out.str();
}
XrPosef Identity() { XrPosef p{};p.orientation.w=1;return p; }
bool PoseOK(const XrPosef& p) {
    return tg3::FinitePose(p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
}
std::string PoseJSON(const XrPosef& p,bool valid) {
    if(!valid)return "null";
    std::ostringstream out;out<<std::setprecision(9)<<"{\"position\":["<<p.position.x<<","<<p.position.y<<","<<p.position.z
        <<"],\"orientation_xyzw\":["<<p.orientation.x<<","<<p.orientation.y<<","<<p.orientation.z<<","<<p.orientation.w<<"]}";
    return out.str();
}

struct Java {
    android_app* app;JNIEnv* env=nullptr;jmethodID frame,status,pair;
    explicit Java(android_app* value):app(value) {
        if(app->activity->vm->AttachCurrentThread(&env,nullptr)!=JNI_OK)throw std::runtime_error("AttachCurrentThread failed");
        jclass cls=env->GetObjectClass(app->activity->clazz);
        frame=env->GetMethodID(cls,"onTrackingFrame","(Ljava/lang/String;)V");
        status=env->GetMethodID(cls,"getLinkStatus","()Ljava/lang/String;");
        pair=env->GetMethodID(cls,"openPairing","()V");env->DeleteLocalRef(cls);
        if(!frame||!status||!pair)throw std::runtime_error("NativeActivity Java bridge mismatch");
    }
    ~Java(){app->activity->vm->DetachCurrentThread();}
    void Send(const std::string& text) {
        auto value=env->NewStringUTF(text.c_str());env->CallVoidMethod(app->activity->clazz,frame,value);env->DeleteLocalRef(value);
        if(env->ExceptionCheck()){env->ExceptionDescribe();env->ExceptionClear();throw std::runtime_error("Java input callback failed");}
    }
    std::string Status() {
        auto value=(jstring)env->CallObjectMethod(app->activity->clazz,status);
        if(env->ExceptionCheck()){env->ExceptionClear();return "NETWORK STATUS UNAVAILABLE";}
        if(!value)return "PAIR ON PC";
        const char* data=env->GetStringUTFChars(value,nullptr);std::string text=data?data:"";
        if(data)env->ReleaseStringUTFChars(value,data);env->DeleteLocalRef(value);return text;
    }
    void Pair(){env->CallVoidMethod(app->activity->clazz,pair);if(env->ExceptionCheck())env->ExceptionClear();}
};

struct Graphics {
    EGLDisplay display=EGL_NO_DISPLAY;EGLConfig config{};EGLContext context=EGL_NO_CONTEXT;EGLSurface surface=EGL_NO_SURFACE;
    GLuint shader=0,buffer=0,indices=0,vao=0,fbo=0;
    void Initialize() {
        display=eglGetDisplay(EGL_DEFAULT_DISPLAY);EGLint major,minor;
        if(!eglInitialize(display,&major,&minor))throw std::runtime_error("eglInitialize failed");
        EGLint options[]={EGL_SURFACE_TYPE,EGL_PBUFFER_BIT,EGL_RENDERABLE_TYPE,EGL_OPENGL_ES3_BIT,
            EGL_RED_SIZE,8,EGL_GREEN_SIZE,8,EGL_BLUE_SIZE,8,EGL_ALPHA_SIZE,8,EGL_NONE};
        EGLint count=0;if(!eglChooseConfig(display,options,&config,1,&count)||count<1)throw std::runtime_error("No GLES3 alpha EGL config");
        EGLint attributes[]={EGL_CONTEXT_CLIENT_VERSION,3,EGL_NONE};context=eglCreateContext(display,config,EGL_NO_CONTEXT,attributes);
        EGLint dimensions[]={EGL_WIDTH,1,EGL_HEIGHT,1,EGL_NONE};surface=eglCreatePbufferSurface(display,config,dimensions);
        if(context==EGL_NO_CONTEXT||surface==EGL_NO_SURFACE||!eglMakeCurrent(display,surface,surface,context))
            throw std::runtime_error("GLES3 context initialization failed");
        const char* vertex="#version 300 es\nlayout(location=0) in vec2 p; uniform vec2 scale; void main(){gl_Position=vec4(-.72+p.x*scale.x,-.50-p.y*scale.y,0,1);}";
        const char* fragment="#version 300 es\nprecision mediump float;out vec4 c;void main(){c=vec4(.15,1.,.45,1.);}";
        auto compile=[](GLenum type,const char* source) {
            GLuint s=glCreateShader(type);glShaderSource(s,1,&source,nullptr);glCompileShader(s);GLint ok=0;glGetShaderiv(s,GL_COMPILE_STATUS,&ok);
            if(!ok){char log[1024];glGetShaderInfoLog(s,sizeof(log),nullptr,log);throw std::runtime_error(std::string("HUD shader: ")+log);}return s;
        };
        GLuint vs=compile(GL_VERTEX_SHADER,vertex),fs=compile(GL_FRAGMENT_SHADER,fragment);
        shader=glCreateProgram();glAttachShader(shader,vs);glAttachShader(shader,fs);glLinkProgram(shader);glDeleteShader(vs);glDeleteShader(fs);
        GLint linked=0;glGetProgramiv(shader,GL_LINK_STATUS,&linked);if(!linked)throw std::runtime_error("HUD shader link failed");
        glGenVertexArrays(1,&vao);glGenBuffers(1,&buffer);glGenBuffers(1,&indices);glGenFramebuffers(1,&fbo);
    }
    void Render(uint32_t texture,int width,int height,const std::string& text) {
        glBindFramebuffer(GL_FRAMEBUFFER,fbo);glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,texture,0);
        if(glCheckFramebufferStatus(GL_FRAMEBUFFER)!=GL_FRAMEBUFFER_COMPLETE)throw std::runtime_error("Swapchain framebuffer incomplete");
        glViewport(0,0,width,height);glDisable(GL_DEPTH_TEST);glDisable(GL_SCISSOR_TEST);glDisable(GL_CULL_FACE);
        glClearColor(0,0,0,0);glClear(GL_COLOR_BUFFER_BIT);
        std::array<char,100000> vertices{};std::string bounded=text.substr(0,600);
        int quads=stb_easy_font_print(0,0,const_cast<char*>(bounded.c_str()),nullptr,vertices.data(),vertices.size());
        std::vector<uint16_t> order;order.reserve(quads*6);
        for(int i=0;i<quads;++i){uint16_t p=static_cast<uint16_t>(i*4);order.insert(order.end(),{p,(uint16_t)(p+1),(uint16_t)(p+2),p,(uint16_t)(p+2),(uint16_t)(p+3)});}
        glUseProgram(shader);glUniform2f(glGetUniformLocation(shader,"scale"),.0038f,.0038f*width/height);
        glBindVertexArray(vao);glBindBuffer(GL_ARRAY_BUFFER,buffer);glBufferData(GL_ARRAY_BUFFER,quads*4*16,vertices.data(),GL_STREAM_DRAW);
        glEnableVertexAttribArray(0);glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,16,nullptr);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,indices);glBufferData(GL_ELEMENT_ARRAY_BUFFER,order.size()*sizeof(uint16_t),order.data(),GL_STREAM_DRAW);
        glDrawElements(GL_TRIANGLES,order.size(),GL_UNSIGNED_SHORT,nullptr);glBindFramebuffer(GL_FRAMEBUFFER,0);glFlush();
    }
    ~Graphics() {
        if(display!=EGL_NO_DISPLAY){if(shader)glDeleteProgram(shader);if(buffer)glDeleteBuffers(1,&buffer);if(indices)glDeleteBuffers(1,&indices);
            if(vao)glDeleteVertexArrays(1,&vao);if(fbo)glDeleteFramebuffers(1,&fbo);
            eglMakeCurrent(display,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);
            if(surface!=EGL_NO_SURFACE)eglDestroySurface(display,surface);if(context!=EGL_NO_CONTEXT)eglDestroyContext(display,context);eglTerminate(display);}
    }
};

struct Hand {
    XrSpace space=XR_NULL_HANDLE;XrSpaceLocation location{XR_TYPE_SPACE_LOCATION};bool connected=false,tracked=false;
    float grip=0,trigger=0;bool primary=false,secondary=false;XrVector2f axis{};unsigned generation=0;
};
struct Swapchain { XrSwapchain handle=XR_NULL_HANDLE;int width=0,height=0;std::vector<XrSwapchainImageOpenGLESKHR> images; };

class Client {
    android_app* app;Java java;Graphics graphics;
    XrInstance instance=XR_NULL_HANDLE;XrSystemId system=XR_NULL_SYSTEM_ID;XrSession session=XR_NULL_HANDLE;
    XrSpace local=XR_NULL_HANDLE,view=XR_NULL_HANDLE;XrActionSet actions=XR_NULL_HANDLE;
    XrAction poseAction{},gripAction{},triggerAction{},primaryAction{},secondaryAction{},axisAction{},menuAction{};
    std::array<XrPath,2> handPaths{};std::array<Hand,2> hands{};std::vector<Swapchain> swapchains;
    std::vector<XrView> views;std::string epoch=Epoch();uint64_t sequence=0;
    XrSessionState state=XR_SESSION_STATE_UNKNOWN;bool running=false,exiting=false,focused=false,menuDown=false;
    XrEnvironmentBlendMode blendMode=XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    bool passthroughReady=false,sceneVisible=false;XrTime relocalizingUntil=0;std::string profileStatus="profiles not selected";
    XrPassthroughFB passthrough=XR_NULL_HANDLE;XrPassthroughLayerFB passthroughLayer=XR_NULL_HANDLE;
    PFN_xrCreatePassthroughFB createPassthrough{};PFN_xrDestroyPassthroughFB destroyPassthrough{};
    PFN_xrPassthroughStartFB startPassthrough{};PFN_xrCreatePassthroughLayerFB createLayer{};
    PFN_xrDestroyPassthroughLayerFB destroyLayer{};PFN_xrPassthroughLayerResumeFB resumeLayer{};
    template<class T> void Function(T& destination,const char* name) {
        XR(xrGetInstanceProcAddr(instance,name,reinterpret_cast<PFN_xrVoidFunction*>(&destination)));
    }
    XrPath Path(const std::string& name) {XrPath result;XR(xrStringToPath(instance,name.c_str(),&result));return result;}
    XrAction Action(const char* name,const char* label,XrActionType type) {
        XrActionCreateInfo info{XR_TYPE_ACTION_CREATE_INFO};std::strncpy(info.actionName,name,sizeof(info.actionName)-1);
        std::strncpy(info.localizedActionName,label,sizeof(info.localizedActionName)-1);info.actionType=type;
        info.countSubactionPaths=2;info.subactionPaths=handPaths.data();XrAction result;XR(xrCreateAction(actions,&info,&result));return result;
    }
    void NewEpoch(){epoch=Epoch();sequence=0;for(auto& h:hands){h.tracked=false;h.generation++;}}
    void Invalid(){focused=false;sceneVisible=false;Emit(false,Identity(),false);}
    void Initialize() {
        PFN_xrInitializeLoaderKHR initializeLoader{};XR(xrGetInstanceProcAddr(XR_NULL_HANDLE,"xrInitializeLoaderKHR",reinterpret_cast<PFN_xrVoidFunction*>(&initializeLoader)));
        XrLoaderInitInfoAndroidKHR loader{XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};loader.applicationVM=app->activity->vm;loader.applicationContext=app->activity->clazz;
        XR(initializeLoader(reinterpret_cast<XrLoaderInitInfoBaseHeaderKHR*>(&loader)));
        uint32_t count=0;XR(xrEnumerateInstanceExtensionProperties(nullptr,0,&count,nullptr));
        std::vector<XrExtensionProperties> extensions(count,{XR_TYPE_EXTENSION_PROPERTIES});XR(xrEnumerateInstanceExtensionProperties(nullptr,count,&count,extensions.data()));
        auto supported=[&](const char* name){return std::any_of(extensions.begin(),extensions.end(),[&](const auto& p){return std::strcmp(p.extensionName,name)==0;});};
        std::vector<const char*> enabled={XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME};
        if(!supported(XR_FB_PASSTHROUGH_EXTENSION_NAME))throw std::runtime_error("XR_FB_passthrough unavailable: input disabled");
        enabled.push_back(XR_FB_PASSTHROUGH_EXTENSION_NAME);
        for(const char* optional:{"XR_BD_controller_interaction","XR_BD_ultra_controller_interaction"})if(supported(optional))enabled.push_back(optional);
        XrInstanceCreateInfoAndroidKHR android{XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};android.applicationVM=app->activity->vm;android.applicationActivity=app->activity->clazz;
        XrInstanceCreateInfo info{XR_TYPE_INSTANCE_CREATE_INFO};info.next=&android;std::strcpy(info.applicationInfo.applicationName,"TG3 Wireless");
        std::strcpy(info.applicationInfo.engineName,"Khronos OpenXR native");info.applicationInfo.applicationVersion=400;info.applicationInfo.apiVersion=XR_API_VERSION_1_0;
        info.enabledExtensionCount=enabled.size();info.enabledExtensionNames=enabled.data();XR(xrCreateInstance(&info,&instance));
        XrSystemGetInfo get{XR_TYPE_SYSTEM_GET_INFO};get.formFactor=XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;XR(xrGetSystem(instance,&get,&system));
        PFN_xrGetOpenGLESGraphicsRequirementsKHR requirements{};Function(requirements,"xrGetOpenGLESGraphicsRequirementsKHR");
        XrGraphicsRequirementsOpenGLESKHR limits{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};XR(requirements(instance,system,&limits));
        graphics.Initialize();
        GLint glMajor=0,glMinor=0;glGetIntegerv(GL_MAJOR_VERSION,&glMajor);glGetIntegerv(GL_MINOR_VERSION,&glMinor);
        XrVersion graphicsVersion=XR_MAKE_VERSION(glMajor,glMinor,0);
        if(graphicsVersion<limits.minApiVersionSupported||graphicsVersion>limits.maxApiVersionSupported)
            throw std::runtime_error("GLES version outside OpenXR runtime graphics requirements");
        XrGraphicsBindingOpenGLESAndroidKHR binding{XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};binding.display=graphics.display;binding.config=graphics.config;binding.context=graphics.context;
        XrSessionCreateInfo sessionInfo{XR_TYPE_SESSION_CREATE_INFO};sessionInfo.next=&binding;sessionInfo.systemId=system;XR(xrCreateSession(instance,&sessionInfo,&session));
        XrReferenceSpaceCreateInfo spaceInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};spaceInfo.poseInReferenceSpace=Identity();spaceInfo.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_LOCAL;
        XR(xrCreateReferenceSpace(session,&spaceInfo,&local));spaceInfo.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_VIEW;XR(xrCreateReferenceSpace(session,&spaceInfo,&view));
        InitializeActions();InitializeSwapchains();
        Function(createPassthrough,"xrCreatePassthroughFB");Function(destroyPassthrough,"xrDestroyPassthroughFB");Function(startPassthrough,"xrPassthroughStartFB");
        Function(createLayer,"xrCreatePassthroughLayerFB");Function(destroyLayer,"xrDestroyPassthroughLayerFB");Function(resumeLayer,"xrPassthroughLayerResumeFB");
        XrPassthroughCreateInfoFB passInfo{XR_TYPE_PASSTHROUGH_CREATE_INFO_FB};XR(createPassthrough(session,&passInfo,&passthrough));
        XrPassthroughLayerCreateInfoFB layerInfo{XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB};layerInfo.passthrough=passthrough;layerInfo.purpose=XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB;
        XR(createLayer(session,&layerInfo,&passthroughLayer));XR(startPassthrough(passthrough));XR(resumeLayer(passthroughLayer));passthroughReady=true;
        Log("OpenXR initialized; passthrough active; no robot authority");
    }
    void InitializeActions() {
        handPaths={Path("/user/hand/left"),Path("/user/hand/right")};
        XrActionSetCreateInfo set{XR_TYPE_ACTION_SET_CREATE_INFO};std::strcpy(set.actionSetName,"tg3_input");std::strcpy(set.localizedActionSetName,"TG3 controller input");
        XR(xrCreateActionSet(instance,&set,&actions));
        poseAction=Action("hand_pose","Hand grip pose",XR_ACTION_TYPE_POSE_INPUT);
        gripAction=Action("grip","Grip clutch",XR_ACTION_TYPE_FLOAT_INPUT);triggerAction=Action("trigger","Grasp trigger",XR_ACTION_TYPE_FLOAT_INPUT);
        primaryAction=Action("primary","X and A",XR_ACTION_TYPE_BOOLEAN_INPUT);secondaryAction=Action("secondary","Y and B open",XR_ACTION_TYPE_BOOLEAN_INPUT);
        axisAction=Action("stick","Thumbstick",XR_ACTION_TYPE_VECTOR2F_INPUT);menuAction=Action("menu","Pairing menu",XR_ACTION_TYPE_BOOLEAN_INPUT);
        int accepted=0;
        for(const char* profile:{"/interaction_profiles/bytedance/pico_ultra_controller_bd","/interaction_profiles/bytedance/pico4s_controller","/interaction_profiles/bytedance/pico4_controller",
                                "/interaction_profiles/bytedance/pico_neo3_controller","/interaction_profiles/oculus/touch_controller"}) {
            std::vector<XrActionSuggestedBinding> bindings;
            for(int i=0;i<2;i++){
                std::string prefix=i?"/user/hand/right/input/":"/user/hand/left/input/";
                bindings.push_back({poseAction,Path(prefix+"grip/pose")});bindings.push_back({gripAction,Path(prefix+"squeeze/value")});
                bindings.push_back({triggerAction,Path(prefix+"trigger/value")});bindings.push_back({primaryAction,Path(prefix+(i?"a/click":"x/click"))});
                bindings.push_back({secondaryAction,Path(prefix+(i?"b/click":"y/click"))});bindings.push_back({axisAction,Path(prefix+"thumbstick")});
                if(i==0)bindings.push_back({menuAction,Path(prefix+"menu/click")});
            }
            XrInteractionProfileSuggestedBinding suggest{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};suggest.interactionProfile=Path(profile);
            suggest.countSuggestedBindings=bindings.size();suggest.suggestedBindings=bindings.data();
            XrResult result=xrSuggestInteractionProfileBindings(instance,&suggest);
            if(XR_SUCCEEDED(result)){accepted++;Log(std::string("Accepted controller bindings: ")+profile);}
            else Log(std::string("Profile unavailable: ")+profile+" code="+std::to_string(result));
        }
        if(!accepted)throw std::runtime_error("No supported PICO/Touch analog controller binding");
        XrActionSpaceCreateInfo space{XR_TYPE_ACTION_SPACE_CREATE_INFO};space.action=poseAction;space.poseInActionSpace=Identity();
        for(int i=0;i<2;i++){space.subactionPath=handPaths[i];XR(xrCreateActionSpace(session,&space,&hands[i].space));}
        XrSessionActionSetsAttachInfo attach{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};attach.countActionSets=1;attach.actionSets=&actions;XR(xrAttachSessionActionSets(session,&attach));
        profileStatus=std::to_string(accepted)+" controller profiles available";
    }
    void InitializeSwapchains() {
        uint32_t blendCount=0;XR(xrEnumerateEnvironmentBlendModes(instance,system,XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,0,&blendCount,nullptr));
        std::vector<XrEnvironmentBlendMode> modes(blendCount);XR(xrEnumerateEnvironmentBlendModes(instance,system,XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,blendCount,&blendCount,modes.data()));
        if(std::find(modes.begin(),modes.end(),XR_ENVIRONMENT_BLEND_MODE_OPAQUE)!=modes.end())blendMode=XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        else if(std::find(modes.begin(),modes.end(),XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND)!=modes.end())blendMode=XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND;
        else throw std::runtime_error("No supported passthrough environment blend mode");
        uint32_t count=0;XR(xrEnumerateViewConfigurationViews(instance,system,XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,0,&count,nullptr));
        if(count!=2)throw std::runtime_error("Stereo view configuration required");
        std::vector<XrViewConfigurationView> settings(count,{XR_TYPE_VIEW_CONFIGURATION_VIEW});
        XR(xrEnumerateViewConfigurationViews(instance,system,XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,count,&count,settings.data()));
        uint32_t formatsCount=0;XR(xrEnumerateSwapchainFormats(session,0,&formatsCount,nullptr));std::vector<int64_t> formats(formatsCount);
        XR(xrEnumerateSwapchainFormats(session,formatsCount,&formatsCount,formats.data()));
        int64_t selected=0;for(int64_t candidate:{(int64_t)GL_SRGB8_ALPHA8,(int64_t)GL_RGBA8})
            if(!selected&&std::find(formats.begin(),formats.end(),candidate)!=formats.end())selected=candidate;
        if(!selected)throw std::runtime_error("No alpha OpenGLES swapchain format");
        for(const auto& setting:settings){Swapchain item;item.width=setting.recommendedImageRectWidth;item.height=setting.recommendedImageRectHeight;
            XrSwapchainCreateInfo info{XR_TYPE_SWAPCHAIN_CREATE_INFO};info.usageFlags=XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT|XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
            info.format=selected;info.sampleCount=1;info.width=item.width;info.height=item.height;info.faceCount=1;info.arraySize=1;info.mipCount=1;
            XR(xrCreateSwapchain(session,&info,&item.handle));uint32_t imageCount=0;XR(xrEnumerateSwapchainImages(item.handle,0,&imageCount,nullptr));
            item.images.resize(imageCount,{XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR});
            XR(xrEnumerateSwapchainImages(item.handle,imageCount,&imageCount,reinterpret_cast<XrSwapchainImageBaseHeader*>(item.images.data())));swapchains.push_back(item);}
        views.resize(2,{XR_TYPE_VIEW});
    }
    void PollEvents() {
        XrEventDataBuffer buffer{XR_TYPE_EVENT_DATA_BUFFER};
        XrResult result;
        while((result=xrPollEvent(instance,&buffer))==XR_SUCCESS){
            if(buffer.type==XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED){
                const auto& event=*reinterpret_cast<XrEventDataSessionStateChanged*>(&buffer);state=event.state;
                focused=state==XR_SESSION_STATE_FOCUSED;
                if(!focused){NewEpoch();Invalid();}
                if(state==XR_SESSION_STATE_READY){XrSessionBeginInfo info{XR_TYPE_SESSION_BEGIN_INFO};info.primaryViewConfigurationType=XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;XR(xrBeginSession(session,&info));running=true;NewEpoch();}
                if(state==XR_SESSION_STATE_STOPPING){running=false;XR(xrEndSession(session));}
                if(state==XR_SESSION_STATE_EXITING||state==XR_SESSION_STATE_LOSS_PENDING)exiting=true;
            }else if(buffer.type==XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING){exiting=true;Invalid();}
            else if(buffer.type==XR_TYPE_EVENT_DATA_PASSTHROUGH_STATE_CHANGED_FB){
                const auto& event=*reinterpret_cast<XrEventDataPassthroughStateChangedFB*>(&buffer);
                // Error bits win over RESTORED if the runtime combines flags.
                // A stale created layer must never certify continuing visibility.
                passthroughReady=false;NewEpoch();Invalid();
                if(event.flags&XR_PASSTHROUGH_STATE_CHANGED_NON_RECOVERABLE_ERROR_BIT_FB)
                    throw std::runtime_error("Passthrough non-recoverable error: input stopped; restart required");
                if(event.flags&XR_PASSTHROUGH_STATE_CHANGED_REINIT_REQUIRED_BIT_FB)
                    throw std::runtime_error("Passthrough reinitialization required: input stopped; reconnect to restart");
                if(event.flags&XR_PASSTHROUGH_STATE_CHANGED_RECOVERABLE_ERROR_BIT_FB)
                    Log("Passthrough temporarily unavailable; input invalidated until RESTORED");
                else if(event.flags==XR_PASSTHROUGH_STATE_CHANGED_RESTORED_ERROR_BIT_FB){
                    passthroughReady=true;
                    Log("Passthrough restored; new source epoch requires released controls");
                }else throw std::runtime_error("Unknown passthrough state event: input stopped");
            }
            else if(buffer.type==XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING){
                auto& event=*reinterpret_cast<XrEventDataReferenceSpaceChangePending*>(&buffer);
                if(event.referenceSpaceType==XR_REFERENCE_SPACE_TYPE_LOCAL){relocalizingUntil=event.changeTime;NewEpoch();Invalid();}
            }else if(buffer.type==XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED){NewEpoch();Invalid();}
            buffer={XR_TYPE_EVENT_DATA_BUFFER};
        }
        if(result!=XR_EVENT_UNAVAILABLE)Check(result,"xrPollEvent");
    }
    float Float(XrAction action,int side,bool& valid) {
        XrActionStateGetInfo get{XR_TYPE_ACTION_STATE_GET_INFO};get.action=action;get.subactionPath=handPaths[side];
        XrActionStateFloat result{XR_TYPE_ACTION_STATE_FLOAT};XR(xrGetActionStateFloat(session,&get,&result));
        valid=valid&&result.isActive&&std::isfinite(result.currentState)&&result.currentState>=0&&result.currentState<=1;
        return tg3::Analog(result.currentState);
    }
    bool Button(XrAction action,int side,bool* valid=nullptr) {
        XrActionStateGetInfo get{XR_TYPE_ACTION_STATE_GET_INFO};get.action=action;get.subactionPath=handPaths[side];
        XrActionStateBoolean result{XR_TYPE_ACTION_STATE_BOOLEAN};XR(xrGetActionStateBoolean(session,&get,&result));
        if(valid)*valid=*valid&&result.isActive;return result.isActive&&result.currentState;
    }
    bool ReadInput(XrTime when,bool usable) {
        XrActiveActionSet active{actions,XR_NULL_PATH};XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};sync.countActiveActionSets=1;sync.activeActionSets=&active;
        XrResult result=xrSyncActions(session,&sync);
        if(result==XR_SESSION_NOT_FOCUSED){for(auto& h:hands)h.tracked=false;return false;}Check(result,"xrSyncActions");
        for(int i=0;i<2;i++){
            auto& h=hands[i];XrActionStateGetInfo get{XR_TYPE_ACTION_STATE_GET_INFO};get.action=poseAction;get.subactionPath=handPaths[i];
            XrActionStatePose pose{XR_TYPE_ACTION_STATE_POSE};XR(xrGetActionStatePose(session,&get,&pose));h.connected=pose.isActive;
            h.location={XR_TYPE_SPACE_LOCATION};XR(xrLocateSpace(h.space,local,when,&h.location));
            bool controls=true;h.grip=Float(gripAction,i,controls);h.trigger=Float(triggerAction,i,controls);
            h.primary=Button(primaryAction,i,&controls);h.secondary=Button(secondaryAction,i,&controls);
            get.action=axisAction;XrActionStateVector2f axis{XR_TYPE_ACTION_STATE_VECTOR2F};XR(xrGetActionStateVector2f(session,&get,&axis));
            h.axis=axis.isActive?axis.currentState:XrVector2f{};
            controls=controls&&axis.isActive;
            bool valid=tg3::Tracked(h.location.locationFlags,pose.isActive,usable)&&controls&&PoseOK(h.location.pose)&&
                std::isfinite(h.axis.x)&&std::isfinite(h.axis.y)&&std::fabs(h.axis.x)<=1&&std::fabs(h.axis.y)<=1;
            if(valid!=h.tracked&&valid)h.generation++;
            h.tracked=valid;
        }
        bool menu=Button(menuAction,0);bool opening=menu&&!menuDown;
        if(opening){Invalid();java.Pair();}menuDown=menu;return !opening;
    }
    std::string HandJSON(int side,bool usable) {
        const auto& h=hands[side];bool valid=usable&&h.tracked;
        std::ostringstream out;out<<std::setprecision(9)<<"{\"pose\":"<<PoseJSON(h.location.pose,valid)<<",\"connected\":"<<(h.connected?"true":"false")
            <<",\"tracked\":"<<(valid?"true":"false")<<",\"grip\":"<<(valid?h.grip:0)<<",\"trigger\":"<<(valid?h.trigger:0)<<",\"buttons\":[";
        bool comma=false;if(valid&&h.primary){out<<'"'<<(side?"A":"X")<<'"';comma=true;}
        if(valid&&h.secondary){if(comma)out<<',';out<<'"'<<(side?"B":"Y")<<'"';}
        out<<"],\"axes\":["<<(valid?std::clamp(h.axis.x,-1.f,1.f):0)<<","<<(valid?std::clamp(h.axis.y,-1.f,1.f):0)
           <<"],\"source_id\":\"openxr-"<<side<<"-"<<h.generation<<"\"}";return out.str();
    }
    void Emit(bool usable,const XrPosef& head,bool headValid) {
        usable=usable&&headValid&&sceneVisible;
        std::ostringstream out;out<<std::setprecision(17)<<"{\"session\":\"openxr-"<<epoch<<"\",\"sequence\":"<<sequence++
            <<",\"source_time_s\":"<<Clock()<<",\"source_age_s\":0,\"source\":\"openxr\",\"focused\":"<<(usable?"true":"false")
            <<",\"scene_visible\":"<<(sceneVisible?"true":"false")<<",\"head\":"<<PoseJSON(head,usable)
            <<",\"left\":"<<HandJSON(0,usable)<<",\"right\":"<<HandJSON(1,usable)<<"}";
        java.Send(out.str());
    }
    void Frame() {
        XrFrameWaitInfo wait{XR_TYPE_FRAME_WAIT_INFO};XrFrameState timing{XR_TYPE_FRAME_STATE};XR(xrWaitFrame(session,&wait,&timing));
        XrFrameBeginInfo begin{XR_TYPE_FRAME_BEGIN_INFO};XR(xrBeginFrame(session,&begin));
        if(relocalizingUntil&&timing.predictedDisplayTime>=relocalizingUntil){relocalizingUntil=0;NewEpoch();}
        sceneVisible=passthroughReady&&timing.shouldRender&&state==XR_SESSION_STATE_FOCUSED;
        bool usable=sceneVisible&&!relocalizingUntil;
        usable=ReadInput(timing.predictedDisplayTime,usable)&&usable;
        XrSpaceLocation head{XR_TYPE_SPACE_LOCATION};XR(xrLocateSpace(view,local,timing.predictedDisplayTime,&head));
        bool headValid=tg3::Tracked(head.locationFlags,true,usable)&&PoseOK(head.pose);Emit(usable,head.pose,headValid);
        std::string hud="TG3 WIRELESS / OPENXR 0.4\n"+java.Status().substr(0,95)+"\n";
        hud+="HEAD "+std::string(headValid?"TRACKED":"LOST")+"  LEFT "+(hands[0].tracked?"TRACKED":"LOST")+"  RIGHT "+(hands[1].tracked?"TRACKED":"LOST")+"\n";
        char details[200];std::snprintf(details,sizeof(details),"L grip %.2f trigger %.2f  R grip %.2f trigger %.2f\n",hands[0].grip,hands[0].trigger,hands[1].grip,hands[1].trigger);hud+=details;
        hud+="GRIP follow / release hold; TRIGGER close+retain\nY / B open hand; LEFT MENU pairing\nRelease BOTH grips before walking\nRobot motion requires explicit enable on PC";
        std::array<XrCompositionLayerProjectionView,2> projectionViews{};XrCompositionLayerProjection projection{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        std::vector<const XrCompositionLayerBaseHeader*> layers;
        XrCompositionLayerPassthroughFB background{XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB};background.layerHandle=passthroughLayer;
        if(timing.shouldRender&&passthroughReady)layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader*>(&background));
        if(timing.shouldRender){
            XrViewLocateInfo locate{XR_TYPE_VIEW_LOCATE_INFO};locate.viewConfigurationType=XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;locate.displayTime=timing.predictedDisplayTime;locate.space=local;
            XrViewState viewState{XR_TYPE_VIEW_STATE};uint32_t count=0;XR(xrLocateViews(session,&locate,&viewState,views.size(),&count,views.data()));
            if(count==2&&(viewState.viewStateFlags&XR_VIEW_STATE_ORIENTATION_VALID_BIT)&&(viewState.viewStateFlags&XR_VIEW_STATE_POSITION_VALID_BIT)){
                for(int i=0;i<2;i++){auto& chain=swapchains[i];uint32_t index=0;XrSwapchainImageAcquireInfo acquire{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};XR(xrAcquireSwapchainImage(chain.handle,&acquire,&index));
                    XrSwapchainImageWaitInfo imageWait{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};imageWait.timeout=XR_INFINITE_DURATION;XR(xrWaitSwapchainImage(chain.handle,&imageWait));
                    graphics.Render(chain.images[index].image,chain.width,chain.height,hud);
                    XrSwapchainImageReleaseInfo release{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};XR(xrReleaseSwapchainImage(chain.handle,&release));
                    auto& item=projectionViews[i];item={XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};item.pose=views[i].pose;item.fov=views[i].fov;
                    item.subImage.swapchain=chain.handle;item.subImage.imageRect.extent={chain.width,chain.height};
                }
                projection.layerFlags=XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;projection.space=local;projection.viewCount=2;projection.views=projectionViews.data();
                layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader*>(&projection));
            }
        }
        XrFrameEndInfo end{XR_TYPE_FRAME_END_INFO};end.displayTime=timing.predictedDisplayTime;end.environmentBlendMode=blendMode;
        end.layerCount=layers.size();end.layers=layers.data();XR(xrEndFrame(session,&end));
    }
public:
    explicit Client(android_app* value):app(value),java(value){}
    void Run() {
        while(!app->window&&!app->destroyRequested){
            int events;android_poll_source* source=nullptr;
            if(ALooper_pollOnce(100,nullptr,&events,reinterpret_cast<void**>(&source))>=0&&source)source->process(app,source);
        }
        if(app->destroyRequested)return;
        Initialize();
        while(!app->destroyRequested&&!exiting){
            int events;android_poll_source* source;
            while(ALooper_pollOnce(running?0:100,nullptr,&events,reinterpret_cast<void**>(&source))>=0){if(source)source->process(app,source);if(app->destroyRequested)break;}
            if(app->destroyRequested)break;PollEvents();if(running)Frame();
        }
        Invalid();
    }
    ~Client(){
        try{Invalid();}catch(...){}
        if(passthroughLayer&&destroyLayer)destroyLayer(passthroughLayer);if(passthrough&&destroyPassthrough)destroyPassthrough(passthrough);
        for(auto& chain:swapchains)if(chain.handle)xrDestroySwapchain(chain.handle);
        for(auto& h:hands)if(h.space)xrDestroySpace(h.space);if(view)xrDestroySpace(view);if(local)xrDestroySpace(local);
        if(session)xrDestroySession(session);if(actions)xrDestroyActionSet(actions);if(instance)xrDestroyInstance(instance);
    }
};
}

extern "C" void android_main(android_app* app) {
    app_dummy();
    try {Client client(app);client.Run();}
    catch(const std::exception& error){Log(std::string("Native XR stopped: ")+error.what());
        JNIEnv* env=nullptr;if(app->activity->vm->AttachCurrentThread(&env,nullptr)==JNI_OK){
            auto cls=env->GetObjectClass(app->activity->clazz);auto method=env->GetMethodID(cls,"onNativeError","(Ljava/lang/String;)V");
            if(method){auto text=env->NewStringUTF(error.what());env->CallVoidMethod(app->activity->clazz,method,text);env->DeleteLocalRef(text);}
            env->DeleteLocalRef(cls);if(env->ExceptionCheck())env->ExceptionClear();app->activity->vm->DetachCurrentThread();
        }
    }
}
