"""Report software, admission and physical acceptance as separate facts."""
from . import __version__

MANUFACTURER_MISSING=(
    '本机厂家双臂控制权申请／确认／释放的真实驱动及接口依据',
    '本机厂家保持／停止恢复接口与独立物理急停反馈',
    '当前控制器版本、实时双臂反馈与原厂仲裁规则确认',
    '需要输出的左右灵巧手实测通道标定、速度及电流限值',
)


def readiness(*,tracking=None,native_ik=False,ros_feedback=False,tools_calibrated=False,
              headset_build=False,hardware_acceptance=False,manufacturer=None,end_effector=None):
    state=tracking or {}
    wireless=bool(state.get('head_tracked') and state.get('left_tracked') and state.get('right_tracked'))
    released=not state.get('left_release_required',True) and not state.get('right_release_required',True)
    missing=[]
    if not headset_build:missing.append('等待原生头显连接与协议确认')
    if not wireless:missing.append('等待头部及双手新鲜跟踪')
    if not native_ik:missing.append('原生 Placo 尚未就绪')
    if not ros_feedback:missing.append('机器人实时反馈尚未就绪')
    reference_ready=bool(end_effector.get('ready')) if end_effector is not None else bool(tools_calibrated)
    if not reference_ready:missing.append('末端参考未确认；可选择模型TCP或填写实测抓握点')
    available=bool(manufacturer and manufacturer.get('available'))
    enabled=bool(available and manufacturer.get('hardware_motion_enabled'))
    factory_ready=False
    output_fault=None
    if not available:
        missing.extend(MANUFACTURER_MISSING)
    else:
        if not manufacturer.get('feedback_ready'):missing.append('厂家臂／手反馈异常或过期')
        if manufacturer.get('physical_estop') is not False:missing.append('独立物理急停未确认解除')
        if manufacturer.get('state')=='fault':missing.append('厂家驱动故障：'+manufacturer.get('reason','unknown'))
        output_fault=((manufacturer.get('xarm_status') or {}).get('bounded_output') or {}).get('fault')
        if output_fault:missing.append('厂家输出故障已锁存，禁止重复接管，需停止并重启本项目：'+str(output_fault))
        factory_ready=bool(manufacturer.get('feedback_ready') and manufacturer.get('physical_estop') is False
                           and manufacturer.get('state')=='disabled' and not output_fault)
    request_ready=bool(available and factory_ready and wireless and released and native_ik and ros_feedback
                       and reference_ready and headset_build)
    if not hardware_acceptance:missing.append('无线故障、低速单／双臂及固定瓶五次现场验收尚未完成')
    return dict(version=__version__,software_input_ready=wireless,
        wireless_input_ready=wireless,native_ik_ready=bool(native_ik),
        hardware_motion_enabled=enabled,ready_to_request_motion=request_ready,
        end_effector_reference=end_effector,tool_offsets_measured=bool(tools_calibrated),
        manufacturer_adapter_available=available,
        hardware_ready=bool(hardware_acceptance and available and enabled),
        physical_acceptance_verified=bool(hardware_acceptance),blockers=missing,
        motion_enable_reason=('可显式申请厂家接管；当前尚未启用' if request_ready else
            'factory_output_fault_latched_restart_required:'+str(output_fault) if output_fault else
            manufacturer.get('reason','等待厂家确认') if available else
            '缺少本机厂家接管／急停／恢复驱动；无法通过布尔开关替代'))
