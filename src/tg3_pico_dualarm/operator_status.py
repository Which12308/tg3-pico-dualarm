"""Presentation only. Queue acknowledgements never grant hardware authority."""
import math


def current_failure(motion):
    failure=motion.get('last_failure') or {}
    if 'attempt_id' in failure and failure['attempt_id']!=motion.get('attempt_id'):return None
    return failure or None


def motion_view(runtime,mode,*,queued=False,stale=False,request_error=None):
    motion=runtime.get('motion',{});ready=runtime.get('readiness',{})
    state=motion.get('state');failure=current_failure(motion)
    if runtime.get('station_fault'):
        return 'fault','控制已停止：'+str(runtime['station_fault'].get('reason','未知故障'))+'；需要重新启动本项目。'
    if runtime.get('connection_error') or stale:
        if mode=='dry_run':return 'unknown','模型接收中断：'+str(runtime.get('connection_error') or '状态超时')+'；硬件输出关闭。'
        return 'unknown','机器人实时状态未确认；不要据此认为已使能或已停止。'
    if mode=='dry_run':return 'dry_run','模型验证：硬件输出关闭；录制由按钮独立控制。'
    factory=runtime.get('manufacturer') or {}
    output_fault=((factory.get('xarm_status') or {}).get('bounded_output') or {}).get('fault')
    if state=='fault' or factory.get('state')=='fault':
        return 'fault','厂家接管故障：'+str(motion.get('reason') or factory.get('reason','未知'))+'；禁止再次启用，先停止本项目并核对停止和恢复。'
    if output_fault:
        return 'fault',('已确认释放；' if factory.get('stop_confirmed') is True else '停止尚未确认；')+'输出故障已锁存：'+str(output_fault)+'。禁止重复接管，需停止并重启本项目。'
    if state=='preparing':return 'preparing','正在准备厂家模式；运动未就绪，请保持全部松键。'
    if state=='acquiring':
        if motion.get('initial_output_alignment'):
            return 'aligning','正在核对厂家初始目标；尚未允许真实机械臂输出，请保持松键。'
        return 'acquiring','正在等待厂家接管确认；尚不能用握持键跟随。'
    if state=='settling':return 'settling','正在稳定保持初始姿态；请全部松键，尚不能开始跟随。'
    if state in ('releasing','holding') and not ready.get('hardware_motion_enabled'):
        return 'stopping','正在停用；等待厂家停止、释放及原模式恢复确认。'
    if ready.get('hardware_motion_enabled') is True and motion.get('hardware_motion_enabled',True) is True and state in ('armed','holding'):
        holds=runtime.get('control',{}).get('safety',{}).get('boundary_holds',[])
        if holds:
            names='、'.join('左臂' if h['side']=='left' else '右臂' for h in holds)
            return 'enabled','厂家输出已启用：'+names+'到达保护边界，保持目标等待反馈；松开握持键暂停。'
        active=runtime.get('control',{}).get('active',{})
        names='、'.join(label for side,label in (('left','左臂'),('right','右臂')) if active.get(side))
        return 'following' if names else 'enabled','厂家输出已启用：'+(names+'正在跟随；松开握持键保持。' if names else '保持当前姿态，可按住握持键开始跟随。')
    if queued:return 'queued','启用请求已排队；尚未确认厂家使能，先不要伸手。'
    if failure and failure.get('actual_output_count',0):
        reason=failure.get('reason','未知原因')
        if reason=='initial_hold_did_not_settle':reason='初始保持稳定检查未通过'
        stopped='厂家已确认停用。' if factory.get('stop_confirmed') is True else '停止状态仍待厂家确认。'
        return 'failed','已接管并实际输出保持指令%s帧，因%s退出；%s'%(failure['actual_output_count'],reason,stopped)
    if request_error:return 'failed','启用请求未完成：'+('请求在队列中过期，未进入接管' if request_error=='motion_request_expired' else str(request_error))+'；运动未确认启用。'
    if failure:
        reason=failure.get('reason','未确认')
        translated={"tracking frame is stale or from the host clock's future":'输入年龄超限',
            'fresh_focused_head_tracking_required':'需要头显前台有效跟踪',
            'both_controllers_tracked_and_released_required':'需要双手柄跟踪正常且全部松键',
            'initial_hold_feedback_corridor_exceeded':'初始化保持偏差过大；请检查接管初始化，勿反复使能'}
        reason=translated.get(reason,reason)
        age=(failure.get('input_timing') or {}).get('total_age_s')
        if isinstance(age,(int,float)) and math.isfinite(age):reason+='（累计输入年龄 %.1fms）'%(age*1000)
        attempt=motion.get('attempt_id')
        return 'failed',('第%s次尝试未完成：'%attempt if attempt else '接管未完成：')+reason+'；运动未启用或已退出。'
    if runtime.get('manufacturer',{}).get('stop_confirmed') and (motion.get('preparation') or {}).get('restored'):
        control=runtime.get('control',{})
        last=control.get('last_control_failure') or (control if control.get('state')=='disarmed' else {})
        if last.get('reason'):
            reason=last['reason']
            interval=(last.get('failure_details') or {}).get('tick_interval_s')
            if reason=='control_tick_overrun':
                reason='控制周期超时'+('（%.1fms）'%(interval*1000) if type(interval) in (int,float) else '')
            return 'disabled','控制已停止：'+reason+'；厂家已确认停用并恢复原模式。'
        return 'disabled','已停用，厂家确认释放并恢复原模式；重新运动需显式启用。'
    return 'disabled','运动未启用；等待实时条件和操作者显式申请。'


def timing_summary(tracking,runtime,relay=None):
    def ms(value):
        return '待确认' if type(value) not in (int,float) or not math.isfinite(value) else '%.1fms'%(value*1000)
    age=ms((tracking or {}).get('sample_age_s'))
    relay_ms=(relay or {}).get('transfer_ms')
    relay_text='待测' if type(relay_ms) not in (int,float) else '%.1fms'%relay_ms
    stats=runtime.get('timing',{})
    wait=stats.get('input_receive_wait',{}).get('p95_ms')
    cycle=stats.get('cycle_processing',{}).get('p95_ms')
    fmt=lambda x:'待测' if type(x) not in (int,float) else '%.1fms'%x
    return ('机器人累计输入年龄：'+age+'；转发一轮耗时：'+relay_text+
        '；接收后等待P95：'+fmt(wait)+'；控制处理P95：'+fmt(cycle)+
        '。转发耗时含隧道及服务器处理，不等于纯网络ping或机械响应时间。')
