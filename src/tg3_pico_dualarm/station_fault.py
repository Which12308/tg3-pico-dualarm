"""Keep authenticated diagnostics available after control has been stopped."""
from copy import deepcopy


def latch_fault(gateway,controller,reason,*,details=None):
    """Disarm once, revoke targets, and require a fresh process to run again.

    Server liveness is deliberately independent of control liveness. No
    successful HTTP request or new tracking sample clears this latch.
    """
    stopped=controller.disarm('station_fault:'+str(reason))
    with gateway.lock:
        gateway.tracking.invalidate('station_fault_restart_required')
        runtime=deepcopy(gateway.runtime)
        control=dict(stopped,state='fault',reason=str(reason),restart_required=True,
                     hardware_motion_enabled=False,robot_commands_sent=False)
        if details is not None:control['failure_details']=deepcopy(details)
        runtime['control']=control
        runtime['station_fault']=dict(reason=str(reason),restart_required=True)
        runtime['motion']=dict(state='fault',reason=str(reason),hardware_motion_enabled=False,
            manufacturer_stop_acknowledged=stopped.get('manufacturer_stop_acknowledged'))
        ready=dict(runtime.get('readiness',{}))
        ready.update(hardware_motion_enabled=False,hardware_ready=False,ready_to_request_motion=False,
            blockers=list(ready.get('blockers',[]))+['station_fault_restart_required:'+str(reason)])
        runtime['readiness']=ready
        gateway.runtime=runtime
        def rejected(_request):
            return dict(accepted=False,reason='station_fault_restart_required',
                        hardware_motion_enabled=False)
        gateway.control_callback=rejected
    return control
