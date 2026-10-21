"""Exercise retry sequencing without ROS or motor hardware."""
import importlib.util
from pathlib import Path
import sys
import types
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('retry', Path(__file__).parents[1]/'scripts/openarm_retry_startup.py')
retry = importlib.util.module_from_spec(spec); spec.loader.exec_module(retry)
N = types.SimpleNamespace
class Request:
    def __init__(self, **kw): self.target_state=N(id=0); self.__dict__.update(kw)
class Service: Request=Request

def run(execute, fail_config=False, mixed=False):
    calls=[]
    components=[N(name=f'openarm_{s}_hardware_interface', state=N(id=1,label='unconfigured')) for s in ('right','left')]
    if mixed: components[1].state=N(id=3,label='active')
    class Client:
        def __init__(self,name): self.name=name
        def wait_for_service(self,**kw): return True
        def call_async(self,req):
            calls.append((self.name,getattr(req,'name',None),req.target_state.id))
            if 'list_hardware' in self.name: answer=N(component=components)
            elif self.name.endswith('/enable'): answer=N(success=True)
            else: answer=N(ok=not(fail_config and req.name.endswith('left_hardware_interface') and req.target_state.id==2))
            return N(done=lambda:True,result=lambda:answer)
    node=N(create_subscription=lambda *a:None,create_client=lambda kind,name:Client(name),destroy_client=lambda c:None,destroy_node=lambda:None)
    modules={'rclpy':N(init=lambda:None,create_node=lambda n:node,spin_once=lambda *a,**k:None,spin_until_future_complete=lambda *a,**k:None,shutdown=lambda:None),
        'rclpy.qos':N(QoSProfile=lambda **k:None,DurabilityPolicy=N(TRANSIENT_LOCAL=1)),
        'controller_manager_msgs.srv':N(ListHardwareComponents=Service,SetHardwareComponentState=Service),
        'std_srvs.srv':N(SetBool=Service),'std_msgs.msg':N(String=object)}
    tick=iter(range(100))
    with patch.dict(sys.modules,modules), patch.object(sys,'argv',['retry']+(['--execute'] if execute else [])), patch.object(retry.time,'monotonic',side_effect=lambda:next(tick)), patch.object(retry.subprocess,'run') as spawn:
        result=retry.main()
        states=[c[2] for c in calls if c[0].endswith('set_hardware_component_state')]
        if not execute: assert result==0 and states==[] and not spawn.called and len(calls)==1
        elif mixed: assert result==1 and states==[] and not spawn.called and len(calls)==1
        elif fail_config: assert result==1 and states==[2,2] and not spawn.called
        else: assert result==0 and states==[2,2,3,3] and spawn.called
run(True,mixed=True)
run(False)
run(True,True)
run(True)
print('Startup retry sequencing checks passed (no hardware).')
