import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
rclpy.init();n=rclpy.create_node('ativar_giro_reabertura');c=n.create_client(SetParameters,'/destravar/set_parameters')
if c.wait_for_service(timeout_sec=90):
 req=SetParameters.Request();req.parameters=[Parameter(name='giro_abertura',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=True))]
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
 print('Giro de abertura:',f.result() if f.done() else 'sem resposta',flush=True)
n.destroy_node();rclpy.shutdown()
