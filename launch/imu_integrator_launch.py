from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
	return LaunchDescription([
		Node(
			package='imu_integrator',
			executable='imu_integrator_node',
			name='imu_integrator_node',
			namespace='/imu',
			#设为‘screen’时，将节点的输出打印到中终端屏幕
			output='screen',
			#respawn: 复位。设为‘true’时，节点停止时自动重启。默认为‘false’
			#arguments: 节点需要输入的参数
			#remappings: 重映射，将默认节点属性（如节点名称、主题名称、服务名称等），重映射为其它名称。
		)
	])
	