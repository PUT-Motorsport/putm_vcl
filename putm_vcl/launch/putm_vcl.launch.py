import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # amk bridge
        launch_ros.actions.Node(
            package='putm_vcl',
            executable='amk_rx_bridge',
            name='amk_rx_bridge'),
        launch_ros.actions.Node(
            package='putm_vcl',
            executable='amk_tx_bridge',
            name='amk_tx_bridge'),
        
        # amk node
        launch_ros.actions.Node(
            package='putm_vcl',
            executable='amk_node',
            name='amk_node'),

        # can bridge
        launch_ros.actions.Node(
            package='putm_vcl',
            executable='can_rx_bridge',
            name='can_rx_bridge'),
        launch_ros.actions.Node(
            package='putm_vcl',
            executable='can_tx_bridge',
            name='can_tx_bridge'),

        # rtd node
        launch_ros.actions.Node(
            package='putm_vcl',
            executable='rtd_node',
            name='rtd_node'),
  ])
