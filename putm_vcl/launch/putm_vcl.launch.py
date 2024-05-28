import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            # can nodes
            launch_ros.actions.Node(
                package="putm_vcl", executable="can_rx_node", name="can_rx_node"
            ),
            launch_ros.actions.Node(
                package="putm_vcl", executable="can_tx_node", name="can_tx_node"
            ),
            # amk node
            launch_ros.actions.Node(
                package="putm_vcl", executable="amk_node", name="amk_node"
            ),
            # rtd node
            launch_ros.actions.Node(
                package="putm_vcl", executable="rtd_node", name="rtd_node"
            ),
        ]
    )
