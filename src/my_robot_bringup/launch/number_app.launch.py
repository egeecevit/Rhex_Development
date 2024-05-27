from launch import LaunchDescription # To create a launch file we need to import LaunchDescription
from launch_ros.actions import Node # To Launch nodes we need to import Node from launch_ros.actions


def generate_launch_description(): # name of the function must be generate_launch_description
    ld = LaunchDescription() # Create a LaunchDescription object

    #remap_number_topic = ("number", "my_number") # or we can use this instead of writing it in each node
    #remappings=[remap_number_topic] like that
    number_publisher_node = Node(
        package="activity_2",
        executable="number_publisher",
        name="my_number_publisher", # For changing name of the node
        remappings=[
            ("number", "my_number") # Remap the topic name
        # Since number_publisher publishes to the topic "number" we can only change it
        ],
        parameters=[
            {"number_to_publish": 4},
            {"publish_freq": 5.0}
        # We can also pass parameters to the node
        ]
    ) # Create a Node object to launch nodes

    counter_node = Node(
        package="activity_2",
        executable="number_counter",
        name="my_number_counter", # For changing name of the node
        remappings=[
            ("number", "my_number"), # Remap the topic name
        # Since number_publisher publishes to topic "number" and 
        # we changed it to "my_number" we need to change it for subscriber too since they need to be same
        ("number_count", "my_number_count")
        ]
    )

    ld.add_action(number_publisher_node) # Add the node to LaunchDescription object
    ld.add_action(counter_node)
    return ld