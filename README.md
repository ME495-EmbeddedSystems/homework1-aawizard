# ME495 Embedded Systems Homework 1
Author: ${Your Name}
1. Use `ros2 launch turtle_control waypoints.launch.xml ` to run the code
2. The `ros2 service call  /load turtle_interfaces/srv/Waypoints "{wpoint:[{x: 1.5, y: 1.7},{x: 2.1 ,y: 9.5 },{x: 7.1 ,y: 6 },{x: 4.1 ,y: 2.5 },{x: 8.1 ,y: 1.4 },{x: 4.1 ,y: 5.2}  ]}"` service loads waypoints for the turtle to follow.
3. The `ros2 service call /toggle std_srvs/srv/Empty` starts and stops the turtle.
4. Here is a video of the turtle in action.

  [hw1.webm](https://github.com/ME495-EmbeddedSystems/homework1-aawizard/assets/58395886/89aced01-42ba-44b7-b506-801a75f24f50)


# Observation while playing ros2 bag

There is a noticeable offset between the original path and the path of the turtle following cmd_vel.
