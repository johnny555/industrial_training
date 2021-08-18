#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* MOVE ARM THROUGH PICK POSES
  Goal:
    - Use the "move_group" object to set the wrist as the end-effector link
    - Use the "move_group" object to set the planning reference frame.
    - Move the robot through the pick motion.
    - Close the gripper when appropriate.

  Hints:
    - The "move_group" interface has useful methods such as "setEndEffectorLink"
        and "setPoseReferenceFrame" that can be used to prepare the robot for planning.
    - The "setPoseTarget" method allows you to set a "pose" as your target
        to move the robot.
*/
void collision_avoidance_pick_and_place::PickAndPlaceApp::pickup_box(std::vector<geometry_msgs::msg::Pose>& pick_poses,const geometry_msgs::msg::Pose& box_pose)
{
    //RCLCPP_ERROR_STREAM(node,"pickup_box is not implemented yet.  Aborting."); exit(1);

    // task variables
    bool success;
    //std::shared_ptr<moveit_cpp::PlanningComponent> planning_component = std::make_shared(cfg.ARM_GROUP_NAME. move_group_ptr);

    /* Fill Code:
     * Goal:
     * - Set the wrist as the end-effector link
     * - The robot will try to move this link to the specified pose
     * - If not specified, moveit will use the last link in the arm group.
     * Hints:
     * - Use the "setEndEffectorLink" in the "move_group_ptr" object.
     * - The WRIST_LINK_NAME field in the "cfg" configuration member contains
     *  the name for the arm's wrist link.
     */

    //planning_component.setEndEffector(cfg.WRIST_LINK_NAME);

    // set allowed planning time
    //move_group_ptr->setPlanningTime(60.0f);


    /* Fill Code:
     * Goal:
     * - Set world frame as the reference
     * - The target position is specified relative to this frame
     * - If not specified, MoveIt will use the parent frame of the SRDF "Virtual Joint"
     * Hints:
     * - Use the "setPoseReferenceFrame" in the "move_group_ptr" object.
     * - The WORLD_FRAME_ID in the "cfg" configuration member contains the name
     * 	for the reference frame.
     */
    //move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

    // move the robot to each wrist pick pose
    for(unsigned int i = 0; i < pick_poses.size(); i++)
    {
      moveit_msgs::msg::RobotState robot_state;

    /* Inspect Code:
     * Goal:
     * - Look in the "set_attached_object()" method to understand
     * 	how to attach a payload using moveit.
     */
    set_attached_object(false,geometry_msgs::msg::Pose(),robot_state);


    /* Inspect Code:
     * Goal:
     * - Look in the "create_motion_plan()" method to observe how an
     * 	entire moveit motion plan is created.
     */
    moveit_cpp::PlanningComponent::PlanSolution plan_solution;
    success = create_motion_plan(pick_poses[i],robot_state,plan_solution) &&
        moveit_cpp->execute(cfg.ARM_GROUP_NAME, plan_solution.trajectory, true);

    // verifying move completion
    if(success)
    {
      RCLCPP_INFO_STREAM(node->get_logger(),"Pick Move " << i <<" Succeeded");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),"Pick Move " << i <<" Failed");
      set_gripper(false);
      exit(1);
    }


    if(i == 0)
    {

      /* Fill Code:
       * Goal:
       * - Turn on gripper suction after approach pose is reached.
       * Hints:
       * - Call the "set_gripper" function to turn on suction.
       * - The input to the set_gripper method takes a "true" or "false"
       *   boolean argument.
       */
      set_gripper(true);

    }

  }

}

