    def move_to_drop_point(self):
	
        pose = PoseStamped()
        pose.header.frame_id = "m1n6s200_link_base"
        pose.pose.position.x = 0.2175546259709541
        pose.pose.position.y = 0.18347985269448372
        pose.pose.position.z = 0.16757751444136426

        pose.pose.orientation.x = 0.6934210704552356
        pose.pose.orientation.y =  0.6589390059796749
        pose.pose.orientation.z = -0.23223137602833943
        pose.pose.orientation.w = -0.17616808290725341
        
        self.arm.set_pose_target(pose, self.end_effector_link)
        plan = self.arm.plan()
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
