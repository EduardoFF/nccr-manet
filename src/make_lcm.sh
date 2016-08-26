

lcm-gen --python  --package-prefix pose --ppath python/ lcm_msgs/pose_t.lcm lcm_msgs/pose_list_t.lcm
lcm-gen --python --package-prefix rnp --ppath python/ lcm_msgs/route2_tree_t.lcm lcm_msgs/route2_table_t.lcm lcm_msgs/route2_entry_t.lcm
lcm-gen --python --package-prefix rnp --ppath python/ lcm_msgs/flow_list_t.lcm lcm_msgs/flow_entry_t.lcm
lcm-gen --python --package-prefix plan --ppath python/ lcm_msgs/plan2_tree_t.lcm lcm_msgs/plan2_table_t.lcm lcm_msgs/plan2_entry_t.lcm
lcm-gen --python --ppath python/ lcm_msgs/plan_waypoint_t.lcm
lcm-gen --python --ppath python/ lcm_msgs/pos_gps_t.lcm
lcm-gen --python --ppath python/ lcm_msgs/endnodedebug_t.lcm
lcm-gen --python --package-prefix rnp --ppath python/ lcm_msgs/route_tree_t.lcm lcm_msgs/route_table_t.lcm lcm_msgs/route_entry_t.lcm



lcm-gen -x lcm_msgs/pose_t.lcm
lcm-gen -x lcm_msgs/route2_tree_t.lcm lcm_msgs/route2_table_t.lcm lcm_msgs/route2_entry_t.lcm
lcm-gen  -x lcm_msgs/plan2_tree_t.lcm lcm_msgs/plan2_table_t.lcm lcm_msgs/plan2_entry_t.lcm
lcm-gen -x lcm_msgs/pos_gps_t.lcm
lcm-gen  -x lcm_msgs/flow_list_t.lcm lcm_msgs/flow_entry_t.lcm
lcm-gen -x lcm_msgs/endnodedebug_t.lcm

