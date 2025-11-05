include "ia_lds_2d.lua"
 
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
-- 提高响应速度和降低计算负载
POSE_GRAPH.optimize_every_n_nodes = 100 -- 减少优化频率

-- -- 调整全局定位搜索范围，这对于初始定位成功非常重要
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5. -- 默认3.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(90.) -- 默认math.rad(30.)

-- -- 调低匹配分数阈值，使得在略有变化的环境中更容易匹配
-- POSE_GRAPH.constraint_builder.min_score = 0.65
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
return options