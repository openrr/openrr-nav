// How to run:
//
// ```sh
// cargo run --release -p openrr-nav-viewer --example dwa_gui
// ```

mod shared;

use clap::Parser;
use grid_map::*;
use openrr_nav::{utils::nearest_path_point, *};
use openrr_nav_viewer::*;
use rand::distributions::{Distribution, Uniform};
use shared::*;

fn main() {
    let nav: NavigationViz = Args::parse().try_into().unwrap();

    let cloned_nav = nav.clone();

    let planner = DwaPlanner::new_from_config(format!(
        "{}/../openrr-nav/config/dwa_parameter_config.yaml",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();

    {
        let mut locked_planner = cloned_nav.planner.lock().unwrap();
        *locked_planner = planner;
    }

    std::thread::spawn(move || loop {
        if !*cloned_nav.is_run.lock().unwrap() {
            std::thread::sleep(std::time::Duration::from_millis(10));
            continue;
        }
        let mut map = new_sample_map();
        let x_range = Uniform::new(map.min_point().x, map.max_point().x);
        let y_range = Uniform::new(map.min_point().y, map.max_point().y);
        let start;
        let goal;
        {
            let locked_start = cloned_nav.start_position.lock().unwrap();
            start = [
                locked_start.translation.x,
                locked_start.translation.y,
                locked_start.rotation.angle(),
            ];
            let locked_goal = cloned_nav.goal_position.lock().unwrap();
            goal = [
                locked_goal.translation.x,
                locked_goal.translation.y,
                locked_goal.rotation.angle(),
            ];
        }
        let is_free = |p: &[f64]| {
            !matches!(
                map.cell(&map.to_grid(p[0], p[1]).unwrap()).unwrap(),
                Cell::Obstacle
            )
        };
        const EXTEND_LENGTH: f64 = 0.05;
        let mut result = rrt::dual_rrt_connect(
            &[start[0], start[1]],
            &[goal[0], goal[1]],
            is_free,
            || {
                let mut rng = rand::thread_rng();
                vec![x_range.sample(&mut rng), y_range.sample(&mut rng)]
            },
            EXTEND_LENGTH,
            4000,
        )
        .unwrap();
        rrt::smooth_path(&mut result, is_free, EXTEND_LENGTH, 1000);
        let result = linear_interpolate_path(result, EXTEND_LENGTH);
        let result = add_target_position_to_path(
            result,
            &Pose::new(Vector2::new(goal[0], goal[1]), goal[2]),
        );
        {
            let mut locked_robot_path = cloned_nav.robot_path.lock().unwrap();
            locked_robot_path.set_global_path(robot_path_from_vec_vec(result.clone()));
        }

        for p in &result {
            map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
        }

        let path_distance_map = path_distance_map(&map, &result).unwrap();

        let goal_distance_map = goal_distance_map(&map, &goal).unwrap();

        let obstacle_distance_map = obstacle_distance_map(&map).unwrap();

        let local_goal_distance_map =
            local_goal_distance_map(&map, &result, [start[0], start[1]]).unwrap();

        {
            let mut locked_layered_grid_map = cloned_nav.layered_grid_map.lock().unwrap();
            locked_layered_grid_map.add_layer(PATH_DISTANCE_MAP_NAME.to_owned(), path_distance_map);
            locked_layered_grid_map.add_layer(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_distance_map);
            locked_layered_grid_map
                .add_layer(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), obstacle_distance_map);
            locked_layered_grid_map.add_layer(
                LOCAL_GOAL_DISTANCE_MAP_NAME.to_owned(),
                local_goal_distance_map,
            );
        }

        {
            let mut locked_angle_table = cloned_nav.angle_table.lock().unwrap();
            locked_angle_table.insert(ROTATION_COST_NAME.to_owned(), start[2]);
            locked_angle_table.insert(PATH_DIRECTION_COST_NAME.to_owned(), start[2]);
            locked_angle_table.insert(GOAL_DIRECTION_COST_NAME.to_owned(), goal[2]);
        }

        let mut current_pose = Pose::new(Vector2::new(start[0], start[1]), start[2]);
        let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), goal[2]);

        let mut current_velocity = Velocity { x: 0.0, theta: 0.0 };
        let mut plan_map = map.clone();

        for i in 0..300 {
            // let dynamic_map = new_dynamic_sample_map(i);
            let dynamic_map = new_sample_map();
            let path_distance_map = openrr_nav::path_distance_map(&dynamic_map, &result).unwrap();

            let goal_distance_map = openrr_nav::goal_distance_map(&dynamic_map, &goal).unwrap();

            let obstacle_distance_map = openrr_nav::obstacle_distance_map(&dynamic_map).unwrap();

            let local_goal_distance_map = openrr_nav::local_goal_distance_map(
                &map,
                &result,
                [current_pose.translation.x, current_pose.translation.y],
            )
            .unwrap();

            {
                let mut locked_layered_grid_map = cloned_nav.layered_grid_map.lock().unwrap();
                locked_layered_grid_map
                    .add_layer(PATH_DISTANCE_MAP_NAME.to_owned(), path_distance_map);
                locked_layered_grid_map
                    .add_layer(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_distance_map);
                locked_layered_grid_map
                    .add_layer(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), obstacle_distance_map);
                locked_layered_grid_map.add_layer(
                    LOCAL_GOAL_DISTANCE_MAP_NAME.to_owned(),
                    local_goal_distance_map,
                );
            }

            {
                let nearest_path_point = nearest_path_point(
                    &result,
                    [current_pose.translation.x, current_pose.translation.y],
                );
                let len = result.len();
                let mut locked_angle_table = cloned_nav.angle_table.lock().unwrap();
                locked_angle_table
                    .insert(ROTATION_COST_NAME.to_owned(), current_pose.rotation.angle());
                const FORWARD_OFFSET: usize = 20;
                if let Some((idx, _)) = nearest_path_point {
                    let look_ahead_idx = (idx + FORWARD_OFFSET).min(len - 1);
                    locked_angle_table.insert(
                        PATH_DIRECTION_COST_NAME.to_owned(),
                        result[look_ahead_idx][2],
                    );
                }
            }

            let (plan, candidates) = {
                let locked_layered_grid_map = cloned_nav.layered_grid_map.lock().unwrap();
                let locked_angle_table = cloned_nav.angle_table.lock().unwrap();
                let locked_planner = cloned_nav.planner.lock().unwrap();
                (
                    locked_planner.plan_local_path(
                        &current_pose,
                        &current_velocity,
                        &locked_layered_grid_map,
                        &locked_angle_table,
                    ),
                    locked_planner.predicted_plan_candidates(&current_pose, &current_velocity),
                )
            };
            {
                let mut locked_robot_path = cloned_nav.robot_path.lock().unwrap();
                locked_robot_path.set_local_path(RobotPath(plan.path.clone()));
                for (i, candidate) in candidates.iter().enumerate() {
                    locked_robot_path.add_user_defined_path(
                        &format!("candidate_{}", i),
                        RobotPath(candidate.path.clone()),
                    );
                }
            }

            current_velocity = plan.velocity;
            current_pose = plan.path[0];

            {
                let mut locked_robot_pose = cloned_nav.robot_pose.lock().unwrap();
                *locked_robot_pose = current_pose;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));

            if let Some(grid) =
                plan_map.to_grid(current_pose.translation.x, current_pose.translation.y)
            {
                let _ = plan_map.set_value(&grid, 9);
            } else {
                println!("OUT OF MAP!");
                return;
            }
            const GOAL_THRESHOLD_DISTANCE: f64 = 0.1;
            const GOAL_THRESHOLD_ANGLE_DIFFERENCE: f64 = 0.4;
            if (goal_pose.translation.vector - current_pose.translation.vector).norm()
                < GOAL_THRESHOLD_DISTANCE
                && (goal_pose.rotation.angle() - current_pose.rotation.angle()).abs()
                    < GOAL_THRESHOLD_ANGLE_DIFFERENCE
            {
                println!("GOAL! count = {i}");
                break;
            }
        }
        {
            let mut is_run = cloned_nav.is_run.lock().unwrap();
            *is_run = false;
        }
    });

    let bevy_cloned_nav = nav.clone();
    let mut app = BevyAppNav::new();
    app.setup(bevy_cloned_nav);
    app.run();
}
