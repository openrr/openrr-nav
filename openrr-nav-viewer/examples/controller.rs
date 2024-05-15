// How to run:
//
// ```sh
// # start viewer
// cargo run --release -p openrr-nav-viewer
// # start controller example
// cargo run --release -p openrr-nav-viewer --example controller -- -f openrr-nav/config/dwa_parameter_config.yaml
// ```

mod shared;

use anyhow::Result;
use clap::Parser;
use grid_map::*;
use openrr_nav::{utils::nearest_path_point, *};
use openrr_nav_viewer::*;
use rand::distributions::{Distribution, Uniform};
use shared::*;

const ENDPOINT: &str = "http://[::1]:50101";

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let mut api = pb::api_client::ApiClient::connect(ENDPOINT).await?;
    api.set_config(pb::Config {
        text: std::fs::read_to_string(&args.planner_config_path)?,
    })
    .await?;
    loop {
        controller(&mut api).await?
    }
}

async fn controller(
    api: &mut openrr_nav_viewer::pb::api_client::ApiClient<tonic::transport::Channel>,
) -> Result<()> {
    if !api.get_is_run(()).await?.into_inner() {
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;
        return Ok(());
    }
    let mut map = new_sample_map();
    let x_range = Uniform::new(map.min_point().x, map.max_point().x);
    let y_range = Uniform::new(map.min_point().y, map.max_point().y);
    let start = Pose::from(api.get_start_position(()).await?.into_inner());
    let start = [
        start.translation.x,
        start.translation.y,
        start.rotation.angle(),
    ];
    let goal = Pose::from(api.get_goal_position(()).await?.into_inner());
    let goal = [
        goal.translation.x,
        goal.translation.y,
        goal.rotation.angle(),
    ];
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
    let result =
        add_target_position_to_path(result, &Pose::new(Vector2::new(goal[0], goal[1]), goal[2]));
    api.set_global_path(pb::RobotPath::from(robot_path_from_vec_vec(result.clone())))
        .await?;

    for p in &result {
        map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
    }

    let path_distance_map = path_distance_map(&map, &result).unwrap();

    let goal_distance_map = goal_distance_map(&map, &goal).unwrap();

    let obstacle_distance_map = obstacle_distance_map(&map).unwrap();

    let local_goal_distance_map =
        local_goal_distance_map(&map, &result, [start[0], start[1]]).unwrap();

    api.set_layered_grid_map(pb::SetLayeredGridMapRequest {
        maps: vec![
            pb::NamedGridMap {
                name: PATH_DISTANCE_MAP_NAME.to_owned(),
                map: Some((&path_distance_map).into()),
            },
            pb::NamedGridMap {
                name: GOAL_DISTANCE_MAP_NAME.to_owned(),
                map: Some((&goal_distance_map).into()),
            },
            pb::NamedGridMap {
                name: OBSTACLE_DISTANCE_MAP_NAME.to_owned(),
                map: Some((&obstacle_distance_map).into()),
            },
            pb::NamedGridMap {
                name: LOCAL_GOAL_DISTANCE_MAP_NAME.to_owned(),
                map: Some((&local_goal_distance_map).into()),
            },
        ],
    })
    .await?;

    api.set_angle_table(pb::SetAngleTableRequest {
        table: vec![
            pb::NamedAngle {
                name: ROTATION_COST_NAME.to_owned(),
                angle: start[2],
            },
            pb::NamedAngle {
                name: PATH_DIRECTION_COST_NAME.to_owned(),
                angle: start[2],
            },
            pb::NamedAngle {
                name: GOAL_DIRECTION_COST_NAME.to_owned(),
                angle: goal[2],
            },
        ],
    })
    .await?;

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

        api.set_layered_grid_map(pb::SetLayeredGridMapRequest {
            maps: vec![
                pb::NamedGridMap {
                    name: PATH_DISTANCE_MAP_NAME.to_owned(),
                    map: Some((&path_distance_map).into()),
                },
                pb::NamedGridMap {
                    name: GOAL_DISTANCE_MAP_NAME.to_owned(),
                    map: Some((&goal_distance_map).into()),
                },
                pb::NamedGridMap {
                    name: OBSTACLE_DISTANCE_MAP_NAME.to_owned(),
                    map: Some((&obstacle_distance_map).into()),
                },
                pb::NamedGridMap {
                    name: LOCAL_GOAL_DISTANCE_MAP_NAME.to_owned(),
                    map: Some((&local_goal_distance_map).into()),
                },
            ],
        })
        .await?;

        {
            let nearest_path_point = nearest_path_point(
                &result,
                [current_pose.translation.x, current_pose.translation.y],
            );
            let len = result.len();
            let mut table = vec![pb::NamedAngle {
                name: ROTATION_COST_NAME.to_owned(),
                angle: current_pose.rotation.angle(),
            }];
            const FORWARD_OFFSET: usize = 20;
            if let Some((idx, _)) = nearest_path_point {
                let look_ahead_idx = (idx + FORWARD_OFFSET).min(len - 1);
                table.push(pb::NamedAngle {
                    name: PATH_DIRECTION_COST_NAME.to_owned(),
                    angle: result[look_ahead_idx][2],
                });
            }
            api.set_angle_table(pb::SetAngleTableRequest { table })
                .await?;
        }

        // TODO: move them into viewer side?
        let (plan, candidates) = {
            (
                api.plan_local_path(pb::PlanRequest {
                    current_pose: Some(current_pose.into()),
                    current_velocity: Some(current_velocity.into()),
                })
                .await?
                .into_inner(),
                api.predicted_plan_candidates(pb::PlanRequest {
                    current_pose: Some(current_pose.into()),
                    current_velocity: Some(current_velocity.into()),
                })
                .await?
                .into_inner()
                .candidates,
            )
        };
        api.set_local_path_and_candidates(pb::PathAndCandidates {
            path: Some(pb::RobotPath {
                path: plan.path.clone(),
            }),
            candidates,
        })
        .await?;

        current_velocity = plan.velocity.unwrap().into();
        current_pose = plan
            .path
            .get(0)
            .cloned()
            .map(Into::into)
            .unwrap_or_default();

        api.set_current_pose(pb::Isometry2::from(current_pose))
            .await?;
        std::thread::sleep(std::time::Duration::from_millis(50));

        if let Some(grid) = plan_map.to_grid(current_pose.translation.x, current_pose.translation.y)
        {
            let _ = plan_map.set_value(&grid, 9);
        } else {
            println!("OUT OF MAP!");
            return Ok(());
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
    api.set_is_run(false).await?;
    Ok(())
}
