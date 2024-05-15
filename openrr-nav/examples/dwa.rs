use grid_map::*;
use openrr_nav::utils::show_ascii_map;
use openrr_nav::*;
use rand::distributions::{Distribution, Uniform};
use std::collections::HashMap;

fn new_sample_map() -> GridMap<u8> {
    let mut map =
        grid_map::GridMap::<u8>::new(Position::new(-1.05, -1.05), Position::new(3.05, 1.05), 0.05);
    for i in 10..50 {
        map.set_obstacle(&Grid::new(i + 10, 5)).unwrap();
        map.set_obstacle(&Grid::new(i + 10, 6)).unwrap();
        for j in 20..30 {
            map.set_obstacle(&Grid::new(i, j)).unwrap();
        }
    }
    map
}

fn main() {
    let mut map = new_sample_map();
    let x_range = Uniform::new(map.min_point().x, map.max_point().x);
    let y_range = Uniform::new(map.min_point().y, map.max_point().y);
    let start = [-0.8, -0.9];
    let goal = [2.5, 0.5];
    let result = rrt::dual_rrt_connect(
        &start,
        &goal,
        |p: &[f64]| {
            !matches!(
                map.cell(&map.to_grid(p[0], p[1]).unwrap()).unwrap(),
                Cell::Obstacle
            )
        },
        || {
            let mut rng = rand::thread_rng();
            vec![x_range.sample(&mut rng), y_range.sample(&mut rng)]
        },
        0.05,
        1000,
    )
    .unwrap();

    for p in result.iter() {
        map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
    }
    show_ascii_map(&map, 1.0);
    let path_distance_map = path_distance_map(&map, &result).unwrap();
    show_ascii_map(&path_distance_map, 1.0);
    println!("=======================");
    let goal_distance_map = goal_distance_map(&map, &goal).unwrap();
    show_ascii_map(&goal_distance_map, 0.1);
    println!("=======================");
    let obstacle_distance_map = obstacle_distance_map(&map).unwrap();
    show_ascii_map(&obstacle_distance_map, 0.1);
    let mut maps = HashMap::new();
    const PATH_DISTANCE_MAP_NAME: &str = "path";
    const GOAL_DISTANCE_MAP_NAME: &str = "goal";
    const OBSTACLE_DISTANCE_MAP_NAME: &str = "obstacle";
    maps.insert(PATH_DISTANCE_MAP_NAME.to_owned(), path_distance_map);
    maps.insert(GOAL_DISTANCE_MAP_NAME.to_owned(), goal_distance_map);
    maps.insert(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), obstacle_distance_map);
    let layered = LayeredGridMap::new(maps);
    let angles = HashMap::new();
    let mut weights = HashMap::new();
    weights.insert(PATH_DISTANCE_MAP_NAME.to_owned(), 0.8);
    weights.insert(GOAL_DISTANCE_MAP_NAME.to_owned(), 0.9);
    weights.insert(OBSTACLE_DISTANCE_MAP_NAME.to_owned(), 0.3);

    let planner = DwaPlanner::new(
        Limits {
            max_velocity: Velocity { x: 0.5, theta: 2.0 },
            max_accel: Acceleration { x: 2.0, theta: 5.0 },
            min_velocity: Velocity {
                x: 0.0,
                theta: -2.0,
            },
            min_accel: Acceleration {
                x: -2.0,
                theta: -5.0,
            },
        },
        weights,
        0.1,
        1.0,
        5,
    );

    let mut current_pose = Pose::new(Vector2::new(start[0], start[1]), 0.0);
    let goal_pose = Pose::new(Vector2::new(goal[0], goal[1]), 0.0);
    let mut current_velocity = Velocity { x: 0.0, theta: 0.0 };
    let mut plan_map = map.clone();
    for i in 0..100 {
        let plan = planner.plan_local_path(&current_pose, &current_velocity, &layered, &angles);
        println!("vel = {:?} cost = {}", current_velocity, plan.cost);
        println!(
            "pose = {:?}, {}",
            current_pose.translation,
            current_pose.rotation.angle()
        );
        current_velocity = plan.velocity;
        current_pose = plan.path[0];
        if let Some(grid) = plan_map.to_grid(current_pose.translation.x, current_pose.translation.y)
        {
            let _ = plan_map.set_value(&grid, 9);
        } else {
            println!("OUT OF MAP!");
            return;
        }
        const GOAL_THRESHOLD: f64 = 0.1;
        if (goal_pose.translation.vector - current_pose.translation.vector).norm() < GOAL_THRESHOLD
        {
            println!("GOAL! count = {i}");
            break;
        }
        show_ascii_map(&plan_map, 1.0);
    }
}
