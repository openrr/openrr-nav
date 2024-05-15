use arci::nalgebra as na;
use grid_map::{Cell, GridMap, LayeredGridMap, Position};
pub use na::Vector2;
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, fs, path::Path};

use crate::Error;

mod serde_cost_name_weight;

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields, from = "[f64; 2]", into = "[f64; 2]")]
pub struct Velocity {
    pub x: f64,
    pub theta: f64,
}

impl From<[f64; 2]> for Velocity {
    fn from(value: [f64; 2]) -> Self {
        Self {
            x: value[0],
            theta: value[1],
        }
    }
}

impl From<Velocity> for [f64; 2] {
    fn from(value: Velocity) -> Self {
        [value.x, value.theta]
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields, from = "[f64; 2]", into = "[f64; 2]")]
pub struct Acceleration {
    pub x: f64,
    pub theta: f64,
}

impl From<[f64; 2]> for Acceleration {
    fn from(value: [f64; 2]) -> Self {
        Self {
            x: value[0],
            theta: value[1],
        }
    }
}

impl From<Acceleration> for [f64; 2] {
    fn from(value: Acceleration) -> Self {
        [value.x, value.theta]
    }
}

pub type Pose = na::Isometry2<f64>;

fn velocity_to_pose(velocity: &Velocity, dt: f64) -> Pose {
    Pose::new(na::Vector2::new(velocity.x * dt, 0.0), velocity.theta * dt)
}

#[derive(Debug, Clone, Default)]
pub struct Plan {
    pub velocity: Velocity,
    pub cost: f64,
    pub path: Vec<Pose>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// Velocity and acceleration limitations of the robot
pub struct Limits {
    /// plus limit of the velocity
    pub max_velocity: Velocity,
    #[serde(rename = "max_acceleration")]
    /// plus limit of the acceleration
    pub max_accel: Acceleration,
    /// minus limit of the velocity (like -0.5)
    pub min_velocity: Velocity,
    #[serde(rename = "min_acceleration")]
    /// minus limit of the acceleration (like -1.0)
    pub min_accel: Acceleration,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// DWA Planner
pub struct DwaPlanner {
    limits: Limits,
    #[serde(with = "serde_cost_name_weight")]
    cost_name_weight: HashMap<String, f64>,
    controller_dt: f64,
    simulation_duration: f64,
    num_vel_sample: i32,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct DwaPlannerConfig {
    #[serde(rename = "DwaPlanner")]
    dwa_planner: DwaPlanner,
}

fn accumulate_values_by_positions(map: &GridMap<u8>, positions: &[Position]) -> f64 {
    if positions.is_empty() {
        return f64::MAX;
    }
    let mut cost: f64 = 0.0;
    for p in positions {
        // TODO: Support allow Unknown
        if let Some(grid) = map.to_grid(p.x, p.y) {
            if let Some(cell) = map.cell(&grid) {
                match cell {
                    Cell::Value(v) => {
                        cost += *v as f64;
                    }
                    Cell::Uninitialized => panic!("Uninitialized is not supported!"),
                    Cell::Obstacle => cost += 255.0,
                    Cell::Unknown => cost += 255.0,
                }
            } else {
                // out of grid (should not happen)
                return f64::MAX;
            }
        } else {
            // out of grid
            return f64::MAX;
        }
    }
    cost
}

impl DwaPlanner {
    pub fn new(
        limits: Limits,
        cost_name_weight: HashMap<String, f64>,
        controller_dt: f64,
        simulation_duration: f64,
        num_vel_sample: i32,
    ) -> Self {
        Self {
            limits,
            cost_name_weight,
            controller_dt,
            simulation_duration,
            num_vel_sample,
        }
    }

    pub fn new_from_config(path: impl AsRef<Path>) -> Result<Self, Error> {
        let source = fs::read_to_string(path).unwrap();
        Self::new_from_config_text(&source)
    }

    pub fn new_from_config_text(source: &str) -> Result<Self, Error> {
        use serde_yaml::from_str;
        let config: DwaPlannerConfig = from_str(source).map_err(grid_map::Error::from)?;
        Ok(config.dwa_planner)
    }

    /// Get candidate velocities from current velocity
    pub(crate) fn sample_velocity(&self, current_velocity: &Velocity) -> Vec<Velocity> {
        let max_x_limit = (current_velocity.x + self.limits.max_accel.x * self.controller_dt)
            .clamp(self.limits.min_velocity.x, self.limits.max_velocity.x);
        let min_x_limit = (current_velocity.x + self.limits.min_accel.x * self.controller_dt)
            .clamp(self.limits.min_velocity.x, self.limits.max_velocity.x);
        let max_theta_limit =
            (current_velocity.theta + self.limits.max_accel.theta * self.controller_dt).clamp(
                self.limits.min_velocity.theta,
                self.limits.max_velocity.theta,
            );
        let min_theta_limit =
            (current_velocity.theta + self.limits.min_accel.theta * self.controller_dt).clamp(
                self.limits.min_velocity.theta,
                self.limits.max_velocity.theta,
            );
        let d_vel_x = (max_x_limit - min_x_limit) / self.num_vel_sample as f64;
        let d_vel_theta = (max_theta_limit - min_theta_limit) / self.num_vel_sample as f64;
        let mut velocities = vec![];
        for i in 0..(self.num_vel_sample + 1) {
            for j in 0..(self.num_vel_sample + 1) {
                velocities.push(Velocity {
                    x: min_x_limit + d_vel_x * j as f64,
                    theta: min_theta_limit + d_vel_theta * i as f64,
                });
            }
            velocities.push(Velocity {
                x: 0.0,
                theta: min_theta_limit + d_vel_theta * i as f64,
            });
        }
        velocities
    }

    fn forward_simulation(&self, current_pose: &Pose, target_velocity: &Velocity) -> Vec<Pose> {
        let mut last_pose = current_pose.to_owned();
        let diff = velocity_to_pose(target_velocity, self.controller_dt);
        let mut poses = vec![];
        for _ in 0..(self.simulation_duration / self.controller_dt) as usize {
            let next_pose = last_pose * diff;
            poses.push(next_pose);
            last_pose = next_pose;
        }
        poses
    }

    /// Get predicted plan candidates
    pub fn predicted_plan_candidates(
        &self,
        current_pose: &Pose,
        current_velocity: &Velocity,
    ) -> Vec<Plan> {
        self.sample_velocity(current_velocity)
            .into_iter()
            .map(|v| Plan {
                velocity: v.to_owned(),
                cost: 0.0,
                path: self.forward_simulation(current_pose, &v),
            })
            .collect::<Vec<_>>()
    }

    /// Plan the path using forward simulation
    pub fn plan_local_path(
        &self,
        current_pose: &Pose,
        current_velocity: &Velocity,
        maps: &LayeredGridMap<u8>,
        angles: &HashMap<String, f64>,
    ) -> Plan {
        let plans = self.predicted_plan_candidates(current_pose, current_velocity);
        let mut min_cost = f64::MAX;
        let mut selected_plan = Plan::default();
        for plan in plans {
            let mut all_layer_cost = 0.0;
            for (cost_name, v) in &self.cost_name_weight {
                let dist_cost = match maps.layer(cost_name) {
                    Some(map) => {
                        v * accumulate_values_by_positions(
                            map,
                            &plan
                                .path
                                .iter()
                                .map(|p| Position::new(p.translation.x, p.translation.y))
                                .collect::<Vec<_>>(),
                        )
                    }
                    None => 0.,
                };
                all_layer_cost += dist_cost;

                let angle_cost = match angles.get(cost_name) {
                    Some(angle) => v * (angle - plan.path.last().unwrap().rotation.angle()).abs(),
                    None => 0.,
                };
                all_layer_cost += angle_cost;
            }

            if all_layer_cost < min_cost {
                min_cost = all_layer_cost;
                selected_plan = plan.clone();
            }
        }
        selected_plan.cost = min_cost;
        selected_plan
    }

    pub fn limits(&self) -> &Limits {
        &self.limits
    }

    pub fn map_name_weight(&self) -> &HashMap<String, f64> {
        &self.cost_name_weight
    }

    pub fn map_name_weight_mut(&mut self) -> &mut HashMap<String, f64> {
        &mut self.cost_name_weight
    }

    pub fn map_names(&self) -> impl Iterator<Item = &String> {
        self.cost_name_weight.keys()
    }

    pub fn controller_dt(&self) -> f64 {
        self.controller_dt
    }

    pub fn simulation_duration(&self) -> f64 {
        self.simulation_duration
    }

    pub fn num_vel_sample(&self) -> i32 {
        self.num_vel_sample
    }
}

#[cfg(test)]
mod tests {
    use grid_map::*;
    use na::Vector2;
    use std::collections::HashMap;

    use crate::dwa_planner::*;
    use crate::utils::show_ascii_map;
    use crate::*;

    fn new_sample_map() -> GridMap<u8> {
        let mut map = grid_map::GridMap::<u8>::new(
            Position::new(-1.05, -1.05),
            Position::new(3.05, 1.05),
            0.05,
        );
        for i in 10..50 {
            map.set_obstacle(&Grid::new(i + 10, 5)).unwrap();
            map.set_obstacle(&Grid::new(i + 10, 6)).unwrap();
            for j in 20..30 {
                map.set_obstacle(&Grid::new(i, j)).unwrap();
            }
        }
        map
    }

    #[test]
    fn new_from_config_test() {
        let _ = DwaPlanner::new_from_config("config/dwa_parameter_config.yaml").unwrap();
    }

    #[test]
    fn dwa_planner_test() {
        use rand::distributions::{Distribution, Uniform};
        use rrt;
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

        let path_grid = result
            .iter()
            .map(|p| map.to_grid(p[0], p[1]).unwrap())
            .collect::<Vec<_>>();

        for p in result {
            map.set_value(&map.to_grid(p[0], p[1]).unwrap(), 0).unwrap();
        }
        show_ascii_map(&map, 1.0);
        let path_distance_map = path_distance_map(&map, &path_grid).unwrap();
        show_ascii_map(&path_distance_map, 1.0);
        println!("=======================");
        let goal_grid = map.to_grid(goal[0], goal[1]).unwrap();
        let goal_distance_map = goal_distance_map(&map, &goal_grid).unwrap();
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
        let mut reached = false;
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
            if let Some(grid) =
                plan_map.to_grid(current_pose.translation.x, current_pose.translation.y)
            {
                let _ = plan_map.set_value(&grid, 9);
            } else {
                println!("OUT OF MAP!");
                return;
            }
            const GOAL_THRESHOLD: f64 = 0.1;
            if (goal_pose.translation.vector - current_pose.translation.vector).norm()
                < GOAL_THRESHOLD
            {
                println!("GOAL! count = {i}");
                reached = true;
                break;
            }
            show_ascii_map(&plan_map, 1.0);
        }
        assert!(reached);
    }

    #[test]
    fn test_sample_velocities() {
        let planner = DwaPlanner::new(
            Limits {
                max_velocity: Velocity { x: 0.1, theta: 0.5 },
                max_accel: Acceleration { x: 0.5, theta: 1.0 },
                min_velocity: Velocity {
                    x: 0.0,
                    theta: -0.5,
                },
                min_accel: Acceleration {
                    x: -0.5,
                    theta: -1.0,
                },
            },
            HashMap::new(),
            0.1,
            3.0,
            5,
        );
        let velocities = planner.sample_velocity(&Velocity { x: 0.0, theta: 0.0 });
        for velocity in velocities {
            println!("{velocity:?}");
        }
        let poses = planner.forward_simulation(
            &Pose::identity(),
            &Velocity {
                x: 0.01,
                theta: 0.1,
            },
        );
        for pose in poses {
            println!("pose = {:?}, {}", pose.translation, pose.rotation.angle());
        }
    }
}
