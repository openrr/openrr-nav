use crate::{Cell, GridMap, LayeredGridMap, Position};
use nalgebra as na;
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, Default)]
pub struct Velocity {
    pub x: f64,
    pub theta: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Acceleration {
    pub x: f64,
    pub theta: f64,
}

pub type Pose = na::Isometry2<f64>;

fn velocity_to_pose(velocity: &Velocity, dt: f64) -> Pose {
    Pose::new(na::Vector2::new(velocity.x * dt, 0.0), velocity.theta * dt)
}

#[derive(Debug, Clone, Default)]
pub struct Plan {
    pub velocity: Velocity,
    pub(crate) cost: f64,
    pub path: Vec<Pose>,
}

#[derive(Debug, Clone, Default)]
pub struct Limits {
    pub max_velocity: Velocity,
    pub max_accel: Acceleration,
    pub min_velocity: Velocity,
    pub min_accel: Acceleration,
}

#[derive(Debug, Clone, Default)]
/// DWA Planner
pub struct DwaPlanner {
    limits: Limits,
    map_name_weight: HashMap<String, f64>,
    controller_dt: f64,
    simulation_duration: f64,
    num_vel_sample: i32,
}

fn accumulate_values_by_positions(map: &GridMap<u8>, positions: &[Position]) -> f64 {
    let mut cost: f64 = 0.0;
    for p in positions {
        if let Some(opt) = map.cell_by_position(p) {
            match opt {
                Cell::Value(v) => cost += v as f64,
                _ => { return f64::MAX }
            }
        } else {
            return f64::MAX;
        }
    }
    cost
}

impl DwaPlanner {
    pub fn new(
        limits: Limits,
        map_name_weight: HashMap<String, f64>,
        controller_dt: f64,
        simulation_duration: f64,
        num_vel_sample: i32,
    ) -> Self {
        Self {
            limits,
            map_name_weight,
            controller_dt,
            simulation_duration,
            num_vel_sample,
        }
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
    pub fn plan_local_path(
        &self,
        current_pose: &Pose,
        current_velocity: &Velocity,
        maps: &LayeredGridMap<u8>,
    ) -> Plan {
        let plans = self
            .sample_velocity(current_velocity)
            .into_iter()
            .map(|v| Plan {
                velocity: v.to_owned(),
                cost: 0.0,
                path: self.forward_simulation(current_pose, &v),
            })
            .collect::<Vec<_>>();
        let mut min_cost = f64::MAX;
        let mut selected_plan = Plan::default();
        for plan in plans {
            let mut all_layer_cost = 0.0;
            for (name, v) in &self.map_name_weight {
                let cost = v * accumulate_values_by_positions(
                    maps.layer(name).unwrap(),
                    &plan
                        .path
                        .iter()
                        .map(|p| Position::new(p.translation.x, p.translation.y))
                        .collect::<Vec<_>>(),
                );
                all_layer_cost += cost;
            }
            if all_layer_cost < min_cost {
                min_cost = all_layer_cost;
                selected_plan = plan.clone();
            }
        }
        selected_plan.cost = min_cost;
        selected_plan
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use na::Vector2;

    use crate::dwa_planner::*;
    use crate::utils::show_ascii_map;
    use crate::*;
    #[test]
    fn dwa_planner_test() {
        use rand::distributions::{Distribution, Uniform};
        use rrt;
        let mut map = grid_map::GridMap::<u8>::new(
            Position::new(-1.05, -1.05),
            Position::new(3.05, 1.05),
            0.05,
        );
        for i in 0..50 {
            map.set_obstacle_by_indices(&Indices::new(i + 10, 5)).unwrap();
            map.set_obstacle_by_indices(&Indices::new(i + 10, 6)).unwrap();
            for j in 20..30 {
                map.set_obstacle_by_indices(&Indices::new(i, j)).unwrap();
            }
        }
        let x_range = Uniform::new(map.min_point().x, map.max_point().x);
        let y_range = Uniform::new(map.min_point().y, map.max_point().y);
        let start = [-0.8, -0.9];
        let goal = [2.5, 0.5];
        let result = rrt::dual_rrt_connect(
            &start,
            &goal,
            |p: &[f64]| {
                !matches!(
                    map.cell_by_position(&Position::new(p[0], p[1])).unwrap(),
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

        let path_indices = result
            .iter()
            .map(|p| {
                map.to_index_by_position(&Position::new(p[0], p[1]))
                    .unwrap()
            })
            .map(|index| map.to_indices_from_index(index).unwrap())
            .collect::<Vec<_>>();
        for p in result {
            map.set_value_by_position(&Position::new(p[0], p[1]), 0)
                .unwrap();
        }
        let path_distance_map = path_distance_map(&map, &path_indices);
        show_ascii_map(&path_distance_map, 1.0);
        println!("=======================");
        let goal_indices = map
            .position_to_indices(&Position::new(goal[0], goal[1]))
            .unwrap();
        let goal_distance_map = goal_distance_map(&map, &goal_indices);
        show_ascii_map(&goal_distance_map, 1.0);
        println!("=======================");
        let obstacle_distance_map = obstacle_distance_map(&map);
        show_ascii_map(&obstacle_distance_map, 0.03);
        let mut maps = HashMap::new();
        maps.insert("path".to_owned(), path_distance_map);
        maps.insert("goal".to_owned(), goal_distance_map);
        maps.insert("obstacle".to_owned(), obstacle_distance_map);
        let layered = LayeredGridMap::new(maps);
        let mut weights = HashMap::new();
        weights.insert("path".to_owned(), 0.9);
        weights.insert("goal".to_owned(), 0.8);
        weights.insert("obstacle".to_owned(), 0.01);

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
        for _ in 0..100 {
            let plan = planner.plan_local_path(&current_pose, &current_velocity, &layered);
            println!("vel = {:?} cost = {}", current_velocity, plan.cost);
            println!(
                "pose = {:?}, {}",
                current_pose.translation,
                current_pose.rotation.angle()
            );
            current_velocity = plan.velocity;
            current_pose = plan.path[0];
            let _  = plan_map
                .set_value_by_position(
                    &Position::new(current_pose.translation.x, current_pose.translation.y),
                    9,
                );
            if (goal_pose.translation.vector - current_pose.translation.vector).norm() < 0.1 {
                println!("GOAL!");
                break;
            }
            show_ascii_map(&plan_map, 1.0);
        }
    }

    #[test]
    fn test_sample_velocities() {
        let planner = DwaPlanner::new(
            Limits {
                max_velocity: Velocity { x: 0.1, theta: 0.5 },
                max_accel: Acceleration { x: 0.2, theta: 1.0 },
                min_velocity: Velocity {
                    x: 0.0,
                    theta: -0.5,
                },
                min_accel: Acceleration {
                    x: -0.2,
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
