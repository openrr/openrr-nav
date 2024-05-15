use arci::nalgebra as na;
use grid_map::{Cell, GridMap};
use rand::{distributions::Uniform, prelude::Distribution};

use crate::Pose;

#[derive(Debug)]
pub struct GlobalPlan {
    map: GridMap<u8>,
    start: [f64; 3],
    goal: [f64; 3],
}

impl GlobalPlan {
    pub fn new(map: GridMap<u8>, start: [f64; 3], goal: [f64; 3]) -> Self {
        Self { map, start, goal }
    }

    pub fn global_plan(&mut self) -> Vec<Vec<f64>> {
        let x_range = Uniform::new(self.map.min_point().x, self.map.max_point().x);
        let y_range = Uniform::new(self.map.min_point().y, self.map.max_point().y);

        let is_free = |p: &[f64]| {
            !matches!(
                self.map
                    .cell(&self.map.to_grid(p[0], p[1]).unwrap())
                    .unwrap(),
                Cell::Obstacle
            )
        };

        const EXTEND_LENGTH: f64 = 0.05;

        let mut result = rrt::dual_rrt_connect(
            &[self.start[0], self.start[1]],
            &[self.goal[0], self.goal[1]],
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
        add_target_position_to_path(
            result,
            &Pose::new(na::Vector2::new(self.goal[0], self.goal[1]), self.goal[2]),
        )
    }
}

fn linear_interpolate_path(path: Vec<Vec<f64>>, extend_length: f64) -> Vec<Vec<f64>> {
    if path.len() < 2 {
        return path;
    }
    let mut interpolated_path = vec![];
    for (p0, p1) in path.iter().zip(path.iter().skip(1)) {
        let diff_x = p1[0] - p0[0];
        let diff_y = p1[1] - p0[1];
        let diff = (diff_x.powi(2) + diff_y.powi(2)).sqrt();
        let direction = diff_y.atan2(diff_x);
        let interpolate_num = (diff / extend_length) as usize;
        if interpolate_num > 0 {
            let unit_diff_x = diff_x / interpolate_num as f64;
            let unit_diff_y = diff_y / interpolate_num as f64;
            for j in 1..interpolate_num {
                interpolated_path.push(vec![
                    p0[0] + unit_diff_x * j as f64,
                    p0[1] + unit_diff_y * j as f64,
                    direction,
                ]);
            }
        } else {
            interpolated_path.push({
                let mut p = p0.to_owned();
                p.push(direction);
                p
            });
        }
    }
    let last_point_angle = interpolated_path.last().unwrap_or(&vec![0.; 3])[2];
    interpolated_path.push({
        let mut end_path = path.last().unwrap().clone();
        end_path.push(last_point_angle);
        end_path
    });
    interpolated_path
}

fn add_target_position_to_path(path: Vec<Vec<f64>>, target_pose: &Pose) -> Vec<Vec<f64>> {
    let mut p = path.clone();
    let target_pose_vec = vec![
        target_pose.translation.x,
        target_pose.translation.y,
        target_pose.rotation.angle(),
    ];
    match p.last_mut() {
        Some(v) => {
            *v = target_pose_vec;
        }
        None => {
            p.push(target_pose_vec);
        }
    }
    p
}
