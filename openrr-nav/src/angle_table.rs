use std::collections::HashMap;

use crate::{utils::nearest_path_point, Pose};

const ROTATION_COST_NAME: &str = "rotation";
const PATH_DIRECTION_COST_NAME: &str = "path_direction";
const GOAL_DIRECTION_COST_NAME: &str = "goal_direction";

#[derive(Debug)]
pub struct AngleTable(HashMap<String, f64>);

impl AngleTable {
    pub fn new(start: f64, goal: f64) -> Self {
        let mut angles = HashMap::new();

        angles.insert(ROTATION_COST_NAME.to_owned(), start);
        angles.insert(PATH_DIRECTION_COST_NAME.to_owned(), start);
        angles.insert(GOAL_DIRECTION_COST_NAME.to_owned(), goal);

        Self(angles)
    }

    pub fn update(&mut self, current_pose: Option<Pose>, path: &[Vec<f64>]) {
        if let Some(pose) = current_pose {
            self.0
                .insert(ROTATION_COST_NAME.to_owned(), pose.rotation.angle());

            if !path.is_empty() {
                let nearest_path_point =
                    nearest_path_point(path, [pose.translation.x, pose.translation.y]);
                if let Some(p) = nearest_path_point {
                    self.0.insert(PATH_DIRECTION_COST_NAME.to_owned(), p.1[2]);
                }
            }
        }
    }

    pub fn angle_table(&self) -> HashMap<String, f64> {
        self.0.clone()
    }
}
