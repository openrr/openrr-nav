use bevy::prelude::*;
use grid_map::*;
use openrr_nav::*;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

#[derive(Debug, Clone, Resource, Default)]
pub struct NavigationViz {
    pub layered_grid_map: Arc<Mutex<LayeredGridMap<u8>>>,
    pub angle_table: Arc<Mutex<HashMap<String, f64>>>,
    pub robot_path: Arc<Mutex<NavigationRobotPath>>,
    pub robot_pose: Arc<Mutex<Pose>>,
    pub is_run: Arc<Mutex<bool>>,
    pub start_position: Arc<Mutex<Pose>>,
    pub goal_position: Arc<Mutex<Pose>>,
    pub planner: Arc<Mutex<DwaPlanner>>,
    planner_config_path: String,
}

impl NavigationViz {
    pub fn new(planner_config_path: &str) -> openrr_nav::Result<Self> {
        let planner = DwaPlanner::new_from_config(planner_config_path)?;
        Ok(Self {
            layered_grid_map: Default::default(),
            angle_table: Default::default(),
            robot_path: Default::default(),
            robot_pose: Default::default(),
            is_run: Arc::new(Mutex::new(true)),
            start_position: Arc::new(Mutex::new(Pose::new(Vector2::new(-1.6, -1.8), 0.0))),
            goal_position: Arc::new(Mutex::new(Pose::new(Vector2::new(5.0, 1.0), 0.0))),
            planner: Arc::new(Mutex::new(planner)),
            planner_config_path: planner_config_path.to_string(),
        })
    }

    pub fn reload_planner(&self) -> openrr_nav::Result<()> {
        let planner = DwaPlanner::new_from_config(&self.planner_config_path)?;
        let mut locked_planner = self.planner.lock().unwrap();
        *locked_planner = planner;
        Ok(())
    }
}
