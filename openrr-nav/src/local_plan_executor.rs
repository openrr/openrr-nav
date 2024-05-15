use crate::{utils::PoseTimeStamped, DwaPlanner, Error, Result, Velocity};
use arci::{BaseVelocity, Isometry2, Localization, MoveBase};
use grid_map::{GridMap, LayeredGridMap};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

#[derive(Debug)]
pub struct LocalPlanExecutor<M, L>
where
    M: MoveBase,
    L: Localization,
{
    /// local planner
    /// TODO: Support more planners
    move_base: Arc<Mutex<M>>,
    localization: Arc<Mutex<L>>,
    frame_id: String,
    local_planner: DwaPlanner,
    cost_maps: LayeredGridMap<u8>,
    angle_table: HashMap<String, f64>,
    goal_threshold: f64,
    last_pose: PoseTimeStamped,
}

impl<M, L> LocalPlanExecutor<M, L>
where
    M: MoveBase,
    L: Localization,
{
    pub fn new(
        move_base: Arc<Mutex<M>>,
        localization: Arc<Mutex<L>>,
        frame_id: String,
        local_planner: DwaPlanner,
        goal_threshold: f64,
    ) -> Self {
        Self {
            move_base,
            localization,
            frame_id,
            local_planner,
            cost_maps: LayeredGridMap::default(),
            angle_table: HashMap::default(),
            goal_threshold,
            last_pose: PoseTimeStamped::default(),
        }
    }
    pub fn set_cost_maps(&mut self, cost_maps: LayeredGridMap<u8>) {
        self.cost_maps = cost_maps;
    }

    pub fn set_cost_map_element(&mut self, key: String, value: GridMap<u8>) {
        self.cost_maps.add_layer(key, value);
    }

    pub fn set_angle_table(&mut self, angle_table: HashMap<String, f64>) {
        self.angle_table = angle_table;
    }

    pub fn set_angle_table_element(&mut self, key: String, value: f64) {
        self.angle_table.insert(key, value);
    }

    pub fn exec_once(&mut self) -> Result<()> {
        let current_pose = self
            .localization
            .lock()
            .unwrap()
            .current_pose(&self.frame_id)
            .unwrap();

        let current_velocity = {
            let v = self.current_velocity().unwrap_or(BaseVelocity {
                x: 0.,
                y: 0.,
                theta: 0.,
            });
            Velocity {
                x: v.x,
                theta: v.theta,
            }
        };

        let plan = self.local_planner.plan_local_path(
            &current_pose,
            &current_velocity,
            &self.cost_maps,
            &self.angle_table,
        );

        self.move_base
            .lock()
            .unwrap()
            .send_velocity(&BaseVelocity {
                x: plan.velocity.x,
                y: 0.,
                theta: plan.velocity.theta,
            })
            .unwrap();

        self.last_pose.set_pose(current_pose);

        Ok(())
    }

    pub fn is_reached_to_goal(&self, goal_pose: Isometry2<f64>) -> bool {
        (goal_pose.translation.vector - self.current_pose().unwrap().translation.vector).norm()
            < self.goal_threshold
    }

    pub fn current_velocity(&self) -> Result<BaseVelocity> {
        match self.move_base.lock().unwrap().current_velocity() {
            Ok(v) => Ok(v),
            Err(_) => match self.current_pose() {
                Ok(current_pose) => {
                    if self.last_pose.is_initialized() {
                        let last_pose = self.last_pose.pose().unwrap();

                        let dt = self.last_pose.elapsed().as_secs_f64();
                        let dx = current_pose.translation.x - last_pose.translation.x;
                        let dy = current_pose.translation.y - last_pose.translation.y;
                        let dtheta = current_pose.rotation.angle() - last_pose.rotation.angle();

                        Ok(BaseVelocity {
                            x: (dx.powi(2) + dy.powi(2)).sqrt() / dt,
                            y: 0.,
                            theta: dtheta / dt,
                        })
                    } else {
                        Err(Error::Other("Cannot get current velocity".to_owned()))
                    }
                }
                Err(_) => Err(Error::Other("Cannot get current velocity.".to_owned())),
            },
        }
    }

    pub fn current_pose(&self) -> Result<Isometry2<f64>> {
        self.localization
            .lock()
            .unwrap()
            .current_pose(&self.frame_id)
            .map_err(|e| Error::Other(e.to_string()))
    }

    pub fn stop(&self) -> Result<()> {
        self.move_base
            .lock()
            .unwrap()
            .send_velocity(&BaseVelocity::new(0., 0., 0.))
            .map_err(|e| Error::Other(e.to_string()))
    }
}
