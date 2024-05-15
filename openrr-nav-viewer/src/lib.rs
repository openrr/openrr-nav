mod bevy_app;
mod converter;
mod map_type;
mod nav_viz;

pub use bevy_app::*;
pub use converter::*;
pub use map_type::*;
pub use nav_viz::*;

pub mod pb {
    #![allow(unreachable_pub)]

    tonic::include_proto!("openrr_nav_viewer");
}

#[tonic::async_trait]
impl pb::api_server::Api for NavigationViz {
    async fn get_start_position(
        &self,
        _request: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::Isometry2>, tonic::Status> {
        let start = self.start_position.lock().unwrap();
        Ok(tonic::Response::new((*start).into()))
    }
    async fn get_goal_position(
        &self,
        _request: tonic::Request<()>,
    ) -> Result<tonic::Response<pb::Isometry2>, tonic::Status> {
        let goal = self.goal_position.lock().unwrap();
        Ok(tonic::Response::new((*goal).into()))
    }
    async fn get_is_run(
        &self,
        _request: tonic::Request<()>,
    ) -> Result<tonic::Response<bool>, tonic::Status> {
        Ok(tonic::Response::new(*self.is_run.lock().unwrap()))
    }
    async fn set_global_path(
        &self,
        request: tonic::Request<pb::RobotPath>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        self.robot_path
            .lock()
            .unwrap()
            .set_global_path(request.into_inner().into());
        Ok(tonic::Response::new(()))
    }
    async fn set_local_path_and_candidates(
        &self,
        request: tonic::Request<pb::PathAndCandidates>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let pb::PathAndCandidates { path, candidates } = request.into_inner();
        let mut robot_path = self.robot_path.lock().unwrap();
        robot_path.set_local_path(path.unwrap().into());
        for (i, candidate) in candidates.into_iter().enumerate() {
            robot_path.add_user_defined_path(
                &format!("candidate_{i}"),
                openrr_nav::RobotPath(candidate.path.into_iter().map(Into::into).collect()),
            );
        }
        Ok(tonic::Response::new(()))
    }
    async fn set_layered_grid_map(
        &self,
        request: tonic::Request<pb::SetLayeredGridMapRequest>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let pb::SetLayeredGridMapRequest { maps } = request.into_inner();
        let mut layered_grid_map = self.layered_grid_map.lock().unwrap();
        for named_map in maps {
            layered_grid_map.add_layer(named_map.name, named_map.map.unwrap().into());
        }
        Ok(tonic::Response::new(()))
    }
    async fn set_angle_table(
        &self,
        request: tonic::Request<pb::SetAngleTableRequest>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let pb::SetAngleTableRequest { table } = request.into_inner();
        let mut angle_table = self.angle_table.lock().unwrap();
        for angle in table {
            angle_table.insert(angle.name, angle.angle);
        }
        Ok(tonic::Response::new(()))
    }
    async fn set_current_pose(
        &self,
        request: tonic::Request<pb::Isometry2>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        *self.robot_pose.lock().unwrap() = request.into_inner().into();
        Ok(tonic::Response::new(()))
    }
    async fn set_config(
        &self,
        request: tonic::Request<pb::Config>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        let text = request.into_inner().text;
        match openrr_nav::DwaPlanner::new_from_config_text(&text) {
            Ok(new) => *self.planner.lock().unwrap() = new,
            Err(e) => {
                return Err(tonic::Status::invalid_argument(format!(
                    "failed to parse config: {e}"
                )));
            }
        }
        Ok(tonic::Response::new(()))
    }
    async fn set_is_run(
        &self,
        request: tonic::Request<bool>,
    ) -> Result<tonic::Response<()>, tonic::Status> {
        *self.is_run.lock().unwrap() = request.into_inner();
        Ok(tonic::Response::new(()))
    }
    async fn plan_local_path(
        &self,
        request: tonic::Request<pb::PlanRequest>,
    ) -> Result<tonic::Response<pb::Plan>, tonic::Status> {
        let pb::PlanRequest {
            current_pose,
            current_velocity,
        } = request.into_inner();
        let layered_grid_map = self.layered_grid_map.lock().unwrap();
        let angle_table = self.angle_table.lock().unwrap();
        let planner = self.planner.lock().unwrap();
        let plan = planner.plan_local_path(
            &current_pose.unwrap().into(),
            &current_velocity.unwrap().into(),
            &layered_grid_map,
            &angle_table,
        );
        Ok(tonic::Response::new(plan.into()))
    }
    async fn predicted_plan_candidates(
        &self,
        request: tonic::Request<pb::PlanRequest>,
    ) -> Result<tonic::Response<pb::Candidates>, tonic::Status> {
        let pb::PlanRequest {
            current_pose,
            current_velocity,
        } = request.into_inner();
        let planner = self.planner.lock().unwrap();
        let candidates = planner.predicted_plan_candidates(
            &current_pose.unwrap().into(),
            &current_velocity.unwrap().into(),
        );
        Ok(tonic::Response::new(pb::Candidates {
            candidates: candidates.into_iter().map(Into::into).collect(),
        }))
    }
}

impl From<openrr_nav::RobotPath> for pb::RobotPath {
    fn from(val: openrr_nav::RobotPath) -> Self {
        Self {
            path: val.0.into_iter().map(Into::into).collect(),
        }
    }
}
impl From<pb::RobotPath> for openrr_nav::RobotPath {
    fn from(val: pb::RobotPath) -> Self {
        Self(val.path.into_iter().map(Into::into).collect())
    }
}

impl From<grid_map::Position> for pb::Position {
    fn from(val: grid_map::Position) -> Self {
        Self { x: val.x, y: val.y }
    }
}
impl From<pb::Position> for grid_map::Position {
    fn from(val: pb::Position) -> Self {
        Self { x: val.x, y: val.y }
    }
}

impl From<pb::Velocity> for openrr_nav::Velocity {
    fn from(value: pb::Velocity) -> Self {
        Self {
            x: value.x,
            theta: value.theta,
        }
    }
}
impl From<openrr_nav::Velocity> for pb::Velocity {
    fn from(value: openrr_nav::Velocity) -> Self {
        Self {
            x: value.x,
            theta: value.theta,
        }
    }
}

impl From<openrr_nav::Plan> for pb::Plan {
    fn from(val: openrr_nav::Plan) -> Self {
        Self {
            velocity: Some(pb::Velocity {
                x: val.velocity.x,
                theta: val.velocity.theta,
            }),
            cost: val.cost,
            path: val.path.into_iter().map(Into::into).collect(),
        }
    }
}
impl From<pb::Plan> for openrr_nav::Plan {
    fn from(val: pb::Plan) -> Self {
        let velocity = val.velocity.unwrap();
        Self {
            velocity: openrr_nav::Velocity {
                x: velocity.x,
                theta: velocity.theta,
            },
            cost: val.cost,
            path: val.path.into_iter().map(Into::into).collect(),
        }
    }
}

impl From<&grid_map::GridMap<u8>> for pb::GridMap {
    fn from(val: &grid_map::GridMap<u8>) -> Self {
        Self {
            grid_converter: Some(pb::GridPositionConverter {
                resolution: val.resolution(),
                min_point: Some((*val.min_point()).into()),
                max_point: Some((*val.max_point()).into()),
                size: Some(pb::Size {
                    width: val.width() as _,
                    height: val.height() as _,
                }),
            }),
            cells: val.cells().iter().map(|c| (*c).into()).collect(),
        }
    }
}
impl From<pb::GridMap> for grid_map::GridMap<u8> {
    fn from(val: pb::GridMap) -> Self {
        let grid_converter = val.grid_converter.unwrap();
        let mut map = grid_map::GridMap::new(
            grid_converter.min_point.unwrap().into(),
            grid_converter.max_point.unwrap().into(),
            grid_converter.resolution,
        );
        assert_eq!(val.cells.len(), map.len());
        for (from, to) in val.cells.into_iter().zip(map.cells_mut()) {
            *to = from.into();
        }
        map
    }
}

impl From<grid_map::Cell<u8>> for pb::Cell {
    fn from(val: grid_map::Cell<u8>) -> Self {
        let kind = match val {
            grid_map::Cell::Uninitialized => pb::cell::Kind::Uninitialized(()),
            grid_map::Cell::Unknown => pb::cell::Kind::Unknown(()),
            grid_map::Cell::Obstacle => pb::cell::Kind::Obstacle(()),
            grid_map::Cell::Value(b) => pb::cell::Kind::Value(b as _),
        };
        Self { kind: Some(kind) }
    }
}
impl From<pb::Cell> for grid_map::Cell<u8> {
    fn from(val: pb::Cell) -> Self {
        let val = val.kind.unwrap();
        match val {
            pb::cell::Kind::Uninitialized(()) => Self::Uninitialized,
            pb::cell::Kind::Unknown(()) => Self::Unknown,
            pb::cell::Kind::Obstacle(()) => Self::Obstacle,
            pb::cell::Kind::Value(v) => Self::Value(v as u8),
        }
    }
}

impl From<arci::nalgebra::Isometry2<f64>> for pb::Isometry2 {
    fn from(val: arci::nalgebra::Isometry2<f64>) -> Self {
        Self {
            rotation: Some(pb::UnitComplex {
                re: val.rotation.re,
                im: val.rotation.im,
            }),
            translation: Some(pb::Translation2 {
                x: val.translation.x,
                y: val.translation.y,
            }),
        }
    }
}
impl From<pb::Isometry2> for arci::nalgebra::Isometry2<f64> {
    fn from(val: pb::Isometry2) -> Self {
        let translation = val.translation.unwrap();
        let rotation = val.rotation.unwrap();
        Self::from_parts(
            arci::nalgebra::Translation2::new(translation.x, translation.y),
            arci::nalgebra::UnitComplex::from_complex(arci::nalgebra::Complex {
                re: rotation.re,
                im: rotation.im,
            }),
        )
    }
}
