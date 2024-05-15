use arci::nalgebra as na;
use std::collections::HashMap;

#[derive(Debug, Clone, Default)]
pub struct RobotPath(pub Vec<na::Isometry2<f64>>);

impl RobotPath {
    pub fn new() -> Self {
        Self(Vec::new())
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }

    pub fn push(&mut self, pose: na::Isometry2<f64>) {
        self.0.push(pose);
    }
}

#[derive(Debug, Clone, Default)]
pub struct NavigationRobotPath {
    local_path: RobotPath,
    global_path: RobotPath,
    user_defined_path: HashMap<String, RobotPath>,
}

impl NavigationRobotPath {
    pub fn new(local_path: RobotPath, global_path: RobotPath) -> Self {
        Self {
            local_path,
            global_path,
            user_defined_path: Default::default(),
        }
    }

    pub fn set_local_path(&mut self, local_path: RobotPath) {
        self.local_path = local_path;
    }

    pub fn set_global_path(&mut self, global_path: RobotPath) {
        self.global_path = global_path;
    }

    pub fn local_path(&self) -> &RobotPath {
        &self.local_path
    }

    pub fn local_path_mut(&mut self) -> &mut RobotPath {
        &mut self.local_path
    }

    pub fn global_path(&self) -> &RobotPath {
        &self.global_path
    }

    pub fn global_path_mut(&mut self) -> &mut RobotPath {
        &mut self.global_path
    }

    pub fn add_user_defined_path(&mut self, name: &str, path: RobotPath) {
        self.user_defined_path.insert(name.to_owned(), path);
    }

    pub fn get_user_defined_path(&self, key: &str) -> Option<&RobotPath> {
        self.user_defined_path.get(key)
    }

    pub fn get_user_defined_path_as_iter(&self) -> impl Iterator<Item = (&String, &RobotPath)> {
        self.user_defined_path.iter()
    }
}
