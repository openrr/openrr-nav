use arci::nalgebra as na;
use clap::Parser;
use grid_map::*;
use openrr_nav::*;
use openrr_nav_viewer::NavigationViz;

#[derive(Debug, Parser)]
pub struct Args {
    #[clap(
        short = 'f',
        long = "config-file",
        default_value = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../openrr-nav/config/dwa_parameter_config.yaml"
        ),
        env = "PLANNER_CONFIG_PATH",
        help = "planner config file path"
    )]
    pub planner_config_path: String,
}

impl TryFrom<Args> for NavigationViz {
    type Error = openrr_nav::Error;

    fn try_from(value: Args) -> openrr_nav::Result<Self> {
        NavigationViz::new(&value.planner_config_path)
    }
}

pub(crate) fn new_sample_map() -> GridMap<u8> {
    let mut map =
        grid_map::GridMap::<u8>::new(Position::new(-2.05, -2.05), Position::new(6.05, 2.05), 0.05);
    for i in 20..100 {
        for j in 10..14 {
            map.set_obstacle(&Grid::new(i + 20, j)).unwrap();
        }
        for j in 40..60 {
            map.set_obstacle(&Grid::new(i, j)).unwrap();
        }
    }
    map
}

pub(crate) fn robot_path_from_vec_vec(path: Vec<Vec<f64>>) -> RobotPath {
    let mut robot_path_inner = vec![];
    for p in path {
        let pose = na::Isometry2::new(na::Vector2::new(p[0], p[1]), 0.);

        robot_path_inner.push(pose);
    }
    RobotPath(robot_path_inner)
}

pub(crate) fn linear_interpolate_path(path: Vec<Vec<f64>>, extend_length: f64) -> Vec<Vec<f64>> {
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
    let last_point_angle = interpolated_path.last().unwrap()[2];
    interpolated_path.push({
        let mut end_path = path.last().unwrap().clone();
        end_path.push(last_point_angle);
        end_path
    });
    interpolated_path
}

pub(crate) fn add_target_position_to_path(
    path: Vec<Vec<f64>>,
    target_pose: &Pose,
) -> Vec<Vec<f64>> {
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
