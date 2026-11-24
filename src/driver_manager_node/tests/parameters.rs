//! Unit tests of `parameters.rs`, included from there with `#[path]`: still a child module of
//! `parameters`, so `use super::*` reaches its private items.

use super::*;

#[test]
fn lidar_msg_type_parses_both_names_and_rejects_others() {
    assert_eq!(LidarMsgType::parse("livox"), Ok(LidarMsgType::Livox));
    assert_eq!(
        LidarMsgType::parse("pointcloud2"),
        Ok(LidarMsgType::PointCloud2)
    );
    assert!(LidarMsgType::parse("PointCloud2").is_err());
    assert!(LidarMsgType::parse("").is_err());
}
