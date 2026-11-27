//! Unit tests of `m20_link.rs`, included from there with `#[path]`: still a child module of
//! `m20_link`, so `use super::*` reaches its private items.

use super::*;

fn loopback(cmd_timeout: Duration) -> (M20Link, UdpSocket) {
    let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
    robot
        .set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let link = M20Link::open(
        M20Config {
            target: robot.local_addr().unwrap(),
            max_vx: 2.0,
            max_vy: 1.0,
            max_wz: 2.0,
            cmd_timeout,
            yaw_map: m20::YawMap::default(),
        },
        Arc::default(),
        Logger::default(),
    )
    .unwrap();
    (link, robot)
}

/// Reads datagrams for `window`, returning each ASDU
fn collect(robot: &UdpSocket, window: Duration) -> Vec<String> {
    let end = Instant::now() + window;
    let mut out = Vec::new();
    let mut buf = [0u8; 2048];
    while Instant::now() < end {
        if let Ok(n) = robot.recv(&mut buf) {
            out.push(m20::parse_packet(&buf[..n]).unwrap().to_owned());
        }
    }
    out
}

fn count(asdus: &[String], kind: (u32, u32)) -> usize {
    let tag = format!("\"Type\":{},\"Command\":{},", kind.0, kind.1);
    asdus.iter().filter(|a| a.contains(&tag)).count()
}

#[test]
fn heartbeat_then_regular_mode_without_any_command() {
    let (_link, robot) = loopback(Duration::from_millis(300));
    let got = collect(&robot, Duration::from_millis(1200));
    assert!(got[0].contains("\"Type\":100,\"Command\":100,"));
    assert!(got[1].contains("\"Items\":{\"Mode\":0}"));
    assert!(count(&got, m20::HEARTBEAT) >= 2);
    assert_eq!(count(&got, m20::AXIS), 0);
}

#[test]
fn velocity_is_repeated_then_stopped_three_times() {
    let (link, robot) = loopback(Duration::from_millis(200));
    link.set_velocity(1.0, 0.0, -1.0);
    let got = collect(&robot, Duration::from_millis(800));
    let moving: Vec<_> = got
        .iter()
        .filter(|a| a.contains("\"X\":0.5000") && a.contains("\"Yaw\":-0.5000"))
        .collect();
    assert!(moving.len() >= 2, "expected the command repeated, got {got:?}");
    let zeros = got
        .iter()
        .filter(|a| a.contains("\"X\":0.0000,\"Y\":0.0000"))
        .count();
    assert_eq!(zeros, usize::from(STOP_REPEATS));
}

#[test]
fn stand_sends_zero_regular_mode_and_motion_param() {
    let (link, robot) = loopback(Duration::from_millis(200));
    collect(&robot, Duration::from_millis(100)); // the startup heartbeat / mode
    link.set_velocity(1.0, 0.0, 0.0);
    link.set_motion_state(m20::MOTION_PARAM_STAND);
    let got = collect(&robot, Duration::from_millis(600));
    let stand = got
        .iter()
        .position(|a| a.contains("\"MotionParam\":1"))
        .expect("stand sent");
    assert!(got[..stand].iter().any(|a| a.contains("\"Mode\":0")));
    // The velocity and its trailing stops were dropped: nothing but heartbeats after the
    // stand (at most one axis command already decided before the stand landed)
    assert!(count(&got[stand..], m20::AXIS) <= 1);
}

#[test]
fn estop_sends_zero_and_soft_estop_and_keeps_the_link_up() {
    let (link, robot) = loopback(Duration::from_millis(200));
    collect(&robot, Duration::from_millis(100)); // the startup heartbeat / mode
    link.set_velocity(1.0, 0.0, 0.0);
    link.set_motion_state(m20::MOTION_PARAM_SOFT_ESTOP);
    let got = collect(&robot, Duration::from_millis(1200));
    let estop = got
        .iter()
        .position(|a| a.contains("\"MotionParam\":2"))
        .expect("soft emergency stop sent");
    assert!(got[..estop]
        .iter()
        .any(|a| a.contains("\"X\":0.0000,\"Y\":0.0000")));
    // The link stays up as on the G23: heartbeats keep going after the stop
    assert!(count(&got[estop..], m20::HEARTBEAT) >= 1, "got {got:?}");
    // ...and a cmd_vel after it is forwarded again
    link.set_velocity(0.5, 0.0, 0.0);
    let after = collect(&robot, Duration::from_millis(150));
    assert!(after.iter().any(|a| a.contains("\"X\":0.5000")), "got {after:?}");
}
