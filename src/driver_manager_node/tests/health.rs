//! Unit tests of `health.rs`, included from there with `#[path]`: still a child module of
//! `health`, so `use super::*` reaches its private items.

use super::*;

const THRESHOLDS: Thresholds = Thresholds {
    telemetry_timeout: Duration::from_secs(1),
    lidar_timeout: Duration::from_millis(500),
    imu_timeout: Duration::from_millis(200),
    camera_timeout: Duration::from_secs(1),
    command_idle: Duration::from_secs(1),
};

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

#[test]
fn rate_counts_arrivals_within_the_window() {
    let t0 = Instant::now();
    let mut stats = StreamStats::default();
    // 10 Hz for 3 s: only the last 2 s count
    for i in 0..30 {
        stats.record(t0 + ms(i * 100));
    }
    let snap = stats.snapshot(t0 + ms(2900));
    assert!((snap.rate_hz - 10.0).abs() < 0.6, "rate {}", snap.rate_hz);
    assert_eq!(snap.total, 30);
    assert_eq!(snap.age, Some(Duration::ZERO));
}

#[test]
fn rate_drops_to_zero_after_the_stream_stops() {
    let t0 = Instant::now();
    let mut stats = StreamStats::default();
    stats.record(t0);
    let snap = stats.snapshot(t0 + RATE_WINDOW + ms(1));
    assert_eq!(snap.rate_hz, 0.0);
    assert_eq!(snap.total, 1);
}

#[test]
fn stream_status_follows_age() {
    let t0 = Instant::now();
    let mut stats = StreamStats::default();
    assert_eq!(
        classify_stream(&stats.snapshot(t0), ms(500)),
        Status::NoData
    );
    stats.record(t0);
    assert_eq!(
        classify_stream(&stats.snapshot(t0 + ms(500)), ms(500)),
        Status::Ok
    );
    assert_eq!(
        classify_stream(&stats.snapshot(t0 + ms(501)), ms(500)),
        Status::Stale
    );
}

#[test]
fn command_status() {
    let t0 = Instant::now();
    let mut stats = StreamStats::default();
    // Never sent anything is idle, not a failure
    assert_eq!(
        classify_command(&stats.snapshot(t0), ms(1000)),
        Status::Idle
    );
    stats.record(t0);
    assert_eq!(
        classify_command(&stats.snapshot(t0 + ms(100)), ms(1000)),
        Status::Ok
    );
    assert_eq!(
        classify_command(&stats.snapshot(t0 + ms(1100)), ms(1000)),
        Status::Idle
    );
    // A failure wins until it leaves the rate window
    stats.record_error(t0 + ms(1200));
    assert_eq!(
        classify_command(&stats.snapshot(t0 + ms(1300)), ms(1000)),
        Status::Error
    );
    let later = t0 + ms(1200) + RATE_WINDOW + ms(1);
    assert_eq!(
        classify_command(&stats.snapshot(later), ms(1000)),
        Status::Idle
    );
    assert_eq!(stats.snapshot(later).total_errors, 1);
}

#[test]
fn status_order_is_best_to_worst() {
    assert!(Status::Ok < Status::Stale);
    assert!(Status::Stale < Status::NoData);
    assert_eq!(Status::Ok.max(Status::NoData), Status::NoData);
    assert!(Status::Idle.is_healthy());
    assert!(!Status::Stale.is_healthy());
}

#[test]
fn rows_before_anything_arrives() {
    let health = SensorHealth::default();
    let rows = health.rows(Instant::now(), &THRESHOLDS, false);
    let statuses: Vec<_> = rows.iter().map(|r| r.status).collect();
    // The monitor starts from INITIAL_STATUSES; it must match, or the first evaluation logs a
    // transition for every sensor that has not started yet
    assert_eq!(statuses, INITIAL_STATUSES);
    let names: Vec<_> = rows.iter().map(|r| r.name).collect();
    assert_eq!(names, ["telemetry", "command", "lidar", "imu", "camera"]);
    assert_eq!(rows[1].detail, "sent 0, failed 0, safety_lock off");
    assert_eq!(rows[2].detail, "points -");
    assert_eq!(rows[3].detail, "received 0");
}

#[test]
fn lidar_and_imu_rows_are_independent() {
    let health = SensorHealth::default();
    health.record_lidar_cloud(19_968);
    let rows = health.rows(Instant::now(), &THRESHOLDS, true);
    // The cloud is fine and the IMU never arrived: each row says so on its own
    assert_eq!(rows[2].status, Status::Ok);
    assert_eq!(rows[2].detail, "points 19968");
    assert_eq!(rows[3].status, Status::NoData);
    assert_eq!(rows[3].age, None);
    assert_eq!(rows[1].detail, "sent 0, failed 0, safety_lock ON");

    health.record_lidar_imu();
    health.record_lidar_imu();
    let rows = health.rows(Instant::now(), &THRESHOLDS, false);
    assert_eq!(rows[3].status, Status::Ok);
    assert_eq!(rows[3].detail, "received 2");
}

#[test]
fn imu_goes_stale_on_its_own_timeout() {
    let t0 = Instant::now();
    let health = SensorHealth::default();
    health.record_lidar_cloud(100);
    health.record_lidar_imu();
    // Past the IMU's 200 ms but inside the lidar's 500 ms: only the IMU is stale
    let rows = health.rows(t0 + ms(300), &THRESHOLDS, false);
    assert_eq!(rows[2].status, Status::Ok);
    assert_eq!(rows[3].status, Status::Stale);
}

#[test]
fn rows_report_telemetry_source_and_camera_size() {
    let health = SensorHealth::default();
    let from: SocketAddr = "192.168.1.120:50051".parse().unwrap();
    health.record_telemetry(from, false);
    health.record_telemetry(from, true);
    health.record_camera(100 * 1024);
    health.record_command(true);
    health.record_command(false);
    let rows = health.rows(Instant::now(), &THRESHOLDS, false);

    assert_eq!(rows[0].status, Status::Ok);
    assert_eq!(
        rows[0].detail,
        "from 192.168.1.120:50051, parse warnings 1 (total 1)"
    );
    assert_eq!(rows[1].status, Status::Error);
    assert_eq!(rows[1].detail, "sent 1, failed 1, safety_lock off");
    assert_eq!(rows[4].status, Status::Ok);
    assert_eq!(rows[4].detail, "last frame 100 KB");
}

#[test]
fn table_layout() {
    let rows = [
        Row {
            name: "telemetry",
            status: Status::Ok,
            rate_hz: 49.8,
            age: Some(ms(20)),
            detail: "from -".into(),
        },
        Row {
            name: "camera",
            status: Status::NoData,
            rate_hz: 0.0,
            age: None,
            detail: "last frame -".into(),
        },
    ];
    let table = render_table(Duration::from_secs(3723), &rows);
    assert_eq!(
        table,
        "[Monitor] Sensor status (uptime 01:02:03)\n\
         \x20 SENSOR     STATUS        RATE       AGE  DETAIL\n\
         \x20 telemetry  OK         49.8 Hz    0.02 s  from -\n\
         \x20 camera     NO DATA     0.0 Hz         -  last frame -"
    );
}
