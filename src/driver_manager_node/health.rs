use std::collections::VecDeque;
use std::fmt::Write as _;
use std::net::SocketAddr;
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

/// Rates are message counts over this trailing window. Two seconds keeps a 10 Hz lidar at a
/// stable reading (20 samples) while a stream that stops still drops to 0 Hz within one report.
pub const RATE_WINDOW: Duration = Duration::from_secs(2);

/// One sensor's health, as the status table and the transition log show it.
///
/// Declared from best to worst: `Ord` is what picks the worse of the lidar's two streams.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Status {
    Ok,
    /// Command link only: nothing has been sent recently, which is normal while nobody drives.
    /// UDP has no acknowledgement, so the command side can never be known to be down.
    Idle,
    /// Seen before, but nothing for longer than the stream's timeout
    Stale,
    /// Never seen since the node started
    NoData,
    /// Command link only: a send failed within the rate window
    Error,
}

impl Status {
    pub fn label(self) -> &'static str {
        match self {
            Status::Ok => "OK",
            Status::Idle => "IDLE",
            Status::Stale => "STALE",
            Status::NoData => "NO DATA",
            Status::Error => "ERROR",
        }
    }

    /// Ok and Idle are both normal operation; a change between them is not worth a log line
    pub fn is_healthy(self) -> bool {
        matches!(self, Status::Ok | Status::Idle)
    }
}

/// Arrival bookkeeping for one stream (a topic, or one side of a UDP link)
#[derive(Debug, Default)]
pub struct StreamStats {
    arrivals: VecDeque<Instant>,
    errors: VecDeque<Instant>,
    last_seen: Option<Instant>,
    /// A per-stream figure for the detail column: the lidar's point count, the camera's JPEG size
    last_value: Option<u64>,
    total: u64,
    total_errors: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StreamSnapshot {
    pub rate_hz: f64,
    /// Time since the last arrival; None if the stream was never seen
    pub age: Option<Duration>,
    pub last_value: Option<u64>,
    pub total: u64,
    /// Errors within the rate window
    pub recent_errors: usize,
    pub total_errors: u64,
}

impl StreamStats {
    pub fn record(&mut self, now: Instant) {
        self.prune(now);
        self.arrivals.push_back(now);
        self.last_seen = Some(now);
        self.total += 1;
    }

    pub fn record_value(&mut self, now: Instant, value: u64) {
        self.record(now);
        self.last_value = Some(value);
    }

    pub fn record_error(&mut self, now: Instant) {
        self.prune(now);
        self.errors.push_back(now);
        self.total_errors += 1;
    }

    pub fn snapshot(&mut self, now: Instant) -> StreamSnapshot {
        self.prune(now);
        StreamSnapshot {
            rate_hz: self.arrivals.len() as f64 / RATE_WINDOW.as_secs_f64(),
            age: self.last_seen.map(|t| now.saturating_duration_since(t)),
            last_value: self.last_value,
            total: self.total,
            recent_errors: self.errors.len(),
            total_errors: self.total_errors,
        }
    }

    fn prune(&mut self, now: Instant) {
        for queue in [&mut self.arrivals, &mut self.errors] {
            while queue
                .front()
                .is_some_and(|&t| now.saturating_duration_since(t) > RATE_WINDOW)
            {
                queue.pop_front();
            }
        }
    }
}

/// A topic or the telemetry link: healthy while something arrived within `timeout`
pub fn classify_stream(snapshot: &StreamSnapshot, timeout: Duration) -> Status {
    match snapshot.age {
        None => Status::NoData,
        Some(age) if age > timeout => Status::Stale,
        Some(_) => Status::Ok,
    }
}

/// The command link: a failed send is the only failure it can see
pub fn classify_command(snapshot: &StreamSnapshot, idle_after: Duration) -> Status {
    if snapshot.recent_errors > 0 {
        return Status::Error;
    }
    match snapshot.age {
        Some(age) if age <= idle_after => Status::Ok,
        _ => Status::Idle,
    }
}

/// How long each stream may go quiet before it counts as STALE (IDLE for the command link)
#[derive(Debug, Clone, Copy)]
pub struct Thresholds {
    pub telemetry_timeout: Duration,
    pub lidar_timeout: Duration,
    pub camera_timeout: Duration,
    pub command_idle: Duration,
}

/// Every sensor the node watches, shared between the threads that feed it (telemetry thread,
/// cmd_vel and services workers through `CommandLink`, the sensor worker) and the monitor
/// thread that reads it. Each stream has its own lock so the 200 Hz IMU never waits on the
/// camera.
#[derive(Debug, Default)]
pub struct SensorHealth {
    telemetry: Mutex<StreamStats>,
    telemetry_from: Mutex<Option<SocketAddr>>,
    command: Mutex<StreamStats>,
    lidar_cloud: Mutex<StreamStats>,
    lidar_imu: Mutex<StreamStats>,
    camera: Mutex<StreamStats>,
}

/// One line of the status table
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    pub name: &'static str,
    pub status: Status,
    pub rate_hz: f64,
    pub age: Option<Duration>,
    pub detail: String,
}

impl SensorHealth {
    /// `parse_warning`: the datagram arrived but some section of it did not parse
    pub fn record_telemetry(&self, from: SocketAddr, parse_warning: bool) {
        let now = Instant::now();
        let mut stats = lock(&self.telemetry);
        stats.record(now);
        if parse_warning {
            stats.record_error(now);
        }
        *lock(&self.telemetry_from) = Some(from);
    }

    pub fn record_command(&self, sent: bool) {
        let now = Instant::now();
        let mut stats = lock(&self.command);
        if sent {
            stats.record(now);
        } else {
            stats.record_error(now);
        }
    }

    pub fn record_lidar_cloud(&self, points: u32) {
        lock(&self.lidar_cloud).record_value(Instant::now(), u64::from(points));
    }

    pub fn record_lidar_imu(&self) {
        lock(&self.lidar_imu).record(Instant::now());
    }

    pub fn record_camera(&self, bytes: usize) {
        lock(&self.camera).record_value(Instant::now(), bytes as u64);
    }

    /// The four rows of the table, in display order: telemetry, command, lidar, camera.
    /// `safety_locked` is shown on the command row, since the lock is what gates that link.
    pub fn rows(&self, now: Instant, thresholds: &Thresholds, safety_locked: bool) -> [Row; 4] {
        let telemetry = lock(&self.telemetry).snapshot(now);
        let command = lock(&self.command).snapshot(now);
        let cloud = lock(&self.lidar_cloud).snapshot(now);
        let imu = lock(&self.lidar_imu).snapshot(now);
        let camera = lock(&self.camera).snapshot(now);
        let from = *lock(&self.telemetry_from);

        let cloud_status = classify_stream(&cloud, thresholds.lidar_timeout);
        let imu_status = classify_stream(&imu, thresholds.lidar_timeout);

        [
            Row {
                name: "telemetry",
                status: classify_stream(&telemetry, thresholds.telemetry_timeout),
                rate_hz: telemetry.rate_hz,
                age: telemetry.age,
                detail: format!(
                    "from {}, parse warnings {} (total {})",
                    from.map_or_else(|| "-".to_owned(), |a| a.to_string()),
                    telemetry.recent_errors,
                    telemetry.total_errors,
                ),
            },
            Row {
                name: "command",
                status: classify_command(&command, thresholds.command_idle),
                rate_hz: command.rate_hz,
                age: command.age,
                detail: format!(
                    "sent {}, failed {}, safety_lock {}",
                    command.total,
                    command.total_errors,
                    if safety_locked { "ON" } else { "off" },
                ),
            },
            // The cloud and the IMU come in on different UDP ports from the MID360, so one can
            // stop without the other; the row takes the worse of the two
            Row {
                name: "lidar",
                status: cloud_status.max(imu_status),
                rate_hz: cloud.rate_hz,
                age: cloud.age,
                detail: format!(
                    "points {}, imu {} {:.1} Hz",
                    cloud
                        .last_value
                        .map_or_else(|| "-".to_owned(), |n| n.to_string()),
                    imu_status.label(),
                    imu.rate_hz,
                ),
            },
            Row {
                name: "camera",
                status: classify_stream(&camera, thresholds.camera_timeout),
                rate_hz: camera.rate_hz,
                age: camera.age,
                detail: format!(
                    "last frame {}",
                    camera
                        .last_value
                        .map_or_else(|| "-".to_owned(), |b| format!("{} KB", b / 1024)),
                ),
            },
        ]
    }
}

/// The periodic stdout table. Fixed column widths so consecutive reports line up in a pane.
pub fn render_table(uptime: Duration, rows: &[Row]) -> String {
    let secs = uptime.as_secs();
    let mut out = format!(
        "[Monitor] Sensor status (uptime {:02}:{:02}:{:02})\n  {:<10} {:<8} {:>9} {:>9}  DETAIL",
        secs / 3600,
        secs / 60 % 60,
        secs % 60,
        "SENSOR",
        "STATUS",
        "RATE",
        "AGE",
    );
    for row in rows {
        let _ = write!(
            out,
            "\n  {:<10} {:<8} {:>6.1} Hz {:>9}  {}",
            row.name,
            row.status.label(),
            row.rate_hz,
            format_age(row.age),
            row.detail,
        );
    }
    out
}

pub fn format_age(age: Option<Duration>) -> String {
    age.map_or_else(|| "-".to_owned(), |a| format!("{:.2} s", a.as_secs_f64()))
}

/// A poisoned lock only means another thread panicked mid-update of plain counters; the data is
/// still usable for a status display.
fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
    use super::*;

    const THRESHOLDS: Thresholds = Thresholds {
        telemetry_timeout: Duration::from_secs(1),
        lidar_timeout: Duration::from_millis(500),
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
        assert_eq!(
            statuses,
            [Status::NoData, Status::Idle, Status::NoData, Status::NoData]
        );
        assert_eq!(rows[1].detail, "sent 0, failed 0, safety_lock off");
    }

    #[test]
    fn lidar_row_takes_the_worse_stream() {
        let health = SensorHealth::default();
        health.record_lidar_cloud(19_968);
        let rows = health.rows(Instant::now(), &THRESHOLDS, true);
        // Cloud is fine but the IMU never arrived
        assert_eq!(rows[2].status, Status::NoData);
        assert_eq!(rows[2].detail, "points 19968, imu NO DATA 0.0 Hz");
        assert_eq!(rows[1].detail, "sent 0, failed 0, safety_lock ON");

        health.record_lidar_imu();
        let rows = health.rows(Instant::now(), &THRESHOLDS, false);
        assert_eq!(rows[2].status, Status::Ok);
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
        assert_eq!(rows[3].status, Status::Ok);
        assert_eq!(rows[3].detail, "last frame 100 KB");
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
}
