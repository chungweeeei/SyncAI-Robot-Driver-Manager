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
/// Declared from best to worst, so `Ord` / `max` give the worse of two statuses.
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
    pub imu_timeout: Duration,
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

/// How many rows `SensorHealth::rows` returns (telemetry, command, lidar, imu, camera)
pub const ROW_COUNT: usize = 5;

/// What every row reads before anything arrives, in `rows` order: the monitor starts from it
/// so the first evaluation does not log a "transition" for a sensor that has not started yet
pub const INITIAL_STATUSES: [Status; ROW_COUNT] = [
    Status::NoData,
    Status::Idle,
    Status::NoData,
    Status::NoData,
    Status::NoData,
];

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

    /// The rows of the table, in display order: telemetry, command, lidar, imu, camera.
    /// `safety_locked` is shown on the command row, since the lock is what gates that link.
    pub fn rows(
        &self,
        now: Instant,
        thresholds: &Thresholds,
        safety_locked: bool,
    ) -> [Row; ROW_COUNT] {
        let telemetry = lock(&self.telemetry).snapshot(now);
        let command = lock(&self.command).snapshot(now);
        let cloud = lock(&self.lidar_cloud).snapshot(now);
        let imu = lock(&self.lidar_imu).snapshot(now);
        let camera = lock(&self.camera).snapshot(now);
        let from = *lock(&self.telemetry_from);

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
            // The cloud and the IMU are separate rows with their own timeouts: they can stop
            // independently (on the G23 they come in on different UDP ports from the MID360,
            // and the M20's /IMU is the vendor's, not the lidar's), and a row of its own gives
            // the IMU its own rate and age
            Row {
                name: "lidar",
                status: classify_stream(&cloud, thresholds.lidar_timeout),
                rate_hz: cloud.rate_hz,
                age: cloud.age,
                detail: format!(
                    "points {}",
                    cloud
                        .last_value
                        .map_or_else(|| "-".to_owned(), |n| n.to_string()),
                ),
            },
            Row {
                name: "imu",
                status: classify_stream(&imu, thresholds.imu_timeout),
                rate_hz: imu.rate_hz,
                age: imu.age,
                detail: format!("received {}", imu.total),
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
#[path = "tests/health.rs"]
mod tests;
