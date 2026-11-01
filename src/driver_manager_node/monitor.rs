use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use rclrs::*;
use ros_env::livox_ros_driver2::msg::CustomMsg;
use ros_env::sensor_msgs::msg::{CompressedImage, Imu};

use super::command::SafetyLock;
use super::health::{self, SensorHealth, Status, Thresholds};

// Statuses are re-evaluated this often, so a transition is logged within half a second of the
// stream crossing its timeout. The thread sleeps in short steps to notice shutdown quickly.
const EVALUATE_PERIOD: Duration = Duration::from_millis(500);
const SLEEP_STEP: Duration = Duration::from_millis(100);

/// The lidar and camera topics, subscribed only to count them: nothing here uses their data.
///
/// One worker of their own, so the camera's 30 Hz JPEGs and the lidar IMU's 200 Hz never queue
/// behind cmd_vel or a service. Every subscription is best-effort, which matches both a
/// reliable and a best-effort publisher; relative names resolve under `/<robot_id>`, where the
/// Livox driver and syncai_camera_feeder publish.
pub struct SensorSubscriptions {
    _worker: Worker<Arc<SensorHealth>>,
    _lidar_cloud: WorkerSubscription<CustomMsg, Arc<SensorHealth>>,
    _lidar_imu: WorkerSubscription<Imu, Arc<SensorHealth>>,
    _camera: WorkerSubscription<CompressedImage, Arc<SensorHealth>>,
}

impl SensorSubscriptions {
    pub fn create(node: &Node, health: Arc<SensorHealth>) -> Result<Self, RclrsError> {
        let worker = node.create_worker(health);

        Ok(Self {
            // CustomMsg is what the Livox driver publishes in the nav and mapping sessions
            // (xfer_format 1). The CALIBRATION session switches it to PointCloud2, but that
            // session does not run this node.
            _lidar_cloud: worker.create_subscription(
                "livox/lidar".sensor_data_qos(),
                |health: &mut Arc<SensorHealth>, msg: CustomMsg| {
                    health.record_lidar_cloud(msg.point_num);
                },
            )?,
            _lidar_imu: worker.create_subscription(
                "livox/imu".sensor_data_qos(),
                |health: &mut Arc<SensorHealth>, _msg: Imu| health.record_lidar_imu(),
            )?,
            _camera: worker.create_subscription(
                "image_raw/compressed".sensor_data_qos(),
                |health: &mut Arc<SensorHealth>, msg: CompressedImage| {
                    health.record_camera(msg.data.len());
                },
            )?,
            _worker: worker,
        })
    }
}

/// Handle to the thread that prints the status table and logs transitions; dropping it stops
/// and joins the thread.
pub struct MonitorWorker {
    running: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl MonitorWorker {
    pub fn spawn(
        health: Arc<SensorHealth>,
        safety: Arc<SafetyLock>,
        thresholds: Thresholds,
        report_period: Duration,
        logger: Logger,
    ) -> io::Result<Self> {
        let running = Arc::new(AtomicBool::new(true));
        let handle = thread::Builder::new().name("monitor".into()).spawn({
            let running = Arc::clone(&running);
            let monitor = MonitorLoop {
                health,
                safety,
                thresholds,
                report_period,
                logger,
            };
            move || monitor.run(&running)
        })?;

        Ok(Self {
            running,
            handle: Some(handle),
        })
    }
}

impl Drop for MonitorWorker {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

struct MonitorLoop {
    health: Arc<SensorHealth>,
    safety: Arc<SafetyLock>,
    thresholds: Thresholds,
    report_period: Duration,
    logger: Logger,
}

impl MonitorLoop {
    fn run(&self, running: &AtomicBool) {
        let started = Instant::now();
        // What every row reads before anything arrives, so the first evaluation does not log a
        // "transition" for a sensor that simply has not started yet
        let mut previous = [Status::NoData, Status::Idle, Status::NoData, Status::NoData];
        let mut next_evaluate = started;
        let mut next_report = started + self.report_period;

        while running.load(Ordering::Relaxed) {
            let now = Instant::now();
            if now >= next_evaluate {
                next_evaluate = now + EVALUATE_PERIOD;
                let rows = self
                    .health
                    .rows(now, &self.thresholds, self.safety.is_engaged());

                for (row, prev) in rows.iter().zip(previous.iter_mut()) {
                    if row.status != *prev {
                        self.log_transition(row, *prev);
                        *prev = row.status;
                    }
                }

                if now >= next_report {
                    next_report = now + self.report_period;
                    log_info!(
                        &self.logger,
                        "{}",
                        health::render_table(now - started, &rows)
                    );
                }
            }
            thread::sleep(SLEEP_STEP);
        }
    }

    fn log_transition(&self, row: &health::Row, from: Status) {
        // OK <-> IDLE is the command link starting and stopping with cmd_vel: normal, and
        // frequent while someone teleoperates, so it is left to the periodic table
        if row.status.is_healthy() && from.is_healthy() {
            return;
        }
        let line = format!(
            "[Monitor] {}: {} -> {} (last message {} ago; {})",
            row.name,
            from.label(),
            row.status.label(),
            health::format_age(row.age),
            row.detail,
        );
        if row.status.is_healthy() {
            log_info!(&self.logger, "{line}");
        } else {
            log_warn!(&self.logger, "{line}");
        }
    }
}
