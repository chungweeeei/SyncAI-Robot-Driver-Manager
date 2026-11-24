use std::time::Duration;

use rclrs::*;
use ros_env::drdds::msg::BatteryData;
use ros_env::sensor_msgs::msg::BatteryState;

use super::m20_battery::{self, PackReading};
use super::publishers;

const LOG_THROTTLE: Duration = Duration::from_secs(10);

/// The Lynx M20's `battery_state`: the vendor's `/BATTERY_DATA` (`drdds/msg/BatteryData`)
/// re-published as the `sensor_msgs/BatteryState` the G23's telemetry thread publishes, on the
/// same topic with the same QoS, so `syncai_robot_state` and charge.xml's `WaitForCharging` read
/// either robot alike. The packs are combined in `m20_battery.rs`.
///
/// One Worker of its own: one message a second, and nothing else on it to wait behind.
pub struct M20BatteryForwarder {
    _worker: Worker<Forwarder>,
    _subscription: WorkerSubscription<BatteryData, Forwarder>,
}

impl M20BatteryForwarder {
    pub fn create(node: &Node, topic: &str) -> Result<Self, RclrsError> {
        let worker = node.create_worker(Forwarder {
            publisher: publishers::create_battery_state(node)?,
            clock: node.get_clock(),
            logger: node.logger().clone(),
            first: true,
        });
        // Reliable keep-last, like the vendor's writer and its own rl_deploy reader (depth 10).
        // A best-effort reader would match too; at 1 Hz there is nothing to gain from it.
        let subscription = worker.create_subscription(
            topic.keep_last(10).reliable(),
            |forwarder: &mut Forwarder, msg: BatteryData| forwarder.on_reading(&msg),
        )?;
        Ok(Self {
            _worker: worker,
            _subscription: subscription,
        })
    }
}

struct Forwarder {
    publisher: Publisher<BatteryState>,
    clock: Clock,
    logger: Logger,
    first: bool,
}

impl Forwarder {
    fn on_reading(&mut self, msg: &BatteryData) {
        let packs: Vec<PackReading> = msg
            .data
            .iter()
            .map(|p| PackReading {
                voltage: p.voltage,
                current: p.current,
                remaining_capacity: p.remaining_capacity,
                nominal_capacity: p.nominal_capacity,
                battery_level: p.battery_level,
                temperatures: &p.battery_temperature,
            })
            .collect();

        let Some(battery) = m20_battery::combine(&packs) else {
            log_warn!(
                self.logger.throttle(LOG_THROTTLE),
                "[M20Battery] /BATTERY_DATA has no pack with a voltage ({} slots); \
                 battery_state not published",
                packs.len(),
            );
            return;
        };

        // Our clock, not the message's: the vendor sends a zero stamp (robot01, 2026-11-24)
        let msg = publishers::battery_state_msg(&battery, publishers::stamp_now(&self.clock));
        if let Err(e) = self.publisher.publish(msg) {
            log_warn!(
                self.logger.throttle(LOG_THROTTLE),
                "[M20Battery] Failed to publish battery_state: {e}"
            );
            return;
        }

        // Once, so the log says the battery arrived and from how many packs
        if std::mem::take(&mut self.first) {
            log_info!(
                &self.logger,
                "[M20Battery] First reading: {} of {} packs, soc {:.1}%, {:.2} V, {:.2} A",
                packs.iter().filter(|p| p.voltage > 0).count(),
                packs.len(),
                battery.soc,
                battery.voltage,
                battery.current,
            );
        }
        log_debug!(
            &self.logger,
            "[M20Battery] soc={:.1}% V={:.2} I={:.2} T={:.1}",
            battery.soc,
            battery.voltage,
            battery.current,
            battery.temperature,
        );
    }
}
