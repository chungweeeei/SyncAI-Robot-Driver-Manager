//! Unit tests of `service.rs`, included from there with `#[path]`: still a child module of
//! `service`, so `use super::*` reaches its private items.

use std::net::UdpSocket;
use std::time::Duration;

use super::super::session::UdpSession;
use super::*;

/// A `CommandLink` connected to a loopback listener, plus the listener to read it from
fn loopback_link() -> (CommandLink, UdpSocket) {
    let listener = UdpSocket::bind("127.0.0.1:0").unwrap();
    listener
        .set_read_timeout(Some(Duration::from_secs(1)))
        .unwrap();
    let session = UdpSession::connect(listener.local_addr().unwrap()).unwrap();
    (
        CommandLink::new(session, Arc::default(), Logger::default()),
        listener,
    )
}

fn recv(listener: &UdpSocket) -> String {
    let mut buf = [0u8; 64];
    let n = listener.recv(&mut buf).unwrap();
    String::from_utf8_lossy(&buf[..n]).into_owned()
}

fn assert_nothing_received(listener: &UdpSocket) {
    let mut buf = [0u8; 64];
    assert!(listener.recv(&mut buf).is_err(), "unexpected datagram");
}

#[test]
fn motion_key_is_sent_when_unlocked() {
    let (command, listener) = loopback_link();
    let safety = SafetyLock::default();

    let resp = set_motion_key(&command, &safety, "0");
    assert!(resp.success);
    assert_eq!(resp.message, "Motion key sent");
    assert_eq!(recv(&listener), "MODE Z\n");

    let resp = set_motion_key(&command, &safety, protocol::ESTOP_KEY);
    assert!(resp.success);
    assert_eq!(resp.message, "Emergency stop sent");
    assert_eq!(recv(&listener), "ESTOP\n");
}

#[test]
fn unknown_motion_key_sends_nothing() {
    let (command, listener) = loopback_link();
    let safety = SafetyLock::default();

    let resp = set_motion_key(&command, &safety, "9");
    assert!(!resp.success);
    assert_eq!(resp.message, "Unknown motion key '9'");
    assert_nothing_received(&listener);
}

#[test]
fn safety_lock_lies_down_once_and_only_lets_estop_through() {
    let (command, listener) = loopback_link();
    let safety = SafetyLock::default();
    let logger = Logger::default();

    safety.trigger("test", &command, &logger);
    safety.trigger("test again", &command, &logger);
    assert!(safety.is_engaged());
    assert_eq!(recv(&listener), protocol::LIE_DOWN_COMMAND);
    assert_nothing_received(&listener);

    let resp = set_motion_key(&command, &safety, "1");
    assert!(!resp.success);
    assert_eq!(resp.message, "LOCKED");
    assert_nothing_received(&listener);

    let resp = set_motion_key(&command, &safety, protocol::ESTOP_KEY);
    assert!(resp.success);
    assert_eq!(recv(&listener), "ESTOP\n");

    assert!(safety.release());
    assert!(!safety.release());
    assert!(set_motion_key(&command, &safety, "1").success);
    assert_eq!(recv(&listener), "MODE C\n");
}

#[test]
fn set_safety_lock_engages_silently_and_releases_with_a_controller_reset() {
    let (command, listener) = loopback_link();
    let safety = SafetyLock::default();
    let logger = Logger::default();

    // Releasing while this node is not locked still resets the controller, which may have
    // locked itself
    let resp = set_safety_lock(&command, &safety, &logger, false);
    assert!(resp.success);
    assert_eq!(
        resp.message,
        "Controller reset. Send motion key 0 to stand up."
    );
    assert!(!safety.is_engaged());
    assert_eq!(recv(&listener), "RESET_SAFETY\n");

    let resp = set_safety_lock(&command, &safety, &logger, true);
    assert!(resp.success);
    assert_eq!(resp.message, "Safety lock engaged. Remote control blocked.");
    assert!(safety.is_engaged());

    let resp = set_safety_lock(&command, &safety, &logger, true);
    assert!(resp.success);
    assert_eq!(resp.message, "System was already locked.");
    assert!(safety.is_engaged());

    // Engaging from outside sends no lie-down, but still gates motion keys
    assert!(!set_motion_key(&command, &safety, "1").success);
    assert_nothing_received(&listener);

    let resp = set_safety_lock(&command, &safety, &logger, false);
    assert!(resp.success);
    assert_eq!(
        resp.message,
        "Safety lock released and controller reset. Send motion key 0 to stand up."
    );
    assert!(!safety.is_engaged());
    assert_eq!(recv(&listener), "RESET_SAFETY\n");
    assert_nothing_received(&listener);
}
