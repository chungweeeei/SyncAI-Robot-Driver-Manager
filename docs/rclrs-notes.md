# rclrs 筆記：Context、Executor 與 Worker

> 依據 rclrs 0.8 原始碼整理（`/opt/ros2_rust_underlay/src/ros2-rust/ros2_rust/rclrs/src/`）

## 1. 為什麼建立 executor 需要 `Context`？

```rust
let mut executor = Context::default_from_env()?.create_basic_executor();
```

**`Context` 代表一個已經初始化好的 ROS 2 環境**，底層包著 `rcl_context_t`。

- **建立時**：`Context::new` 會呼叫 `rcl_init(argc, argv, init_options, ...)`（`context.rs:158`）。它會做這幾件事：
  - 讀取命令列參數，例如 `--ros-args -r __node:=xxx`、`-p param:=value`（`default_from_env` 就是把 `std::env::args()` 傳進去）
  - 決定 `ROS_DOMAIN_ID`，也就是哪些 node 之間能互相通訊
  - 初始化底層的 RMW/DDS 中介層
- **結束時**：`Drop` 會呼叫 `rcl_shutdown` 和 `rcl_context_fini`（`context.rs:36`）。

所以 executor 跟它建立的所有 node 都必須掛在某個 context 下：

- node 要靠 context 才能在 rcl 層被建立出來（`rcl_node_init` 需要 context）
- executor 的 wait set 也要綁定 context，才能知道「這個 ROS 環境是否已經 shutdown」，例如按 Ctrl+C 後 context 失效，spin 就會停下來

rclrs 的設計是 `Context → Executor → Node`。`create_executor` 只是把 `Arc<ContextHandle>` 交給 executor（`context.rs:203-208`），之後 `executor.create_node()` 建立的 node 就會共用這個 context。透過 `Arc` 的生命週期管理，**只要還有 node 或 executor 存在，context 就不會被提前 shutdown**。這一點用型別系統避免了 C++ 裡常見的「先 `rclcpp::shutdown()` 再用 node」這類錯誤。

對照 rclcpp 的寫法：

```cpp
rclcpp::init(argc, argv);                       // ≈ Context::default_from_env()
rclcpp::executors::SingleThreadedExecutor exec; // ≈ create_basic_executor()
```

rclcpp 用的是全域的 default context，rclrs 則要求你明確持有 context。好處是一個 process 裡可以有多個獨立的 context，例如不同的 domain ID。

## 2. `node.create_worker` 是做什麼的？

```rust
let worker = node.create_worker::<u64>(0);
```

原始碼的說明（`worker.rs:18`）是：

> A worker that carries a payload and synchronizes callbacks for subscriptions and services. Workers share much in common with "callback groups" from rclcpp, with the addition of holding a data payload to share between the callbacks.

**Worker 等於 rclcpp 的 callback group 再加上一份共享狀態（payload）**：

1. **持有一份 payload**：傳入的 `0` 就是初始值，型別是 `u64`，可以換成任何 `Send + Sync + 'static` 的 struct。
2. **同一個 worker 底下的 callback 保證互斥、依序執行**，效果類似 `MutuallyExclusive` callback group。
3. 因為有這個互斥保證，每個 callback 都能直接拿到 `&mut Payload`，**不需要自己包 `Arc<Mutex<T>>`**。

`src/syncai_driver_manager/src/main.rs` 裡的用法：

```rust
worker.create_timer_repeating(Duration::from_secs(1), move |n: &mut u64| {
    *n += 1;   // 直接改 worker 裡的 payload
    ...
})
```

這裡的 `n` 就是 worker 裡那個 `u64`。如果不用 worker，改成 `node.create_timer_repeating`，callback 就拿不到 `&mut` 狀態，得自己寫 `Arc<Mutex<u64>>` 或 `AtomicU64`。

### 什麼時候適合用 worker

當好幾個 callback 要共用、修改同一份狀態時最適合：

```rust
#[derive(Default)]
struct DriverState {
    last_cmd: Option<Twist>,
    fault: bool,
}

let worker = node.create_worker(DriverState::default());

let _sub = worker.create_subscription("cmd_vel", |s: &mut DriverState, msg: Twist| {
    s.last_cmd = Some(msg);
})?;

let _timer = worker.create_timer_repeating(Duration::from_millis(10), |s: &mut DriverState| {
    if !s.fault { /* 用 s.last_cmd 控制馬達 */ }
})?;
```

subscription 跟 timer 都能改 `DriverState`，而且不會發生 data race。

### 其他用途

- `worker.run(|payload| ...)`：從 worker 外面（例如別的 worker 的 callback）送一個任務進去讀或改 payload，會回傳一個 `Promise`，可以 `.await` 取得結果。
- 如果想讓兩組 callback 平行執行、互不阻塞，就建立多個 worker。每個 worker 是一個獨立的互斥單位，需要搭配多執行緒的 executor runtime。

## 總結

| 元件 | 角色 | rclcpp 對應 |
|---|---|---|
| `Context` | ROS 環境本身（`rcl_init`、參數、domain ID、shutdown） | `rclcpp::init` / `rclcpp::Context` |
| `Executor` | 等待事件並分派 callback | `rclcpp::Executor` |
| `Node` | 通訊端點，建立 pub/sub/service | `rclcpp::Node` |
| `Worker<T>` | 互斥的 callback 群組加上共享狀態 `T` | `MutuallyExclusive` callback group 加上自己管的成員變數 |

## 延伸閱讀

- `rclrs/src/node.rs:240-338`：`create_worker` 的 doc comment 與範例
- `rclrs/src/worker.rs`：`WorkerState` 的完整 API
- ros2_rust repo 的 `examples/` 目錄
