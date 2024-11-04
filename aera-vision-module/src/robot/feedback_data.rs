use serde::Deserialize;
use serde_big_array::BigArray;


#[derive(Debug, Clone, Deserialize)]
pub struct FeedbackData {
    pub message_size: u16, // Total message size in bytes
    pub reserved1: [i16; 3],

    pub digital_inputs: i32, // Digital inputs
    pub digital_outputs: i32, // Digital outputs
    pub robot_mode: i32, // Robot mode, 9 indicates an error
    pub time_stamp: i32, // Timestamp (in ms)

    pub reserved2: i32,
    pub test_value: i32, // Memory structure test standard value 0x0123 4567 89AB CDEF
    pub reserved3: f64,

    pub speed_scaling: f64, // Speed scaling
    pub linear_momentum_norm: f64, // Current robot momentum
    pub v_main: f64, // Control board voltage
    pub v_robot: f64, // Robot voltage
    pub i_robot: f64, // Robot current

    pub reserved4: f64,
    pub reserved5: f64,

    pub tool_accelerometer_values: [f64; 3], // TCP acceleration
    pub elbow_position: [f64; 3], // Elbow position
    pub elbow_velocity: [f64; 3], // Elbow velocity

    pub q_target: [f64; 6], // Target joint position
    pub qd_target: [f64; 6], // Target joint speed
    pub qdd_target: [f64; 6], // Target joint acceleration
    pub i_target: [f64; 6], // Target joint current
    pub m_target: [f64; 6], // Target joint torque
    pub q_actual: [f64; 6], // Actual joint position
    pub qd_actual: [f64; 6], // Actual joint speed
    pub i_actual: [f64; 6], // Actual joint current
    pub i_control: [f64; 6], // TCP sensor force value
    pub tool_vector_actual: [f64; 6], // TCP actual Cartesian coordinates
    pub tcp_speed_actual: [f64; 6], // TCP actual speed value
    pub tcp_force: [f64; 6], // TCP force value
    pub tool_vector_target: [f64; 6], // TCP target Cartesian coordinates
    pub tcp_speed_target: [f64; 6], // TCP target speed value
    pub motor_temperatures: [f64; 6], // Joint temperatures
    pub joint_modes: [f64; 6], // Joint control mode
    pub v_actual: [f64; 6], // Joint voltage

    pub hand_type: [u8; 4], // Hand type
    pub user: u8, // User coordinates
    pub tool: u8, // Tool coordinates
    pub run_queued_cmd: u8, // Queue algorithm run flag
    pub pause_cmd_flag: u8, // Queue algorithm pause flag
    pub velocity_ratio: u8, // Joint speed ratio (0~100)
    pub acceleration_ratio: u8, // Joint acceleration ratio (0~100)
    pub jerk_ratio: u8, // Joint jerk ratio (0~100)
    pub xyz_velocity_ratio: u8, // Cartesian position speed ratio (0~100)
    pub r_velocity_ratio: u8, // Cartesian attitude speed ratio (0~100)
    pub xyz_acceleration_ratio: u8, // Cartesian position acceleration ratio (0~100)
    pub r_acceleration_ratio: u8, // Cartesian attitude acceleration ratio (0~100)
    pub xyz_jerk_ratio: u8, // Cartesian position jerk ratio (0~100)
    pub r_jerk_ratio: u8, // Cartesian attitude jerk ratio (0~100)

    pub brake_status: u8, // Robot brake status
    pub enable_status: u8, // Robot enable status
    pub drag_status: u8, // Robot drag status
    pub running_status: u8, // Robot running status
    pub error_status: u8, // Robot error status
    pub jog_status: u8, // Robot jog status
    pub robot_type: u8, // Robot type
    pub drag_button_signal: u8, // Drag button signal on control panel
    pub enable_button_signal: u8, // Enable button signal on control panel
    pub record_button_signal: u8, // Record button signal on control panel
    pub reappear_button_signal: u8, // Reappear button signal on control panel
    pub jaw_button_signal: u8, // Jaw control signal on control panel
    pub six_force_online: u8, // Six-axis force online status

    #[serde(with = "BigArray")]
    pub reserved6: [u8; 82],

    pub m_actual: [f64; 6], // Actual torque
    pub load: f64, // Load weight in kg
    pub center_x: f64, // Offset distance in X direction (mm)
    pub center_y: f64, // Offset distance in Y direction (mm)
    pub center_z: f64, // Offset distance in Z direction (mm)
    pub user_value: [f64; 6], // User coordinate values
    pub tools: [f64; 6], // Tool coordinate values
    pub trace_index: f64, // Trace replay index
    pub six_force_value: [f64; 6], // Raw six-axis force data
    pub target_quaternion: [f64; 4], // Target quaternion [qw, qx, qy, qz]
    pub actual_quaternion: [f64; 4], // Actual quaternion [qw, qx, qy, qz]

    pub reserved7: [u8; 24],
}