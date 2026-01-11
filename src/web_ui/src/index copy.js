// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

let velocity_actuation = 0;
let velocity_feedback = 0;
let steering_actuation = 0;
let steering_feedback = 0;
let battery_value = 45;
let global_fsm_value = 0;
let master_fsm_value = 0;
let master_transmission_value = 0;
let detected_aruco_value = -1;
let obs_emergency_value = 0;
let slam_ref_id_value = 0;
let slam_lc_id_value = 0;
let slam_prox_id_value = 0;
let slam_lm_id_value = 0;

let ui_control_btn = 0;
let ui_target_velocity = -1.0;
let ui_target_steering = 0;

// ================================================================

// Connect to the ROS bridge WebSocket server
var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
});

ros.on("connection", function () {
    console.log("Connected to WebSocket server.");
});

ros.on("error", function (error) {
    console.log("Error connecting to WebSocket server:", error);
});

ros.on("close", function () {
    console.log("Connection to WebSocket server closed.");
});

var sub_master2ui = new ROSLIB.Topic({
    ros: ros,
    name: "/master/to_ui",
    messageType: "std_msgs/Float32MultiArray",
});

sub_master2ui.subscribe(function (message) {
    velocity_actuation = message.data[0];
    steering_actuation = message.data[1];
    velocity_feedback = message.data[2];
    steering_feedback = message.data[3];
});

var sub_master_global_fsm = new ROSLIB.Topic({
    ros: ros,
    name: "/master/global_fsm",
    messageType: "std_msgs/Int16",
});

sub_master_global_fsm.subscribe(function (message) {
    global_fsm_value = message.data;
});

var sub_master_local_fsm = new ROSLIB.Topic({
    ros: ros,
    name: "/master/local_fsm",
    messageType: "std_msgs/Int16",
});

sub_master_local_fsm.subscribe(function (message) {
    master_fsm_value = message.data;
});

var sub_battery = new ROSLIB.Topic({
    ros: ros,
    name: "/can/battery",
    messageType: "std_msgs/Int16",
});

sub_battery.subscribe(function (message) {
    battery_value = message.data;
});

var sub_master_transmission = new ROSLIB.Topic({
    ros: ros,
    name: "/master/transmission",
    messageType: "std_msgs/Int16",
});

sub_master_transmission.subscribe(function (message) {
    master_transmission_value = message.data;
});

var sub_aruco = new ROSLIB.Topic({
    ros: ros,
    name: "/aruco_detection/aruco_nearest_marker_id",
    messageType: "std_msgs/Int16",
});

sub_aruco.subscribe(function (message) {
    detected_aruco_value = message.data;
});

var sub_obs_emergency = new ROSLIB.Topic({
    ros: ros,
    name: "/master/obs_find",
    messageType: "std_msgs/Float32",
});
sub_obs_emergency.subscribe(function (message) {
    obs_emergency_value = message.data;
});

var slam_info = new ROSLIB.Topic({
    ros: ros,
    name: '/slam/info',
    messageType: 'rtabmap_msgs/Info'
});

slam_info.subscribe(function (message) {
    slam_ref_id_value = message.ref_id;
    slam_lc_id_value = message.loop_closure_id;
    slam_prox_id_value = message.proximity_detection_id;
    slam_lm_id_value = message.landmark_id;
}
);

const topic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/ui_control_btn',
    messageType: 'std_msgs/UInt16'
});

const topic_velocity_and_steering = new ROSLIB.Topic({
    ros: ros,
    name: '/master/ui_target_velocity_and_steering',
    messageType: 'std_msgs/Float32MultiArray'
});

document.addEventListener('keydown', function (event) {
    if (event.key == "j") {
        ui_target_velocity = 0.48;
    }
    else if (event.key == 'u') {
        ui_target_velocity = 1.0;
    }
    else if (event.key == 'i') {
        ui_target_velocity = 1.97;
    }
    else if (event.key == 'm') {
        ui_target_steering = steering_feedback - 0.1;
    }
    else if (event.key == 'n') {
        ui_target_steering = steering_feedback;
    }
    else if (event.key == 'b') {
        ui_target_steering = steering_feedback + 0.1;
    }
    else if (event.key == ' ') {
        ui_target_velocity = -1;
        ui_target_steering = 0;
    }
    else if (event.key == 'g') {
        ui_target_velocity = -2;
    }
    else if (event.key == 'v') {
        ui_target_velocity = -3;
    }

});

// ================================================================

function set_velocity(selector1, selector2, id_text, value1, value2) {

    if (value1 > 100) value1 = 100;
    if (value2 > 100) value2 = 100;

    const new_value1 = value1 / 100 * 75;
    const new_value2 = value2 / 100 * 75;

    const progressText = document.getElementById(id_text);
    var circle = document.querySelector(selector1);
    var circle2 = document.querySelector(selector2);
    var length = circle.getTotalLength(); // Get the total length of the circle's path
    var length2 = circle2.getTotalLength();

    circle.style.strokeDasharray = length; // Set the stroke dasharray to the total length of the circle
    circle.style.strokeDashoffset = length - (length * (new_value1 / 100)); // Initially hide the stroke

    circle2.style.strokeDasharray = length2; // Set the stroke dasharray to the total length of the circle
    circle2.style.strokeDashoffset = length2 - (length2 * (new_value2 / 100)); // Initially hide the stroke

    value1_display = value1 * 0.1;
    progressText.textContent = `${value1_display.toFixed(2)}`;
}

function set_steering(selector1, selector2, id_text, value1, value2, raw_actuation) {

    value1 = value1 * -1;
    value2 = value2 * -1;


    if (value1 > 100) value1 = 100;
    if (value2 > 100) value2 = 100;

    const new_value1 = value1 / 100 * 75;
    const new_value2 = value2 / 100 * 75;

    const progressText = document.getElementById(id_text);
    var circle = document.querySelector(selector1);
    var circle2 = document.querySelector(selector2);
    var length = circle.getTotalLength(); // Get the total length of the circle's path
    var length2 = circle2.getTotalLength();

    circle.style.strokeDasharray = length; // Set the stroke dasharray to the total length of the circle
    circle.style.strokeDashoffset = length - (length * (new_value1 / 100)); // Initially hide the stroke

    circle2.style.strokeDasharray = length2; // Set the stroke dasharray to the total length of the circle
    circle2.style.strokeDashoffset = length2 - (length2 * (new_value2 / 100)); // Initially hide the stroke

    circle.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle.style.transformOrigin = "50% 50%"; // Ensure rotation is centered
    circle2.style.transform = "rotate(-90deg)"; // Adjust the rotation as needed
    circle2.style.transformOrigin = "50% 50%"; // Ensure rotation is centered

    progressText.textContent = `${raw_actuation.toFixed(2)}`;
}

function control_display_fsm() {
    if (global_fsm_value == 0) {
        global_fsm.innerHTML = ": INIT"
    }
    else if (global_fsm_value == 1) {
        global_fsm.innerHTML = ": Pre-Operation"
    }
    else if (global_fsm_value == 2) {
        global_fsm.innerHTML = ": Safe-Operation"
    }
    else if (global_fsm_value == 3) {
        global_fsm.innerHTML = ": Operation Mode 3"
    }
    else if (global_fsm_value == 4) {
        global_fsm.innerHTML = ": Operation Mode 4"
    }
    else if (global_fsm_value == 5) {
        global_fsm.innerHTML = ": Operation Mode 5"
    }
    else if (global_fsm_value == 6) {
        global_fsm.innerHTML = ": Operation Mode 2"
    }


    if (master_fsm_value == 0) {
        master_fsm.innerHTML = ": Pre-Follow lane"
    }
    else if (master_fsm_value == 1) {
        master_fsm.innerHTML = ": Follow Lane"
    }
    else if (master_fsm_value == 2) {
        master_fsm.innerHTML = ": Menunggu di Station 1"
    }
    else if (master_fsm_value == 3) {
        master_fsm.innerHTML = ": Menunggu di Station 2"
    }
    else if (master_fsm_value == 4) {
        master_fsm.innerHTML = ": Berhenti Sejenak"
    }

    if (master_transmission_value == 0) {
        master_transmission.innerHTML = ": Auto"
    }
    else if (master_transmission_value == 1) {
        master_transmission.innerHTML = ": Neutral"
    }
    else if (master_transmission_value == 3) {
        master_transmission.innerHTML = ": Forward"
    }
    else if (master_transmission_value == 5) {
        master_transmission.innerHTML = ": Reverse"
    }
}

function ui_control_controller() {
    /**
     * Bit 0 untuk enable control 
     * Bit 1 untuk mode kontrol (0 = otomatis, 1 = manual)
     * Bit 2-3-4 untuk mode manual (0b011 = full otomatis, 0b101 = steer manual, 0b110 = gas manual)
     * Bit 5-6-7 untuk transmisi (0b011 = forward, 0b001 = neutral, 0b110 = reverse)
     */
    ui_control_btn = 0;

    if (kontrol_enable.checked) {
        mode_otomatis.disabled = false;
        mode_manual.disabled = false;

        if (kontrol_enable.checked) ui_control_btn |= 1;
        else ui_control_btn &= ~(1);

        if (mode_otomatis.checked) ui_control_btn &= ~(1 << 1);
        else ui_control_btn |= (1 << 1);

        if (mode_manual.checked) {
            full_manual.disabled = false;
            steer_manual.disabled = false;
            gas_manual.disabled = false;
            kontrol_hardware.disabled = false;

            if (full_manual.checked) {
                ui_control_btn &= ~(7 << 2);
                ui_control_btn |= (6 << 2);
            }
            else if (steer_manual.checked) {
                ui_control_btn &= ~(7 << 2);
                ui_control_btn |= (4 << 2);
            }
            else if (gas_manual.checked) {
                ui_control_btn &= ~(7 << 2);
                ui_control_btn |= (5 << 2);
            }
            else if (kontrol_hardware.checked) {
                ui_control_btn &= ~(7 << 2);
                ui_control_btn |= (2 << 2);
            }

            if (gas_manual.checked || full_manual.checked) {
                transmisi_forward.disabled = false;
                transmisi_neutral.disabled = false;
                transmisi_reverse.disabled = false;
                transmisi_auto.disabled = false;
                if (transmisi_forward.checked) {
                    ui_control_btn &= ~(7 << 5);
                    ui_control_btn |= (3 << 5);
                }
                else if (transmisi_neutral.checked) {
                    ui_control_btn &= ~(7 << 5);
                    ui_control_btn |= (1 << 5);
                }
                else if (transmisi_reverse.checked) {
                    ui_control_btn &= ~(7 << 5);
                    ui_control_btn |= (5 << 5);
                }
                else if (transmisi_auto.checked) {
                    ui_control_btn &= ~(7 << 5);
                }
            }
            /**
             * Sementara karena menunggu beckhoff dari cina
             */
            else if (kontrol_hardware.checked) {
                transmisi_forward.disabled = false;
                transmisi_neutral.disabled = false;
                transmisi_reverse.disabled = false;
                transmisi_auto.disabled = false;
                if (transmisi_forward.checked) {
                    ui_control_btn &= ~(7 << 5);
                    ui_control_btn |= (3 << 5);
                }
                else if (transmisi_neutral.checked) {
                    ui_control_btn &= ~(7 << 5);
                    ui_control_btn |= (1 << 5);
                }
                else if (transmisi_reverse.checked) {
                    ui_control_btn &= ~(7 << 5);
                    ui_control_btn |= (5 << 5);
                }
                else if (transmisi_auto.checked) {
                    ui_control_btn &= ~(7 << 5);
                }
            }
            else {
                transmisi_forward.disabled = true;
                transmisi_neutral.disabled = true;
                transmisi_reverse.disabled = true;
                transmisi_auto.disabled = true;
            }
        }
        else {
            full_manual.disabled = true;
            steer_manual.disabled = true;
            gas_manual.disabled = true;
            kontrol_hardware.disabled = true;
            transmisi_forward.disabled = true;
            transmisi_neutral.disabled = true;
            transmisi_reverse.disabled = true;
            transmisi_auto.disabled = true;
        }

        topic.publish({ data: ui_control_btn });
        topic_velocity_and_steering.publish({ data: [ui_target_velocity, ui_target_steering] });
    }
    else {
        mode_otomatis.disabled = true;
        mode_manual.disabled = true;
        full_manual.disabled = true;
        steer_manual.disabled = true;
        gas_manual.disabled = true;
        kontrol_hardware.disabled = true;
        transmisi_forward.disabled = true;
        transmisi_neutral.disabled = true;
        transmisi_reverse.disabled = true;
        transmisi_auto.disabled = true;
    }
}

// ================================================================

const velocity_kmph = document.getElementById('velocity-kmph');
const steering_rad = document.getElementById('steering-rad');
const baterai_dom = document.getElementById('baterai-dom');
const global_fsm = document.getElementById('global-fsm');
const master_fsm = document.getElementById('master-fsm');
const mode_otomatis = document.getElementById('mode-otomatis');
const mode_manual = document.getElementById('mode-manual');
const full_manual = document.getElementById('full-manual');
const steer_manual = document.getElementById('steer-manual');
const gas_manual = document.getElementById('gas-manual');
const kontrol_hardware = document.getElementById('kontrol-hardware');
const transmisi_forward = document.getElementById('transmisi-forward');
const transmisi_neutral = document.getElementById('transmisi-neutral');
const transmisi_reverse = document.getElementById('transmisi-reverse');
const transmisi_auto = document.getElementById('transmisi-auto');
const kontrol_disable = document.getElementById('kontrol-disable');
const kontrol_enable = document.getElementById('kontrol-enable');
const master_transmission = document.getElementById('master-transmission');
const detected_aruco = document.getElementById('detected-aruco');
const obs_emergency = document.getElementById('obs-emergency');
const slam_ref_id = document.getElementById('slam-ref-id');
const slam_lc_id = document.getElementById('slam-lc-id');
const slam_prox_id = document.getElementById('slam-prox-id');
const slam_lm_id = document.getElementById('slam-lm-id');

setInterval(() => {
    let velocity_actuation_display = velocity_actuation * 10 * 3.6;
    let velocity_feedback_display = velocity_feedback * 10 * 3.6;
    let steering_actuation_display = (steering_actuation) * 135 / 6.28;
    let steering_feedback_display = (steering_feedback) * 135 / 6.28;

    let velocity_feedback_kmh = velocity_feedback * 3.6;

    set_velocity('.velocity-circle1', '.velocity-circle2', 'velocity-text', velocity_actuation_display, velocity_feedback_display);
    velocity_kmph.textContent = `${velocity_feedback_kmh.toFixed(2)} km/h`;
    set_steering('.steering-circle1', '.steering-circle2', 'steering-text', steering_actuation_display, steering_feedback_display, steering_actuation);
    steering_rad.textContent = `${steering_feedback.toFixed(2)} rad`;

    if (battery_value > 50) {
        baterai_dom.className = "progress is-success"
    }
    else if (battery_value > 40) {
        baterai_dom.className = "progress is-warning"
    }
    else {
        baterai_dom.className = "progress is-danger"
    }
    baterai_dom.value = battery_value;

    control_display_fsm();

    detected_aruco.textContent = detected_aruco_value;
    obs_emergency.textContent = obs_emergency_value;
    slam_ref_id.textContent = slam_ref_id_value;
    slam_lc_id.textContent = slam_lc_id_value;
    slam_prox_id.textContent = slam_prox_id_value;
    slam_lm_id.textContent = slam_lm_id_value;

    ui_control_controller();
}, 50);
