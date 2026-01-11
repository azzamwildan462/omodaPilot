class Robot {
    constructor(ip, conv_circle, conv_line, x, y, theta, radius) {
        this.ip = ip;
        this.conv_circle = conv_circle;
        this.conv_line = conv_line;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.radius = radius;
        this.routes = [];
        this.routes_line = [];

        this.color_r = 0;
        this.color_g = 0;
        this.color_b = 0;

        this.has_finished_routes_init = false;
        this.prev_has_finished_routes_init = false;
        this.has_route_drawed = false;

    }
}

// ================================================================================================================================

let robots = [];

// ================================================================================================================================


let wtf_skala = 10;
// let wtf_skala = 60;


// Create a Konva Stage
const stage = new Konva.Stage({
    container: 'map',
    width: window.innerWidth,
    height: window.innerHeight,
});

// Create a layer for grid and shapes
const gridLayer = new Konva.Layer();
const robotLayer = new Konva.Layer();
const mapLayer = new Konva.Layer();
const lidarLayer = new Konva.Layer();
const waypointsLayer = new Konva.Layer();
const terminalsLayer = new Konva.Layer();
const filteredLidarLayer = new Konva.Layer();
stage.add(mapLayer);
stage.add(gridLayer);
stage.add(waypointsLayer);
stage.add(terminalsLayer);
stage.add(lidarLayer);
stage.add(filteredLidarLayer);
stage.add(robotLayer);

// Draw the grid
const gridSize = 50;
for (let x = 0; x < stage.width(); x += gridSize) {
    gridLayer.add(
        new Konva.Line({
            points: [x, 0, x, stage.height()],
            stroke: '#ddd',
            strokeWidth: 1,
        })
    );
}
for (let y = 0; y < stage.height(); y += gridSize) {
    gridLayer.add(
        new Konva.Line({
            points: [0, y, stage.width(), y],
            stroke: '#ddd',
            strokeWidth: 1,
        })
    );
}
gridLayer.draw();

// ================================================================================================================================

// Enable zooming
stage.on('wheel', (e) => {
    e.evt.preventDefault();
    const scaleBy = 1.1;
    const oldScale = stage.scaleX();
    const pointer = stage.getPointerPosition();
    const mousePointTo = {
        x: (pointer.x - stage.x()) / oldScale,
        y: (pointer.y - stage.y()) / oldScale,
    };

    const newScale = e.evt.deltaY > 0 ? oldScale / scaleBy : oldScale * scaleBy;
    stage.scale({ x: newScale, y: newScale });

    const newPos = {
        x: pointer.x - mousePointTo.x * newScale,
        y: pointer.y - mousePointTo.y * newScale,
    };
    stage.position(newPos);
    stage.batchDraw();
});

// Enable map shifting (panning) with the right mouse button
let isDragging = false;
let dragStartPos = { x: 0, y: 0 };

stage.on('mousedown', (e) => {
    if (e.evt.button === 2) { // Check if the right mouse button is pressed
        isDragging = true;
        dragStartPos = stage.getPointerPosition();
    }

    if (e.evt.button === 1 || e.evt.button === 2) {
        isDragging = true;
        dragStartPos = stage.getPointerPosition();
    }
});

stage.on('mousemove', (e) => {
    if (!isDragging) return;

    const pointer = stage.getPointerPosition();
    const dx = pointer.x - dragStartPos.x;
    const dy = pointer.y - dragStartPos.y;

    stage.position({
        x: stage.x() + dx,
        y: stage.y() + dy,
    });
    stage.batchDraw();
    dragStartPos = pointer;
});

stage.on('mouseup', () => {
    isDragging = false;
});

stage.on('contextmenu', (e) => {
    // Prevent the browser's context menu from appearing on right-click
    e.evt.preventDefault();
});

// Handle window resizing
window.addEventListener('resize', () => {
    const width = window.innerWidth;
    const height = window.innerHeight;

    stage.width(width);
    stage.height(height);

    gridLayer.batchDraw();
    robotLayer.batchDraw();
    mapLayer.batchDraw();
    lidarLayer.batchDraw();
});

// ================================================================================================================================

function addRobot(ip, x, y, theta, radius, colourr) {
    const original_x = x;
    const original_y = y;
    const original_theta = theta;

    ip = ip;
    x = x + stage.width() * 0.5 / wtf_skala;
    y = stage.height() * 0.5 / wtf_skala - y;
    theta = theta;
    radius = radius;


    for (let i = 0; i < robots.length; i++) {
        if (robots[i].ip == ip) {
            robots[i].conv_circle.position({ x: x * wtf_skala, y: y * wtf_skala });
            robots[i].conv_line.points([x * wtf_skala, y * wtf_skala, x * wtf_skala + radius * wtf_skala * Math.cos(theta), y * wtf_skala - radius * wtf_skala * Math.sin(theta)]);
            robots[i].x = original_x;
            robots[i].y = original_y;
            robots[i].theta = original_theta;
            robots[i].radius = radius;

            robotLayer.batchDraw();

            return;
        }
    }

    const conv_circle = new Konva.Circle({
        x: x * wtf_skala,
        y: y * wtf_skala,
        radius: radius * wtf_skala,
        fill: colourr,
        draggable: true,
    });

    // Calculate the end point of the line based on theta
    const thetaRadians = theta; // Convert to radians
    const lineEndX = x * wtf_skala + radius * wtf_skala * Math.cos(thetaRadians);
    const lineEndY = y * wtf_skala - radius * wtf_skala * Math.sin(thetaRadians);

    // Draw the line inside the circle
    const conv_line = new Konva.Line({
        points: [x * wtf_skala, y * wtf_skala, lineEndX, lineEndY],
        stroke: 'Cyan',
        strokeWidth: 5,
    });

    let robot_buffer = new Robot(ip, conv_circle, conv_line, x, y, theta, radius);

    robots.push(robot_buffer);

    robotLayer.add(robot_buffer.conv_circle);
    robotLayer.add(robot_buffer.conv_line);
    robotLayer.draw();
}

// ================================================================================================================================

// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

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

const robotTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/slam/odometry/filtered',
    // name: '/odom',
    messageType: 'nav_msgs/Odometry'
});

robotTopic.subscribe(function (message) {
    const x = message.pose.pose.position.x;
    const y = message.pose.pose.position.y;
    const q = message.pose.pose.orientation;
    const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const radius = 2.0;

    addRobot("0.0.0.0", x, y, theta, radius, 'magenta');
}
);

// const robotTopic2 = new ROSLIB.Topic({
//     ros: ros,
//     // name: '/lidar_obstacle_filter/icp_estimate',
//     name: '/master/pose_filtered',
//     messageType: 'nav_msgs/Odometry'
// });

// robotTopic2.subscribe(function (message) {
//     const x = message.pose.pose.position.x;
//     const y = message.pose.pose.position.y;
//     const q = message.pose.pose.orientation;
//     const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
//     const radius = 0.35;

//     addRobot("1.1.1.1", x, y, theta, radius, 'green');
// }
// );

const robotTopic2 = new ROSLIB.Topic({
    ros: ros,
    name: '/slam/localization_pose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
});

robotTopic2.subscribe(function (message) {
    const x = message.pose.pose.position.x;
    const y = message.pose.pose.position.y;
    const q = message.pose.pose.orientation;
    const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const radius = 1.8;

    // console.log("x: " + x + " y: " + y + " theta: " + theta);

    addRobot("1.1.1.1", x, y, theta, radius, 'green');
}
);

let rtabmap_current_ref_id = 0;

const slam_info = new ROSLIB.Topic({
    ros: ros,
    name: '/slam/info',
    messageType: 'rtabmap_msgs/Info'
});

slam_info.subscribe(function (message) {
    rtabmap_current_ref_id = message.ref_id;
}
);

const lidar_tf_x = 1.378;
const lidar_tf_y = 0.0;
const lidar_tf_theta = 0; // radians

// const lidarTopic = new ROSLIB.Topic({
//     ros: ros,
//     name: '/scan',
//     messageType: 'sensor_msgs/LaserScan'
// });

// lidarTopic.subscribe(function (message) {
//     // Wait until rtabmap/robot ready
//     if (robots.length === 0 || rtabmap_current_ref_id === 0) return;

//     const ranges = message.ranges;
//     const angleIncrement = message.angle_increment;
//     const angleMin = message.angle_min; // <-- don't add robot theta here

//     const xr = robots[0].x;
//     const yr = robots[0].y;
//     const theta_r = robots[0].theta;
//     const cos_r = Math.cos(theta_r);
//     const sin_r = Math.sin(theta_r);

//     const theta_L = lidar_tf_theta;
//     const w = stage.width();
//     const h = stage.height();
//     const scale = wtf_skala;

//     const cx = w * 0.5 / scale; // world meters corresponding to screen center
//     const cy = h * 0.5 / scale;

//     const laserPoints = [];
//     for (let i = 0; i < ranges.length; i++) {
//         const r = ranges[i];
//         if (!Number.isFinite(r)) continue; // skip NaN/Inf

//         // 1) laser frame angle, include lidar yaw
//         const a = angleMin + i * angleIncrement + theta_L;

//         // 2) to base frame (rotate by lidar yaw already via +theta_L, then translate)
//         const x_b = r * Math.cos(a) + lidar_tf_x;
//         const y_b = -r * Math.sin(a) + lidar_tf_y;

//         // 3) base -> world (rotate by robot yaw, then translate by robot position)
//         const x_w = xr + (x_b * cos_r - y_b * sin_r);
//         const y_w = yr + (x_b * sin_r + y_b * cos_r);

//         // 4) world -> screen (center-at-screen, invert y, then scale)
//         const x_screen = (x_w + cx) * scale;
//         const y_screen = (cy - y_w) * scale;

//         laserPoints.push(x_screen, y_screen);
//     }

//     // Draw
//     lidarLayer.destroyChildren();
//     const laserLine = new Konva.Line({
//         points: laserPoints,
//         stroke: 'red',
//         strokeWidth: 1
//     });
//     lidarLayer.add(laserLine);
//     lidarLayer.draw();
// });


const filteredLidarTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/obstacle_filter/filtered_lidar_pcl',
    messageType: 'sensor_msgs/PointCloud'
});

filteredLidarTopic.subscribe(function (message) {
    if (robots.length == 0) {
        return;
    }

    const points = message.points;
    const filtered_lidar = [];

    const rotated_x = lidar_tf_x * Math.cos(robots[0].theta) - lidar_tf_y * Math.sin(robots[0].theta);
    const rotated_y = lidar_tf_x * Math.sin(robots[0].theta) + lidar_tf_y * Math.cos(robots[0].theta);

    for (let i = 0; i < points.length; i++) {
        const x = points[i].x + robots[0].x + rotated_x;
        const y = -points[i].y + robots[0].y + rotated_y;

        const x_tf = x + stage.width() * 0.5 / wtf_skala;
        const y_tf = stage.height() * 0.5 / wtf_skala - y;

        filtered_lidar.push(x_tf * wtf_skala, y_tf * wtf_skala);
    }

    // Clear the shape layer
    filteredLidarLayer.destroyChildren();

    // Draw the filtered_lidar
    const FilteredLidar = new Konva.Line({
        points: filtered_lidar,
        stroke: 'orange',
        strokeWidth: 3,
    });
    filteredLidarLayer.add(FilteredLidar);

    filteredLidarLayer.draw();
}
);

const waypointsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/waypoints',
    messageType: 'sensor_msgs/PointCloud'
});

waypointsTopic.subscribe(function (message) {
    const points = message.points;
    const waypoints = [];

    for (let i = 0; i < points.length; i++) {
        const x = points[i].x;
        const y = points[i].y;

        const x_tf = x + stage.width() * 0.5 / wtf_skala;
        const y_tf = stage.height() * 0.5 / wtf_skala - y;

        waypoints.push(x_tf * wtf_skala, y_tf * wtf_skala);
    }

    // Clear the shape layer
    waypointsLayer.destroyChildren();

    // Draw the waypoints
    const waypointsLine = new Konva.Line({
        points: waypoints,
        stroke: 'blue',
        strokeWidth: 5,
    });
    waypointsLayer.add(waypointsLine);

    waypointsLayer.draw();
}
);

const terminalsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/master/terminals',
    messageType: 'ros2_interface/TerminalArray'
});

terminalsTopic.subscribe(function (message) {
    const points = message.terminals;

    // Clear the shape layer
    terminalsLayer.destroyChildren();

    // Draw terminals as outlined circles
    for (let i = 0; i < points.length; i++) {
        const x = points[i].target_pose_x;
        const y = points[i].target_pose_y;
        const radius = points[i].radius_area;
        const arah = points[i].target_pose_theta
        const id = points[i].id

        // Transform coordinates according to your logic
        const x_tf = x + (stage.width() * 0.5) / wtf_skala;
        const y_tf = (stage.height() * 0.5) / wtf_skala - y;

        let color = 'orange'; // Default color

        if (points[i].type == 1)
            color = 'orange';
        else if (points[i].type == 2)
            color = 'yellow';
        else if (points[i].type == 32)
            color = 'green';

        const terminalCircle = new Konva.Circle({
            x: x_tf * wtf_skala,
            y: y_tf * wtf_skala,
            radius: radius * wtf_skala,
            stroke: color,    // Outline color
            strokeWidth: 5,      // Outline thickness
            fill: null,          // Ensure circle is not filled
        });

        const lineEndX = x_tf * wtf_skala + radius * wtf_skala * Math.cos(arah);
        const lineEndY = y_tf * wtf_skala - radius * wtf_skala * Math.sin(arah);

        // Draw the line inside the circle
        const conv_line = new Konva.Line({
            points: [x_tf * wtf_skala, y_tf * wtf_skala, lineEndX, lineEndY],
            stroke: 'Cyan',
            strokeWidth: 5,
        });

        const idText = new Konva.Text({
            x: x_tf * wtf_skala + 5,
            y: y_tf * wtf_skala + 5,
            text: id.toString(),
            fontSize: 24,
            fontFamily: 'Calibri',
            fill: 'magenta',
        });

        // Add circle to the terminalsLayer
        terminalsLayer.add(terminalCircle);
        terminalsLayer.add(conv_line);
        terminalsLayer.add(idText);
    }

    terminalsLayer.draw();
}
);



// Create a ROSLIB Topic to subscribe to the 'ui_test' topic
const mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/map', // Change topic name as needed
    messageType: 'nav_msgs/OccupancyGrid'
});

//===============================================================================================================================
let last_time_update_map = 0;
let mapCanvas = document.createElement("canvas");
let mapCtx = mapCanvas.getContext("2d");

// Subscribe to map messages and draw the map
mapTopic.subscribe(function (message) {
    if (new Date().getTime() - last_time_update_map < 1000) {
        return;
    }
    // console.log(message.info.origin);
    last_time_update_map = new Date().getTime();

    let map = message.data;
    let width = message.info.width;
    let height = message.info.height;

    // Resize the canvas buffer
    mapCanvas.width = width;
    mapCanvas.height = height;

    // Draw the map to the canvas buffer
    let imageData = mapCtx.createImageData(width, height);
    for (let i = 0; i < map.length; i++) {
        let occupancy = map[i];

        let r, g, b;

        if (occupancy === -1) {
            // Unknown space -> Dark Green
            r = 71;
            g = 128;
            b = 118;
        } else if (occupancy === 0) {
            // Free space -> White
            r = 255;
            g = 255;
            b = 255;
        } else {
            // Occupied space -> Color gradient (Red to Orange)
            let t = occupancy / 100; // Normalize to 0-1

            // Interpolate red and green between red (255, 0, 0) and orange (255, 165, 0)
            r = 0; // Red is fixed
            g = Math.round(165 - t * 165); // From 165 to 0 (green scale)
            b = 0; // Blue stays 0 for red-to-orange color
        }

        // Assign colors to image data
        imageData.data[i * 4 + 0] = r;  // Red
        imageData.data[i * 4 + 1] = g;  // Green
        imageData.data[i * 4 + 2] = b;  // Blue
        imageData.data[i * 4 + 3] = 255; // Fully opaque
    }
    mapCtx.putImageData(imageData, 0, 0);

    // console.log(message.info.origin.position.x, " ", message.info.origin.position.y);

    // Convert canvas to Konva image
    // x = x + stage.width() * 0.5 / wtf_skala;
    // y = stage.height() * 0.5 / wtf_skala - y;
    let imageObj = new Image();
    imageObj.onload = function () {
        let mapImage = new Konva.Image({
            image: imageObj,
            width: width * wtf_skala * message.info.resolution,
            height: height * wtf_skala * message.info.resolution,
            x: (message.info.origin.position.x + stage.width() * 0.5 / wtf_skala) * wtf_skala,
            y: (stage.height() * 0.5 / wtf_skala - message.info.origin.position.y) * wtf_skala,
        });

        // Flip Y-axis if the map's origin is bottom-left
        mapImage.scaleY(-1);
        // mapImage.offsetY(height);

        // Clear and add the new map image
        mapLayer.destroyChildren();
        mapLayer.add(mapImage);
        mapLayer.draw();
    };
    imageObj.src = mapCanvas.toDataURL();

    mapTopic.unsubscribe();
});


// ================================================================================================================================

document.addEventListener('keydown', function (event) {
    if (event.key == 's') {
        const startRequest = new ROSLIB.ServiceRequest({
            // Pass any necessary parameters for start_trajectory here, if any
        });

        startTrajectoryService.callService(startRequest, function (response) {
            console.log('Start trajectory response:', response);
            alert('Trajectory started!');
        });
    }
    else if (event.key == 'd') {
        const finishRequest = new ROSLIB.ServiceRequest({
            // Pass any necessary parameters for finish_trajectory here, if any
        });

        finishTrajectoryService.callService(finishRequest, function (response) {
            console.log('Finish trajectory response:', response);
            alert('Trajectory finished!');
        });
    }
});

// ================================================================================================================================

function add_terminal_here(add_terminal) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: add_terminal
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/set_terminal',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Set Terminal:', response);
    });
}

function remove_terminal(remove_all) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: remove_all
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/rm_terminal',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Set Terminal:', response);
    });
}

function rtabmap_set_mode_mapping() {
    const modeRequest = new ROSLIB.ServiceRequest({
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/slam/rtabmap/set_mode_mapping',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Set mode mapping:', response);
    });
}

function rtabmap_set_mode_localization() {
    const modeRequest = new ROSLIB.ServiceRequest({
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/slam/rtabmap/set_mode_localization',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Set mode localization:', response);
    });
}

function rtabmap_trigger_new_map() {
    const modeRequest = new ROSLIB.ServiceRequest({
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/slam/rtabmap/trigger_new_map',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('New Map:', response);
    });
}
function rtabmap_reset() {
    offset_odometry(0, 0, 0);

    // Delay 
    setTimeout(() => {
    }, 2000); // 2000ms = 2 seconds

    const modeRequest = new ROSLIB.ServiceRequest({
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/slam/rtabmap/reset',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Reset:', response);
    });

    const modeRequest_icp = new ROSLIB.ServiceRequest({
    });

    const modeService_icp = new ROSLIB.Service({
        ros: ros,
        name: '/slam_icp/reset_odom',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService_icp.callService(modeRequest_icp, function (response) {
        console.log('Reset:', response);
    });

    const modeRequest_rgbd = new ROSLIB.ServiceRequest({
    });

    const modeService_rgbd = new ROSLIB.Service({
        ros: ros,
        name: '/slam_vo/reset_odom',
        serviceType: 'std_srvs/srv/Empty'
    });

    modeService_rgbd.callService(modeRequest_rgbd, function (response) {
        console.log('Reset:', response);
    });
}

function rtabmap_reset_map2odom(x, y, theta) {
    // Initialize ROSLIB topic publisher for initialpose
    const initialPoseTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/slam/rtabmap/initialpose',  // Typically this topic is used for initial pose
        messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    });

    // Construct the message
    const initialPoseMsg = new ROSLIB.Message({
        header: {
            seq: 0,
            stamp: { secs: 0, nsecs: 0 },
            frame_id: "map" // or the appropriate frame your RTAB-Map uses
        },
        pose: {
            pose: {
                position: { x: x, y: y, z: 0.0 }, // position in meters
                orientation: {
                    x: 0, y: 0, z: Math.sin(theta / 2), w: Math.cos(theta / 2) // quaternion
                }
            },
            covariance: [  // Standard covariance (can be identity or small values)
                0.25, 0, 0, 0, 0, 0,
                0, 0.25, 0, 0, 0, 0,
                0, 0, 0.25, 0, 0, 0,
                0, 0, 0, 0.068, 0, 0,
                0, 0, 0, 0, 0.068, 0,
                0, 0, 0, 0, 0, 0.068
            ]
        }
    });

    // Publish the message
    initialPoseTopic.publish(initialPoseMsg);
    console.log('Published initial pose (0,0,0)');
}

function route_record(record) {
    const modeRequest = new ROSLIB.ServiceRequest({
        data: record
    });

    const modeService = new ROSLIB.Service({
        ros: ros,
        name: '/master/set_record_route_mode',
        serviceType: 'std_srvs/srv/SetBool'
    });

    modeService.callService(modeRequest, function (response) {
        console.log('Record:', response);
    });
}

function offset_odometry(x, y, theta) {
    var pub_pose_offset = new ROSLIB.Topic({
        ros: ros,
        name: '/master/pose_offset',
        messageType: 'nav_msgs/msg/Odometry'
    });

    // Construct the Odometry message
    var msg_pose_offset = new ROSLIB.Message({
        header: {
            stamp: { sec: 0, nanosec: 0 },  // Set timestamps dynamically if needed
            frame_id: "pose_offset"
        },
        child_frame_id: "base_link",
        pose: {
            pose: {
                position: { x: x, y: y, z: 0.0 },
                orientation: { x: 0, y: 0, z: theta, w: 1 }
            }
        },
        twist: {
            twist: {
                linear: { x: 0.0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0.0 }
            }
        }
    });
    pub_pose_offset.publish(msg_pose_offset);
}

const global_fsm = document.getElementById('global-fsm');
var sub_master_global_fsm = new ROSLIB.Topic({
    ros: ros,
    name: "/master/global_fsm",
    messageType: "std_msgs/Int16",
});

sub_master_global_fsm.subscribe(function (message) {
    if (message.data == 0) {
        global_fsm.innerHTML = "INIT"
    }
    else if (message.data == 1) {
        global_fsm.innerHTML = "Pre-Operation"
    }
    else if (message.data == 2) {
        global_fsm.innerHTML = "Safe-Operation"
    }
    else if (message.data == 3) {
        global_fsm.innerHTML = "Operation Mode 3"
    }
    else if (message.data == 4) {
        global_fsm.innerHTML = "Operation Mode 4"
    }
    else if (message.data == 5) {
        global_fsm.innerHTML = "Operation Mode 5"
    }
    else if (message.data == 6) {
        global_fsm.innerHTML = "Operation Mode 2"
    }
    else if (message.data == 7) {
        global_fsm.innerHTML = "Routing Mode"
    }
    else if (message.data == 8) {
        global_fsm.innerHTML = "Mapping Mode"
    }
});

// const icp_score = document.getElementById('icp-score');
// var sub_icp_score = new ROSLIB.Topic({
//     ros: ros,
//     name: "/lidar_obstacle_filter/icp_score",
//     messageType: "std_msgs/Float32",
// });

// sub_icp_score.subscribe(function (message) {
//     icp_score.innerHTML = "ICP Score: " + message.data.toFixed(2);
// });

// const obs_score = document.getElementById('obs-score');
// var sub_obs_score = new ROSLIB.Topic({
//     ros: ros,
//     name: "/all_obstacle_filter/result_all_obstacle",
//     messageType: "std_msgs/Float32MultiArray",
// });

// sub_obs_score.subscribe(function (message) {
//     obs_score.innerHTML = "Obs Score (Kiri, kanan, kamera, tbkiri, tbkanan): " + message.data[0].toFixed(2) + " " + message.data[1].toFixed(2) + " " + message.data[2].toFixed(2) + " " + message.data[3].toFixed(2) + " " + message.data[4].toFixed(2) + " " + message.data[5].toFixed(2) + " " + message.data[6].toFixed(2);
// });



// ================================================================

// ================================================================


// setInterval(() => {
// }, 50);
