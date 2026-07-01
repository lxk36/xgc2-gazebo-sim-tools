#include <algorithm>
#include <cctype>
#include <cmath>
#include <deque>
#include <map>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

constexpr const char* kFs150BodyMesh = "package://gazebo_sim_fs150_sitl/models/fs150/meshes/iris.stl";
constexpr const char* kFs150PropCcwMesh = "package://gazebo_sim_fs150_sitl/models/fs150/meshes/iris_prop_ccw.dae";
constexpr const char* kFs150PropCwMesh = "package://gazebo_sim_fs150_sitl/models/fs150/meshes/iris_prop_cw.dae";
constexpr const char* kScoutBodyMesh = "package://scout_description/meshes/scout_mini_base_link2.dae";
constexpr const char* kScoutBoxMesh = "package://scout_description/meshes/box_link.STL";
constexpr const char* kScoutWheelMesh = "package://scout_description/meshes/wheel.dae";
constexpr double kScoutWheelRadius = 0.08;

struct RotorVisual {
    const char* name;
    geometry_msgs::Vector3 offset;
    const char* mesh;
    double direction;
};

struct WheelVisual {
    const char* joint_name;
    const char* link_name;
    geometry_msgs::Vector3 offset;
    geometry_msgs::Quaternion joint_origin_rotation;
};

geometry_msgs::Vector3 makeVector3(double x, double y, double z) {
    geometry_msgs::Vector3 out;
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

geometry_msgs::Point makePoint(double x, double y, double z) {
    geometry_msgs::Point out;
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

geometry_msgs::Quaternion makeQuaternion(double x, double y, double z, double w) {
    geometry_msgs::Quaternion out;
    out.x = x;
    out.y = y;
    out.z = z;
    out.w = w;
    return out;
}

geometry_msgs::Pose makePose(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation) {
    geometry_msgs::Pose out;
    out.position = position;
    out.orientation = orientation;
    return out;
}

std_msgs::ColorRGBA makeColor(double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

geometry_msgs::Quaternion rpyQuaternion(double roll, double pitch, double yaw) {
    const double cr = std::cos(0.5 * roll);
    const double sr = std::sin(0.5 * roll);
    const double cp = std::cos(0.5 * pitch);
    const double sp = std::sin(0.5 * pitch);
    const double cy = std::cos(0.5 * yaw);
    const double sy = std::sin(0.5 * yaw);
    return makeQuaternion(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
                          cr * cp * cy + sr * sp * sy);
}

geometry_msgs::Pose makePoseFromXyzRpy(double x, double y, double z, double roll, double pitch, double yaw) {
    return makePose(makePoint(x, y, z), rpyQuaternion(roll, pitch, yaw));
}

const std::vector<RotorVisual>& fs150Rotors() {
    static const std::vector<RotorVisual> rotors = {
        {"rotor_0", makeVector3(0.13, -0.22, 0.023), kFs150PropCcwMesh, 1.0},
        {"rotor_1", makeVector3(-0.13, 0.20, 0.023), kFs150PropCcwMesh, 1.0},
        {"rotor_2", makeVector3(0.13, 0.22, 0.023), kFs150PropCwMesh, -1.0},
        {"rotor_3", makeVector3(-0.13, -0.20, 0.023), kFs150PropCwMesh, -1.0},
    };
    return rotors;
}

const std::vector<WheelVisual>& scoutWheels() {
    static const std::vector<WheelVisual> wheels = {
        {"front_left_wheel", "front_left_wheel_link", makeVector3(0.2319755, 0.2082515, -0.100998),
         rpyQuaternion(-1.57, 0.0, 0.0)},
        {"front_right_wheel", "front_right_wheel_link", makeVector3(0.2319755, -0.2082515, -0.099998),
         rpyQuaternion(1.57, 0.0, 0.0)},
        {"rear_left_wheel", "rear_left_wheel_link", makeVector3(-0.2319755, 0.2082515, -0.100998),
         rpyQuaternion(-1.57, 0.0, 0.0)},
        {"rear_right_wheel", "rear_right_wheel_link", makeVector3(-0.2319755, -0.2082515, -0.099998),
         rpyQuaternion(1.57, 0.0, 0.0)},
    };
    return wheels;
}

bool isFinite(double value) {
    return std::isfinite(value);
}

bool isFinite(const geometry_msgs::Pose& pose) {
    return isFinite(pose.position.x) && isFinite(pose.position.y) && isFinite(pose.position.z) &&
           isFinite(pose.orientation.x) && isFinite(pose.orientation.y) && isFinite(pose.orientation.z) &&
           isFinite(pose.orientation.w);
}

geometry_msgs::Quaternion normalize(const geometry_msgs::Quaternion& q) {
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 1.0e-9) {
        return makeQuaternion(0.0, 0.0, 0.0, 1.0);
    }
    return makeQuaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
}

geometry_msgs::Quaternion multiply(const geometry_msgs::Quaternion& lhs, const geometry_msgs::Quaternion& rhs) {
    const geometry_msgs::Quaternion a = normalize(lhs);
    const geometry_msgs::Quaternion b = normalize(rhs);
    return makeQuaternion(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

geometry_msgs::Quaternion multiplyRaw(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b) {
    return makeQuaternion(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
    return makeQuaternion(0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw));
}

geometry_msgs::Vector3 rotateVector(const geometry_msgs::Quaternion& q, const geometry_msgs::Vector3& v) {
    const geometry_msgs::Quaternion qn = normalize(q);
    const geometry_msgs::Quaternion vq = makeQuaternion(v.x, v.y, v.z, 0.0);
    const geometry_msgs::Quaternion qi = makeQuaternion(-qn.x, -qn.y, -qn.z, qn.w);
    const geometry_msgs::Quaternion out = multiplyRaw(multiplyRaw(qn, vq), qi);
    return makeVector3(out.x, out.y, out.z);
}

geometry_msgs::Pose copyPose(const geometry_msgs::Pose& pose) {
    geometry_msgs::Pose out;
    out.position = pose.position;
    out.orientation = normalize(pose.orientation);
    return out;
}

geometry_msgs::Pose composePose(const geometry_msgs::Pose& parent, const geometry_msgs::Pose& child) {
    geometry_msgs::Pose out;
    const geometry_msgs::Vector3 child_translation = makeVector3(child.position.x, child.position.y, child.position.z);
    const geometry_msgs::Vector3 rotated = rotateVector(parent.orientation, child_translation);
    out.position =
        makePoint(parent.position.x + rotated.x, parent.position.y + rotated.y, parent.position.z + rotated.z);
    out.orientation = multiply(parent.orientation, child.orientation);
    return out;
}

double pointDistance(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool isUavModel(const std::string& name) {
    static const std::regex pattern("^(uav|tello)[0-9]+$");
    return std::regex_match(name, pattern);
}

bool isUgvModel(const std::string& name) {
    static const std::regex pattern("^ugv[0-9]+$");
    const std::string normalized = lower(name);
    return std::regex_match(normalized, pattern) || normalized.find("scout") != std::string::npos;
}

std::string trailingNumber(const std::string& name) {
    static const std::regex pattern("([0-9]+)$");
    std::smatch match;
    return std::regex_search(name, match, pattern) ? match.str(1) : std::string();
}

} // namespace

class GazeboAutoVisualizer {
  public:
    GazeboAutoVisualizer()
        : private_nh_("~"), marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("markers", 1)) {
        private_nh_.param<std::string>("frame_id", frame_id_, "world");
        private_nh_.param<std::string>("model_states_topic", model_states_topic_, "/gazebo/model_states");
        private_nh_.param<std::string>("vrpn_pose_prefix", vrpn_pose_prefix_, "/vrpn_client_node");
        private_nh_.param("publish_rate", publish_rate_, 30.0);
        private_nh_.param("path_publish_rate", path_publish_rate_, 10.0);
        private_nh_.param("path_limit", path_limit_, 3000);
        private_nh_.param("path_history_duration", path_history_duration_sec_, 15.0);
        private_nh_.param("vrpn_timeout", vrpn_timeout_sec_, 0.5);
        private_nh_.param("joint_state_timeout", joint_state_timeout_sec_, 0.5);
        private_nh_.param("rotor_speed_rad_s", rotor_speed_rad_s_, 70.0);
        private_nh_.param("uav_mesh_scale", uav_mesh_scale_, 1.0);
        private_nh_.param("ugv_mesh_scale", ugv_mesh_scale_, 1.0);

        publish_rate_ = std::max(1.0, publish_rate_);
        path_publish_rate_ = std::max(1.0, path_publish_rate_);
        path_history_duration_sec_ = std::max(0.0, path_history_duration_sec_);
        if (path_history_duration_sec_ > 0.0) {
            path_limit_ = static_cast<int>(std::ceil(path_history_duration_sec_ * path_publish_rate_)) + 1;
        }
        path_limit_ = std::max(2, path_limit_);

        model_states_sub_ = nh_.subscribe(model_states_topic_, 5, &GazeboAutoVisualizer::modelStatesCallback, this);
        publish_timer_ =
            nh_.createTimer(ros::Duration(1.0 / publish_rate_), &GazeboAutoVisualizer::publishCallback, this);
    }

  private:
    enum class ModelKind { kUav, kUgv };

    struct TrackedModel {
        std::string name;
        ModelKind kind;
        geometry_msgs::Pose gazebo_pose;
        geometry_msgs::Pose vrpn_pose;
        bool has_gazebo_pose{false};
        bool has_vrpn_pose{false};
        ros::Time vrpn_stamp;
        ros::Time last_path_stamp;
        std::deque<geometry_msgs::Point> path;
        int marker_base_id{0};
        std::map<std::string, double> joint_positions;
        bool has_previous_wheel_pose{false};
        double fallback_wheel_phase{0.0};
        geometry_msgs::Point previous_wheel_position;
        ros::Time joint_state_stamp;
        ros::Subscriber vrpn_subscriber;
        ros::Subscriber joint_states_subscriber;
    };

    void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg) {
        for (std::size_t i = 0; i < msg->name.size() && i < msg->pose.size(); ++i) {
            const std::string& name = msg->name[i];
            const bool uav = isUavModel(name);
            const bool ugv = isUgvModel(name);
            if (!uav && !ugv) {
                continue;
            }
            if (!isFinite(msg->pose[i])) {
                ROS_WARN_THROTTLE(2.0,
                                  "[gazebo_auto_visualizer] Ignoring non-finite pose "
                                  "for model '%s'",
                                  name.c_str());
                continue;
            }

            auto it = models_.find(name);
            if (it == models_.end()) {
                TrackedModel model;
                model.name = name;
                model.kind = uav ? ModelKind::kUav : ModelKind::kUgv;
                model.marker_base_id = static_cast<int>(models_.size()) * 20;
                model.vrpn_subscriber = subscribeVrpn(name);
                if (ugv) {
                    model.joint_states_subscriber = subscribeJointStates(name);
                }
                it = models_.emplace(name, std::move(model)).first;
                ROS_INFO("[gazebo_auto_visualizer] Tracking %s model '%s'", uav ? "uav" : "ugv", name.c_str());
            }
            it->second.gazebo_pose = copyPose(msg->pose[i]);
            it->second.has_gazebo_pose = true;
        }
    }

    ros::Subscriber subscribeVrpn(const std::string& name) {
        const std::string topic = vrpn_pose_prefix_ + "/" + name + "/pose";
        return nh_.subscribe<geometry_msgs::PoseStamped>(topic, 10,
                                                         [this, name](const geometry_msgs::PoseStampedConstPtr& msg) {
                                                             vrpnPoseCallback(name, msg);
                                                         });
    }

    ros::Subscriber subscribeJointStates(const std::string& name) {
        const std::string topic = "/" + name + "/joint_states";
        return nh_.subscribe<sensor_msgs::JointState>(topic, 10,
                                                      [this, name](const sensor_msgs::JointStateConstPtr& msg) {
                                                          jointStatesCallback(name, msg);
                                                      });
    }

    void vrpnPoseCallback(const std::string& name, const geometry_msgs::PoseStampedConstPtr& msg) {
        auto it = models_.find(name);
        if (it == models_.end() || !isFinite(msg->pose)) {
            return;
        }
        it->second.vrpn_pose = copyPose(msg->pose);
        it->second.vrpn_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        it->second.has_vrpn_pose = true;
    }

    void jointStatesCallback(const std::string& name, const sensor_msgs::JointStateConstPtr& msg) {
        auto it = models_.find(name);
        if (it == models_.end()) {
            return;
        }
        const std::size_t count = std::min(msg->name.size(), msg->position.size());
        for (std::size_t i = 0; i < count; ++i) {
            if (std::isfinite(msg->position[i])) {
                it->second.joint_positions[msg->name[i]] = msg->position[i];
            }
        }
        it->second.joint_state_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    }

    const geometry_msgs::Pose* selectPose(const TrackedModel& model) const {
        if (model.has_vrpn_pose && ros::Time::now() - model.vrpn_stamp <= ros::Duration(vrpn_timeout_sec_)) {
            return &model.vrpn_pose;
        }
        return model.has_gazebo_pose ? &model.gazebo_pose : nullptr;
    }

    void publishCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        if (!last_publish_time_.isZero()) {
            const double dt = std::max(0.0, (now - last_publish_time_).toSec());
            rotor_phase_ = std::fmod(rotor_phase_ + rotor_speed_rad_s_ * dt, 2.0 * M_PI);
        }
        last_publish_time_ = now;

        visualization_msgs::MarkerArray markers;
        std::vector<geometry_msgs::TransformStamped> transforms;
        for (auto& entry : models_) {
            TrackedModel& model = entry.second;
            const geometry_msgs::Pose* pose = selectPose(model);
            if (pose == nullptr) {
                continue;
            }
            updatePath(model, *pose, now);
            transforms.push_back(makeBaseTransform(model.name, *pose, now));
            if (model.kind == ModelKind::kUav) {
                addUavMarkers(model, *pose, now, &markers, &transforms);
            } else {
                updateWheelFallback(model, *pose);
                addUgvMarkers(model, *pose, now, &markers, &transforms);
            }
            addPathMarker(model, now, &markers);
            addLabelMarker(model, *pose, now, &markers);
        }

        if (!transforms.empty()) {
            tf_broadcaster_.sendTransform(transforms);
        }
        marker_pub_.publish(markers);
    }

    void updatePath(TrackedModel& model, const geometry_msgs::Pose& pose, const ros::Time& now) {
        if (!model.last_path_stamp.isZero() && (now - model.last_path_stamp).toSec() < 1.0 / path_publish_rate_) {
            return;
        }
        if (static_cast<int>(model.path.size()) >= path_limit_) {
            model.path.pop_front();
        }
        model.path.push_back(pose.position);
        model.last_path_stamp = now;
    }

    void updateWheelFallback(TrackedModel& model, const geometry_msgs::Pose& pose) const {
        if (model.has_previous_wheel_pose) {
            model.fallback_wheel_phase +=
                pointDistance(pose.position, model.previous_wheel_position) / kScoutWheelRadius;
        }
        model.previous_wheel_position = pose.position;
        model.has_previous_wheel_pose = true;
    }

    geometry_msgs::TransformStamped makeBaseTransform(const std::string& name, const geometry_msgs::Pose& pose,
                                                      const ros::Time& stamp) const {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = frame_id_;
        transform.child_frame_id = name + "/base_link";
        transform.transform.translation = makeVector3(pose.position.x, pose.position.y, pose.position.z);
        transform.transform.rotation = pose.orientation;
        return transform;
    }

    geometry_msgs::TransformStamped makeRelativeTransform(const std::string& parent_frame,
                                                          const std::string& child_frame,
                                                          const geometry_msgs::Pose& pose,
                                                          const ros::Time& stamp) const {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation = makeVector3(pose.position.x, pose.position.y, pose.position.z);
        transform.transform.rotation = pose.orientation;
        return transform;
    }

    geometry_msgs::TransformStamped makeRotorTransform(const std::string& model_name, const RotorVisual& rotor,
                                                       const ros::Time& stamp) const {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = model_name + "/base_link";
        transform.child_frame_id = model_name + "/" + rotor.name;
        transform.transform.translation = rotor.offset;
        transform.transform.rotation =
            multiply(makeQuaternion(0.0, 0.0, 0.0, 1.0), yawQuaternion(rotor.direction * rotor_phase_));
        return transform;
    }

    void addUavMarkers(TrackedModel& model, const geometry_msgs::Pose& pose, const ros::Time& stamp,
                       visualization_msgs::MarkerArray* markers,
                       std::vector<geometry_msgs::TransformStamped>* transforms) {
        markers->markers.push_back(makeMeshMarker(model.name + "_body", model.marker_base_id, kFs150BodyMesh, pose,
                                                  stamp, makeColor(0.84, 0.71, 0.10, 1.0), uav_mesh_scale_));
        int rotor_id = 1;
        for (const RotorVisual& rotor : fs150Rotors()) {
            transforms->push_back(makeRotorTransform(model.name, rotor, stamp));

            geometry_msgs::Pose rotor_pose;
            const geometry_msgs::Vector3 offset = rotateVector(pose.orientation, rotor.offset);
            rotor_pose.position =
                makePoint(pose.position.x + offset.x, pose.position.y + offset.y, pose.position.z + offset.z);
            rotor_pose.orientation = multiply(pose.orientation, yawQuaternion(rotor.direction * rotor_phase_));
            const bool gazebo_blue_rotor = std::string(rotor.name) == "rotor_0" || std::string(rotor.name) == "rotor_2";
            const std_msgs::ColorRGBA rotor_color =
                gazebo_blue_rotor ? makeColor(0.10, 0.20, 0.90, 1.0) : makeColor(0.12, 0.12, 0.12, 1.0);
            markers->markers.push_back(makeMeshMarker(model.name + "_" + rotor.name, model.marker_base_id + rotor_id,
                                                      rotor.mesh, rotor_pose, stamp, rotor_color, uav_mesh_scale_));
            ++rotor_id;
        }
    }

    void addUgvMarkers(const TrackedModel& model, const geometry_msgs::Pose& pose, const ros::Time& stamp,
                       visualization_msgs::MarkerArray* markers,
                       std::vector<geometry_msgs::TransformStamped>* transforms) const {
        const geometry_msgs::Pose body_visual_pose =
            composePose(pose, makePoseFromXyzRpy(0.0, 0.0, 0.0, 1.57, 0.0, -1.57));
        markers->markers.push_back(makeMeshMarker(model.name + "_body", model.marker_base_id, kScoutBodyMesh,
                                                  body_visual_pose, stamp, makeColor(1.0, 1.0, 1.0, 1.0),
                                                  ugv_mesh_scale_, true));

        const geometry_msgs::Pose box_joint_pose = makePoseFromXyzRpy(0.0, 0.0, 0.055, 0.0, 0.0, 3.14);
        const geometry_msgs::Pose box_visual_pose =
            composePose(composePose(pose, box_joint_pose), makePoseFromXyzRpy(0.0, 0.0, 0.0, 0.0, 0.0, 3.14));
        markers->markers.push_back(makeMeshMarker(model.name + "_box", model.marker_base_id + 1, kScoutBoxMesh,
                                                  box_visual_pose, stamp, makeColor(1.0, 1.0, 1.0, 1.0),
                                                  ugv_mesh_scale_, false));
        transforms->push_back(
            makeRelativeTransform(model.name + "/base_link", model.name + "/box_link", box_joint_pose, stamp));

        int wheel_marker_offset = 2;
        for (const WheelVisual& wheel : scoutWheels()) {
            const geometry_msgs::Pose wheel_pose = makeWheelPose(model, wheel);
            const geometry_msgs::Pose wheel_world_pose = composePose(pose, wheel_pose);
            markers->markers.push_back(makeMeshMarker(
                model.name + "_" + wheel.link_name, model.marker_base_id + wheel_marker_offset, kScoutWheelMesh,
                wheel_world_pose, stamp, makeColor(1.0, 1.0, 1.0, 1.0), ugv_mesh_scale_, true));
            transforms->push_back(makeRelativeTransform(model.name + "/base_link", model.name + "/" + wheel.link_name,
                                                        wheel_pose, stamp));
            ++wheel_marker_offset;
        }
    }

    geometry_msgs::Pose makeWheelPose(const TrackedModel& model, const WheelVisual& wheel) const {
        const double angle = wheelJointPosition(model, wheel.joint_name);
        return makePose(makePoint(wheel.offset.x, wheel.offset.y, wheel.offset.z),
                        multiply(wheel.joint_origin_rotation, yawQuaternion(-angle)));
    }

    double wheelJointPosition(const TrackedModel& model, const std::string& joint_name) const {
        if (model.has_gazebo_pose && !model.joint_state_stamp.isZero() &&
            ros::Time::now() - model.joint_state_stamp <= ros::Duration(joint_state_timeout_sec_)) {
            auto it = model.joint_positions.find(joint_name);
            if (it != model.joint_positions.end()) {
                return it->second;
            }
        }
        return model.fallback_wheel_phase;
    }

    visualization_msgs::Marker makeMeshMarker(const std::string& ns, int id, const std::string& mesh,
                                              const geometry_msgs::Pose& pose, const ros::Time& stamp,
                                              const std_msgs::ColorRGBA& color, double scale = 1.0,
                                              bool use_embedded_materials = false) const {
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = frame_id_;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.mesh_resource = mesh;
        marker.mesh_use_embedded_materials = use_embedded_materials;
        marker.pose = pose;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color = color;
        return marker;
    }

    void addPathMarker(const TrackedModel& model, const ros::Time& stamp,
                       visualization_msgs::MarkerArray* markers) const {
        if (model.path.size() < 2) {
            return;
        }
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = frame_id_;
        marker.ns = model.name + "_actual_path";
        marker.id = model.marker_base_id + 10;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.points.assign(model.path.begin(), model.path.end());
        marker.scale.x = 0.018;
        marker.color.r = model.kind == ModelKind::kUav ? 0.0 : 1.0;
        marker.color.g = model.kind == ModelKind::kUav ? 0.55 : 0.05;
        marker.color.b = model.kind == ModelKind::kUav ? 1.0 : 0.02;
        marker.color.a = 1.0;
        markers->markers.push_back(marker);
    }

    void addLabelMarker(const TrackedModel& model, const geometry_msgs::Pose& pose, const ros::Time& stamp,
                        visualization_msgs::MarkerArray* markers) const {
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = frame_id_;
        marker.ns = model.name + "_label";
        marker.id = model.marker_base_id + 11;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = makePoint(pose.position.x, pose.position.y, pose.position.z + labelHeight(model.kind));
        marker.pose.orientation = makeQuaternion(0.0, 0.0, 0.0, 1.0);
        marker.scale.z = 0.32;
        marker.color = makeColor(1.0, 1.0, 1.0, 1.0);
        marker.text = displayName(model);
        markers->markers.push_back(marker);
    }

    double labelHeight(ModelKind kind) const { return kind == ModelKind::kUav ? 0.55 : 0.65; }

    std::string displayName(const TrackedModel& model) const {
        const std::string number = trailingNumber(model.name);
        const std::string prefix = model.kind == ModelKind::kUav ? "UAV " : "UGV ";
        return prefix + (number.empty() ? model.name : number);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber model_states_sub_;
    ros::Timer publish_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::map<std::string, TrackedModel> models_;

    std::string frame_id_;
    std::string model_states_topic_;
    std::string vrpn_pose_prefix_;
    double publish_rate_{30.0};
    double path_publish_rate_{10.0};
    int path_limit_{3000};
    double path_history_duration_sec_{15.0};
    double vrpn_timeout_sec_{0.5};
    double joint_state_timeout_sec_{0.5};
    double rotor_speed_rad_s_{70.0};
    double uav_mesh_scale_{1.0};
    double ugv_mesh_scale_{1.0};
    double rotor_phase_{0.0};
    ros::Time last_publish_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_auto_visualizer");
    GazeboAutoVisualizer visualizer;
    ros::spin();
    return 0;
}
