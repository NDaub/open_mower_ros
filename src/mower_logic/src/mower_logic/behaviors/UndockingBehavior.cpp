// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "UndockingBehavior.h"

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern xbot_msgs::AbsolutePose getPose();
extern mower_msgs::Status getStatus();

extern void setRobotPose(geometry_msgs::Pose &pose);
extern void goReverse();
extern void stopMoving();
extern bool isGpsGood();
extern bool setGPS(bool enabled);

UndockingBehavior UndockingBehavior::INSTANCE(&MowingBehavior::INSTANCE);
UndockingBehavior UndockingBehavior::RETRY_INSTANCE(&DockingBehavior::INSTANCE);

std::string UndockingBehavior::state_name() {
    return "UNDOCKING";
}

Behavior *UndockingBehavior::execute() {
   // ROS_INFO_STREAM("om_mower_logic: stopMoving() - stopping bot movement");
    goReverse();

    ros::Rate reverseTimer(3);
    reverseTimer.sleep();

    stopMoving();

    ROS_INFO_STREAM("Undock success. Waiting for GPS.");
    bool hasGps = waitForGPS();

    if (!hasGps) {
        ROS_ERROR_STREAM("Could not get GPS.");
        return &IdleBehavior::INSTANCE;
    }

    // TODO return mow area
    return nextBehavior;

}

void UndockingBehavior::enter() {
    reset();
    paused = aborted = false;

    // Get the docking pose in map
    mower_map::GetDockingPointSrv get_docking_point_srv;
    dockingPointClient.call(get_docking_point_srv);
    docking_pose_stamped.pose = get_docking_point_srv.response.docking_pose;
    docking_pose_stamped.header.frame_id = "map";
    docking_pose_stamped.header.stamp = ros::Time::now();

    // set the robot's position to the dock if we're actually docked
    if(getStatus().v_charge > 5.0) {
        ROS_INFO_STREAM("Currently inside the docking station, we set the robot's pose to the docks pose.");
        setRobotPose(docking_pose_stamped.pose);
    }
}

void UndockingBehavior::exit() {

}

void UndockingBehavior::reset() {
    gpsRequired = false;
}

bool UndockingBehavior::needs_gps() {
    return gpsRequired;
}

bool UndockingBehavior::mower_enabled() {
    // No mower during docking
    return false;
}

bool UndockingBehavior::waitForGPS() {
    gpsRequired = false;
    setGPS(true);
    ros::Rate odom_rate(1.0);
    while (ros::ok() && !aborted) {
        if (isGpsGood()) {
            ROS_INFO("Got good gps, let's go");
            break;
        } else {
            ROS_INFO_STREAM("waiting for gps. current accuracy: " << getPose().position_accuracy);
            odom_rate.sleep();
        }
    }
    if (!ros::ok() || aborted) {
        return false;
    }

    // wait additional time for odometry filters to converge
    ros::Rate r(ros::Duration(config.gps_wait_time, 0));
    r.sleep();

    gpsRequired = true;

    return true;
}

UndockingBehavior::UndockingBehavior(Behavior* next) {
    this->nextBehavior = next;
}

void UndockingBehavior::command_home() {

}

void UndockingBehavior::command_start() {

}

void UndockingBehavior::command_s1() {

}

void UndockingBehavior::command_s2() {

}

bool UndockingBehavior::redirect_joystick() {
    return false;
}


uint8_t UndockingBehavior::get_sub_state() {
    return 2;

}
uint8_t UndockingBehavior::get_state() {
    return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void UndockingBehavior::handle_action(std::string action) {
}
