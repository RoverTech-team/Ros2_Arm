#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>

using FollowJointTrajectoryAction =
    control_msgs::action::FollowJointTrajectory;

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

using namespace std::chrono_literals;


// ==========================================================
// ATTESA DI UN FUTURE
// ==========================================================

template <typename FutureT>
static bool waitFuture(
    const FutureT & future,
    const rclcpp::Node::SharedPtr & node,
    double sim_timeout_s,
    double wall_timeout_s)
{
    const auto sim_deadline =
        node->now() +
        rclcpp::Duration::from_seconds(sim_timeout_s);

    const auto wall_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(wall_timeout_s);

    while (rclcpp::ok()) {

        if (future.wait_for(20ms) ==
            std::future_status::ready) {
            return true;
        }

        if (node->now() > sim_deadline ||
            std::chrono::steady_clock::now() > wall_deadline) {
            return false;
        }
    }

    return false;
}


// ==========================================================
// INVIO TRAIETTORIA CON CANCELLAZIONE
// ==========================================================

static bool sendTrajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    rclcpp_action::Client<FollowJointTrajectoryAction>::SharedPtr & client,
    const rclcpp::Node::SharedPtr & node,
    const rclcpp::Logger & logger,
    double expected_s,
    const std::atomic<bool> & aruco_found)
{
    if (aruco_found.load()) { //verifica continua di questa condizione: continua a eseguire i movimenti decisi se non viene trovato alcun aruco dal python.
        return false;
    }

    if (!client->action_server_is_ready()) {
        RCLCPP_ERROR(
            logger,
            "Controller del braccio non disponibile."
        );
        return false;
    }

    FollowJointTrajectoryAction::Goal goal;
    goal.trajectory = traj;

    // Invia la traiettoria al controller
    auto send_future = client->async_send_goal(goal);

    if (!waitFuture(send_future, node, 5.0, 60.0)) {
        RCLCPP_ERROR(
            logger,
            "Timeout nell'invio della traiettoria."
        );
        return false;
    }

    auto goal_handle = send_future.get();

    if (!goal_handle) {
        RCLCPP_ERROR(
            logger,
            "Traiettoria rifiutata dal controller."
        );
        return false;
    }

    auto result_future =
        client->async_get_result(goal_handle);

    const double sim_timeout = expected_s + 10.0;

    const auto sim_deadline =
        node->now() +
        rclcpp::Duration::from_seconds(sim_timeout);

    const auto wall_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(
            std::max(60.0, 20.0 * sim_timeout)
        );

    bool cancel_sent = false;

    while (rclcpp::ok()) {

        // La camera ha riconosciuto un ArUco.
        if (aruco_found.load() && !cancel_sent) {

            cancel_sent = true;

            RCLCPP_WARN(
                logger,
                "ARUCO TROVATO: cancello la traiettoria!"
            );

            auto cancel_future =
                client->async_cancel_goal(goal_handle);

            if (!waitFuture(
                    cancel_future, node, 5.0, 10.0)) {

                RCLCPP_ERROR(
                    logger,
                    "Timeout richiesta cancellazione."
                );
                return false;
            }

            auto response = cancel_future.get();

            if (response->goals_canceling.empty()) {

                // Potrebbe essere gia' terminata.
                if (result_future.wait_for(0ms) !=
                    std::future_status::ready) {

                    RCLCPP_WARN(
                        logger,
                        "Cancellazione non accettata dal controller."
                    );
                }
            }

            // Attendi conferma del risultato del goal
            if (!waitFuture(
                    result_future, node, 5.0, 10.0)) {

                RCLCPP_WARN(
                    logger,
                    "Arresto non ancora confermato dal controller."
                );

                return false;
            }

            auto result = result_future.get();

            RCLCPP_INFO(
                logger,
                "Risultato cancellazione: code=%d",
                static_cast<int>(result.code)
            );

            return false;
        }

        // Movimento terminato normalmente
        if (result_future.wait_for(20ms) ==
            std::future_status::ready) {

            auto result = result_future.get();

            // Anche se il movimento e' terminato,
            // non avviamo altri goal se il marker e' stato visto.
            if (aruco_found.load()) {
                return false;
            }

            if (result.code !=
                rclcpp_action::ResultCode::SUCCEEDED) {

                RCLCPP_ERROR(
                    logger,
                    "Traiettoria fallita: code=%d",
                    static_cast<int>(result.code)
                );

                return false;
            }

            return true;
        }

        if (node->now() > sim_deadline ||
            std::chrono::steady_clock::now() > wall_deadline) {

            RCLCPP_ERROR(
                logger,
                "Timeout esecuzione traiettoria."
            );

            // Anche un timeout deve evitare che il
            // controller continui a eseguire il goal.
            auto cancel_future =
                client->async_cancel_goal(goal_handle);

            if (waitFuture(
                    cancel_future, node, 5.0, 10.0)) {

                auto response = cancel_future.get();

                if (!response->goals_canceling.empty()) {
                    waitFuture(
                        result_future, node, 5.0, 10.0
                    );
                }
            }

            return false;
        }
    }

    return false;
}


// ==========================================================
// MOVIMENTO MORBIDO NELLO SPAZIO DEI GIUNTI
// ==========================================================

static bool moveSmooth(
    MoveGroupInterface & move_group,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & goal,
    rclcpp_action::Client<FollowJointTrajectoryAction>::SharedPtr & client,
    const rclcpp::Logger & logger,
    double joint_speed,
    double min_time,
    const rclcpp::Node::SharedPtr & node,
    const std::atomic<bool> & aruco_found)
{
    if (aruco_found.load()) {
        return false;
    }

    const auto start =
        move_group.getCurrentJointValues();

    if (start.size() != goal.size() ||
        joint_names.size() != goal.size()) {

        RCLCPP_ERROR(
            logger,
            "Dimensioni dei giunti non valide."
        );

        return false;
    }

    double max_delta = 0.0;

    for (size_t j = 0; j < goal.size(); ++j) {

        max_delta = std::max(
            max_delta,
            std::fabs(goal[j] - start[j])
        );
    }

    // Profilo minimum jerk
    const double T = std::max(
        min_time,
        1.875 * max_delta /
        std::max(joint_speed, 1e-3)
    );

    const int N = std::max(
        10,
        static_cast<int>(std::ceil(T * 20.0))
    );

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names;

    for (int k = 1; k <= N; ++k) {

        const double tau =
            static_cast<double>(k) /
            static_cast<double>(N);

        const double s =
            tau * tau * tau *
            (10.0 - 15.0 * tau + 6.0 * tau * tau);

        const double ds =
            (30.0 * tau * tau -
             60.0 * tau * tau * tau +
             30.0 * tau * tau * tau * tau) / T;

        const double dds =
            (60.0 * tau -
             180.0 * tau * tau +
             120.0 * tau * tau * tau) / (T * T);

        trajectory_msgs::msg::JointTrajectoryPoint pt;

        for (size_t j = 0; j < goal.size(); ++j) {

            const double D = goal[j] - start[j];

            pt.positions.push_back(
                start[j] + D * s
            );

            pt.velocities.push_back(D * ds);
            pt.accelerations.push_back(D * dds);
        }

        pt.time_from_start =
            rclcpp::Duration::from_seconds(T * tau);

        traj.points.push_back(pt);
    }

    traj.header.stamp = builtin_interfaces::msg::Time();

    // Ultimo controllo prima dell'invio
    if (aruco_found.load()) {
        return false;
    }

    RCLCPP_INFO(
        logger,
        "Movimento: delta massimo %.2f rad, durata %.2f s",
        max_delta,
        T
    );

    return sendTrajectory(
        traj,
        client,
        node,
        logger,
        T,
        aruco_found
    );
}


// ==========================================================
// MAIN
// ==========================================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.append_parameter_override(
        "use_sim_time", true
    );

    auto node = std::make_shared<rclcpp::Node>(
        "panel_interaction_demo",
        options
    );

    auto logger = node->get_logger();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]() {
        executor.spin();
    });

    auto finish = [&](int code) {

        rclcpp::shutdown();

        if (spinner.joinable()) {
            spinner.join();
        }

        return code;
    };


    // ======================================================
    // PARAMETRI
    // ======================================================

    node->declare_parameter<std::string>(
        "trajectory_action_topic",
        "/arm_controller/follow_joint_trajectory"
    );

    node->declare_parameter<double>(
        "joint_speed", 0.5
    );

    node->declare_parameter<double>(
        "min_move_time", 1.0
    );

    node->declare_parameter<double>(
        "limit_margin", 0.05
    );

    node->declare_parameter<double>(
        "max_arm_bend", 0.5
    );

    const auto traj_topic =
        node->get_parameter(
            "trajectory_action_topic"
        ).as_string();

    const double joint_speed =
        node->get_parameter(
            "joint_speed"
        ).as_double();

    const double min_move_time =
        node->get_parameter(
            "min_move_time"
        ).as_double();

    const double limit_margin =
        std::clamp(
            node->get_parameter(
                "limit_margin"
            ).as_double(),
            0.0,
            0.3
        );

    const double max_arm_bend =
        std::max(
            0.0,
            node->get_parameter(
                "max_arm_bend"
            ).as_double()
        );


    // ======================================================
    // RICONOSCIMENTO ARUCO
    // ======================================================

    std::atomic<bool> aruco_found{false};

    auto marker_sub =
        node->create_subscription<std_msgs::msg::Bool>(
            "/panel/markers_detected",
            10,
            [&](const std_msgs::msg::Bool::SharedPtr msg) {

                if (msg->data &&
                    !aruco_found.exchange(true)) {

                    RCLCPP_WARN(
                        logger,
                        "ARUCO RICONOSCIUTO! STOP RICERCA."
                    );
                }
            }
        );


    // ======================================================
    // CLIENT CONTROLLER BRACCIO
    // ======================================================

    auto action_client =
        rclcpp_action::create_client<
            FollowJointTrajectoryAction
        >(node, traj_topic);

    auto controller_deadline =
        std::chrono::steady_clock::now() + 15s;

    while (rclcpp::ok() &&
           !action_client->action_server_is_ready() &&
           std::chrono::steady_clock::now() <
               controller_deadline) {

        std::this_thread::sleep_for(50ms);
    }

    if (!action_client->action_server_is_ready()) {

        RCLCPP_FATAL(
            logger,
            "Controller '%s' non disponibile.",
            traj_topic.c_str()
        );

        return finish(2);
    }


    // ======================================================
    // ATTESA STATO GIUNTI E CLOCK
    // ======================================================

    std::atomic<bool> joint_states_received{false};

    auto sub_js =
        node->create_subscription<
            sensor_msgs::msg::JointState
        >(
            "/joint_states",
            10,
            [&](const sensor_msgs::msg::JointState::SharedPtr msg) {

                if (!msg->position.empty()) {
                    joint_states_received = true;
                }
            }
        );

    RCLCPP_INFO(
        logger,
        "Attendo stato giunti e clock Gazebo..."
    );

    while (rclcpp::ok() &&
           (!joint_states_received.load() ||
            node->now().seconds() == 0.0)) {

        std::this_thread::sleep_for(20ms);
    }

    if (!rclcpp::ok()) {
        return finish(130);
    }


    // ======================================================
    // INIZIALIZZAZIONE MOVEIT
    // ======================================================

    MoveGroupInterface move_group(node, "arm");

    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setPlanningTime(5.0);

    const auto joint_names =
        move_group.getActiveJoints();

    const size_t n = joint_names.size();

    if (n == 0) {

        RCLCPP_ERROR(
            logger,
            "Nessun giunto nel gruppo arm."
        );

        return finish(2);
    }

    struct JointLimit {
        double lo;
        double hi;
        bool bounded;
    };

    std::vector<JointLimit> limits;

    auto robot_model = move_group.getRobotModel();

    for (size_t i = 0; i < n; ++i) {

        JointLimit limit{
            -1e9,
            1e9,
            false
        };

        const auto * jm =
            robot_model->getJointModel(joint_names[i]);

        if (jm != nullptr &&
            !jm->getVariableBounds().empty()) {

            const auto & bounds =
                jm->getVariableBounds()[0];

            if (bounds.position_bounded_) {

                limit.lo = bounds.min_position_;
                limit.hi = bounds.max_position_;
                limit.bounded = true;
            }
        }

        limits.push_back(limit);
    }


    // ======================================================
    // LIMITI GIUNTI
    // ======================================================

    auto clampToLimits =
        [&](std::vector<double> q) {

            for (size_t i = 0;
                 i < q.size() && i < limits.size();
                 ++i) {

                if (!limits[i].bounded) {
                    continue;
                }

                const double margin =
                    limit_margin *
                    (limits[i].hi - limits[i].lo);

                q[i] = std::clamp(
                    q[i],
                    limits[i].lo + margin,
                    limits[i].hi - margin
                );
            }

            return q;
        };

    auto clampBend =
        [&](std::vector<double> q) {

            q = clampToLimits(q);

            for (size_t i = 1;
                 i <= 2 && i < q.size();
                 ++i) {

                q[i] = std::clamp(
                    q[i],
                    -max_arm_bend,
                    max_arm_bend
                );
            }

            return q;
        };


    // ======================================================
    // FUNZIONE MOVIMENTO BRACCIO
    // ======================================================

    auto goTo =
        [&](const std::vector<double> & q) {

            if (aruco_found.load()) {
                return false;
            }

            return moveSmooth(
                move_group,
                joint_names,
                clampToLimits(q),
                action_client,
                logger,
                joint_speed,
                min_move_time,
                node,
                aruco_found
            );
        };


    // ======================================================
    // POSA INIZIALE
    // ======================================================

    std::vector<double> neutral(n, 0.0);

    if (n > 1) {
        neutral[1] = 0.2;
    }

    neutral = clampBend(neutral);

    RCLCPP_INFO(
        logger,
        "Porto il braccio nella posa iniziale..."
    );

    if (!aruco_found.load()) {
        if (!goTo(neutral) && !aruco_found.load()) {

            RCLCPP_WARN(
                logger,
                "Posa iniziale non raggiunta."
            );
        }
    }


    // ======================================================
    // SCANSIONE ARUCO
    // ======================================================

    const std::vector<std::vector<double>> scan_poses = {

        { 1.0,   0.0,  0.0 },
        {-1.57, -0.8,  0.2 },
        { 0.0,   0.4,  0.4 },
        {-0.4,   0.2,  0.2 },
        {-0.8,   0.0,  0.0 }

    };

    RCLCPP_INFO(
        logger,
        "Inizio ricerca ArUco..."
    );

    for (size_t i = 0;
         i < scan_poses.size() && rclcpp::ok();
         ++i) {

        // Marker gia' trovato: nessun altro movimento
        if (aruco_found.load()) {
            break;
        }

        std::vector<double> q(n, 0.0);

        q[0] = scan_poses[i][0];

        if (n > 2) {
            q[1] = scan_poses[i][1];
            q[2] = scan_poses[i][2];
        }

        q = clampBend(q);

        RCLCPP_INFO(
            logger,
            "Scansione posa %zu/%zu",
            i + 1,
            scan_poses.size()
        );

        // Il movimento viene cancellato se arriva
        // il segnale dal Python.
        bool moved = goTo(q);

        if (aruco_found.load()) {
            break;
        }

        if (!moved) {

            RCLCPP_WARN(
                logger,
                "Movimento di scansione fallito."
            );

            continue;
        }

        // Attesa per permettere alla camera di
        // riconoscere il marker a braccio fermo.
        const auto wait_end =
            std::chrono::steady_clock::now() +
            1200ms;

        while (rclcpp::ok() &&
               !aruco_found.load() &&
               std::chrono::steady_clock::now() <
                   wait_end) {

            std::this_thread::sleep_for(20ms);
        }
    }


    // ======================================================
    // FINE RICERCA
    // ======================================================

    if (!rclcpp::ok()) {
        return finish(130);
    }

    if (aruco_found.load()) {

        RCLCPP_INFO(
            logger,
            "ARUCO TROVATO: ricerca interrotta."
        );

        RCLCPP_INFO(
            logger,
            "Il braccio non ricevera' altri movimenti."
        );

        // Manteniamo il nodo attivo.
        // Non torniamo in neutral.
        // Non avviamo la navigazione.
        // Non eseguiamo i target del pannello.

        while (rclcpp::ok()) {
            std::this_thread::sleep_for(100ms);
        }

        return finish(0);
    }

    RCLCPP_WARN(
        logger,
        "Ricerca terminata: nessun ArUco trovato."
    );

    // Anche in questo caso nessun altro movimento.
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(100ms);
    }

    return finish(1);
}
