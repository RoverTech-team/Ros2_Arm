//rispetto a quello aim aruco, questo cpp ha 5 configurazioni pre impostate e interrompe la ricerca quando riceve il segnale di rilevamento. Quindi la ricerca dipende da configurazioni stabilite in anticipo.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
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
// ATTESA IN TEMPO SIMULATO
// ==========================================================

static void sleepSim(
    const rclcpp::Node::SharedPtr & node,
    double seconds)
{
    const auto sim_end =
        node->now() +
        rclcpp::Duration::from_seconds(seconds);

    const auto wall_end =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(
            std::max(10.0, 30.0 * seconds)
        );

    while (rclcpp::ok() &&
           node->now() < sim_end &&
           std::chrono::steady_clock::now() < wall_end) {

        std::this_thread::sleep_for(20ms);
    }
}


// ==========================================================
// ERRORE DEI MARKER NELL'IMMAGINE (dal nodo Python)
// ==========================================================

struct ImageError
{
    double ex = 0.0;    // errore orizzontale normalizzato [-1, 1]; + = marker a destra
    double ey = 0.0;    // errore verticale normalizzato   [-1, 1]; + = marker in basso
    int markers = 0;    // quanti marker sono visibili (1 o 2)
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};   // istante di arrivo (tempo simulato)
    bool valid = false;
};


// ==========================================================
// INVIO TRAIETTORIA CON CANCELLAZIONE
// ==========================================================

static bool sendTrajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    rclcpp_action::Client<FollowJointTrajectoryAction>::SharedPtr & client,
    const rclcpp::Node::SharedPtr & node,
    const rclcpp::Logger & logger,
    double expected_s,
    const std::atomic<bool> & stop_flag)
{
    if (stop_flag.load()) { // verifica continua: i movimenti vanno avanti finche' il chiamante non chiede lo stop (per la scansione: ArUco trovato).
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

        // Stop richiesto (nella scansione: la camera ha riconosciuto gli ArUco).
        if (stop_flag.load() && !cancel_sent) {

            cancel_sent = true;

            RCLCPP_WARN(
                logger,
                "STOP richiesto: cancello la traiettoria!"
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
            // non avviamo altri goal se e' stato richiesto lo stop.
            if (stop_flag.load()) {
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
    const std::atomic<bool> & stop_flag)
{
    if (stop_flag.load()) {
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
    if (stop_flag.load()) {
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
        stop_flag
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

    // --- centraggio dei marker dopo il rilevamento ---

    // Se true: trovati gli ArUco, il braccio si abbassa/inclina finche'
    // i marker sono al centro dell'immagine, poi parte la stima finale.
    node->declare_parameter<bool>(
        "center_after_found", true
    );

    // Tolleranza sull'errore normalizzato (0 = centro, 1 = bordo immagine)
    node->declare_parameter<double>(
        "center_tolerance", 0.08
    );

    node->declare_parameter<int>(
        "center_max_steps", 15
    );

    // Frazione della correzione calcolata applicata ad ogni passo
    node->declare_parameter<double>(
        "center_gain", 0.7
    );

    // Limite del movimento di un singolo giunto in un passo [rad]
    node->declare_parameter<double>(
        "center_max_step", 0.25
    );

    // Durata minima di un passo di centraggio e attesa a braccio fermo [s sim]
    node->declare_parameter<double>(
        "center_move_time", 0.8
    );

    node->declare_parameter<double>(
        "center_settle_time", 0.7
    );

    // Camera: FOV orizzontale [rad] e rapporto larghezza/altezza
    // (Gemini 305 simulata: 1.64061 rad, 1280x800)
    node->declare_parameter<double>(
        "camera_hfov", 1.64061
    );

    node->declare_parameter<double>(
        "camera_aspect", 1280.0 / 800.0
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

    const bool center_after_found =
        node->get_parameter(
            "center_after_found"
        ).as_bool();

    const double center_tol =
        std::clamp(
            node->get_parameter(
                "center_tolerance"
            ).as_double(),
            0.01,
            0.5
        );

    const int center_max_steps =
        std::max(
            1,
            static_cast<int>(
                node->get_parameter(
                    "center_max_steps"
                ).as_int()
            )
        );

    const double center_gain =
        std::clamp(
            node->get_parameter(
                "center_gain"
            ).as_double(),
            0.1,
            1.0
        );

    const double center_max_step =
        std::clamp(
            node->get_parameter(
                "center_max_step"
            ).as_double(),
            0.01,
            0.6
        );

    const double center_move_time =
        std::max(
            0.2,
            node->get_parameter(
                "center_move_time"
            ).as_double()
        );

    const double center_settle =
        std::max(
            0.1,
            node->get_parameter(
                "center_settle_time"
            ).as_double()
        );

    const double cam_hfov =
        std::clamp(
            node->get_parameter(
                "camera_hfov"
            ).as_double(),
            0.2,
            3.0
        );

    const double cam_aspect =
        std::max(
            0.2,
            node->get_parameter(
                "camera_aspect"
            ).as_double()
        );


    // ======================================================
    // RICONOSCIMENTO ARUCO
    // ======================================================

    std::atomic<bool> aruco_found{false};

    // Ultimo stato dei giunti e stato dei giunti nell'istante del
    // rilevamento (serve per tornare indietro se i marker si perdono).
    std::mutex js_mutex;
    std::map<std::string, double> latest_joints;
    std::map<std::string, double> joints_at_detection;

    // Ultimo errore dei marker nell'immagine (da /panel/image_error)
    std::mutex err_mutex;
    ImageError latest_err;

    auto marker_sub =
        node->create_subscription<std_msgs::msg::Bool>(
            "/panel/markers_detected",
            10,
            [&](const std_msgs::msg::Bool::SharedPtr msg) {

                if (msg->data && !aruco_found.load()) {

                    // foto dei giunti prima di segnalare il rilevamento
                    {
                        std::lock_guard<std::mutex> lock(js_mutex);
                        joints_at_detection = latest_joints;
                    }

                    if (!aruco_found.exchange(true)) {

                        RCLCPP_WARN(
                            logger,
                            "ARUCO RICONOSCIUTO! STOP RICERCA."
                        );
                    }
                }
            }
        );

    // Posizione dei marker nell'immagine, pubblicata dal Python:
    // data = [errore_orizzontale, errore_verticale, numero_marker_visibili]
    auto err_sub =
        node->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/panel/image_error",
            10,
            [&](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

                if (msg->data.size() < 3) {
                    return;
                }

                std::lock_guard<std::mutex> lock(err_mutex);

                latest_err.ex = msg->data[0];
                latest_err.ey = msg->data[1];
                latest_err.markers =
                    static_cast<int>(std::lround(msg->data[2]));
                latest_err.stamp = node->now();
                latest_err.valid = true;
            }
        );

    // Segnale per il Python: braccio fermo nella posa finale,
    // si puo' fare la stima del pannello. QoS "transient local" cosi'
    // il messaggio arriva anche se il Python si collega dopo.
    auto align_done_pub =
        node->create_publisher<std_msgs::msg::Bool>(
            "/panel/align_done",
            rclcpp::QoS(1).reliable().transient_local()
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

                {
                    std::lock_guard<std::mutex> lock(js_mutex);

                    for (size_t i = 0;
                         i < msg->name.size() &&
                         i < msg->position.size();
                         ++i) {

                        latest_joints[msg->name[i]] =
                            msg->position[i];
                    }
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
    // CENTRAGGIO DEI MARKER NELL'IMMAGINE
    // ======================================================
    //
    // Dopo lo stop il braccio si muove a piccoli passi, in anello chiuso:
    // ad ogni passo legge dove stanno i marker nell'immagine (errore
    // normalizzato dal Python) e corregge
    //   - l'inclinazione della camera con il gomito j3 (se arriva al limite,
    //     il residuo passa alla spalla j2): "abbassa" il braccio;
    //   - l'orientamento orizzontale con la base j1.
    // Il verso dei movimenti e' dedotto dalla cinematica; se dopo un passo
    // l'errore peggiora, il verso di quell'asse viene invertito da solo.

    std::atomic<bool> never_stop{false};

    // Attende un errore arrivato dopo l'istante 'after'
    auto waitFreshError =
        [&](const rclcpp::Time & after,
            double timeout_s,
            ImageError & out) {

            const auto sim_deadline =
                node->now() +
                rclcpp::Duration::from_seconds(timeout_s);

            const auto wall_deadline =
                std::chrono::steady_clock::now() +
                std::chrono::duration<double>(
                    std::max(10.0, 30.0 * timeout_s)
                );

            while (rclcpp::ok()) {

                {
                    std::lock_guard<std::mutex> lock(err_mutex);

                    if (latest_err.valid &&
                        latest_err.stamp > after) {

                        out = latest_err;
                        return true;
                    }
                }

                if (node->now() > sim_deadline ||
                    std::chrono::steady_clock::now() >
                        wall_deadline) {

                    return false;
                }

                std::this_thread::sleep_for(20ms);
            }

            return false;
        };

    auto centerOnMarkers =
        [&]() {

            if (n < 3) {

                RCLCPP_WARN(
                    logger,
                    "Centraggio impossibile: servono almeno 3 giunti."
                );

                return false;
            }

            const double tan_h = std::tan(cam_hfov / 2.0);
            const double tan_v = tan_h / cam_aspect;

            // Posa dei giunti nel momento del rilevamento
            std::vector<double> q_detect;

            {
                std::lock_guard<std::mutex> lock(js_mutex);

                for (const auto & name : joint_names) {

                    auto it = joints_at_detection.find(name);

                    if (it == joints_at_detection.end()) {
                        q_detect.clear();
                        break;
                    }

                    q_detect.push_back(it->second);
                }
            }

            bool recovered = false;

            double sign_yaw = 1.0;
            double sign_pitch = 1.0;

            ImageError prev;
            bool have_prev = false;
            double last_dyaw = 0.0;
            double last_dpitch = 0.0;

            // Il controller tiene la posizione dopo la cancellazione:
            // aspetto che il braccio si fermi davvero.
            sleepSim(node, center_settle);

            for (int step = 0;
                 step < center_max_steps && rclcpp::ok();
                 ++step) {

                ImageError err;

                bool seen =
                    waitFreshError(node->now(), 3.0, err);

                // Marker persi (es. il braccio e' scivolato oltre mentre
                // frenava): torno una volta alla posa del rilevamento.
                if (!seen &&
                    !recovered &&
                    q_detect.size() == n) {

                    recovered = true;

                    RCLCPP_WARN(
                        logger,
                        "Marker non visibili: torno alla posa del rilevamento."
                    );

                    moveSmooth(
                        move_group,
                        joint_names,
                        clampToLimits(q_detect),
                        action_client,
                        logger,
                        joint_speed,
                        center_move_time,
                        node,
                        never_stop
                    );

                    sleepSim(node, center_settle);

                    seen = waitFreshError(node->now(), 3.0, err);
                }

                if (!seen) {

                    RCLCPP_WARN(
                        logger,
                        "Centraggio: marker non visibili, mi fermo qui."
                    );

                    return false;
                }

                RCLCPP_INFO(
                    logger,
                    "Centraggio %d/%d: errore orizzontale %+.2f, "
                    "verticale %+.2f (marker visibili: %d)",
                    step + 1,
                    center_max_steps,
                    err.ex,
                    err.ey,
                    err.markers
                );

                if (std::fabs(err.ex) <= center_tol &&
                    std::fabs(err.ey) <= center_tol) {

                    RCLCPP_INFO(
                        logger,
                        "Marker centrati nell'immagine."
                    );

                    return true;
                }

                // Se l'errore e' peggiorato dopo l'ultimo movimento, il
                // verso scelto per quell'asse era sbagliato: lo inverto.
                if (have_prev) {

                    if (std::fabs(last_dyaw) > 1e-6 &&
                        std::fabs(err.ex) >
                            std::fabs(prev.ex) + 0.05) {

                        sign_yaw = -sign_yaw;

                        RCLCPP_WARN(
                            logger,
                            "Centraggio: inverto il verso orizzontale (j1)."
                        );
                    }

                    if (std::fabs(last_dpitch) > 1e-6 &&
                        std::fabs(err.ey) >
                            std::fabs(prev.ey) + 0.05) {

                        sign_pitch = -sign_pitch;

                        RCLCPP_WARN(
                            logger,
                            "Centraggio: inverto il verso verticale (j3/j2)."
                        );
                    }
                }

                // Correzione in radianti.
                // Marker a destra (ex > 0)  -> j1 diminuisce.
                // Marker in basso (ey > 0)  -> camera verso il basso ->
                //                              j3 diminuisce.
                double dyaw = 0.0;
                double dpitch = 0.0;

                if (std::fabs(err.ex) > center_tol) {

                    dyaw =
                        -sign_yaw * center_gain *
                        std::atan(err.ex * tan_h);
                }

                if (std::fabs(err.ey) > center_tol) {

                    dpitch =
                        -sign_pitch * center_gain *
                        std::atan(err.ey * tan_v);
                }

                dyaw = std::clamp(
                    dyaw, -center_max_step, center_max_step
                );

                dpitch = std::clamp(
                    dpitch, -center_max_step, center_max_step
                );

                const auto q = move_group.getCurrentJointValues();

                if (q.size() != n) {

                    RCLCPP_WARN(
                        logger,
                        "Centraggio: stato dei giunti non valido."
                    );

                    return false;
                }

                std::vector<double> q_goal = q;

                q_goal[0] += dyaw;

                // Inclinazione: prima sul gomito (j3); se e' al limite
                // il residuo va sulla spalla (j2), cioe' il braccio si abbassa.
                const double q3_wanted = q[2] + dpitch;

                const double q3_limited = std::clamp(
                    q3_wanted, -max_arm_bend, max_arm_bend
                );

                q_goal[2] = q3_limited;
                q_goal[1] += (q3_wanted - q3_limited);

                q_goal = clampBend(q_goal);

                double max_delta = 0.0;

                for (size_t j = 0; j < n; ++j) {

                    max_delta = std::max(
                        max_delta,
                        std::fabs(q_goal[j] - q[j])
                    );
                }

                if (max_delta < 0.002) {

                    RCLCPP_WARN(
                        logger,
                        "Centraggio: giunti al limite, non posso muovermi oltre."
                    );

                    return false;
                }

                prev = err;
                have_prev = true;
                last_dyaw = dyaw;
                last_dpitch = dpitch;

                if (!moveSmooth(
                        move_group,
                        joint_names,
                        q_goal,
                        action_client,
                        logger,
                        joint_speed,
                        center_move_time,
                        node,
                        never_stop)) {

                    RCLCPP_WARN(
                        logger,
                        "Movimento di centraggio fallito."
                    );

                    return false;
                }

                sleepSim(node, center_settle);
            }

            RCLCPP_WARN(
                logger,
                "Centraggio: raggiunto il numero massimo di passi."
            );

            return false;
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

        bool centered = false;

        if (center_after_found) {

            RCLCPP_INFO(
                logger,
                "Abbasso/inclino il braccio per inquadrare meglio i marker..."
            );

            centered = centerOnMarkers();

        } else {

            sleepSim(node, center_settle);
        }

        // Braccio fermo nella posa finale: il Python puo' stimare il pannello.
        std_msgs::msg::Bool done;
        done.data = true;
        align_done_pub->publish(done);

        if (centered) {

            RCLCPP_INFO(
                logger,
                "Marker centrati: stima finale del pannello richiesta."
            );

        } else {

            RCLCPP_WARN(
                logger,
                "Centraggio non completato: stima finale richiesta comunque."
            );
        }

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
