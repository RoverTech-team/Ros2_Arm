//Prima esegue un pre-orientamento del braccio: cerca un angolo del primo giunto che punti la telecamera verso una posizione approssimativa del pannello, mantenendo una configurazione inclinata di spalla e gomito.

//Sto codice funzione che il braccio punta la camera verso una posizione approssimativa del pannello modificando progressicamente gli angoli dei giunti. Se non vede Aruco esegue una scansione con j1 modificando periodicamente anche l'inclinazione tramite j2 e j3, poi se ne trova uno usa l'errore nell'immagine inviato da python per orientare meglio la camera. se trova aruco interrompe la ricerca.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
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
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>

using FollowJointTrajectoryAction =
    control_msgs::action::FollowJointTrajectory;

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

using namespace std::chrono_literals;


// ==========================================================
// ATTESA DI UN FUTURE
// ==========================================================

//
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
// MOVIMENTO DIRETTO "MINIMUM JERK" + CANCELLAZIONE ARUCO
// ==========================================================
// Versione per la demo in Gazebo. Come nel C++ originale, la
// traiettoria viene inviata direttamente ad arm_controller, senza
// chiamare MoveIt::plan(): si evita cosi' che contatti interni gia'
// presenti nel modello impediscano anche il primo movimento.
//
// IMPORTANTE: questa modalita' NON verifica le collisioni lungo
// la traiettoria. Non usarla su un rover reale. Con il pannello
// vicino, controllare in Gazebo le distanze di sicurezza.
// A differenza del vecchio codice, verifichiamo anche target, limiti
// del modello, accelerazione e ampiezza del singolo spostamento.
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
    if (!rclcpp::ok() || stop_flag.load()) {
        return false;
    }

    const auto start = move_group.getCurrentJointValues();
    if (start.size() != goal.size() ||
        joint_names.size() != goal.size() || goal.empty()) {
        RCLCPP_ERROR(logger, "Movimento diretto: dimensione dei giunti non valida.");
        return false;
    }

    const auto robot_model = move_group.getRobotModel();
    if (!robot_model) {
        RCLCPP_ERROR(logger, "Movimento diretto: modello robot non disponibile.");
        return false;
    }

    double max_delta = 0.0;
    for (size_t j = 0; j < goal.size(); ++j) {
        if (!std::isfinite(start[j]) || !std::isfinite(goal[j])) {
            RCLCPP_ERROR(logger, "Movimento diretto: valore non finito nel giunto %s.",
                         joint_names[j].c_str());
            return false;
        }

        const auto * joint = robot_model->getJointModel(joint_names[j]);
        if (joint && !joint->getVariableBounds().empty()) {
            const auto & bounds = joint->getVariableBounds()[0];
            if (bounds.position_bounded_ &&
                (goal[j] < bounds.min_position_ - 1e-5 ||
                 goal[j] > bounds.max_position_ + 1e-5)) {
                RCLCPP_ERROR(logger, "Movimento diretto: target di %s fuori limite.",
                             joint_names[j].c_str());
                return false;
            }
        }

        max_delta = std::max(max_delta, std::fabs(goal[j] - start[j]));
    }

    // La scansione ArUco lavora gia' a piccoli passi (0.20/0.12).
    // Impediamo richieste dirette troppo grandi senza collision checking.
    constexpr double kMaxDeltaPerCommand = 0.40;
    if (max_delta > kMaxDeltaPerCommand) {
        RCLCPP_WARN(logger,
            "Movimento diretto rifiutato: delta %.3f supera il limite %.2f "
            "(unita' del giunto).",
            max_delta, kMaxDeltaPerCommand);
        return false;
    }

    if (max_delta < 1e-5) {
        // Nessun movimento necessario.
        return true;
    }

    // Quintica minimum-jerk: velocita' e accelerazione nulle agli estremi.
    // Il fattore 1.875 e' il massimo della derivata normalizzata.
    // Il fattore ~5.8 limita il picco di accelerazione della quintica.
    const double speed = std::clamp(joint_speed, 0.03, 0.50);
    constexpr double kMaxAccel = 0.50;
    const double T = std::max({
        std::max(min_time, 0.3),
        1.875 * max_delta / speed,
        std::sqrt(5.8 * max_delta / kMaxAccel)
    });
    const int N = std::max(10, static_cast<int>(std::ceil(T * 20.0)));

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names;
    traj.header.stamp = builtin_interfaces::msg::Time();
    traj.points.reserve(static_cast<size_t>(N));

    for (int k = 1; k <= N; ++k) {
        const double tau = static_cast<double>(k) / static_cast<double>(N);
        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;
        const double tau4 = tau3 * tau;
        const double tau5 = tau4 * tau;

        const double s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
        const double ds = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / T;
        const double dds = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (T * T);

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.reserve(goal.size());
        pt.velocities.reserve(goal.size());
        pt.accelerations.reserve(goal.size());

        for (size_t j = 0; j < goal.size(); ++j) {
            const double D = goal[j] - start[j];
            pt.positions.push_back(start[j] + D * s);
            pt.velocities.push_back(D * ds);
            pt.accelerations.push_back(D * dds);
        }

        pt.time_from_start = rclcpp::Duration::from_seconds(T * tau);
        traj.points.push_back(std::move(pt));
    }

    // Anche se ArUco appare mentre generiamo i punti, non inviare il goal.
    if (!rclcpp::ok() || stop_flag.load()) {
        return false;
    }

    static std::atomic<bool> warning_printed{false};
    if (!warning_printed.exchange(true)) {
        RCLCPP_WARN(logger,
            "Demo Gazebo: movimento diretto minimum-jerk; "
            "collision checking MoveIt NON eseguito.");
    }

    RCLCPP_INFO(logger, "Movimento diretto: delta massimo %.3f, durata %.2f s, %d punti.",
                max_delta, T, N);
    return sendTrajectory(traj, client, node, logger, T, stop_flag);
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
    // (Gemini 305 simulata: 1.53589 rad, risoluzione 848x530 o 1280x800)
    node->declare_parameter<double>(
        "camera_hfov", 1.535890
    );

    node->declare_parameter<double>(
        "camera_aspect", 848.0 / 530.0
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

    // Ultimo stato dei giunti e istante del rilevamento (diagnostica).
    std::mutex js_mutex;
    std::map<std::string, double> latest_joints;
    std::map<std::string, double> joints_at_detection;

    // Ultimo errore dei marker nell'immagine (da /panel/image_error)
    std::mutex err_mutex;
    ImageError latest_err;
    // Conserva la conferma PnP anche se i marker escono dalla camera.
    std::atomic<bool> pnp_confirmed{false};

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
                if (latest_err.markers >= 2 &&
                    std::isfinite(latest_err.ex) &&
                    std::isfinite(latest_err.ey)) {
                    pnp_confirmed.store(true);
                }
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
    // CENTRAGGIO DEL PANNELLO NELL'IMMAGINE
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

            // Nessuna posa di recupero: la posizione dei marker e' gia'
            // stata rilevata dal Python; la depth usera' la posa salvata.
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

                if (!seen || err.markers < 2) {
                    RCLCPP_WARN(logger,
                        "Centraggio: ArUco usciti dall'immagine. NON torno indietro; "
                        "mantengo la posa attuale per la misura depth.");
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
    // RICERCA VISIVA ATTIVA (NESSUNA POSA NOTA DEL PANNELLO)
    // ======================================================
    // Partiamo dallo stato corrente del braccio: non imponiamo una
    // configurazione che conosca gia' l'orientamento del pannello.
    // 0 marker: sweep progressivo su j1; inverte ai limiti, varia pitch.
    // 1 marker: si orienta usando il suo errore nell'immagine.
    // 2 marker: l'azione corrente e' cancellata; segue il centraggio.

    node->declare_parameter<double>("search_yaw_step", 0.20);
    node->declare_parameter<double>("search_pitch_step", 0.12);
    node->declare_parameter<int>("search_max_steps", 85);
    node->declare_parameter<double>("search_observe_wall_s", 0.7);
    node->declare_parameter<double>("search_one_marker_gain", 0.65);

    const double yaw_step = std::clamp(
        node->get_parameter("search_yaw_step").as_double(), 0.04, 0.30);
    const double pitch_step = std::clamp(
        node->get_parameter("search_pitch_step").as_double(), 0.03, 0.20);
    const int max_search_steps = std::max(1,
        static_cast<int>(node->get_parameter("search_max_steps").as_int()));
    const double search_observe_wall_s = std::max(0.3,
        node->get_parameter("search_observe_wall_s").as_double());
    const double one_marker_gain = std::clamp(
        node->get_parameter("search_one_marker_gain").as_double(), 0.1, 0.9);

    int yaw_direction = +1;
    int pitch_direction = -1;
    int local_search_direction = +1;
    int failed_moves = 0;
    double tracking_yaw_sign = 1.0;
    double previous_marker_ex = 0.0;
    double previous_marker_yaw_step = 0.0;
    bool have_previous_marker = false;

    // ======================================================
    // PRE-ORIENTAMENTO: VECCHIA POSA INCLINATA + YAW VERSO PANNELLO
    // ======================================================
    // La demo precedente piegava il braccio usando una posa ESPLICITA:
    // { j1=-1.57, j2=-0.8, j3=+0.2 } (poi limitata da clampBend).
    // Qui manteniamo quell'inclinazione per j2/j3, ma scegliamo j1
    // cinematicamente in base alla posizione del pannello nel launch.
    // Non facciamo ottimizzare j2/j3: deve piegarsi davvero prima dello sweep.
    // Solo per Gazebo: moveSmooth NON controlla collisioni lungo la traiettoria.
    node->declare_parameter<bool>("preaim_enabled", true);
    node->declare_parameter<double>("preaim_target_x", 0.40);
    node->declare_parameter<double>("preaim_target_y", 0.0);
    node->declare_parameter<double>("preaim_target_z", 0.65);
    node->declare_parameter<double>("preaim_max_rotation", 2.8);
    node->declare_parameter<double>("preaim_shoulder_angle", -0.8);
    node->declare_parameter<double>("preaim_elbow_angle", 0.12);

    if (node->get_parameter("preaim_enabled").as_bool() &&
        !aruco_found.load() && rclcpp::ok()) {

        const auto yaw_it = std::find(joint_names.begin(), joint_names.end(), "j1_joint");
        const auto shoulder_it = std::find(joint_names.begin(), joint_names.end(), "j2_joint");
        const auto elbow_it = std::find(joint_names.begin(), joint_names.end(), "j3_joint");

        if (yaw_it == joint_names.end() || shoulder_it == joint_names.end() ||
            elbow_it == joint_names.end()) {
            RCLCPP_ERROR(logger, "PRE-AIM: giunti j1/j2/j3 non trovati.");
        } else {
            const size_t yi = static_cast<size_t>(std::distance(joint_names.begin(), yaw_it));
            const size_t si = static_cast<size_t>(std::distance(joint_names.begin(), shoulder_it));
            const size_t ei = static_cast<size_t>(std::distance(joint_names.begin(), elbow_it));
            const auto current = move_group.getCurrentJointValues();

            if (current.size() != n) {
                RCLCPP_WARN(logger, "PRE-AIM: stato giunti non disponibile.");
            } else {
                std::vector<double> desired = current;
                const double old_pose_j2 =
                    node->get_parameter("preaim_shoulder_angle").as_double();
                const double old_pose_j3 =
                    node->get_parameter("preaim_elbow_angle").as_double();

                // Esattamente il criterio della vecchia demo: posa inclinata
                // e limiti di j2/j3 rispettati tramite clampBend().
                desired[si] = old_pose_j2;
                desired[ei] = old_pose_j3;
                desired = clampBend(desired);

                // Per j1 manteniamo il vantaggio del pre-aim precedente:
                // scegliamo la rotazione con la cinematica, ma tenendo
                // j2 e j3 FISSI alla posa inclinata della vecchia demo.
                const auto * optical_link =
                    robot_model->getLinkModel("gemini_camera_optical_frame");
                const auto * base_link = robot_model->getLinkModel("base_footprint");
                const auto state = move_group.getCurrentState(3.0);
                if (state && optical_link && base_link) {
                    const Eigen::Vector3d target(
                        node->get_parameter("preaim_target_x").as_double(),
                        node->get_parameter("preaim_target_y").as_double(),
                        node->get_parameter("preaim_target_z").as_double());
                    if (target.allFinite()) {
                        const double max_rotation = std::clamp(
                            node->get_parameter("preaim_max_rotation").as_double(),
                            0.2, 3.0);
                        double low = current[yi] - max_rotation;
                        double high = current[yi] + max_rotation;
                        const auto * joint = robot_model->getJointModel("j1_joint");
                        if (joint && !joint->getVariableBounds().empty()) {
                            const auto & b = joint->getVariableBounds()[0];
                            if (b.position_bounded_) {
                                const double margin = limit_margin *
                                    (b.max_position_ - b.min_position_);
                                low = std::max(low, b.min_position_ + margin);
                                high = std::min(high, b.max_position_ - margin);
                            }
                        }
                        if (low <= high) {
                            double best_yaw = std::clamp(current[yi], low, high);
                            double best_score = -std::numeric_limits<double>::infinity();
                            // Una copia modificabile dello stato, senza inviare comandi.
                            auto test_state = *state;
                            test_state.setVariablePosition("j2_joint", desired[si]);
                            test_state.setVariablePosition("j3_joint", desired[ei]);
                            for (int i = 0; i <= 160; ++i) {
                                const double yaw = low + (high-low) * (double(i)/160.0);
                                test_state.setVariablePosition("j1_joint", yaw);
                                test_state.update();
                                const Eigen::Isometry3d cam =
                                    test_state.getGlobalLinkTransform(base_link).inverse() *
                                    test_state.getGlobalLinkTransform(optical_link);
                                const Eigen::Vector3d to_panel = target - cam.translation();
                                if (!to_panel.allFinite() || to_panel.norm() < 0.06) continue;
                                const Eigen::Vector3d forward =
                                    cam.linear() * Eigen::Vector3d::UnitZ();
                                const double score = forward.dot(to_panel.normalized()) -
                                    0.003 * std::fabs(yaw - current[yi]);
                                if (score > best_score) {
                                    best_score = score;
                                    best_yaw = yaw;
                                }
                            }
                            if (std::isfinite(best_score)) {
                                desired[yi] = best_yaw;
                                RCLCPP_INFO(logger,
                                    "PRE-AIM: yaw cinematica %.3f rad (score %.3f), "
                                    "inclinazione vecchia demo mantenuta.",
                                    best_yaw, best_score);
                            }
                        }
                    }
                } else {
                    RCLCPP_WARN(logger,
                        "PRE-AIM: TF ottico/stato MoveIt assente, "
                        "piego comunque j2 e j3 come nella vecchia demo.");
                }

                desired = clampBend(desired);
                RCLCPP_INFO(logger,
                    "PRE-AIM: j1 %.3f -> %.3f; j2 %.3f -> %.3f; "
                    "j3 %.3f -> %.3f rad (j2 vecchia posa %.2f, con clamp).",
                    current[yi], desired[yi], current[si], desired[si],
                    current[ei], desired[ei], old_pose_j2);

                // Piccoli segmenti minimum-jerk: la routine moveSmooth
                // rifiuta delta > 0.40, quindi ciascun giunto varia <= 0.30.
                // j1, j2, j3 sono comandati INSIEME a ogni segmento.
                constexpr double kStep = 0.30;
                constexpr int kMaxSteps = 24;
                bool completed = false;
                for (int attempt = 0; attempt < kMaxSteps &&
                     rclcpp::ok() && !aruco_found.load(); ++attempt) {
                    const auto now = move_group.getCurrentJointValues();
                    if (now.size() != n) break;
                    const double e1 = desired[yi] - now[yi];
                    const double e2 = desired[si] - now[si];
                    const double e3 = desired[ei] - now[ei];
                    if (std::max({std::fabs(e1), std::fabs(e2), std::fabs(e3)}) < 0.025) {
                        completed = true;
                        break;
                    }
                    std::vector<double> q = now;
                    q[yi] += std::clamp(e1, -kStep, kStep);
                    q[si] += std::clamp(e2, -kStep, kStep);
                    q[ei] += std::clamp(e3, -kStep, kStep);
                    if (!goTo(q)) {
                        if (!aruco_found.load())
                            RCLCPP_WARN(logger,
                                "PRE-AIM: movimento non riuscito; controllare "
                                "log del controller e joint_states.");
                        break;
                    }
                }
                if (completed) {
                    RCLCPP_INFO(logger,
                        "PRE-AIM: inclinazione iniziale eseguita; inizio ricerca ArUco.");
                } else if (!aruco_found.load()) {
                    RCLCPP_WARN(logger,
                        "PRE-AIM: posa non confermata; inizio comunque ricerca ArUco.");
                }
            }
        }
    }

    RCLCPP_INFO(logger,
        "RICERCA ATTIVA: pre-orientamento con inclinazione completato (o saltato); "
        "ora correggo l'inquadratura con gli ArUco.");

    for (int step = 0; step < max_search_steps && rclcpp::ok(); ++step) {
        if (aruco_found.load()) break;

        // Lascio arrivare nuovi frame senza tenere sempre il braccio in moto.
        const auto until = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(search_observe_wall_s);
        while (rclcpp::ok() && !aruco_found.load() &&
               std::chrono::steady_clock::now() < until) {
            std::this_thread::sleep_for(20ms);
        }
        if (aruco_found.load() || !rclcpp::ok()) break;

        const auto current = move_group.getCurrentJointValues();
        if (current.size() != n) {
            RCLCPP_ERROR(logger, "Impossibile leggere i giunti durante la ricerca.");
            break;
        }

        ImageError observed;
        bool recent = false;
        {
            std::lock_guard<std::mutex> lock(err_mutex);
            const auto age = node->now() - latest_err.stamp;
            recent = latest_err.valid && age.nanoseconds() >= 0 &&
                     age.seconds() < 1.5;
            if (recent) observed = latest_err;
        }

        std::vector<double> target = current;
        const bool one_marker = recent && observed.markers == 1;
        if (one_marker) {
            // Se l'errore orizzontale peggiora, inverto il verso della
            // correzione: dipende dalla convenzione dei giunti del braccio.
            if (have_previous_marker &&
                std::fabs(previous_marker_yaw_step) > 0.015 &&
                std::fabs(observed.ex) > std::fabs(previous_marker_ex) + 0.06) {
                tracking_yaw_sign = -tracking_yaw_sign;
                RCLCPP_WARN(logger, "Inverto il verso del tracking del marker.");
            }
            // Un marker: inseguimento visuale, senza conoscere quale lato
            // del mondo occupi il pannello. Se e' gia' centrato, piccola
            // esplorazione locale per portare in vista anche il secondo.
            const double tan_h = std::tan(cam_hfov / 2.0);
            const double tan_v = tan_h / cam_aspect;
            double dyaw = -tracking_yaw_sign * one_marker_gain *
                std::atan(observed.ex * tan_h);
            double dpitch = -one_marker_gain *
                std::atan(observed.ey * tan_v);
            dyaw = std::clamp(dyaw, -yaw_step, yaw_step);
            dpitch = std::clamp(dpitch, -pitch_step, pitch_step);
            if (std::fabs(observed.ex) < 0.09) {
                dyaw = 0.06 * local_search_direction;
                local_search_direction = -local_search_direction;
            }
            target[0] += dyaw;
            if (n >= 3) {
                target[2] += dpitch;
            }
            previous_marker_ex = observed.ex;
            previous_marker_yaw_step = dyaw;
            have_previous_marker = true;
            RCLCPP_INFO(logger,
                "Marker singolo: correzione immagine ex=%+.2f ey=%+.2f",
                observed.ex, observed.ey);
        } else {
            have_previous_marker = false;
            // Nessun marker recente: sweep orizzontale e, ogni quattro
            // passi, variazione VERTICALE della spalla j2. Prima lo sweep
            // muoveva solo j1 finche' non raggiungeva il limite.
            target[0] += yaw_direction * yaw_step;
            if (n >= 3 && (step % 4 == 3)) {
                const double shoulder_before = current[1];
                target[1] += pitch_direction * pitch_step;
                target = clampBend(target);
                if (std::fabs(target[1] - shoulder_before) < 0.01) {
                    pitch_direction = -pitch_direction;
                    target[1] = shoulder_before + pitch_direction * pitch_step;
                }
            }
        }

        target = clampBend(target);

        // Se il giunto di base e' arrivato al limite, inverto il senso
        // e cambio leggermente l'inclinazione per esplorare un'altra fascia.
        if (std::fabs(target[0] - current[0]) < 0.015) {
            yaw_direction = -yaw_direction;
            target[0] = current[0] + yaw_direction * yaw_step;
            if (n >= 3) {
                target[2] += pitch_direction * pitch_step;
                target = clampBend(target);
                if (std::fabs(target[2] - current[2]) < 0.01)
                    pitch_direction = -pitch_direction;
            }
            target = clampBend(target);
        }

        double biggest_change = 0.0;
        for (size_t j = 0; j < n; ++j)
            biggest_change = std::max(biggest_change,
                                      std::fabs(target[j] - current[j]));
        if (biggest_change < 0.01) {
            ++failed_moves;
            if (failed_moves >= 5) break;
            continue;
        }

        RCLCPP_INFO(logger, "Ricerca passo %d/%d: %s",
            step + 1, max_search_steps,
            one_marker ? "seguo marker visibile" : "sweep senza marker");

        const bool moved = goTo(target);
        if (aruco_found.load()) break;
        if (!moved) {
            ++failed_moves;
            yaw_direction = -yaw_direction;
            if (failed_moves >= 5) {
                RCLCPP_ERROR(logger,
                    "Troppi movimenti non riusciti: fermo il braccio.");
                break;
            }
        } else {
            failed_moves = 0;
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

        // La ricerca dei marker ha gia' fornito una posa PnP al Python.
        // Se vengono persi abbassando il braccio, NON e' necessario
        // recuperarli: il Python memorizza la posa del pannello in base_link
        // e usa TF ottica aggiornata per la depth a braccio fermo.
        // Non autorizzare acquisizioni durante il movimento.
        sleepSim(node, center_settle);
        const auto stable_before = move_group.getCurrentJointValues();
        sleepSim(node, 0.5);
        const auto stable_after = move_group.getCurrentJointValues();
        bool stable = stable_before.size() == n && stable_after.size() == n;
        double max_change = 0.0;
        if (stable) {
            for (size_t j = 0; j < n; ++j) {
                if (!std::isfinite(stable_before[j]) ||
                    !std::isfinite(stable_after[j])) {
                    stable = false;
                    break;
                }
                max_change = std::max(max_change,
                    std::fabs(stable_after[j] - stable_before[j]));
            }
            stable = stable && max_change < 0.005;
        }
        // Conferma storica del PnP: il Python salva la posa in base_link.
        // Il marker puo' essere uscito dal campo visivo dopo il tilt.
        const bool pnp_seen = pnp_confirmed.load();
        if (stable && pnp_seen) {
            std_msgs::msg::Bool done;
            done.data = true;
            align_done_pub->publish(done);
            RCLCPP_INFO(logger,
                "DEPTH: /panel/align_done PUBBLICATO, nessun ritorno ai marker "
                "(centraggio=%s, delta giunti=%.4f).",
                centered ? "completato" : "interrotto", max_change);
        } else {
            RCLCPP_WARN(logger,
                "DEPTH: non abilitata: braccio_fermo=%s PnP_precedente=%s "
                "(delta=%.4f). Nessun movimento di recupero.",
                stable ? "si" : "no", pnp_seen ? "si" : "no", max_change);
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
