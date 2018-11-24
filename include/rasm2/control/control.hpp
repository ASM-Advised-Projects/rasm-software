/**
 * Defines the Controller class.
 */

#ifndef RASM2_CONTROL_CONTROL_HPP
#define RASM2_CONTROL_CONTROL_HPP

#include <map>
#include <thread>
#include <pthread.h>
#include <unistd.h>

#include <Poco/Semaphore.h>
#include <Poco/Exception.h>

#include "pose_estimation_async.hpp"
#include "trajectory_generation.hpp"
#include "trajectory_tracking.hpp"
#include "rasm2/periphery/uc_board.hpp"
//#include "rasm2/battery.hpp"
#include "rasm2/util/time.hpp"
#include "rasm2/util/pose.hpp"
#include "rasm2/configuration.hpp"
#include "../battery.hpp"

/**
 * TODO - pose rotation
 * TODO - feedforward
 * TODO - feedback
 */
class Controller
{
protected:
  /**
   * This enumeration represents the different states that this controller can
   * be within when running.
   */
  enum State {
    OVERRIDE,
    SEARCH,
    FACE,
    MARKER,
    FACE_MOVING,
    MARKER_MOVING,
    YIELD
  };

  State state;  // current state of this controller

  UCBoard &uc_board;
  BatteryEstimator &battery_estimator;

  AsyncPoseEstimator pose_estimator;
  TrajectoryGenerator traj_generator;
  FeedforwardGenerator ff_generator;
  PIDFeedbackGenerator fb_generator;
  MotionSentinel motion_sentinel;

  bool running;  // indicates if control method has been called
  bool shutdown;  // indicates if the state machine should exit
  Poco::Semaphore shutdown_sema;  // blocking structure for control method

  int precision_check_period;
  int moving_check_period;
  int force_check_period;

  std::array<double, 6> joint_positions;
  Poco::Event joint_positions_required;
  Poco::Event joint_positions_updated;

  /**
   * State-to-time maps that describe what estimation periods for faces and
   * markers should be used for each state.
   */
  std::map<State, int> face_est_period_map;
  std::map<State, int> marker_est_period_map;

  /**
   * References to the current face and marker pose values held in the pose
   * estimation instance being used.
   */
  Pose &face_pose;
  Pose &marker_pose;

  /**
   * The relative pose that should result when the screen is perfectly aligned
   * with the object being tracked (for both faces and markers).
   */
  Pose aligned_pose;

  /**
   * The pose vector that holds the thresholds for differences between the
   * aligned pose and the current pose for each pose element. If any of these
   * thresholds are exceeded then the screen should be moved closer to the
   * aligned pose.
   */
  Pose pose_diff_threshold;

  /**
   * The pose vector that holds the required precision when moving to the
   * destination pose for each pose element. The values of these elements should
   * each be less than their corresponding value in pose_diff_threshold.
   */
  Pose pose_precision;

  /**
   * Returns true if any one of the pose elements of the given relative pose
   * differs from the aligned pose by more than the difference threshold;
   * returns false otherwise.
   */
  bool exceeds_threshold(const Pose &pose)
  {
    Pose diff = PoseOps::subtract(pose, aligned_pose);
    return PoseOps::magnitude_exceeds(diff, pose_diff_threshold);
  }

  /**
   * Sets the pose estimation periods for the given state.
   */
  void set_estimation_periods(State state)
  {
    pose_estimator.set_face_period(face_est_period_map[state]);
    pose_estimator.set_marker_period(marker_est_period_map[state]);
  }

  /**
   * Runs this controller's finite state machine until shutdown is true.
   */
  void run_state_machine()
  {
    Pose zero_pose = {0, 0, 0, 0, 0, 0};

    // settings
    int wait_period = 100;

    // initial state transition
    state = State::SEARCH;
    pose_estimator.unlock_face();
    pose_estimator.unlock_marker();
    set_estimation_periods(state);

    // repeately execute blocks of code corresponding to the current state
    // until it's time to shutdown
    bool new_pose;
    Trajectory6D trajectory;
    Pose final_global_pose;
    Pose pose_diff;
    while (!shutdown)
    {
      switch (state)
      {
        case OVERRIDE:

          break;

        case SEARCH:
          // wait on new marker or face pose event w/timeout
          new_pose = pose_estimator.general_pose_event.tryWait(wait_period);

          // potential state transitions
          if (new_pose)  // if timeout did not occur
          {
            if (!PoseOps::equal(marker_pose, zero_pose))  // to MARKER
            {
              state = State::MARKER;
              pose_estimator.lock_marker();
              set_estimation_periods(state);
            }
            else if (!PoseOps::equal(face_pose, zero_pose))  // to FACE
            {
              state = State::FACE;
              pose_estimator.lock_face();
              set_estimation_periods(state);
              pose_estimator.general_pose_event.set();
            }
          }

          break;

        case FACE:
          // wait on new marker or face pose event w/timeout
          new_pose = pose_estimator.general_pose_event.tryWait(wait_period);

          // potential state transitions
          if (new_pose)  // if timeout did not occur
          {
            if (PoseOps::equal(face_pose, zero_pose))  // to SEARCH
            {
              state = State::SEARCH;
              pose_estimator.unlock_face();
              set_estimation_periods(state);
            }
            else if (!PoseOps::equal(marker_pose, zero_pose))  // to MARKER
            {
              state = State::MARKER;
              pose_estimator.unlock_face();
              pose_estimator.lock_marker();
              set_estimation_periods(state);
            }
            else if (exceeds_threshold(face_pose))  // to FACE_MOVING
            {
              state = State::FACE_MOVING;
              set_estimation_periods(state);
            }
          }

          break;

        case MARKER:
          // wait on new marker or face pose event w/timeout
          new_pose = pose_estimator.marker_pose_event.tryWait(wait_period);

          // potential state transitions
          if (new_pose)  // if timeout did not occur
          {
            if (PoseOps::equal(marker_pose, zero_pose))  // to SEARCH
            {
              state = State::SEARCH;
              pose_estimator.unlock_marker();
              set_estimation_periods(state);
              pose_estimator.general_pose_event.reset();
            }
            else if (exceeds_threshold(marker_pose))  // to MARKER_MOVING
            {
              state = State::MARKER_MOVING;
              set_estimation_periods(state);
            }
          }

          break;

        case FACE_MOVING:
          generate_trajectory(face_pose, trajectory, final_global_pose, pose_diff);

          // if difference between initial and final global poses is greater
          // than precision then start moving to track the trajectory
          if (PoseOps::magnitude_exceeds(pose_diff, pose_precision))
            track_trajectory(trajectory, final_global_pose);

          if (state != State::YIELD)
            state = State::FACE;
          set_estimation_periods(state);

          break;

        case MARKER_MOVING:
          generate_trajectory(face_pose, trajectory, final_global_pose, pose_diff);

          // if difference between initial and final global poses is greater
          // than precision then start moving to track the trajectory
          if (PoseOps::magnitude_exceeds(pose_diff, pose_precision))
            track_trajectory(trajectory, final_global_pose);

          if (state != State::YIELD)
            state = State::MARKER;
          set_estimation_periods(state);

          break;

        case YIELD:

          break;
      }
    }
  }

  /**
   * Generates a trajectory to re-align with the given relative object pose.
   * The resultant trajectory is placed into the 'trajectory' argument.
   * Returns the pose difference between initial and final global poses of the
   * trajectory. The actual final global pose that the trajectory terminates at
   * may be restricted due to the limited range of positions the RASM is capable
   * of.
   */
  Pose generate_trajectory(const Pose &relative_object_pose, Trajectory6D &trajectory,
      Pose &final_global_pose, Pose &pose_diff)
  {
    // find initial global pose of screen center
    joint_positions_required.set();
    joint_positions_updated.wait();
    Pose initial_global_pose;
    traj_generator.joint_to_cartesian(joint_positions, initial_global_pose);

    // find final global pose of screen center
    Pose rotated_aligned_pose = PoseOps::rotate_extrinsic(aligned_pose, relative_object_pose);
    final_global_pose = PoseOps::add(initial_global_pose, relative_object_pose);
    final_global_pose = PoseOps::subtract(final_global_pose, rotated_aligned_pose);

    // generate trajectory between initial and final global poses
    final_global_pose = traj_generator.generate_trajectory(
        initial_global_pose, final_global_pose, trajectory);

    pose_diff = PoseOps::subtract(final_global_pose, initial_global_pose);
  }

  /**
   * Tracks the given trajectory until one of the following occurs:
   *  - Shutdown is set to true.
   *  - An external force is detected (detected by ff_generator; state is
   *    subsequently set to YIELD).
   *  - All joints have stopped moving for more than 300 milliseconds.
   *  - The difference between the current pose and the final desired pose
   *    becomes less than pose_precision (for all elements).
   * The battery estimator is disabled throughout this entire method so that
   * noisy battery voltage readings aren't taken and so there exists more
   * throughput on the serial communication line for reading encoders and
   * setting motors.
   */
  void track_trajectory(const Trajectory6D &trajectory, const Pose &final_global_pose)
  {
    battery_estimator.disable_reading();

    long start_time = ProgramTime::current_millis();
    long current_time = start_time;

    //Pose current_precision;
    bool precision_achieved = false;
    long last_precision_check = current_time;

    int not_moving_count = 0;
    long last_moving_check = current_time;

    bool external_force = false;
    long last_force_check = current_time;

    std::array<float, 6> ff_motor_pwms;
    std::array<float, 6> fb_motor_pwms;
    std::array<float, 6> motor_pwms;

    joint_positions_required.set();

    while (!shutdown && not_moving_count < 3 && !precision_achieved && !external_force)
    {
      joint_positions_updated.wait();
      motion_sentinel.update(joint_positions);
      joint_positions_required.set();
      current_time = ProgramTime::current_millis();

      // get feedforward control output
      ff_generator.generate_pwms(trajectory, current_time-start_time, ff_motor_pwms);

      // get feedback control output
      fb_generator.generate_pwms(trajectory, current_time-start_time, joint_positions, fb_motor_pwms);

      // combine feedforward and feedback control outputs
      for (int i = 0; i < 6; ++i)
        motor_pwms[i] = ff_motor_pwms[i] + fb_motor_pwms[i];

      // set motor levels
      uc_board.set_motor_pwms(motor_pwms);

      // determine if precision has been met
      if (current_time - last_precision_check > precision_check_period)
      {
        Pose current_global_pose;
        traj_generator.joint_to_cartesian(joint_positions, current_global_pose);
        Pose current_precision = PoseOps::subtract(current_global_pose, final_global_pose);
        precision_achieved = !PoseOps::exceeds(current_precision, pose_precision);
        last_precision_check = current_time;
      }

      // determine if joints are still moving
      if (current_time - last_moving_check > moving_check_period)
      {
        if (motion_sentinel.motion_exists())
          ++not_moving_count;
        else
          not_moving_count = 0;
        last_moving_check = current_time;
      }

      // determine if there is an external force
      if (current_time - last_force_check > force_check_period)
      {
        external_force = motion_sentinel.external_force_exists();
        last_force_check = current_time;
      }
    }

    // make sure all joints have stopped moving
    motor_pwms.fill(0);
    uc_board.set_motor_pwms(motor_pwms);

    // change state to YIELD if an external force was detected
    if (external_force)
      state = State::YIELD;

    battery_estimator.enable_reading();
  }

  /**
   * Repeatedly waits for the joint_positions_required event to be signaled.
   * Everytime this event is signaled it will update the joint positions and
   * subsequently signal the joint_positions_updated event.
   */
  void read_joint_positions()
  {
    std::array<int, 6> encoder_outputs;
    while (true)
    {
      joint_positions_required.wait();
      uc_board.get_encoder_outputs(encoder_outputs);

      for (int i = 0; i < 6; ++i)
        joint_positions[i] = encoder_outputs[i];

      joint_positions_updated.set();
    }
  }

  /**
   * Repeatedly checks if the power is switched on or off with the given
   * delay in milliseconds between checks. Once the power is off, the shutdown
   * semaphore is set and this method returns.
   */
  void check_power_loop(int delay_millis)
  {
    while (true)
    {
      usleep(delay_millis*1000);
      if (!uc_board.is_pmb_on())
      {
        shutdown_sema.set();
        break;
      }
    }
  }

  /**
   * Repeatedly checks if the battery level is less than 1% with the given
   * delay in milliseconds between checks. Once the level is too low, the
   * shutdown semaphore is set and this method returns.
   */
  void check_battery_loop(int delay_millis)
  {
    while (true)
    {
      usleep(delay_millis*1000);
      if (battery_estimator.battery_percent() < 1.0)
      {
        shutdown_sema.set();
        break;
      }
    }
  }

  static Pose string_to_pose(const std::string num_list)
  {
    Pose result;

  }

public:
  /**
   * Constructs a new controller.
   */
  Controller(UCBoard &ucb, BatteryEstimator &be)
  : uc_board(ucb)
  , battery_estimator(be)
  , pose_estimator()
  , traj_generator()
  , ff_generator()
  , fb_generator()
  , motion_sentinel()
  , shutdown_sema(0, 1)
  {
    running = false;
    shutdown = false;


    // get configuration group for control subsystem
    const MapConfiguration *configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManagerImpl::Group::CONTROL);

    face_est_period_map[State::OVERRIDE] = 0;
    face_est_period_map[State::SEARCH] = configs->getInt("face_estimation_period.search");
    face_est_period_map[State::FACE] = configs->getInt("face_estimation_period.face");
    face_est_period_map[State::MARKER] = configs->getInt("face_estimation_period.marker");
    face_est_period_map[State::FACE_MOVING] = configs->getInt("face_estimation_period.face_moving");
    face_est_period_map[State::MARKER_MOVING] = configs->getInt("face_estimation_period.marker_moving");
    face_est_period_map[State::YIELD] = 0;

    marker_est_period_map[State::OVERRIDE] = 0;
    marker_est_period_map[State::SEARCH] = configs->getInt("marker_estimation_period.search");
    marker_est_period_map[State::FACE] = configs->getInt("marker_estimation_period.face");
    marker_est_period_map[State::MARKER] = configs->getInt("marker_estimation_period.marker");
    marker_est_period_map[State::FACE_MOVING] = configs->getInt("marker_estimation_period.face_moving");
    marker_est_period_map[State::MARKER_MOVING] = configs->getInt("marker_estimation_period.marker_moving");
    marker_est_period_map[State::YIELD] = 0;

    face_pose = pose_estimator.get_face_pose();
    marker_pose = pose_estimator.get_marker_pose();

    aligned_pose = string_to_pose(configs->getString("aligned_pose"));
    pose_diff_threshold = string_to_pose(configs->getString("pose_diff_threshold"));
    pose_precision = string_to_pose(configs->getString("pose_precision"));

    precision_check_period = configs->getInt("precision_check_period");
    moving_check_period = configs->getInt("moving_check_period");
    force_check_period = configs->getInt("force_check_period");
  }

  /**
   * Destructs this controller, causing it to exit the control method if it's
   * currently active.
   */
  ~Controller()
  {
    shutdown_sema.set();
  }

  /**
   * Runs this controller. This method will return either when the destructor is
   * called or when the UCBoard instance indicates the power has been switched
   * off, whichever comes first. If this method has already been called then
   * subsequent calls to this method will throw a Poco::IllegalStateException.
   */
  void control()
  {
    // continue only if this method hasn't been called yet
    if (running)
      throw Poco::IllegalStateException("control() method has already been called.");
    running = true;

    // get configuration group for control subsystem
    const MapConfiguration *configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManagerImpl::Group::CONTROL);

    // start a thread for repeatedly checking if the power has been switched off
    int power_check_period = configs->getInt("power_check_period");
    std::thread check_power_thread(&Controller::check_power_loop, this, power_check_period);

    // start a thread for repeatedly checking if the battery level is too low
    int battery_check_period = configs->getInt("battery_check_period");
    std::thread check_battery_thread(&Controller::check_battery_loop, this, battery_check_period);

    // start a thread for updating the joint positions array
    std::thread joint_positions_thread(&Controller::read_joint_positions, this);

    // start a thread for running the control state machine
    std::thread state_machine_thread(&Controller::run_state_machine, this);

    // set thread scheduling policy and priority of state machine
    sched_param sch_params;
    sch_params.sched_priority = configs->getInt("state_machine_thread_priority");
    pthread_setschedparam(state_machine_thread.native_handle(), SCHED_RR, &sch_params);

    // block till destructor is called, battery is too low, or power is turned off
    shutdown_sema.wait();

    // wait for the state machine to exit
    shutdown = true;
    state_machine_thread.join();
  }
};

#endif
