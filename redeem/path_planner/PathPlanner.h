/*
  This file is part of Redeem - 3D Printer control software

  Author: Mathieu Monney
  License: GNU GPLv3 http://www.gnu.org/copyleft/gpl.html


  This file is based on Repetier-Firmware licensed under GNU GPL v3 and
  available at https://github.com/repetier/Repetier-Firmware

  Redeem is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Redeem is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Redeem.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef __PathPlanner__PathPlanner__
#define __PathPlanner__PathPlanner__

#include "Delta.h"
#include "Path.h"
#include "PathOptimizer.h"
#include "PathQueue.h"
#include "PruTimer.h"
#include "config.h"
#include "vectorN.h"
#include <assert.h>
#include <atomic>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <string.h>
#include <thread>
#include <vector>

class AlarmCallback;

/*
 * The path planner is fed a sequence of paths.
 * These paths are in *ideal* coordinates.
 * If required the path is manipulated such that all conditions, such as
 *  path segment length, are met.
 * The movement vector is calculated using the discrete stepper motion.
 * This is combined with a tracked machine state to give a final stepper
 * 	movement vector that approaches the desired position.
 * The machine state is then updated accordingly and the path is sent to
 *	the queue.
 * This approach gives the effect of constantly 'hunting' the ideal
 * 	position.
 */

class PathPlanner
{
private:
    friend class PathPlannerRunMoveTest;
    friend class PathPlannerTest;

    VectorN machineToWorld(const IntVectorN& machinePos);
    IntVectorN worldToMachine(const VectorN& worldPos, bool* possible = nullptr);

    void pruAlarmCallback();

    AlarmCallback& alarmCallback;

    VectorN maxSpeeds;
    VectorN maxAccelerationStepsPerSquareSecond;
    VectorN maxAccelerationMPerSquareSecond;

    double minimumSpeed;
    VectorN axisStepsPerM;

    std::thread runningThread;
    bool stop;
    std::atomic_bool acceptingPaths;

    PruInterface& pru;
    PathOptimizer optimizer;
    PathQueue<PathOptimizer> pathQueue;
    void recomputeParameters();
    void run();

    void runMove(
        const int moveMask,
        const int cancellableMask,
        const bool sync,
        const bool wait,
        const double moveEndTime,
        std::array<std::vector<Step>, NUM_AXES>& steps,
        std::unique_ptr<SteppersCommand[]> const& commands,
        const size_t commandsLength,
        IntVectorN* probeDistanceTraveled = nullptr,
        SyncCallback* syncCallback = nullptr);

    // pre-processor functions
    int softEndStopApply(const VectorN& endPos);
    void applyBedCompensation(VectorN& endPos);
    void backlashCompensation(IntVectorN& delta);
    bool queue_move_fail;

    // soft endstops
    VectorN soft_endstops_min;
    VectorN soft_endstops_max;

    bool stop_on_soft_endstops_hit;
    bool stop_on_physical_endstops_hit;

    // bed compensation
    std::vector<double> matrix_bed_comp;

    // axis configuration (see config.h for options)
    int axis_config;

    // the current state of the machine
    IntVectorN state;

    // distance of the last bed probe movement
    double lastProbeDistance;

    // slaves
    bool has_slaves;
    std::vector<int> master;
    std::vector<int> slave;
    std::array<uint8_t, NUM_AXES> axes_stepping_together;
    void clearSlaveAxesMovements(VectorN& startWorldPos, VectorN& stopWorldPos);

    // backlash compensation
    VectorN backlash_compensation;
    IntVectorN backlash_state;

    Vector3 worldToHBelt(const Vector3&);
    Vector3 hBeltToWorld(const Vector3&);
    Vector3 worldToCoreXY(const Vector3&);
    Vector3 coreXYToWorld(const Vector3&);

public:
    Delta delta_bot;

    /**
   * @brief Create a new path planner that is used to compute paths parameters and send it to the PRU for execution
   * @details Create a new path planner that is used to compute paths parameters and send it to the PRU for execution
   * @param cacheSize Size of the movement planner cache
   */
    PathPlanner(unsigned int cacheSize, AlarmCallback& alarmCallback, PruInterface& pru);
    PathPlanner(unsigned int cacheSize, AlarmCallback& alarmCallback);

    /**
   * @brief  Init the internal PRU co-processors
   * @details Init the internal PRU co-processors with the provided firmware
   *
   * @param firmware_stepper The firmware for the stepper step generation, will be executed on PRU0
   * @param firmware_endstops The firmware for the endstop checks, will be executed on PRU1
   *
   * @return true in case of success, false otherwise.
   */
    bool initPRU(const std::string& firmware_stepper, const std::string& firmware_endstops)
    {
        return pru.initPRU(firmware_stepper, firmware_endstops);
    }

    void queueSyncEvent(SyncCallback& callback, bool isBlocking = false);
    WaitEvent* queueWaitEvent();

    /**
   * @brief Queue a line move for execution
   * @details Queue a line move execution in the path planner. Note that the path planner
   * has no internal state in term of printer head position. Therefore you have
   * to pass the correct start and end position everytime.
   *
   * The coordinates unit is in meters. As a general rule, every public method of this class use SI units.
   *
   * @param endPos The end position of the path in meters
   * @param speed The feedrate (aka speed) of the move in m/s
   * @param cancelable flags the move as cancelable.
   * @param optimize Wait for additional commands to fill the buffer, to optimize speed.
   * @param enable_soft_endstops use soft end stop values to clip path
   * @param use_bed_matrix use a bed leveling correction
   * @param use_backlash_compensation use backlash compensation
   * @param is_probe move is a probe and probe distance needs to be measured
   * @param tool_axis which axis is our tool attached to
   * @param virgin Flag to indicate if this is a newly passed in value or if it is somewhere in a recursion loop
   */
    void queueMove(VectorN endPos,
        double speed, double accel,
        bool cancelable, bool optimize,
        bool enable_soft_endstops, bool use_bed_matrix,
        bool use_backlash_compensation, bool is_probe, int tool_axis = 3);
    /**
   * @brief Run the path planner thread
   * @details Run the path planner thread that is in charge to compute the different delays and submit it to the PRU for execution.
   */
    void runThread();

    /**
   * @brief Stop the path planner thread
   * @details Stop the path planner thread and optionnaly wait until it is stopped before returning.
   *
   * @param join If true, the method does not return until the thread is effectively stopped.
   */
    void stopThread(bool join);

    /**
   * @brief Wait until all queued move are finished to be executed
   * @details Wait until all queued move are finished to be executed
   */
    void waitUntilFinished();

    /**
   * @brief Set the maximum feedrates of the different axis X,Y,Z
   * @details Set the maximum feedrates of the different axis in m/s
   *
   * @param rates The feedrate for each of the axis, consisting of a NUM_AXES length array.
   */
    void setMaxSpeeds(VectorN speeds);

    /**
   * @brief Set the number of steps required to move each axis by 1 meter
   * @details Set the number of steps required to move each axis by 1 meter
   *
   * @param stepPerM the number of steps required to move each axis by 1 meter, consisting of a NUM_AXES length array.
   */
    void setAxisStepsPerMeter(VectorN stepPerM);

    /**
   * @brief Set the max acceleration for all moves
   * @details Set the max acceleration for moves when the extruder is activated
   *
   * @param accel The acceleration in m/s^2
   */
    void setAcceleration(VectorN accel);

    /**
   * @brief Set the maximum speed that can be used when in a corner
   * @details The speed jump determines your start speed and the maximum speed at the join of two segments.
   *
   * Its unit is m/s.
   *
   * If the printer is standing still, the start speed is speed jump/2. At the join of two segments, the speed
   * difference is limited to the speed jump value.
   *
   * @param speedJumps the maximum speed jump for each axis in m/s^2
   */
    void setMaxSpeedJumps(VectorN speedJumps);

    void suspend()
    {
        pru.suspend();
    }

    void resume()
    {
        pru.resume();
    }

    void setSoftEndstopsMin(VectorN stops);
    void setSoftEndstopsMax(VectorN stops);
    void setStopPrintOnSoftEndstopHit(bool stop);
    void setStopPrintOnPhysicalEndstopHit(bool stop);
    void setBedCompensationMatrix(std::vector<double> matrix);
    void setAxisConfig(int axis);
    void setState(VectorN set);
    void enableSlaves(bool enable);
    void addSlave(int master_in, int slave_in);
    void setBacklashCompensation(VectorN set);
    void resetBacklash();

    VectorN getState();
    bool getLastQueueMoveStatus();

    double getLastProbeDistance();

    void reset();

    virtual ~PathPlanner();
};

class InputSizeError
{
};

#endif /* defined(__PathPlanner__PathPlanner__) */
