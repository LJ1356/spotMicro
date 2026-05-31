#include "spot_micro_transition_stand.h"

#include <cmath>

#include "spot_micro_stand.h"
#include "spot_micro_motion_cmd.h"
#include "spot_micro_state.h"

SpotMicroTransitionStandState::SpotMicroTransitionStandState() {
  sequential_stand_ = true;
  phase_ = FRONT_LEGS;
}

SpotMicroTransitionStandState::~SpotMicroTransitionStandState() {
}

void SpotMicroTransitionStandState::setFrontLegFilterCommands(
    const smk::LegsFootPos& feet_pos,
    BodyStateFilters* body_state_filters) {
  body_state_filters->leg_right_front.x.setCommand(feet_pos.right_front.x);
  body_state_filters->leg_right_front.y.setCommand(feet_pos.right_front.y);
  body_state_filters->leg_right_front.z.setCommand(feet_pos.right_front.z);

  body_state_filters->leg_left_front.x.setCommand(feet_pos.left_front.x);
  body_state_filters->leg_left_front.y.setCommand(feet_pos.left_front.y);
  body_state_filters->leg_left_front.z.setCommand(feet_pos.left_front.z);
}

void SpotMicroTransitionStandState::setRearLegFilterCommands(
    const smk::LegsFootPos& feet_pos,
    BodyStateFilters* body_state_filters) {
  body_state_filters->leg_right_back.x.setCommand(feet_pos.right_back.x);
  body_state_filters->leg_right_back.y.setCommand(feet_pos.right_back.y);
  body_state_filters->leg_right_back.z.setCommand(feet_pos.right_back.z);

  body_state_filters->leg_left_back.x.setCommand(feet_pos.left_back.x);
  body_state_filters->leg_left_back.y.setCommand(feet_pos.left_back.y);
  body_state_filters->leg_left_back.z.setCommand(feet_pos.left_back.z);
}

void SpotMicroTransitionStandState::setBodyFilterCommands(
    const smk::BodyState& body_state,
    BodyStateFilters* body_state_filters) {
  body_state_filters->body_pos.x.setCommand(body_state.xyz_pos.x);
  body_state_filters->body_pos.y.setCommand(body_state.xyz_pos.y);
  body_state_filters->body_pos.z.setCommand(body_state.xyz_pos.z);

  body_state_filters->body_angs.x.setCommand(body_state.euler_angs.phi);
  body_state_filters->body_angs.y.setCommand(body_state.euler_angs.theta);
  body_state_filters->body_angs.z.setCommand(body_state.euler_angs.psi);
}

bool SpotMicroTransitionStandState::checkRearLegsEquality(
    const smk::BodyState& body_state1,
    const smk::BodyState& body_state2,
    float tol) {
  if (std::fabs(body_state1.leg_feet_pos.right_back.x -
                body_state2.leg_feet_pos.right_back.x) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.right_back.y -
                body_state2.leg_feet_pos.right_back.y) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.right_back.z -
                body_state2.leg_feet_pos.right_back.z) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.left_back.x -
                body_state2.leg_feet_pos.left_back.x) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.left_back.y -
                body_state2.leg_feet_pos.left_back.y) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.left_back.z -
                body_state2.leg_feet_pos.left_back.z) > tol) {
    return false;
  }
  return true;
}

bool SpotMicroTransitionStandState::checkFrontLegsEquality(
    const smk::BodyState& body_state1,
    const smk::BodyState& body_state2,
    float tol) {
  if (std::fabs(body_state1.leg_feet_pos.right_front.x -
                body_state2.leg_feet_pos.right_front.x) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.right_front.y -
                body_state2.leg_feet_pos.right_front.y) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.right_front.z -
                body_state2.leg_feet_pos.right_front.z) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.left_front.x -
                body_state2.leg_feet_pos.left_front.x) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.left_front.y -
                body_state2.leg_feet_pos.left_front.y) > tol) {
    return false;
  }
  if (std::fabs(body_state1.leg_feet_pos.left_front.z -
                body_state2.leg_feet_pos.left_front.z) > tol) {
    return false;
  }
  return true;
}

void SpotMicroTransitionStandState::init(const smk::BodyState& body_state,
                                         const SpotMicroNodeConfig& smnc,
                                         const Command& cmd,
                                         SpotMicroMotionCmd* smmc) {
  start_body_state_ = body_state;
  sequential_stand_ = smnc.sequential_stand;
  phase_ = FRONT_LEGS;

  end_body_state_.leg_feet_pos = smmc->getNeutralStance();
  end_body_state_.euler_angs.phi = 0.0f;
  end_body_state_.euler_angs.theta = 0.0f;
  end_body_state_.euler_angs.psi = 0.0f;
  end_body_state_.xyz_pos.x = 0.0f;
  end_body_state_.xyz_pos.y = smnc.default_stand_height;
  end_body_state_.xyz_pos.z = 0.0f;

  float dt = smnc.dt;
  float tau = smnc.transit_tau;
  float rl = smnc.transit_rl;
  float rl_ang = smnc.transit_angle_rl;

  initBodyStateFilters(dt, tau, rl, rl_ang,
                       body_state, &body_state_filters_);

  if (sequential_stand_) {
    // Phase 1: only front legs move to stand foot positions; body and rear legs stay put
    setFrontLegFilterCommands(end_body_state_.leg_feet_pos, &body_state_filters_);
  } else {
    setBodyStateFilterCommands(end_body_state_, &body_state_filters_);
  }
}

void SpotMicroTransitionStandState::handleInputCommands(
                                   const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc,
                                   smk:: BodyState* body_state_cmd) {
  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Transition Stand State";
    if (sequential_stand_) {
      const char* phase_name = "front legs";
      if (phase_ == REAR_LEGS) {
        phase_name = "rear legs";
      } else if (phase_ == BODY_RISE) {
        phase_name = "body rise";
      }
      std::cout << " (phase " << phase_name << ")";
    }
    std::cout << std::endl;
  }

  if (sequential_stand_) {
    if (phase_ == BODY_RISE &&
        checkBodyStateEquality(body_state, end_body_state_, 0.001f)) {
      changeState(smmc, std::make_unique<SpotMicroStandState>());
      return;
    }

    if (phase_ == FRONT_LEGS &&
        checkFrontLegsEquality(body_state, end_body_state_, 0.001f)) {
      phase_ = REAR_LEGS;
      setRearLegFilterCommands(end_body_state_.leg_feet_pos, &body_state_filters_);
    } else if (phase_ == REAR_LEGS &&
               checkRearLegsEquality(body_state, end_body_state_, 0.001f)) {
      phase_ = BODY_RISE;
      setBodyFilterCommands(end_body_state_, &body_state_filters_);
    }
  } else if (checkBodyStateEquality(body_state, end_body_state_, 0.001f)) {
    changeState(smmc, std::make_unique<SpotMicroStandState>());
    return;
  }

  runFilters(&body_state_filters_);
  assignFilterValuesToBodyState(body_state_filters_, body_state_cmd);
  smmc->setServoCommandMessageData();
  smmc->publishServoProportionalCommand();
}
