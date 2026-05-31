#pragma once

#include <iostream>

#include "spot_micro_state.h"
#include "command.h"
#include "rate_limited_first_order_filter.h"

#include "spot_micro_kinematics/spot_micro_kinematics.h"

class SpotMicroTransitionStandState : public SpotMicroState {
 public:
  SpotMicroTransitionStandState(); // Constructor
  ~SpotMicroTransitionStandState(); // Destructor

  virtual void handleInputCommands(const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc,
                                   smk::BodyState* body_state_cmd);

  virtual void init(const smk::BodyState& body_state,
                    const SpotMicroNodeConfig& smnc,
                    const Command& cmd,
                    SpotMicroMotionCmd* smmc); 

  // Returns current state name as a string
  virtual std::string getCurrentStateName() {
    return "Transit Stand";
  }
 private:
  enum StandTransitionPhase {
    FRONT_LEGS,
    REAR_LEGS,
    BODY_RISE
  };

  void setFrontLegFilterCommands(const smk::LegsFootPos& feet_pos,
                                 BodyStateFilters* body_state_filters);

  void setRearLegFilterCommands(const smk::LegsFootPos& feet_pos,
                                BodyStateFilters* body_state_filters);

  void setBodyFilterCommands(const smk::BodyState& body_state,
                             BodyStateFilters* body_state_filters);

  bool checkFrontLegsEquality(const smk::BodyState& body_state1,
                              const smk::BodyState& body_state2,
                              float tol);

  bool checkRearLegsEquality(const smk::BodyState& body_state1,
                               const smk::BodyState& body_state2,
                               float tol);

  smk::BodyState start_body_state_;
  smk::BodyState end_body_state_;
  RateLmtdFirstOrderFilter rlfof; 
  BodyStateFilters body_state_filters_;
  bool sequential_stand_;
  StandTransitionPhase phase_;
};
