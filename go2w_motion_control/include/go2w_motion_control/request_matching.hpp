#pragma once

#include <cstdint>

namespace go2w_motion_control {

inline bool ResponseMatches(int64_t expected_request_id,
                            int64_t expected_api_id,
                            int64_t response_request_id,
                            int64_t response_api_id) {
  return expected_request_id == response_request_id &&
         expected_api_id == response_api_id;
}

}  // namespace go2w_motion_control
