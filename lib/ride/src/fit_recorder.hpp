#pragma once

#include "ride.hpp"

namespace fit_recorder {
enum class Status { Unavailable, Idle, WaitingForTime, Recording, Error };
bool init();
bool start();
bool submit(const ride::RecordingSample& sample);
bool stop(const ride::RideData& totals);
Status status();
}
