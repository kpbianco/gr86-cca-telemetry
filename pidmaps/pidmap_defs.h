#pragma once

#include <stddef.h>
#include <stdint.h>

namespace pidmaps {

struct PidRule {
  uint32_t pid;
  uint8_t divider;
};

struct PidMapDefinition {
  const char *name;
  const PidRule *rules;
  size_t ruleCount;
  bool (*isCanIdWhitelisted)(uint32_t id);
  uint8_t (*policyDividerForId)(uint32_t id);
};

}  // namespace pidmaps
