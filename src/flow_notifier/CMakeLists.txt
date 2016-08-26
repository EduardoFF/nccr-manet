
set(flow_notifier_HEADERS
  flow_notifier.h
)

set(flow_notifier_IMPLEMENTATIONS
  flow_notifier.cc
)


add_library(flow_notifier ${flow_notifier_IMPLEMENTATIONS})

target_link_libraries(flow_notifier)


