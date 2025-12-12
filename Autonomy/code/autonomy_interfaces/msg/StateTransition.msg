# State Transition Event Message
# Records state transitions for logging and debugging

std_msgs/Header header

# Transition details
string from_state                # State transitioning from
string to_state                  # State transitioning to
builtin_interfaces/Time start_time    # When transition started
builtin_interfaces/Time end_time      # When transition completed
float64 transition_duration      # Duration in seconds

# Transition metadata
bool success                     # True if transition completed successfully
string reason                    # Reason for transition
string initiated_by              # Source of transition request (frontend, safety, auto)
string failure_reason            # If failed, why it failed

# Context information
string[] preconditions_checked   # List of preconditions that were validated
string[] entry_actions_executed  # Actions executed on entering new state
string[] exit_actions_executed   # Actions executed on leaving old state
