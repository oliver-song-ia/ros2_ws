# Behavior Tree Visual Architecture - Simplified

## Simplified Mission Behavior Tree

```
┌─────────────────────────────────────────────────────────────┐
│                    MISSION SEQUENCE                          │
│                  (Sequence Node - Loops)                     │
└────────────────────────────┬────────────────────────────────┘
                             │
        ┌────────────────────┼────────────────────────┐
        │                    │                        │
        ▼                    ▼                        ▼
┌──────────────┐    ┌────────────────┐     ┌──────────────────┐
│ Wait for     │    │ Wait for       │     │  MANIPULATION    │
│ Goal Pose    │ ──▶│ Navigation     │ ──▶ │  PHASE           │
│              │    │ Complete       │     │  (Sequence)      │
│ (Condition)  │    │ (Condition)    │     └────────┬─────────┘
└──────────────┘    └────────────────┘              │
                                          ┌─────────┼─────────┐
                                          │         │         │
                                          ▼         ▼         ▼
                                    ┌─────────┬──────────┬─────────┐
                                    │ Start   │ Wait for │ Stop    │
                                    │ Manip   │ Complete │ Manip   │
                                    └─────────┴──────────┴─────────┘
```

## Data Flow

```
┌─────────────────┐
│   /goal_pose    │ (External)
│   (Topic)       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐         ┌──────────────────┐
│  BT: Goal       │         │  Nav2: Navigate  │
│  Received       │ ──────▶ │  (Automatic)     │
└─────────────────┘         └────────┬─────────┘
         │                           │
         │                           ▼
         │                  ┌──────────────────┐
         │                  │ /navigate_to_pose│
         │                  │ /_action/status  │
         │                  └────────┬─────────┘
         │                           │
         ▼                           ▼
┌─────────────────────────────────────────┐
│       BLACKBOARD (Shared Data)          │
│  • goal_received: Bool                  │
│  • nav_status: GoalStatus               │
│  • manipulation_active: Bool            │
│  • ik_process: Process                  │
│  • mocap_process: Process               │
└─────────────────────────────────────────┘
```

## Execution Timeline Example

```
Time  │ State                │ What's Happening
──────┼──────────────────────┼─────────────────────────────────
  0s  │ IDLE                 │ Waiting for goal_pose
      │                      │
  5s  │ GOAL_RECEIVED        │ /goal_pose published
      │                      │ → BT: goal_received = True
      │                      │ → Nav2: Starts navigation automatically
      │                      │
 10s  │ NAVIGATION           │ Nav2 navigating
      │                      │ BT monitoring status topic
      │                      │
 45s  │ NAVIGATION           │ Still navigating...
      │                      │ Distance: 2.3m remaining
      │                      │
 52s  │ NAVIGATION SUCCESS ✓ │ Nav2 reached goal
      │                      │ BT detects STATUS_SUCCEEDED
      │                      │
 53s  │ MANIPULATION         │ BT starts manipulation
      │                      │ → Launch IK controller (PID: 12345)
      │                      │ → Launch mocap publisher (PID: 12346)
      │                      │
 60s  │ MANIPULATION         │ Processes running...
      │                      │ (30s elapsed)
      │                      │
113s  │ MANIPULATION DONE ✓  │ Timeout reached (60s)
      │                      │ or Process terminated
      │                      │
114s  │ CLEANUP              │ Stopping processes
      │                      │ → Terminate IK controller
      │                      │ → Terminate mocap publisher
      │                      │
115s  │ IDLE                 │ Ready for next goal
      │                      │ Tree resets, loop continues
```


## State Transitions

```
       ┌──────────────┐
       │     IDLE     │◀──────────┐
       └──────┬───────┘           │
              │                   │
      Goal pose received          │
              │                   │
              ▼                   │
       ┌──────────────┐           │
       │ GOAL_RECEIVED│           │
       └──────┬───────┘           │
              │                   │
    Nav2 starts automatically     │
              │                   │
              ▼                   │
       ┌──────────────┐           │
       │  NAVIGATION  │           │
       └──────┬───────┘           │
              │                   │
     Nav2 completes (SUCCESS)     │
              │                   │
              ▼                   │
       ┌──────────────┐           │
       │ MANIPULATION │           │
       └──────┬───────┘           │
              │                   │
      Timeout or process exit     │
              │                   │
              ▼                   │
          Cleanup ─────────────────┘
```


## Node Communication Diagram

```
┌──────────────┐         ┌──────────────┐
│   External   │         │     Nav2     │
│   Systems    │         │              │
└──────┬───────┘         └──────┬───────┘
       │                        │
       │ /goal_pose             │ /navigate_to_pose
       │                        │ /_action/status
       ▼                        ▼
┌─────────────────────────────────────┐
│     Mission BT Node                 │
│                                     │
│  Subscriptions:                     │
│   • /goal_pose                      │
│   • /navigate_to_pose/_action/status│
│                                     │
│  Publications:                      │
│   • /mission_status                 │
│                                     │
│  Manages:                           │
│   • IK controller process           │
│   • Mocap publisher process         │
└─────────────────────────────────────┘
```

## Legend

```
┌────────────────────────────────────────────────┐
│ NODE TYPE          │ BEHAVIOR                  │
├────────────────────┼───────────────────────────┤
│ Sequence           │ Execute children in order │
│ Condition          │ Check something (RUNNING/ │
│                    │ SUCCESS/FAILURE)          │
│ Action             │ Do something              │
└────────────────────────────────────────────────┘
```

---

**Architecture Version**: 0.2.0 (Simplified)  
**Last Updated**: October 2025
