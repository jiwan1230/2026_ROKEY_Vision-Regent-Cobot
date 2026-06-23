# 전체 동작 순서도

```
Start[Start System] --> Init[Initialize Camera, ROS2 Nodes, Robot]
Init --> Capture[Capture Side-view Image]
Capture --> Infer[Infer Liquid Height (YOLOv8n)]
Infer --> Publish[Publish Height/State Data]
Publish --> Decision[main_decision_node: safety gate + dispatch]

Decision --> HasState2{Any State 2?}
HasState2 -- Yes --> Dispose[Pick Tube(s) and Move to Waste Zone]
Dispose --> Recheck1[Request Recheck]

HasState2 -- No --> HasState0{Any State 0?}
HasState0 -- Yes --> Refill[Move to Tube(s) and Refill Reagent]
Refill --> Recheck2[Request Recheck]

HasState0 -- No --> AllNormal{All State 1?}
AllNormal -- Yes --> MoveTray[Move Tray to Normal Zone]
AllNormal -- No --> Idle[No action this cycle]

Recheck1 --> Capture
Recheck2 --> Capture
MoveTray --> End[Task Complete]
Idle --> Capture
```

## 구현 매핑

- `Capture` ~ `Publish`: `side_camera_node` → `liquid_height_detector_node` → `tube_state_publisher_node` (계속 반복 실행되는 파이프라인, 정지 없음).
- `Decision`: `main_decision_node.on_tube_state()`. 카메라 상태/손 감지/UNKNOWN 여부를 먼저 확인하고, 통과하면 `/robot/start_task`를 호출한다.
- `HasState2? / HasState0? / AllNormal?`과 그에 따른 한 가지 작업 수행: `robot_task_manager_node.handle_start_task()` 내부의 우선순위 분기 (`dispose_idxs` > `refill_idxs` > `all_normal`).
- `Recheck1 / Recheck2 --> Capture`: 명세서 원본에서는 recheck가 캡처 단계로 되돌아가는 루프로 그려져 있다. 본 구현에서는 이를 "하나의 블로킹 호출 내부 루프"로 만들지 않고, **각 사이클마다 최고 우선순위 작업 1건만 처리한 뒤 종료**하고, 그 다음 자연스럽게 들어오는 새로운 `/vision/tube_state`(카메라 파이프라인은 멈추지 않으므로 계속 갱신됨)가 다음 사이클의 `Decision`을 다시 트리거하는 방식으로 구현했다. 결과적으로 동일한 순서도를 여러 ROS2 사이클에 걸쳐 재현한다.

## Scenario 매핑

| Scenario | 입력 예시 | 결과 |
|---|---|---|
| A: 일부 State 0 | `[0, 1, 1]` | `refill_idxs=[0]` → tube 0 보충 → recheck → (정상이면) 다음 사이클에서 트레이 이송 |
| B: 일부 State 2 | `[1, 2, 1]` | `dispose_idxs=[1]` → tube 1 폐기 → recheck |
| C: 전체 State 1 | `[1, 1, 1]` | `all_normal=True` → 트레이 전체 정상 구역 이송 |
| D: State 0 + State 2 동시 | `[0, 2, 1]` | 1순위인 `dispose_idxs=[1]` 먼저 처리 → recheck → 다음 사이클에서 `refill_idxs=[0]` 처리 → recheck → 모두 정상이면 트레이 이송 |

Scenario D처럼 여러 상태가 동시에 존재해도, `robot_task_manager_node`가 매 호출마다 최우선순위 1건만 처리하므로 "폐기 > 보충 > 정상 이송" 순서가 항상 보장된다.
