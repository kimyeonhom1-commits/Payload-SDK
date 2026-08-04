# Manifold 3 자원 제한형 LiDAR 품질 파이프라인

## 구현 위치

- `samples/sample_c++/module_sample/perception/lidar_quality_pipeline.hpp`
- `samples/sample_c++/module_sample/perception/lidar_quality_pipeline.cpp`
- DJI 콜백 연결: `samples/sample_c++/module_sample/perception/test_lidar_entry.cpp`

기존 Manifold 3 CMake가 `module_sample/perception/*.cpp`를 자동 포함하므로 별도 라이브러리
설치 없이 함께 빌드된다. 구현은 C++11 표준 라이브러리만 사용하며 PCL, Eigen, Open3D를
필수 의존성으로 추가하지 않는다.

## 3단계 처리

### 1차: 밀도 계산

- 20 cm 기본 voxel
- voxel별 점 개수, 위치·공분산·intensity 누적
- DJI noise/not-return 라벨 제외
- 최대 200,000점을 균일 간격으로 선택
- 활성 voxel 최대 50,000개
- 25프레임 동안 관측되지 않은 voxel 제거
- 5프레임마다 누적 통계를 절반으로 감쇠

### 2차: 슈퍼복셀 구조 진단

- 세밀 voxel을 60 cm seed cell 단위로 병합
- 3×3 공분산의 주축을 8회 power iteration으로 근사
- 외부 선형대수 라이브러리 없이 선형성 계산
- 최소 12점, 선형성 0.55 이상을 목부 구조 후보로 유지
- 슈퍼복셀 최대 5,000개

현재 구현은 자원 제한을 우선한 고정 seed-cell 슈퍼복셀이다. PCL VCCS와 같은 반복적
경계 최적화는 수행하지 않는다. 실시간 성능이 확인된 뒤 관심영역에만 정밀 슈퍼복셀을
추가할 수 있다.

### 3차: 그래프 연결성

- 같은 seed cell의 26개 인접 cell만 탐색
- 거리 1.2 m 이하, 주축 정렬도 0.55 이상만 연결
- 전체 간선 최대 30,000개
- union-find로 연결요소 계산
- 최대 연결요소 비율과 고립 노드 수 출력

현재 단계는 연결성 품질의 안전한 최소 구현이다. articulation point, bridge, rooted-tree와
gap-edge는 실제 L3 데이터에서 노드 안정성을 확인한 뒤 추가한다.

## 자원 보호

- LiDAR 콜백 큐는 최대 2프레임이다.
- 분석이 밀리면 가장 오래된 프레임을 버리고 최신 프레임을 유지한다.
- 큐에서 버린 프레임 수를 `backpressure_drops`로 출력한다.
- 프레임별 PCD 쓰기는 기본적으로 비활성화된다.
- 입력점, voxel, 슈퍼복셀, 간선에 각각 고정 상한이 있다.
- 상한 도달 여부는 JSON의 `capped` 필드에 기록된다.
- 분석 프레임을 버려도 PSDK 수신 스레드가 무한 대기하거나 큐가 무한 증가하지 않는다.

## 출력 예시

```text
LIDAR_QUALITY {"frame":10,"input_points":120000,"sampled_points":119800,
"active_voxels":18320,"dense_voxel_ratio":0.7210,"supervoxels":2100,
"wood_candidates":340,"edges":520,"components":28,"isolated_nodes":11,
"largest_component_ratio":0.6147,"capped":{"input":false,"voxel":false,
"supervoxel":false,"edge":false}} backpressure_drops=0
```

해석:

- `dense_voxel_ratio`: 1차 밀도 상태
- `wood_candidates`: 2차 선형 목부 후보 슈퍼복셀 수
- `largest_component_ratio`: 3차에서 가장 큰 목부 연결 구조의 비율
- `isolated_nodes`: 연결되지 않은 목부 후보
- `backpressure_drops`: 분석기가 실시간 입력을 따라가지 못한 횟수
- `capped`: 설정한 자원 상한에 도달했는지 여부

## RGB 정합 오버레이

`lidar_rgb_overlay.*`와 `lidar_rgb_overlay_bridge.*`는 저밀도 voxel 중심을 RGB packed 또는
RGBA 프레임에 투영해 빨간 반투명 원으로 표시한다. 정합 모듈은 다음 조건을 모두 확인한다.

- LiDAR–RGB 외부표정 `R`, `t`
- RGB 내부 파라미터 `fx, fy, cx, cy`
- 동일한 프레임 크기와 RGB 채널 수
- 콜백 도착 monotonic timestamp 차이 100 ms 이하
- 카메라 뒤쪽과 화면 밖 점 제거

캘리브레이션 파일 형식은 다음과 같다. 마지막 `maxTimestampSkewNs`는 선택 항목이다.

```text
image_width image_height fx fy cx cy
R00 R01 R02 R10 R11 R12 R20 R21 R22
tx ty tz
maxTimestampSkewNs
```

실행 전 환경변수를 지정한다.

```bash
export DJI_L3_RGB_CALIBRATION=/path/to/l3_rgb_calibration.txt
```

LiDAR 품질 샘플은 저밀도 영역을 bridge에 발행하고, RGB liveview 콜백은 최근 결과를 복사한
프레임에만 overlay를 적용한다. 캘리브레이션이 없거나 timestamp가 어긋나면 원본 RGB 프레임을
그대로 전송한다. 따라서 잘못된 위치에 경고가 그려지는 것보다 overlay가 생략되는 쪽을
선택한다.

현재 PSDK liveview callback은 RGB packed 버퍼와 frame ID를 제공하므로, bridge는 센서 timestamp
대신 두 콜백의 host monotonic timestamp를 사용한다. 실제 장비에서 허용 시간창과 callback
지연을 측정한 뒤 100 ms 값을 조정한다.

## 수목 ROI와 건물·지면 제거

`tree_roi_filter.*`는 RGB 세그멘테이션 mask를 LiDAR 점에 투영해 수목 점만 통과시키는
필터다. mask label은 다음 네 가지를 사용한다.

```text
0 unknown
1 tree
2 building
3 ground
```

처리 순서는 다음과 같다.

```text
RGB segmentation mask
        ↓ LiDAR–RGB 정합
LiDAR 점별 label 조회
        ├─ building → 제거
        ├─ ground   → 제거
        ├─ tree     → 수목 그래프 입력
        └─ unknown  → 설정에 따라 보존 또는 제거
```

세그멘테이션 모델은 파이프라인과 분리돼 있다. Manifold에서는 경량 tree/building 모델을
사용하거나, 외부에서 생성한 동일 해상도 mask를 공급할 수 있다. 현재 filter는 모델을
가정하지 않으므로 RGB 모델 교체가 LiDAR 그래프 코드에 영향을 주지 않는다.

지면은 별도 지면 필터(CSF 또는 동등한 저해상도 지면 분류)로 만든 `ground` mask를 함께
공급한다. 지면 점은 수목 그래프에서 제외하지만 높이 정규화와 DTM 계산용으로는 별도로
보존한다. RGB mask가 없거나 정합이 실패하면 tree-only 필터를 적용하지 않고 기존 점군
품질 분석으로 fallback한다.

## 빌드

Manifold 프로젝트에서 기존 방식으로 빌드한다.

```bash
cd /path/to/Payload-SDK
mkdir -p build
cd build
cmake .. -DUSE_SYSTEM_ARCH=LINUX
make -j2
```

`-j2`는 Manifold에서 빌드 중 메모리 급증을 피하기 위한 보수적 값이다. 기존 빌드 디렉터리가
있다면 새로 만들기 전에 실행 중인 빌드가 없는지 확인한다.

## 실행

기존 C++ Manifold 샘플을 실행하고 메뉴의 LiDAR data sample 항목을 선택한다. 현재 샘플은
10초 동안만 구독한 뒤 자동 해제한다. 실제 L3 스트림 시작은 기체·Pilot·L3 상태를 확인한
시험 절차에서만 수행한다.

## PCD 기록을 다시 켜는 방법

디버깅 목적으로만 컴파일 정의를 추가한다.

```text
DJI_LIDAR_QUALITY_WRITE_PCD=1
```

장시간 시험에서는 저장공간과 I/O 부하 때문에 기본값 `0`을 유지한다.

## 초기 튜닝 순서

1. `backpressure_drops`가 증가하면 입력점 상한을 먼저 낮춘다.
2. `capped.voxel`이 반복되면 voxel 크기를 20 cm에서 25~30 cm로 키운다.
3. `capped.supervoxel`이 반복되면 seed 크기를 키우거나 수목 ROI를 제한한다.
4. 목부 후보가 지나치게 많으면 `minLinearity`와 최소 점 수를 높인다.
5. 연결요소가 지나치게 분리되면 거리·축 정렬 조건을 실제 점 간격에 맞춰 조정한다.
6. 실시간 안정화 전에는 해상도를 낮추지 말고 상한과 ROI부터 조정한다.

## 남은 장비 검증

- Manifold 3 aarch64 컴파일
- M400·L3 조합의 `DjiPerception_SubscribeLidarData` 콜백 지원
- 프레임당 실제 점 수와 콜백 주기
- 좌표계와 pose 적용 여부
- intensity 분포 및 label 의미 확인
- 10초 시험에서 backpressure와 각 `capped` 상태 확인
