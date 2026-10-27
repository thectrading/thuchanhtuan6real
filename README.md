# Bài 6 — Điều khiển TurtleBot 4 bằng cử chỉ tay

**Tô Văn Chúc · 23020727** — Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress.
Hai node: `gesture_recognition_node` đọc webcam bằng OpenCV + MediaPipe Tasks;
`robot_behavior_node` nhận String trên `/detected_gesture` và phát Twist `/cmd_vel`.
Không cần Nav2 hoặc bản đồ. Đây là package mới, không thay package Nav2/YOLO bài trước.

## Ánh xạ và thao tác demo

| Cử chỉ | String | Tác dụng |
| --- | --- | --- |
| Vẫy bàn tay trái–phải ít nhất hai nhịp | `Wave` | Chọn chào: xoay +45°, −90°, +45° |
| Một ngón trỏ hướng lên | `Pointing_Up` | Chọn tiến 0.50 m rồi lùi về điểm đầu, lặp **4 chu kỳ** |
| Hai ngón hình chữ V | `Victory` | Chọn đi một vòng tròn, bán kính 0.75 m / đường kính 1.50 m |
| Ngón cái hướng lên | `Thumb_Up` | Xác nhận lựa chọn và chạy |
| Xòe bàn tay tĩnh | `Open_Palm` | Dừng; phải chọn và xác nhận lại để chạy từ đầu |
| Nắm tay | `Closed_Fist` | Hủy, xóa lựa chọn và về IDLE |

**Tất cả hành vi di chuyển, kể cả chào, đều cần Thumb_Up sau khi chọn.** Giữ cử chỉ
chọn/Thumb_Up khoảng 0.4–0.6 giây. Khi đang chạy, duy trì một bàn tay được nhìn thấy,
ví dụ giữ Thumb_Up. Không cất tay khỏi webcam rồi mong robot tiếp tục chạy.
Mất bàn tay, camera hỏng hoặc inference ngừng cập nhật làm hết heartbeat và robot
dừng trong khoảng timeout 0.65 s, cộng tối đa một chu kỳ điều khiển 0.05 s.
Khi có khung hình cũ, node không phát lại nó thành heartbeat hợp lệ vô thời hạn.

MediaPipe cung cấp 21 điểm khớp AI và nhãn phân loại. `pose_geometry.py` kiểm tra
góc khớp và độ duỗi từng ngón để bổ sung nhận diện 5 tư thế khi nhãn AI yếu/None,
đồng thời tránh nhãn Open_Palm sai làm dừng khi các khớp thể hiện rõ ngón cái.
Bàn tay xòe đủ các ngón vẫn phát Open_Palm ngay. Nhãn AI mâu thuẫn với các ngón
thấy rõ không được dùng để phát lệnh. Giao diện giữ nguyên xác suất **model thô**,
hiển thị riêng nhãn Detect; không gán độ tin cậy AI giả cho luật hình học.
`/gesture_debug` có pose_source và finger_states để giải thích quyết định.

**Wave là luật theo thời gian** trên tư thế bàn tay xòe đã kiểm tra và
landmark cổ tay: biên độ x ≥ 0.12 chiều rộng ảnh, ít nhất 2 lần đảo
chiều trong 2.4 s, tối thiểu 6 mẫu. Một frame AI không chắc chắn ngắn không xóa
toàn bộ lịch sử vẫy. Có thể dùng điểm khớp xòe tay dù model trả None; nhãn model
Open_Palm 0.50 chỉ được dùng khi khớp ngón không mâu thuẫn. Không
phát nhãn Wave khi mất tay, chỉ giữ lịch sử tối đa 0.25 s để nối một lần che khuất
ngắn. Không gọi Wave là một class AI dựng sẵn. Khi đang RUNNING, Wave
bị tắt để mọi Open_Palm đều ưu tiên dừng. Sau khi Wave được chọn, giữ lòng bàn tay
hoặc chuyển thẳng sang Thumb_Up; Closed_Fist luôn hủy lựa chọn.
Khi đã SELECTED chào, giữ bàn tay xòe tĩnh không xóa lựa chọn; robot vẫn đứng yên
chờ Thumb_Up. Khi robot RUNNING, Open_Palm vẫn dừng ngay cả trong lúc xoay chào.

## 1. Cài đặt và build

Đặt package vào `<workspace>/src/turtlebot4_gesture_hri`, chạy ở thư mục workspace:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions python3-rosdep python3-venv python3-pip
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/turtlebot4_gesture_hri --ignore-src -r -y
colcon build --symlink-install --packages-select turtlebot4_gesture_hri
source install/setup.bash
/usr/bin/python3 -m venv --system-site-packages ~/venvs/tb4-gesture
~/venvs/tb4-gesture/bin/python -m pip install -r src/turtlebot4_gesture_hri/requirements.txt
python3 src/turtlebot4_gesture_hri/tools/download_model.py
```

Model `.task` tải từ Google, không kèm weights/venv trong GitHub. Cần Python 3.10 của
Ubuntu 22.04 để dùng rclpy apt. Venv system-site-packages sử dụng ROS hiện có.

## 2. Mở simulation và chuẩn bị vùng trống

Máy thực hành này cần Ogre 2 để LiDAR đúng trên Intel GPU; các helper chỉ tạo
overlay tại cache người dùng, không sửa `/opt/ros`. Chạy một lần:

```bash
python3 src/turtlebot4_gesture_hri/scripts/prepare_ogre2_overlay.py
python3 src/turtlebot4_gesture_hri/scripts/prepare_simulation_overlay.py
```

Overlay riêng của bài 6 bỏ sensor RGB-D mô phỏng không sử dụng, giảm tải render
để webcam laptop và Gazebo ít cạnh tranh tài nguyên hơn. Camera laptop thật,
LiDAR, cảm biến cliff, hình học robot và an toàn driver vẫn hoạt động. Chỉ cache
`~/.cache/turtlebot4_gesture_hri/simulation` được tạo; package Nav2/YOLO giữ riêng.

Mỗi terminal dùng cùng môi trường:

```bash
cd /home/chuc/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/turtlebot4_gesture_hri/scripts/lab_env.bash
```

Mở Gazebo với arena 8×8 m, robot chính thức và `/clock`, không chạy Nav2/teleop:

```bash
ros2 launch turtlebot4_gesture_hri simulation.launch.py
```

Terminal khác, **trước lúc demo bằng cử chỉ**:

```bash
ros2 run turtlebot4_gesture_hri prepare_arena
```

Lệnh này dời **dock** đến góc (3,3) bằng service scene Gazebo, kiểm tra is_docked=false.
Sau khi xác minh world Gazebo tồn tại và driver `use_sim_time=true`, nó cấu hình
`/motion_control safety_override=backup_only` để lùi xa hơn giới hạn mặc định
0.15 m. Chế độ này chỉ bỏ giới hạn độ dài lùi trên robot mô phỏng; không dùng
`full`. LiDAR phía sau, stop/watchdog và các hazard khác vẫn được kiểm tra.
Nó không gửi cmd_vel, không dời/reset robot. Robot đứng ở tâm arena và mọi chuyển
động của bài sau đó phải có chọn + xác nhận. Dock được dời để không chắn quỹ đạo tròn
và hướng lùi. Không gọi prepare_arena khi đang thực hiện hành vi.

## 3. Chạy hai node và webcam

```bash
ros2 launch turtlebot4_gesture_hri gesture.launch.py \
  python_executable:="$HOME/venvs/tb4-gesture/bin/python" camera_index:=0
```

Trên workspace này, venv kiểm thử là `.venv-tb4-gesture/bin/python`; truyền đường
dẫn tuyệt đối của nó nếu dùng lại môi trường đã chuẩn bị. `show_window:=false` chỉ
dùng khi kiểm tra không giao diện; demo phải hiển thị webcam và Gazebo.
Đưa **một bàn tay** vào khung hình, đủ sáng, toàn bộ các ngón nhìn thấy được.
Hai bàn tay làm vô hiệu điều khiển cho tới khi chỉ còn một bàn tay.
Không có phím điều khiển robot; đóng cửa sổ nhận diện sẽ dừng pipeline.

## Node, topic và cơ chế điều khiển

| Thành phần/topic | Vai trò |
| --- | --- |
| `gesture_recognition_node` | Capture mới nhất, MediaPipe VIDEO mode, vẽ landmarks/tên/confidence |
| `/detected_gesture` — String | Nhãn chuẩn hóa, Best Effort/Volatile/depth 1, không replay lệnh cũ |
| `robot_behavior_node` | Máy trạng thái chọn–xác nhận, điều khiển 20 Hz bằng đồng hồ wall/steady |
| `/cmd_vel` — Twist | linear.x và angular.z; các thành phần còn lại bằng 0 |
| `/odom` — Odometry | Đo quãng đường/góc thật; không hoàn thành chỉ vì hết thời gian |
| `/scan` — LaserScan | Dừng khi vật cản phía tiến/lùi gần hơn 0.42 m hoặc xoay thiếu khoảng trống |
| `/dock_status`, `/hazard_detection` | Chỉ chạy khi undocked, dữ liệu mới và không BUMP/CLIFF/STALL/WHEEL_DROP/PROXIMITY |
| `/behavior_state` — String JSON | Trạng thái, lựa chọn, phân đoạn, lý do dừng; hiển thị trên webcam |
| `/gesture_debug` — String JSON | Nhãn AI, confidence và nhãn phát; không phải lệnh vận tốc |

IDLE → SELECTED → (Thumb_Up) RUNNING → COMPLETED. Nhiễu nhãn đơn lẻ không chọn/chạy;
cần nhãn ổn định ≥0.30 s và ≥3 message. Giữ Thumb_Up sau khi xong không lặp hành vi.
Open_Palm đưa về STOPPED, Closed_Fist về IDLE, mất tín hiệu/sensor/vật cản về FAULT.
Sau STOPPED/FAULT/COMPLETED phải chọn lại và xác nhận lại; không tự resume.

Trong lúc đang chạy, chọn **hành vi khác** bằng 1/2 ngón ổn định sẽ dừng hành vi
cũ và chuyển về SELECTED cho hành vi mới; phải giơ ngón cái xác nhận lại. Giữ
cử chỉ của chính hành vi hiện tại không khởi động lại hoặc kéo dài chương trình.

Khi đang SELECTED, robot đứng yên và cho phép khoảng chuyển tư thế mất tay tối
đa 1.8 s. Nắm tay trong trạng thái này cần giữ 0.25 s để tránh hủy nhầm lúc chuyển
ngón trỏ sang ngón cái. Khi đang RUNNING, Open_Palm/Closed_Fist vẫn dừng/hủy ngay,
và mất tín hiệu quá 0.65 s vẫn gây dừng. Sau FAULT có thể chọn lại **cùng cử chỉ**;
không còn bị giữ cạnh kích hoạt cũ. Ngón cái đơn lẻ vẫn không tự chạy lại.

Tiến–lùi là 8 phân đoạn đo từ odometry. Chào là 3 góc quay tương đối. Đi tròn dùng
v=0.26 m/s, ω=v/r≈0.347 rad/s, kết thúc sau gần 2π rad **và** đủ quãng đường; unwrap
yaw qua ±π. Khi Gazebo chậm, hành vi vẫn dựa trên odometry. Khi pause, odom/scan
không có timestamp mới sẽ hết timeout và hủy hành vi. Robot driver chính thức có
cmd_vel_timeout 0.5 s. Tiến–lùi dùng hai đích cố định so với tư thế lúc xác nhận:
đích trước cách 0.50 m, đích sau là chính điểm đầu. Mỗi chu kỳ kết thúc gần điểm
đầu, không tích lũy độ dời về trước. Giao diện hiển thị LẦN 1/4…4/4 và TIẾN/LÙI.

Tốc độ hiện tại: tiến/lùi tối đa **0.28 m/s**, xoay chào tối đa **1.0 rad/s**,
đi tròn **0.26 m/s**. Bộ điều khiển giảm tốc khi gần cuối đoạn; góc nhìn Gazebo mặc
định cũng gần robot hơn. Bước chọn/xác nhận, timeout và các lệnh dừng vẫn áp dụng.

## Kiểm tra và nộp bài

Đã kiểm chứng trên máy thực hành: build thành công, **52 kiểm thử logic đạt**;
Gazebo hoàn thành tiến–lùi đủ 4 chu kỳ, một vòng tròn và xoay chào; Open_Palm,
Closed_Fist và mất tín hiệu đều làm dừng đúng trạng thái. Vòng tròn có sai lệch vị
trí đầu/cuối được ghi trong `artifacts/simulation_test.json`. Dữ liệu tự động và ghi chú phương pháp
nằm trong `artifacts/`; webcam thật 640×480 và MediaPipe Tasks cũng đã chạy được.
Các kết quả này chưa thay thế video demo đủ cử chỉ bằng người thật.
Log Gazebo ghi rõ tốc độ thực tế đã dùng trong `active_controller_parameters`;
quãng đường, góc quay và số phân đoạn đều được kiểm tra từ odometry. Không dùng
log String tự động để khẳng định webcam đã demo đủ cử chỉ.

```bash
PYTHONPATH=src/turtlebot4_gesture_hri python3 -m pytest -q src/turtlebot4_gesture_hri/test
ros2 topic echo /detected_gesture
ros2 topic echo /behavior_state
```

Chẩn đoán không gửi lệnh robot, chỉ lưu metadata:

```bash
python3 src/turtlebot4_gesture_hri/tools/audit_live.py --seconds 15 \
  --output /tmp/gesture_audit.json
```

Kết quả có độ trễ frame, số bàn tay, nhãn AI, trạng thái, odometry và real-time
factor của Gazebo. RTF=0.25 nghĩa là 1 giây mô phỏng mất khoảng 4 giây ngoài đời;
đây là lý do có thể thấy robot chậm dù vận tốc cấu hình lớn.

Kiểm thử bằng message giả lập chỉ xác minh logic/ROS/simulation, **không thay video
webcam người thật**. Video cần cho thấy chọn không chạy, Thumb_Up mới chạy, đủ 4
chu kỳ tiến–lùi, đủ một vòng tròn, chào, Open_Palm dừng, Closed_Fist hủy, cất tay/mất
tín hiệu làm dừng và giữ một cử chỉ không kích hoạt lặp. Tài liệu trong `docs/` ghi
rõ phần đã kiểm chứng và phần còn chờ demo bằng người thật.

GitHub Public chứa package này; video upload Drive và bật Anyone with the link.
Họ tên/MSSV đã có; lớp, link GitHub Public và link Drive chờ bổ sung.

Báo cáo PDF: `docs/BAO_CAO_BAI_6_TO_VAN_CHUC_23020727.pdf` (5 trang). Sơ đồ, ảnh
Gazebo và đồ thị đo từ odometry đã kèm theo. Chỉnh nội dung ở `tools/build_report.py`,
hoặc bản Markdown `docs/REPORT.md`. Để xuất lại PDF sau khi có liên kết:

```bash
sudo apt install python3-reportlab python3-matplotlib fonts-dejavu-core
python3 src/turtlebot4_gesture_hri/tools/build_report.py \
  --github 'https://github.com/<tai-khoan>/<repo>' \
  --video 'https://drive.google.com/file/d/<video-id>/view'
```

Kịch bản quay video: `docs/DEMO_CHECKLIST.md`. Chỉ upload video/ảnh webcam khi bạn
đã xem lại nội dung muốn chia sẻ.

## Nguồn chính thức

- [MediaPipe Gesture Recognizer Python](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/python)
- [MediaPipe categories/model](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer)
- [OpenCV webcam](https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html)
- [TurtleBot 4 cmd_vel](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html)
- [Create 3 safety và giới hạn lùi](https://iroboteducation.github.io/create3_docs/api/safety/)
