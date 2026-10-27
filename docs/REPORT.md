# Bài tập 6 — Giao tiếp phi ngôn ngữ và điều khiển TurtleBot 4 bằng cử chỉ tay

**Sinh viên:** Tô Văn Chúc · **MSSV:** 23020727 · **Lớp:** chờ bổ sung.

## Mục tiêu và kiến trúc

Webcam laptop → OpenCV → MediaPipe Gesture Recognizer → bộ lọc theo thời gian
→ `/detected_gesture` (std_msgs/String) → robot_behavior_node → `/cmd_vel`
(geometry_msgs/Twist) → driver Create 3 → TurtleBot 4 trong Gazebo.
`/odom`, `/scan`, `/dock_status`, `/hazard_detection` cung cấp phản hồi và điều kiện
cho phép chạy. `/behavior_state` trả trạng thái robot về cửa sổ webcam.

Hai node độc lập cho phép tách xử lý AI khỏi vòng điều khiển 20 Hz. Capture và
inference chạy trong hai thread, chỉ xử lý frame mới nhất; inference VIDEO mode
dùng timestamp tăng đơn điệu. Mỗi frame được mirror, đổi BGR → RGB rồi đưa vào
MediaPipe Tasks. Một bàn tay hợp lệ cung cấp 21 landmarks và danh mục dự đoán.
Hai bàn tay hợp lệ hoặc không có tay đầy đủ vô hiệu điều khiển. Nhãn/confidence và landmarks
được vẽ trực tiếp trên ảnh webcam thật.

## Nhận diện và ánh xạ

| Cử chỉ | Nhãn | Tác dụng |
| --- | --- | --- |
| Vẫy tay | Wave | Chọn chào +45°, −90°, +45° |
| Một ngón trỏ | Pointing_Up | Tiến 0.50 m rồi lùi về điểm đầu, lặp 4 chu kỳ |
| Hai ngón chữ V | Victory | Chọn một vòng tròn r=0.75 m |
| Ngón cái lên | Thumb_Up | Xác nhận lựa chọn |
| Xòe tay tĩnh | Open_Palm | Dừng ngay |
| Nắm tay | Closed_Fist | Hủy và về chờ |

Năm tư thế tĩnh được nhận bằng nhãn AI kết hợp luật góc khớp/duỗi ngón trên 21
landmarks AI. Luật bổ sung khi model trả None hoặc nhãn sai; nhãn mâu thuẫn với
các ngón rõ ràng không phát lệnh. Confidence hiển thị là xác suất model thô;
không gán xác suất AI giả cho kết quả hình học. Wave không phải class có sẵn:
thuật toán riêng theo dõi cổ tay x, yêu cầu biên độ ≥0.12 chiều rộng ảnh, ≥2 lần
đảo chiều trong cửa sổ 2.4 s, tối thiểu 6 mẫu và tư thế xòe tay được xác nhận từ
điểm khớp hoặc nhãn AI ≥0.50 không mâu thuẫn với các ngón. Khi đã nhận ra
Wave, giữ nhãn cho tới khi đổi tư thế/mất tay. Khi robot RUNNING, tắt Wave để
Open_Palm luôn ưu tiên dừng. Ngưỡng Open_Palm là 0.50; các lệnh khác là 0.65.
Giữ bàn tay xòe sau khi đã chọn chào không xóa lựa chọn khi robot đứng yên;
ngón cái vẫn cần để thực hiện. Xòe tay khi đang xoay chào dừng ngay.

## Chọn, xác nhận và điều khiển

IDLE → SELECTED → Thumb_Up → RUNNING → COMPLETED. Wave, Pointing_Up và Victory
chỉ chọn, không chuyển động. Cần nhãn ổn định ít nhất 0.30 s và ít nhất 3 message
để chọn/xác nhận. Thumb_Up giữ nguyên không lặp hành vi. Sau dừng/hủy/lỗi/hoàn tất,
phải chọn và xác nhận lại. Lựa chọn hết hạn sau 12 s. Khi đang SELECTED và đứng
yên, cho phép khoảng chuyển tay tối đa 1.8 s; nắm tay cần giữ 0.25 s để hủy lựa
chọn. Khi RUNNING, mất tín hiệu quá 0.65 s vẫn dừng và nắm tay vẫn hủy ngay.
Sau lỗi có thể chọn lại cùng cử chỉ mà không cần đổi sang cử chỉ khác trước.
Chọn hành vi khác khi đang RUNNING sẽ dừng chương trình cũ, về SELECTED và cần
Thumb_Up mới để thực hiện lựa chọn mới. Giữ cùng cử chỉ không khởi động lại.

Điều khiển dùng odometry thật: hai đích tiến–lùi cố định so với tư thế lúc xác
nhận là +0.50 m và 0 m, lặp 4 chu kỳ, tổng 8 đoạn. Robot trở về gần điểm đầu sau
mỗi chu kỳ, không cộng dồn dịch chuyển. Tốc độ tiến/lùi tối đa 0.28 m/s, xoay
chào tối đa 1.0 rad/s. Đi tròn dùng v=0.26 m/s, r=0.75 m, ω≈0.347 rad/s, tích
lũy yaw qua ±π cho tới gần 2π và kiểm tra cả quãng đường.
Chào gồm ba đoạn quay tương đối +π/4, −π/2, +π/4. Không dùng thời gian trôi qua
đơn thuần để kết luận robot đã hoàn thành chuyển động.

Open_Palm gửi Twist 0 ngay trong callback. Closed_Fist xóa cả lựa chọn. Mất nhãn
hợp lệ quá 0.65 s, cảm biến cũ quá 0.8 s, gặp vật cản, đang dock hoặc hazard
nguy hiểm đều khiến robot dừng và không tự resume. Kết quả inference chỉ dùng
khi tuổi frame ≤0.30 s; ngân sách dừng khi mất ảnh gồm độ mới frame, heartbeat,
một tick 0.05 s và thời gian hãm cơ khí/mô phỏng. Không cam kết robot dừng vật lý
tức thời trong đúng một timestamp. Driver có timeout riêng 0.5 s khi mất cmd_vel.

## Môi trường và kiểm chứng

Arena 8×8 m có tường bao; robot ở vùng tâm. Chuẩn bị scene dời dock sang (3,3)
trước demo, không dời robot và không gửi vận tốc. Không chạy Nav2 hoặc teleop.
LiDAR kiểm tra phía tiến/lùi 0.42 m, khoảng trống xoay 0.26 m. prepare_arena xác
minh driver dùng simulation time, rồi đặt safety_override=backup_only để robot
Gazebo lùi đủ 0.50 m. Không dùng full; các hazard khác và LiDAR phía sau vẫn
được kiểm tra. Việc này chỉ cấu hình mô phỏng bài 6.

Kết quả cụ thể được lấy từ artifacts/unit_tests.txt và artifacts/simulation_test.json
khi tạo PDF. Kiểm thử tự động gửi nhãn String có kiểm soát, đo odometry và trạng
thái thật trong Gazebo; không phải video webcam người thật.
Các log/quỹ đạo Gazebo ghi rõ tốc độ thực tế trong active_controller_parameters.
Kiểm tra đủ 8 đoạn tiến–lùi, góc quay vòng tròn và sai lệch vị trí đầu/cuối.
Overlay riêng của bài 6 bỏ sensor RGB-D mô phỏng không dùng để giảm tải render;
webcam laptop thật và cảm biến điều khiển vẫn hoạt động. Webcam và model đã
mở/chạy inference; chưa có bằng chứng demo đủ sáu cử chỉ bằng người thật nếu
chưa quay video. Chỉ điền kết quả nhận diện thực nghiệm khi có ảnh/video thật.

## Nộp bài và tài liệu

Các lệnh cài đặt/build/chạy nằm trong README.md; kịch bản quay ở DEMO_CHECKLIST.md.
PDF được tạo bằng `python3 tools/build_report.py`. Sau khi có URL, truyền
`--github https://github.com/... --video https://drive.google.com/...` để xuất lại.

**GitHub Public:** chờ bổ sung URL thật. **Video Drive Public:** chờ bổ sung URL thật.

- MediaPipe: https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/python
- OpenCV: https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html
- TurtleBot 4: https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html
- Create 3 safety: https://iroboteducation.github.io/create3_docs/api/safety/
