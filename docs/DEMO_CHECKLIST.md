# Kịch bản video Bài 6 — Tô Văn Chúc, 23020727

Video phải ghi webcam người thật và Gazebo đồng thời. Log kiểm thử bằng String
trong artifacts chỉ chứng minh điều khiển, không thay thế video nhận diện cử chỉ.

1. Khởi chạy arena, chạy `prepare_arena`, rồi mở `gesture.launch.py` theo README.
   Chỉ chạy một robot_behavior_node và một gesture_recognition_node.
2. Hiển thị tên/MSSV, Gazebo với robot, cửa sổ webcam có nhãn AI, confidence,
   nhãn publish và trạng thái robot. Đưa một bàn tay vào khung hình, đủ sáng.
3. Giơ Pointing Up khoảng 0.5 s: robot SELECTED và vẫn đứng yên. Chuyển trực tiếp
   sang Thumb Up khoảng 0.5 s: RUNNING. Giữ Thumb Up trong tầm webcam suốt hành vi.
   Quay đủ **4 chu kỳ tiến 0.50 m rồi lùi về điểm đầu** (8 phân đoạn), quan sát
   LẦN 1/4…4/4 trên cửa sổ, chờ COMPLETED. Giữ Thumb Up thêm
   vài giây để chứng minh không tự chạy lại.
4. Giơ Victory rồi Thumb Up: robot đi tròn đủ một vòng đường kính 1.50 m. Duy trì tay trong khung.
5. Xòe bàn tay và vẫy trái–phải ít nhất 2 nhịp; đợi published=Wave, selected=greet.
   Chuyển trực tiếp sang Thumb Up: robot xoay trái–phải chào. Chào cũng cần xác nhận.
6. Chọn Pointing Up rồi Thumb Up; khi robot đang chạy, xòe Open Palm: STOPPED,
   vận tốc 0. Giữ Thumb Up ngay sau đó không được tự chạy tiếp.
7. Chọn và xác nhận lại; nắm Closed Fist: IDLE và xóa lựa chọn. Có thể nắm tay ngay
   ở trạng thái SELECTED để minh họa hủy lựa chọn trước khi chạy.
8. Chọn và xác nhận lại; cất tay khỏi webcam: FAULT do mất tín hiệu, robot dừng.
   Đưa lại Thumb Up không được tự tiếp tục; phải chọn và xác nhận lại.
9. Kết thúc bằng Closed Fist. Không dùng phím teleop hoặc gửi cmd_vel thủ công
   trong toàn bộ video demo. Nếu chọn nhầm, nắm tay và thao tác lại.

Sau khi quay: upload video lên Drive, chọn **Anyone with the link / Viewer**.
Đăng source package lên GitHub Public. Điền hai URL thật vào báo cáo rồi xuất PDF
lại. Lớp chưa được cung cấp. Không dùng video Nav2/YOLO của bài trước cho bài này.
