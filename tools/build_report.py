#!/usr/bin/env python3
"""Build an honest Vietnamese report from source and measured test artifacts.

Requires python3-reportlab, python3-matplotlib and fonts-dejavu-core.
Figures are engineering diagrams, never fabricated webcam experiment images.
"""
import argparse
import json
from pathlib import Path
from xml.sax.saxutils import escape

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Image, Table,
                               TableStyle, PageBreak)

ROOT = Path(__file__).resolve().parents[1]
IMAGES = ROOT / 'docs' / 'images'
IMAGES.mkdir(parents=True, exist_ok=True)
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})


def diagrams():
    fig, ax = plt.subplots(figsize=(10, 4.5)); ax.set(xlim=(0, 10), ylim=(0, 4.5)); ax.axis('off')
    def box(x, y, w, h, title, body, fill='#e7f0fa'):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=.08',
                                   facecolor=fill, edgecolor='#36566d', linewidth=1.2))
        ax.text(x+w/2, y+h*.68, title, ha='center', va='center', weight='bold', fontsize=10)
        ax.text(x+w/2, y+h*.30, body, ha='center', va='center', fontsize=9)
    def arrow(a, b, text=''):
        ax.annotate('', xy=b, xytext=a, arrowprops={'arrowstyle': '-|>', 'color': '#38536b', 'lw': 1.5})
        if text: ax.text((a[0]+b[0])/2, (a[1]+b[1])/2+.13, text, ha='center', fontsize=9)
    box(.15, 2.8, 2, 1.2, 'Webcam laptop', 'OpenCV / BGR → RGB')
    box(2.8, 2.8, 3, 1.2, 'gesture_recognition_node', 'MediaPipe + Wave + độ mới ảnh')
    box(6.5, 2.8, 3.2, 1.2, 'robot_behavior_node', 'Chọn → xác nhận / 20 Hz')
    box(6.5, .35, 3.2, 1.1, 'TurtleBot 4 / Gazebo', 'Create 3 driver / bánh xe', '#e3f1e9')
    box(.4, .35, 4.8, 1.1, 'Phản hồi và giám sát', '/odom · /scan · dock · hazard', '#fff1db')
    arrow((2.23, 3.4), (2.72, 3.4))
    arrow((5.88, 3.4), (6.42, 3.4), 'String')
    ax.text(6.1, 4.27, '/detected_gesture', ha='center', fontsize=9)
    arrow((8.1, 2.72), (8.1, 1.53), '/cmd_vel')
    arrow((6.42, .9), (5.28, .9))
    arrow((5.2, 1.45), (6.55, 2.7), 'điều kiện chạy')
    arrow((7, 2.7), (4.9, 2.7), '/behavior_state')
    fig.tight_layout(); fig.savefig(IMAGES/'architecture.png', dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 3.4)); ax.set(xlim=(0, 10), ylim=(0, 3.4)); ax.axis('off')
    centers = {'IDLE': (1, 2.4), 'SELECTED': (3.5, 2.4), 'RUNNING': (6.1, 2.4),
               'COMPLETED': (8.7, 2.4), 'STOPPED / FAULT': (5.5, .6)}
    for name, (x, y) in centers.items():
        w = 2 if '/' in name else 1.75
        ax.add_patch(FancyBboxPatch((x-w/2, y-.3), w, .6, boxstyle='round,pad=.06',
                                   fc='#e7f0fa' if y>1 else '#fff1db', ec='#36566d'))
        ax.text(x, y, name, ha='center', va='center', fontsize=10)
    for start, end, label in [('IDLE','SELECTED','chọn ổn định'),
                              ('SELECTED','RUNNING','Thumb_Up'),
                              ('RUNNING','COMPLETED','đủ odometry')]:
        a, b = centers[start], centers[end]
        ax.annotate('', xy=(b[0]-.95,b[1]), xytext=(a[0]+.95,a[1]), arrowprops={'arrowstyle':'->'})
        ax.text((a[0]+b[0])/2, a[1]+.47, label, ha='center', fontsize=9)
    ax.annotate('', xy=(5.5,.96), xytext=(6.1,2.03), arrowprops={'arrowstyle':'->'})
    ax.text(7.35, 1.3, 'Open_Palm / mất tín hiệu\nhoặc sensor / vật cản', ha='center', fontsize=9)
    ax.text(2.2,.65,'Closed_Fist → IDLE\nXóa lựa chọn',ha='center',fontsize=10)
    ax.text(5.2,3.23,'Sau khi dừng / hoàn tất: chọn và xác nhận lại',ha='center',fontsize=10)
    fig.tight_layout();fig.savefig(IMAGES/'state_machine.png',dpi=180);plt.close(fig)


def trajectory(result):
    samples = result.get('trajectory', [])
    if not samples: return None
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.2))
    for ax, stage, title in zip(axes, ['shuttle_4_cycles','one_circle','greeting'],
                               ['Tiến–lùi 4 chu kỳ','Một vòng tròn','Xoay chào']):
        group=[s for s in samples if s['stage']==stage]
        if group:
            points=[s['pose'] for s in group]
            if stage == 'one_circle':
                xs,ys=zip(*[(p[0],p[1]) for p in points])
                ax.set_aspect('equal', adjustable='datalim')
                ax.set(xlabel='odom x (m)',ylabel='odom y (m)')
            else:
                xs=[s['wall']-group[0]['wall'] for s in group]
                index=0 if stage == 'shuttle_4_cycles' else 2
                scale=1 if index == 0 else 180/3.141592653589793
                ys=[(p[index]-points[0][index])*scale for p in points]
                ax.set(xlabel='Thời gian wall (s)',ylabel='Độ dời x (m)' if index == 0 else 'Góc quay tương đối (°)')
            ax.plot(xs,ys,color='#246d95',lw=1.7)
            ax.scatter([xs[0]],[ys[0]],color='green',label='Bắt đầu',s=18)
            ax.scatter([xs[-1]],[ys[-1]],color='red',label='Kết thúc',s=18)
        else:ax.text(.5,.5,'Chưa có dữ liệu',ha='center',transform=ax.transAxes)
        ax.set(title=title);ax.grid(alpha=.25)
    fig.suptitle('ODOMETRY THẬT — đầu vào String tự động, không phải demo webcam',fontsize=10)
    fig.tight_layout();path=IMAGES/'measured_trajectory.png';fig.savefig(path,dpi=180);plt.close(fig)
    return path


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--github',default='');ap.add_argument('--video',default='')
    args=ap.parse_args();diagrams()
    result_path=ROOT/'artifacts/simulation_test.json'
    result=json.loads(result_path.read_text()) if result_path.exists() else {}
    plot=trajectory(result)
    font='/usr/share/fonts/truetype/dejavu/'
    pdfmetrics.registerFont(TTFont('DV',font+'DejaVuSans.ttf'))
    pdfmetrics.registerFont(TTFont('DVB',font+'DejaVuSans-Bold.ttf'))
    pdfmetrics.registerFontFamily('DV',normal='DV',bold='DVB',italic='DV',boldItalic='DVB')
    styles=getSampleStyleSheet()
    styles.add(ParagraphStyle(name='VN',fontName='DV',fontSize=9.3,leading=14,spaceAfter=7))
    styles.add(ParagraphStyle(name='VNTitle',fontName='DVB',fontSize=20,leading=28,spaceAfter=15))
    styles.add(ParagraphStyle(name='VNHead',fontName='DVB',fontSize=13,leading=19,spaceAfter=9))
    styles.add(ParagraphStyle(name='VNCaption',fontName='DV',fontSize=8,leading=12,alignment=TA_CENTER,spaceAfter=10))
    story=[]
    def p(text,style='VN'):story.append(Paragraph(text,styles[style]))
    def h(text):p(text,'VNHead')
    def img(path,w,h):story.append(Image(str(path),width=w*mm,height=h*mm))
    def table(rows,widths):
        data=[[Paragraph(escape(str(cell)),styles['VN']) for cell in row] for row in rows]
        t=Table(data,colWidths=[w*mm for w in widths],hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e7f0fa')),
            ('VALIGN',(0,0),(-1,-1),'TOP'),('GRID',(0,0),(-1,-1),.4,colors.HexColor('#b3c3ce')),
            ('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),
            ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),3)]))
        story.append(t);story.append(Spacer(1,8))

    p('BÀI TẬP 6','VNTitle')
    h('Giao tiếp phi ngôn ngữ và điều khiển TurtleBot 4 bằng cử chỉ tay')
    p('<b>Tô Văn Chúc · MSSV 23020727</b><br/>Lớp: chờ bổ sung<br/>Ubuntu 22.04 · ROS 2 Humble · Gazebo Fortress')
    h('1. Mục tiêu và kiến trúc hệ thống')
    p('Xây dựng hai ROS 2 node để nhận diện tay từ webcam laptop và điều khiển robot trong vùng trống. Người dùng chọn hành vi bằng cử chỉ, xác nhận bằng ngón cái; robot phản hồi trạng thái trên cửa sổ webcam. Bài này sử dụng điều khiển vận tốc và phản hồi cảm biến.')
    img(IMAGES/'architecture.png',175,79)
    p('Hình 1. Luồng ảnh, nhãn cử chỉ, vận tốc và phản hồi của hệ thống.','VNCaption')
    table([['Node','Nhiệm vụ'],['gesture_recognition_node','OpenCV đọc webcam; MediaPipe phân loại; lọc nhãn; hiển thị ảnh và trạng thái.'],
           ['robot_behavior_node','Chọn–xác nhận; phát Twist ở 20 Hz; đo tiến trình bằng odometry; giám sát tín hiệu và cảm biến.']], [66,109])
    p('Source độc lập trong package <b>turtlebot4_gesture_hri</b>. Các file control.py và wave.py tách logic khỏi ROS để kiểm thử; behavior_node.py và recognition_node.py nối logic với topic, camera và giao diện.')
    story.append(PageBreak())

    h('2. Nhận diện AI và ánh xạ cử chỉ')
    p('OpenCV lấy frame mới nhất, mirror ảnh và đổi BGR sang RGB. MediaPipe Tasks chạy VIDEO mode, cung cấp 21 điểm khớp AI và nhãn/confidence. Luật góc khớp, duỗi/gập ngón bổ sung khi model trả None hoặc nhãn mâu thuẫn. Giao diện ghi riêng Model và Detect, giữ xác suất model thô, không gán xác suất AI giả cho luật. Hai bàn tay hợp lệ hoặc tay bị cắt/không thấy đều vô hiệu điều khiển.')
    table([['Cử chỉ / String','Ý nghĩa và chuyển động'],
           ['Vẫy tay / Wave','Chọn chào: quay +45°, −90°, +45°.'],
           ['Một ngón / Pointing_Up','Tiến 0.50 m rồi lùi về điểm đầu, lặp 4 chu kỳ. Tổng 8 đoạn.'],
           ['Hai ngón / Victory','Chọn một vòng tròn r = 0.75 m, đường kính 1.50 m.'],
           ['Ngón cái / Thumb_Up','Xác nhận và bắt đầu lựa chọn hợp lệ.'],
           ['Xòe tay / Open_Palm','Dừng ngay; không tự tiếp tục.'],
           ['Nắm tay / Closed_Fist','Hủy hành vi, xóa lựa chọn, về IDLE.']], [68,107])
    p('<b>Wave dùng thêm thuật toán theo thời gian.</b> Tư thế xòe tay được xác nhận từ khớp AI hoặc nhãn Open_Palm ≥0.50 phù hợp với các ngón. Cổ tay dịch ≥0.12 chiều rộng ảnh, ≥2 lần đảo chiều trong 2.4 s, ≥6 mẫu; chịu được một frame không chắc chắn ngắn. Wave giữ khi cùng tay còn thấy để chuyển sang Thumb_Up xác nhận. Khi RUNNING, Wave tắt để Open_Palm luôn dừng robot.')
    h('3. Máy trạng thái và chống nhiễu')
    img(IMAGES/'state_machine.png',175,60)
    p('Hình 2. Mọi chuyển động, kể cả xoay chào, đều cần bước xác nhận.','VNCaption')
    p('Chọn/xác nhận ổn định ≥0.30 s và ≥3 message; lựa chọn hết hạn 12 s. Khi SELECTED đứng yên, cho phép mất tay 1.8 s; nắm tay giữ 0.25 s để hủy. Khi RUNNING, Open_Palm/Closed_Fist dừng ngay, mất tín hiệu 0.65 s gây dừng. Chọn hành vi khác dừng chương trình cũ, về SELECTED và cần Thumb_Up mới. Giữ cùng cử chỉ không lặp hành vi. Sau lỗi có thể chọn lại cùng cử chỉ.')
    story.append(PageBreak())

    h('4. Topic, điều khiển và cơ chế dừng')
    table([['Topic / kiểu message','Vai trò'],
           ['/detected_gesture / String','Nhãn cử chỉ; Best Effort, Volatile, depth 1; không replay lệnh cũ.'],
           ['/cmd_vel / Twist','linear.x là vận tốc tiến/lùi; angular.z là tốc độ quay.'],
           ['/odom / Odometry','Độ dời và yaw thực tế từ mô phỏng.'],
           ['/scan / LaserScan','Khoảng cách vật cản phía tiến, lùi và quanh robot.'],
           ['/dock_status / DockStatus','Chỉ cho chạy khi robot không dock.'],
           ['/hazard_detection / HazardDetectionVector','Khóa chuyển động khi có va chạm, cliff, stall, wheel drop hoặc proximity.'],
           ['/behavior_state / String JSON','Mode, lựa chọn, phân đoạn, lý do dừng và trạng thái sẵn sàng.'],
           ['/gesture_debug / String JSON','Nhãn AI, confidence và nhãn đã publish.']], [84,91])
    p('<b>Tiến–lùi:</b> hai đích cố định +0.50 m và 0 m theo tư thế lúc xác nhận; lặp 4 chu kỳ, tối đa 0.28 m/s. Mỗi chu kỳ trở về gần điểm đầu; giao diện ghi lượt 1/4…4/4 và TIẾN/LÙI. <b>Đi tròn:</b> r=0.75 m, v=0.26 m/s, ω≈0.347 rad/s; kiểm tra cả góc gần 2π và quãng đường. <b>Chào:</b> quay +π/4, −π/2, +π/4, tối đa 1.0 rad/s. Giảm tốc khi gần cuối đoạn.')
    p('Đủ odometry mới hoàn thành; đứng yên không thể hoàn tất bằng thời gian trôi qua. Không tiến triển 12 s hoặc quá 180 s gây FAULT. prepare_arena xác minh driver Gazebo dùng use_sim_time=true rồi đặt backup_only để lùi đủ 0.50 m; không dùng full, các hazard khác và LiDAR phía sau vẫn hoạt động.')
    p('<b>Dừng khi mất điều khiển:</b> quá 0.65 s không có nhãn hợp lệ, node phát Twist 0. Kết quả AI chỉ hợp lệ khi frame còn mới ≤0.30 s; vòng điều khiển dùng steady clock 20 Hz. Cảm biến cũ quá 0.8 s, obstacle phía tiến/lùi &lt;0.42 m, khoảng trống xoay &lt;0.26 m hoặc hazard nguy hiểm cũng gây dừng. Driver có timeout cmd_vel riêng 0.5 s.')
    p('Các timeout là ngân sách phản ứng phần mềm; dừng vật lý còn phụ thuộc chu kỳ truyền, driver và hãm bánh xe. Không có cơ chế tự resume sau khi đưa tay trở lại. Hai thread capture/inference không được giữ heartbeat sống bằng một frame cũ vô thời hạn.')
    story.append(PageBreak())

    h('5. Môi trường và kết quả kiểm chứng')
    profile=result.get('active_controller_parameters',{})
    p('Arena 8×8 m, tường bao và vùng tâm trống. prepare_arena dời dock đến (3,3), kiểm tra undocked, không dời robot hoặc gửi vận tốc. Overlay bài 6 bỏ RGB-D mô phỏng không dùng; giữ webcam laptop, LiDAR và an toàn driver. Cấu hình đo: tiến/lùi '+str(profile.get('linear_speed','chưa ghi'))+' m/s, quay '+str(profile.get('angular_speed','chưa ghi'))+' rad/s, đi tròn '+str(profile.get('circle_speed','chưa ghi'))+' m/s.')
    arena=IMAGES/'gazebo_arena.png'
    if arena.exists():
        img(arena,75,62.5)
        p('Hình 3. Ảnh chụp Gazebo đang chạy trên máy; dock đã dời khỏi vùng thao tác.','VNCaption')
    unit=(ROOT/'artifacts/unit_tests.txt').read_text().strip().splitlines()[-1]
    p('<b>Kiểm thử logic:</b> '+escape(unit)+'. Kiểm tra chọn–xác nhận, nhiễu nhãn, dừng/hủy, mất tín hiệu/cảm biến, vật cản, toàn bộ các hành vi, unwrap yaw và phát hiện Wave.')
    names={'shuttle_4_cycles':'Tiến–lùi đủ 4 chu kỳ','one_circle':'Đi đủ một vòng tròn',
           'greeting':'Xoay chào đủ 3 đoạn','palm_stop':'Open_Palm dừng','fist_cancel':'Closed_Fist hủy','signal_loss':'Mất tín hiệu dừng'}
    rows=[['Kiểm thử ROS / Gazebo với nhãn String tự động','Kết quả']]
    for key,label in names.items():
        hit=next((t for t in result.get('tests',[]) if t['test']==key),None)
        rows.append([label,'Đạt' if hit and hit.get('passed') else 'Chưa đạt / chưa có kết quả'])
    table(rows,[128,47])
    if plot:
        img(plot,175,56)
        p('Hình 4. Quỹ đạo đo từ /odom của lần kiểm thử được lưu trong artifacts.','VNCaption')
    p('<b>Giới hạn bằng chứng:</b> các chuyển động trên dùng nhãn String tự động để kiểm tra ROS/Gazebo. Webcam và model đã chạy inference trên ảnh thật; chưa có video người thật thực hiện đủ các cử chỉ. Không coi log tự động hoặc sơ đồ là bằng chứng nhận diện tay thành công.')
    story.append(PageBreak())

    h('6. Chạy chương trình và quay video')
    p('Cài đặt và build theo README.md. Tạo venv Python 3.10 có system-site-packages, cài requirements.txt, tải model bằng tools/download_model.py. Mọi terminal phải source ROS, install/setup.bash và scripts/lab_env.bash để dùng chung ROS_DOMAIN_ID và middleware.')
    table([['Bước','Lệnh / thao tác'],
           ['1. Simulation','ros2 launch turtlebot4_gesture_hri simulation.launch.py'],
           ['2. Chuẩn bị scene','ros2 run turtlebot4_gesture_hri prepare_arena'],
           ['3. Hai node + webcam','ros2 launch turtlebot4_gesture_hri gesture.launch.py python_executable:=<venv>/bin/python camera_index:=0'],
           ['4. Quan sát','Webcam hiển thị nhãn/confidence, robot mode; Gazebo hiển thị robot thực hiện hành vi.']], [37,138])
    p('Kịch bản đầy đủ ở docs/DEMO_CHECKLIST.md. Quay đồng thời webcam và Gazebo: chọn không chạy → Thumb_Up mới chạy → 4 chu kỳ tiến/lùi → đi tròn → xoay chào → Open_Palm dừng → Closed_Fist hủy → cất tay gây dừng. Giữ một tay trong webcam khi đang chạy; giữ nguyên Thumb_Up sau hoàn tất để minh họa không lặp hành vi. Không dùng phím điều khiển robot hoặc gửi cmd_vel thủ công trong demo.')
    h('7. Liên kết nộp bài')
    for name,value in [('GitHub Public',args.github),('Video Google Drive Public',args.video)]:
        if value:
            safe=escape(value);p(f'<b>{name}:</b> <link href="{safe}" color="#146c9e">{safe}</link>')
        else:p(f'<b>{name}:</b> CHỜ BỔ SUNG URL THẬT.')
    p('Repository chứa source package, launch, config, world, README và báo cáo. Không đưa venv/build/install/log hoặc model weights lên GitHub. Video cần upload Drive và bật Anyone with the link / Viewer; kiểm tra liên kết bằng cửa sổ riêng tư. Xuất PDF lại sau khi điền các URL thật.')
    h('8. Tài liệu tham khảo chính thức')
    for name,url in [('MediaPipe Gesture Recognizer Python','https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/python'),
                     ('MediaPipe model và danh mục cử chỉ','https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer'),
                     ('OpenCV đọc webcam','https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html'),
                     ('TurtleBot 4 điều khiển vận tốc','https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html'),
                     ('Create 3 safety và backup limit','https://iroboteducation.github.io/create3_docs/api/safety/')]:
        p(f'<link href="{url}" color="#146c9e">{name}</link>')
    output=ROOT/'docs/BAO_CAO_BAI_6_TO_VAN_CHUC_23020727.pdf'
    def page(canvas,doc):
        canvas.setFont('DV',8);canvas.setFillColor(colors.HexColor('#526372'))
        canvas.drawString(18*mm,11*mm,'Tô Văn Chúc · 23020727 | TurtleBot 4 · Gesture HRI')
        canvas.drawRightString(192*mm,11*mm,f'{doc.page}')
    SimpleDocTemplate(str(output),pagesize=A4,leftMargin=18*mm,rightMargin=17*mm,
                      topMargin=17*mm,bottomMargin=18*mm,
                      title='Bài 6 — TurtleBot 4 điều khiển bằng cử chỉ tay',author='Tô Văn Chúc — 23020727').build(story,onFirstPage=page,onLaterPages=page)
    print(output)


if __name__=='__main__':main()
