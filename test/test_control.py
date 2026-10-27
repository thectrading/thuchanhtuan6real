import math
import pytest
from turtlebot4_gesture_hri.control import Controller, Settings
from turtlebot4_gesture_hri.wave import WaveDetector


def sensors(c, now, pose=(0, 0, 0)):
    c.on_pose(*pose, now)
    c.on_scan(4., 4., 4., now)
    c.last_dock = c.last_hazard = now
    c.docked = c.hazard = False


def hold(c, gesture, start, duration=.4):
    for i in range(int(duration / .05) + 1):
        now = start + i * .05
        sensors(c, now)
        c.on_gesture(gesture, now)
        c.tick(now)
    return start + duration


def running():
    c = Controller()
    hold(c, 'Pointing_Up', 0)
    hold(c, 'Thumb_Up', .5)
    assert c.mode == 'RUNNING'
    return c


def test_confirmation_and_noise_gate():
    c = Controller()
    hold(c, 'Thumb_Up', 0)
    assert c.mode == 'IDLE'
    c.on_gesture('Victory', .45)
    c.on_gesture('Thumb_Up', .50)
    assert c.mode != 'RUNNING'
    hold(c, 'Pointing_Up', 1)
    assert c.mode == 'SELECTED'
    assert c.tick(1.4) == (0, 0)


@pytest.mark.parametrize('gesture,mode,selection', [('Open_Palm', 'STOPPED', 'shuttle'),
                                                  ('Closed_Fist', 'IDLE', None)])
def test_stop_and_cancel(gesture, mode, selection):
    c = running()
    c.on_gesture(gesture, 1)
    assert c.tick(1) == (0, 0)
    assert c.mode == mode and c.selected == selection and not c.armed


def test_heartbeat_loss_does_not_auto_resume():
    c = running()
    sensors(c, 1.7)
    assert c.tick(1.7) == (0, 0)
    assert c.mode == 'FAULT'
    hold(c, 'Thumb_Up', 1.8)
    assert c.mode != 'RUNNING'


def test_reselect_same_gesture_after_loss_before_confirmation():
    c = Controller(); hold(c, 'Pointing_Up', 0)
    sensors(c, 2.3); c.on_gesture('No_Hand', 2.3); c.tick(2.3)
    assert c.mode == 'FAULT'
    hold(c, 'Pointing_Up', 2.4)
    assert c.mode == 'SELECTED' and c.armed
    assert c.tick(2.8) == (0, 0)
    hold(c, 'Thumb_Up', 2.9)
    assert c.mode == 'RUNNING'


def test_stationary_handover_gap_then_confirmation():
    c = Controller(); hold(c, 'Victory', 0)
    for now in (.7, 1.0, 1.3):
        sensors(c, now);c.on_gesture('No_Hand', now)
        assert c.tick(now) == (0, 0) and c.mode == 'SELECTED'
    hold(c, 'Thumb_Up', 1.4)
    assert c.mode == 'RUNNING'


def test_brief_transition_fist_does_not_clear_stationary_choice():
    c = Controller();hold(c, 'Pointing_Up', 0)
    c.on_gesture('Closed_Fist', .5);c.on_gesture('Closed_Fist', .6)
    assert c.mode == 'SELECTED' and c.tick(.6) == (0, 0)
    hold(c, 'Thumb_Up', .65)
    assert c.mode == 'RUNNING'


def test_deliberate_fist_cancels_stationary_choice():
    c = Controller();hold(c, 'Victory', 0);hold(c, 'Closed_Fist', .5)
    assert c.mode == 'IDLE' and c.selected is None
    hold(c, 'Thumb_Up', 1)
    assert c.mode == 'IDLE'


@pytest.mark.parametrize('first,new,selection', [('Pointing_Up','Victory','circle'),
                                               ('Victory','Pointing_Up','shuttle')])
def test_change_running_behavior_stops_and_requires_fresh_confirmation(first,new,selection):
    c = Controller();hold(c, first, 0);hold(c, 'Thumb_Up', .5)
    assert c.mode == 'RUNNING'
    hold(c, new, 1)
    assert c.mode == 'SELECTED' and c.selected == selection and c.armed
    assert c.tick(1.4) == (0, 0) and not c.program
    hold(c, new, 1.5)
    assert c.mode == 'SELECTED'
    hold(c, 'Thumb_Up', 2)
    assert c.mode == 'RUNNING' and c.selected == selection


def test_repeated_current_choice_does_not_restart_running_program():
    c = running();program=c.program;started=c.started
    hold(c, 'Pointing_Up', 1)
    assert c.mode == 'RUNNING' and c.program is program and c.started == started


def test_static_open_hand_after_wave_preserves_greeting_until_confirmation():
    c=Controller();hold(c,'Wave',0);hold(c,'Open_Palm',.5,duration=1.)
    assert c.mode=='SELECTED' and c.selected=='greet' and c.armed
    assert c.tick(1.5)==(0,0)
    hold(c,'Thumb_Up',1.6)
    assert c.mode=='RUNNING' and c.selected=='greet'
    c.on_gesture('Open_Palm',2.1)
    assert c.tick(2.1)==(0,0) and c.mode=='STOPPED' and not c.armed


def test_low_confidence_and_invalid_commands_do_not_refresh_signal():
    c = running()
    c.on_gesture('Unknown', 1.3)
    c.on_gesture('garbage', 1.4)
    sensors(c, 1.7)
    assert c.tick(1.7) == (0, 0) and c.mode == 'FAULT'


@pytest.mark.parametrize('kind', ['odom', 'scan', 'dock', 'hazard'])
def test_stale_sensor_stops(kind):
    c = running()
    sensors(c, 2)
    c.on_gesture('Thumb_Up', 2)
    setattr(c, 'last_' + kind, 0)
    assert c.tick(2) == (0, 0) and c.mode == 'FAULT'


def test_obstacle_and_dock_interlock():
    c = running(); sensors(c, 1)
    c.on_scan(.20, 4., .20, 1)
    assert c.tick(1) == (0, 0) and c.mode == 'FAULT'
    c = Controller(); c.docked = True
    # A valid-looking confirmation with no sensors cannot start motion.
    for gesture, start in [('Victory', 0), ('Thumb_Up', .5)]:
        for i in range(10):c.on_gesture(gesture, start + i * .05)
    assert c.mode == 'FAULT' and not c.program


def test_selection_expiry_and_held_confirmation():
    c = Controller(); hold(c, 'Pointing_Up', 0)
    sensors(c, 13);c.on_gesture('Hand_Present', 13);c.tick(13)
    assert not c.armed
    c = running();c.mode = 'COMPLETED';c.program = []
    hold(c, 'Thumb_Up', 1)
    assert c.mode == 'COMPLETED'


@pytest.mark.parametrize('gesture,segments', [('Wave', 3), ('Pointing_Up', 8), ('Victory', 1)])
def test_entire_behavior_against_unicycle_plant(gesture, segments):
    c = Controller();hold(c, gesture, 0);hold(c, 'Thumb_Up', .5)
    x = y = yaw = 0.;now = 1.;dt = .02;signs = set()
    for _ in range(12000):
        sensors(c, now, (x, y, wrap(yaw)))
        c.on_gesture('Thumb_Up', now)
        v, w = c.tick(now)
        signs.add(1 if v > .001 else -1 if v < -.001 else 0)
        x += v * math.cos(yaw) * dt;y += v * math.sin(yaw) * dt;yaw += w * dt
        now += dt
        if c.mode != 'RUNNING':break
    assert c.mode == 'COMPLETED', c.reason
    assert sum('Finished motion segment' in e['reason'] for e in c.events) == segments
    assert c.tick(now) == (0, 0)
    if gesture == 'Pointing_Up':
        assert {-1, 1} <= signs and math.hypot(x, y) < .025
        assert c.status(now)['shuttle_cycles_completed'] == 4
    if gesture == 'Victory':
        assert yaw > 6.1 and math.hypot(x, y) < .06


def wrap(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def test_stationary_robot_cannot_complete_from_wall_time():
    c = running()
    for i in range(1, 150):
        now = 1 + i * .1;sensors(c, now);c.on_gesture('Thumb_Up', now);c.tick(now)
        if c.mode == 'FAULT':break
    assert c.mode == 'FAULT' and 'progress' in c.reason


def test_wave_is_temporal_not_an_open_palm_or_jitter():
    wave = WaveDetector()
    assert not any(wave.update(i*.04, .5 + .005*math.sin(i), .5, 'Open_Palm', True)
                   for i in range(60))
    wave.reset()
    assert any(wave.update(i*.04, .5 + .12*math.sin(2*math.pi*2*i*.04), .5,
                           'Open_Palm', True) for i in range(60))
    assert not wave.update(3, .5, .5, 'Open_Palm', True, enabled=False)


def test_wave_selection_survives_static_palm_until_pose_changes():
    wave = WaveDetector()
    detected = False
    for i in range(60):
        detected |= wave.update(i*.04, .5 + .12*math.sin(2*math.pi*2*i*.04),
                                .5, 'Open_Palm', True)
    assert detected
    # A stationary palm after waving must not erase the greeting selection
    # before the user can switch directly to the confirmation gesture.
    for i in range(60, 150):
        assert wave.update(i*.04, .5, .5, 'Open_Palm', True)
    assert not wave.update(6.0, .5, .5, 'Thumb_Up', True)
    assert not wave.update(6.04, .5, .5, 'Open_Palm', True)


def test_wave_with_slow_inference_and_one_uncertain_frame():
    wave=WaveDetector();detected=False
    for i in range(15):
        now=i*.2;label='None' if i==4 else 'Open_Palm'
        detected |= wave.update(now,.5+.12*math.sin(2*math.pi*.8*now),.5,label,label=='Open_Palm')
    assert detected
    assert not wave.update(3.1,.5,.5,'Open_Palm',True,enabled=False)


def test_wave_short_occlusion_history_cannot_emit_a_control_signal():
    wave=WaveDetector();detected=False
    for i in range(15):
        now=i*.15
        if i==4:
            assert wave.missing(now) is None
            continue
        detected |= wave.update(now,.5+.12*math.sin(2*math.pi*.9*now),.5,'Open_Palm',True)
    assert detected
    wave.missing(5)
    assert not wave.update(5.1,.5,.5,'Open_Palm',True)


@pytest.mark.parametrize('heading', [0., math.pi/2, -.9])
def test_four_shuttle_cycles_return_to_same_endpoints(heading):
    c=Controller();x0,y0=1.,-1.;x,y,yaw=x0,y0,heading
    for gesture,start in [('Pointing_Up',0),('Thumb_Up',.5)]:
        for i in range(9):
            now=start+i*.05;sensors(c,now,(x,y,yaw));c.on_gesture(gesture,now);c.tick(now)
    now=1.;directions=[];last=0;returns=[];previous_segment=0
    for _ in range(10000):
        sensors(c,now,(x,y,wrap(yaw)));c.on_gesture('Thumb_Up',now);v,w=c.tick(now)
        sign=1 if v>.001 else -1 if v<-.001 else 0
        if sign and sign!=last:directions.append(sign);last=sign
        if c.segment!=previous_segment:
            if c.segment%2==0:returns.append(math.hypot(x-x0,y-y0))
            previous_segment=c.segment
        x+=v*math.cos(yaw)*.02;y+=v*math.sin(yaw)*.02;yaw+=w*.02;now+=.02
        if c.mode!='RUNNING':break
    assert c.mode=='COMPLETED',c.reason
    assert directions == [1,-1]*4
    assert len(returns)==4 and max(returns)<.025


@pytest.mark.parametrize('values', [{'linear_speed': 1.}, {'circle_radius': .01},
                                   {'signal_timeout': float('nan')}, {'stable_samples': 1}])
def test_bad_config(values):
    with pytest.raises(ValueError):Controller(Settings(**values))
