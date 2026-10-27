import math
from types import SimpleNamespace as P
import pytest

from turtlebot4_gesture_hri.pose_geometry import analyze, resolve
from turtlebot4_gesture_hri.wave import WaveDetector


def hand(extended=(), thumb_up=False):
    points=[P(x=.5,y=.8,z=0.) for _ in range(21)]
    points[1]=P(x=.43,y=.66,z=0.)
    points[2]=P(x=.38,y=.59,z=0.)
    points[3]=P(x=.39,y=.61,z=0.)
    points[4]=P(x=.43,y=.63,z=0.)
    if thumb_up:
        points[2]=P(x=.35,y=.56,z=0.);points[3]=P(x=.35,y=.40,z=0.);points[4]=P(x=.35,y=.25,z=0.)
    for finger,base,x in zip(range(4),(5,9,13,17),(.42,.48,.54,.60)):
        ys=(.55,.42,.31,.20) if finger in extended else (.55,.45,.53,.59)
        for i,y in enumerate(ys):points[base+i]=P(x=x,y=y,z=0.)
    return points


@pytest.mark.parametrize('fingers,thumb,label', [((0,),False,'Pointing_Up'),
    ((0,1),False,'Victory'),((),True,'Thumb_Up'),((0,1,2,3),False,'Open_Palm'),
    ((),False,'Closed_Fist')])
@pytest.mark.parametrize('mirror', [False,True])
def test_static_gestures_from_ai_landmarks_even_if_category_is_none(fingers,thumb,label,mirror):
    points=hand(fingers,thumb)
    if mirror:
        for p in points:p.x=1-p.x
    pose=analyze(points)
    assert pose.visible and pose.label==label
    assert resolve(pose,'None',.99)==(label,'AI_landmarks_joint_rules')


def test_false_category_palm_cannot_cancel_a_clear_thumb_up():
    pose=analyze(hand(thumb_up=True))
    assert resolve(pose,'Open_Palm',.95)[0]=='Thumb_Up'


def test_true_palm_stops_even_if_canned_classifier_is_wrong():
    pose=analyze(hand((0,1,2,3)))
    assert resolve(pose,'Thumb_Up',.95)[0]=='Open_Palm'


def test_three_fingers_do_not_become_victory_from_a_conflicting_category():
    pose=analyze(hand((0,1,2)))
    assert pose.label is None
    assert resolve(pose,'Victory',.98)[0]=='Hand_Present'


@pytest.mark.parametrize('problem', ['cropped','tiny','nan','missing'])
def test_bad_or_incomplete_landmarks_cannot_emit_control(problem):
    points=hand((0,))
    if problem=='cropped':points[8].y=-.10
    elif problem=='tiny':
        for p in points:p.x=.5+(p.x-.5)*.05;p.y=.5+(p.y-.5)*.05
    elif problem=='nan':points[0].x=float('nan')
    else:points.pop()
    assert resolve(analyze(points),'Pointing_Up',.99)[0]=='No_Hand'


def test_wave_uses_moving_open_hand_landmarks_when_category_is_none():
    wave=WaveDetector();detected=False
    for i in range(18):
        now=i*.15;points=hand((0,1,2,3));offset=.12*math.sin(2*math.pi*.9*now)
        for p in points:p.x+=offset
        pose=analyze(points);label,_=resolve(pose,'None',.95)
        detected |= wave.update(now,points[0].x,points[0].y,label,label=='Open_Palm')
    assert detected
    assert not wave.update(3,.5,.5,'Open_Palm',True,enabled=False)
