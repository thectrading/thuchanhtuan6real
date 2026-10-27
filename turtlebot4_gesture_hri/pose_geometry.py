"""Validate static gestures using the 21 AI-estimated MediaPipe landmarks.

Joint-angle rules supplement the task model when its canned classifier returns
None/low confidence. Rule results are not assigned a fabricated AI confidence.
"""
from dataclasses import dataclass
import math


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))


def angle(a, b, c):
    u=[x-y for x,y in zip(a,b)];v=[x-y for x,y in zip(c,b)]
    norm=math.sqrt(sum(x*x for x in u)*sum(x*x for x in v))
    if norm<1e-10:return 0.
    cosine=max(-1.,min(1.,sum(x*y for x,y in zip(u,v))/norm))
    return math.degrees(math.acos(cosine))


@dataclass
class HandPose:
    visible: bool
    label: str | None
    fingers: tuple = ()
    thumb_up: bool = False
    thumb_down: bool = False
    reason: str = ''


def analyze(landmarks, aspect=0.75):
    if len(landmarks)!=21:return HandPose(False,None)
    raw=[(float(p.x),float(p.y),float(p.z)) for p in landmarks]
    if not all(math.isfinite(v) for p in raw for v in p):return HandPose(False,None)
    # Occluded folded fingertips can be estimated just beyond an image edge.
    # Use the visible palm/wrist to validate presence instead of rejecting a
    # whole hand because of one extrapolated point.
    if any(not(-.08<=raw[i][0]<=1.08 and -.08<=raw[i][1]<=1.08) for i in (0,5,9,13,17)):
        return HandPose(False,None,reason='palm_or_wrist_outside_frame')
    points=[(x,y*aspect,z) for x,y,z in raw]
    scale=distance(points[0],points[9])
    if (scale<.04 or max(p[0] for p in raw)-min(p[0] for p in raw)<.055
            or max(p[1] for p in raw)-min(p[1] for p in raw)<.09):
        return HandPose(False,None)
    fingers=[]
    for mcp,pip,dip,tip in [(5,6,7,8),(9,10,11,12),(13,14,15,16),(17,18,19,20)]:
        pip_angle=angle(points[mcp],points[pip],points[dip])
        dip_angle=angle(points[pip],points[dip],points[tip])
        reach=distance(points[tip],points[0])/max(1e-8,distance(points[pip],points[0]))
        if pip_angle>=145 and dip_angle>=140 and reach>=1.10:fingers.append('extended')
        elif pip_angle<=115 or reach<1.02:fingers.append('folded')
        else:fingers.append('uncertain')
    for state,tip in zip(fingers,(8,12,16,20)):
        if state=='extended' and not(-.08<=raw[tip][0]<=1.08 and -.08<=raw[tip][1]<=1.08):
            return HandPose(False,None,reason='extended_finger_outside_frame')
    thumb_straight=angle(points[2],points[3],points[4])>=145
    up=points[2][1]-points[4][1];side=abs(points[4][0]-points[2][0])
    thumb_up=(thumb_straight and up>.45*scale and up>.8*side
              and points[4][1]<points[3][1]-.12*scale)
    thumb_down=(thumb_straight and up<-.45*scale and -up>.8*side)
    if thumb_up and not(-.08<=raw[4][0]<=1.08 and -.08<=raw[4][1]<=1.08):
        return HandPose(False,None,reason='thumb_outside_frame')
    label=None
    if fingers==['extended']*4:label='Open_Palm'
    elif fingers==['extended','extended','folded','folded']:label='Victory'
    elif (fingers==['extended','folded','folded','folded']
          and points[8][1]<points[6][1]-.15*scale):label='Pointing_Up'
    elif fingers==['folded']*4:
        if thumb_up:label='Thumb_Up'
        elif not thumb_straight or distance(points[4],points[5])<.8*scale:label='Closed_Fist'
    return HandPose(True,label,tuple(fingers),thumb_up,thumb_down)


def resolve(pose, raw_label, score, confidence=.65, stop_confidence=.50):
    """Return (label, evidence source), keeping stop/cancel rules explicit."""
    if not pose.visible:return 'No_Hand','incomplete_or_small_hand'
    if pose.label:return pose.label,'AI_landmarks_joint_rules'
    states=pose.fingers;extended=states.count('extended')
    compatible={
        'Open_Palm':extended>=3,
        'Pointing_Up':states[0]!='folded' and states[1:].count('extended')==0,
        'Victory':states[0]!='folded' and states[1]!='folded' and states[2:].count('extended')==0,
        'Thumb_Up':extended==0 and not pose.thumb_down,
        'Closed_Fist':extended==0 and not pose.thumb_up,
    }
    limit=stop_confidence if raw_label=='Open_Palm' else confidence
    if compatible.get(raw_label,False) and score>=limit:
        return raw_label,'AI_category_with_landmark_validation'
    return 'Hand_Present','uncertain_pose'
