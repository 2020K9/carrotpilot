"""Driving-only raylib adaptation of GRT47/HKG_5W_TurzxMonitor android-hud.
Source: 1e0538c223dc78e09a5f74a624875d24ba8e1e99, HudRenderer.java
shared camera / drawRoad3d / drawFeatheredPath3d (NOT drawRoad fallback).
User-reported private permission, 2026-11-28; see docs/hud2-source.md.
No public sublicense asserted. Existing K9 outer HUD and alerts remain in owner.
"""
from __future__ import annotations
import math
from contextvars import ContextVar
from dataclasses import dataclass
from itertools import islice

WHITE, MUTED, BLUE, AMBER = (237,242,250), (142,153,168), (28,145,255), (255,180,66)


def finite(value):
    if isinstance(value, (bool, str, bytes)):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


# HudRenderer.stepSimulation shared camera (not Car3dView's local fallback).
# Original864:576 viewport uniformly fitted inside the retained road region.
# Letterbox, never stretch camera x or multiply sprite scale by the wide HUD.
SCENE=(392.,0.,1552.,480.)
VIEW=(612.,0.,1332.,480.)
CENTER_X=(VIEW[0]+VIEW[2])/2
PITCH=math.radians(3.60)
SIN_P,COS_P=math.sin(PITCH),math.cos(PITCH)
CAMERA_D=23.
TARGET_X,TARGET_Y=-.24,3.10
CAMERA_H=TARGET_Y+CAMERA_D*SIN_P
EYE_Z=CAMERA_D*COS_P
FOCAL=240/math.tan(math.radians(10))
KX=FOCAL*(VIEW[2]-VIEW[0])/480./(864./576.)
HORIZON_Y=240-FOCAL*math.tan(PITCH)


@dataclass(frozen=True)
class Camera:
    """General shared look-at basis; source orbit parameters at reset by default."""
    eye: tuple
    right: tuple
    up: tuple
    back: tuple
    focal: float

    def clip(self,x,y,z):
        d=(x-self.eye[0],y-self.eye[1],z-self.eye[2])
        dot=lambda v:sum(a*b for a,b in zip(d,v))
        return dot(self.right),dot(self.up),-dot(self.back)

    def screen(self,p):
        x,y,w=p
        ky=self.focal/w
        # Original projection aspect and matching virtual viewport: kx == ky.
        # drawNav uses that same physical px/metre; no anisotropic stretch.
        kx=ky*(VIEW[2]-VIEW[0])/480./(864./576.)
        return CENTER_X+x*kx,240-y*ky,kx


def camera(curve=0.,yaw=0.,pitch=0.,zoom=1.):
    """Literal reset/orbit/chase formula from HudRenderer.java:561-597."""
    raw=min(1.,math.hypot(yaw/35.,pitch/18.))
    om=raw*raw*(3-2*raw)
    target=(-.24*(1-om),3.10+(.65-3.10)*om,0.)
    bird=max(0.,pitch-18)*.22
    dist=23*((12.5+bird)/23)**om*zoom
    chase=max(-6.5,min(6.5,curve/.001*4.2))
    yr=math.radians(yaw-chase*(1-om*.7))
    pr=math.radians(3.60+pitch)
    back=(math.cos(pr)*math.sin(yr),math.sin(pr),math.cos(pr)*math.cos(yr))
    right=(math.cos(yr),0.,-math.sin(yr))
    up=(-math.sin(pr)*math.sin(yr),math.cos(pr),-math.sin(pr)*math.cos(yr))
    eye=tuple(t+dist*b for t,b in zip(target,back))
    focal=240/math.tan(math.radians(20+(32-20)*om)/2)
    return Camera(eye,right,up,back,focal)


_CAMERA=ContextVar("hud2_camera",default=camera())

def project_world(x,y,z):
    cam=_CAMERA.get();p=cam.clip(x,y,z)
    return None if p[2]<.85 else cam.screen(p)


def project(forward_m,lateral_m):
    f,lat=finite(forward_m),finite(lateral_m)
    if f is None or lat is None or not 0<=f<=140 or abs(lat)>30:
        return None
    return project_world(lat,0.,-f)


EGO_Y=project_world(0.,0.,0.)[1]
EGO_WIDTH=project_world(0.,0.,0.)[2]*1.9


@dataclass
class Motion:
    now: float
    speed: float
    steer: float
    curve: float
    lead: float
    lateral: float
    phase: float=0.


def advance_motion(previous,state,objects,now):
    """Source exponential speed/steer14Hz and curve/lead9Hz interpolation.
    Render-only adapters: repeated same timestamp doesn't advance simulation;
    no stale lead is emitted after detection loss. A long inactive gap reinitializes.
    """
    number=lambda name:finite(getattr(state,name,None)) or 0.
    lead=next((v for v in objects if v[2]),None)
    target=(number("speed_kph"),number("steering_angle_deg"),number("planned_curvature_m_inv"),
            0. if lead is None else lead[0],0. if lead is None else lead[1])
    if previous is None or now<previous.now or now-previous.now>.5:
        return Motion(now,*target)
    elapsed=now-previous.now
    dt=0. if elapsed==0 else min(.1,max(.001,elapsed))
    fast,med=1-math.exp(-dt*14),1-math.exp(-dt*9)
    values=[old+(new-old)*a for old,new,a in zip(
        (previous.speed,previous.steer,previous.curve,previous.lead,previous.lateral),target,(fast,fast,med,med,med))]
    phase=(previous.phase+max(0.,values[0])/3.6*dt)%7000
    return Motion(now,*values,phase)


def effective_curve(motion):
    curve=max(-.0025,min(.0025,motion.curve))
    if abs(curve)<1e-6 and abs(motion.steer)>.05:
        curve=max(-.0025,min(.0025,-motion.steer*.000025))
    return curve


def _world_quad(rl,r,vertices,color,near=.85):
    """HudRenderer.clipAddQuad: homogeneous near clipping before division."""
    cam=_CAMERA.get();points=[cam.clip(*v) for v in vertices];out=[]
    for a,b in zip(points,points[1:]+points[:1]):
        ain,bin=a[2]>=near,b[2]>=near
        if ain:out.append(a)
        if ain!=bin:
            t=(near-a[2])/(b[2]-a[2])
            out.append(tuple(a[i]+(b[i]-a[i])*t for i in range(3)))
    if len(out)<3:return
    pts=[cam.screen(v) for v in out]
    for i in range(1,len(pts)-1):
        r._k9_turzx_quad(*(rl.Vector2(*v[:2]) for v in (pts[0],pts[i],pts[i+1],pts[i+1])),rl.Color(*color))


def _road_band(rl,r,curve,start,end,left,right,color,step):
    """Literal fillRoadBand curved world geometry; decorative ground, not sensing."""
    f=start
    while f<end-.01:
        n=min(end,f+step)
        verts=[]
        for depth,lat in ((f,left),(f,right),(n,right),(n,left)):
            th=curve*depth
            verts.append((.5*curve*depth*depth+lat*math.cos(th),0.,-depth+lat*math.sin(th)))
        _world_quad(rl,r,verts,color)
        f=n

def clip_segment(a, b, rect=None):
    """Liang-Barsky design-space clip, without clamping endpoints to an edge."""
    rect = SCENE if rect is None else rect
    if len(a) != 2 or len(b) != 2 or len(rect) != 4:
        return None
    v = tuple(finite(n) for n in (*a, *b, *rect))
    if any(n is None for n in v):
        return None
    x,y,bx,by,x0,y0,x1,y1 = v
    if x1 < x0 or y1 < y0:
        return None
    dx,dy,lo,hi = bx-x,by-y,0.0,1.0
    for p,q in ((-dx,x-x0),(dx,x1-x),(-dy,y-y0),(dy,y1-y)):
        if p == 0:
            if q < 0:
                return None
            continue
        t = q/p
        if p < 0:
            lo = max(lo,t)
        else:
            hi = min(hi,t)
        if lo > hi:
            return None
    return (x+lo*dx,y+lo*dy),(x+hi*dx,y+hi*dy)


def ground_segments(points, shift=0.0):
    """Break at bad samples/nonmonotonic depth; no interpolated NaN bridges."""
    shift = finite(shift)
    if shift is None:
        return ()
    out,previous = [],None
    for p in islice(points or (),256):
        f,lat = finite(getattr(p,"forward_m",None)),finite(getattr(p,"lateral_m",None))
        current = None if f is None or lat is None else (f,lat+shift)
        if current is None or project(*current) is None:
            previous = None
            continue
        if previous is not None and 0 < current[0]-previous[0] <= 30:
            out.append((previous,current))
        previous = current
    return tuple(out)


def detected_objects(state):
    """Measured coordinates only. L1-priority dedup; no nominal BSD cars.
    Pass current state, not mode-0 held state. Offscreen cars are clipped, never
    moved clear of ego or snapped to lanes. Rear objects are outside this view.
    """
    candidates,kept = [],[]
    for v in islice(getattr(state,"detected_vehicles",()) or (),128):
        f,lat = finite(getattr(v,"longitudinal_m",None)),finite(getattr(v,"lateral_m",None))
        if f is None or f <= 0 or lat is None or project(f,lat) is None:
            continue
        if str(getattr(v,"source","")).lower() == "carstate bsd":
            continue
        prob = finite(getattr(v,"probability",1.0))
        if prob is None or prob <= 0:
            continue
        lead = bool(getattr(v,"primary",False)) and str(getattr(v,"label","")).upper() == "L1"
        absolute=finite(getattr(v,"absolute_speed_kph",None))
        speed=finite(getattr(state,"speed_kph",None))
        if not lead and speed is not None and speed>=20 and absolute is not None and abs(absolute/3.6)<1.5:
            continue
        candidates.append((f,lat,lead))
    for item in sorted(candidates,key=lambda p:(not p[2],p[0])):
        if not any(abs(item[0]-k[0]) < 2 and abs(item[1]-k[1]) < .7 for k in kept):
            kept.append(item)
    return tuple(sorted(kept,reverse=True))


def display_number(value, scale=1.0, maximum=400.0):
    value = finite(value)
    return "--" if value is None or not 0 <= value <= maximum else str(round(value*scale))


def _line(rl,a,b,width,color):
    clipped = clip_segment(a,b)
    if clipped is not None:
        rl.draw_line_ex(rl.Vector2(*clipped[0]),rl.Vector2(*clipped[1]),width,rl.Color(*color))


def _ribbon(rl,r,segments,half_width,color):
    # Project source-decorative behind-ego stations too; measured validation
    # remains in ground_segments(), which rejects negative/invalid input samples.
    for a,b in segments:
        vertices=[(lat+offset,0.,-f) for (f,lat),offset in
                  ((a,-half_width),(a,half_width),(b,half_width),(b,-half_width))]
        _world_quad(rl,r,vertices,color)


def sprite_metadata():
    """Original rear-ground anchors; no centre-bottom approximation."""
    from functools import lru_cache
    return _metadata()


from functools import lru_cache
from pathlib import Path
ASSETS = Path(__file__).resolve().parent / "assets" / "hud2"


@lru_cache(maxsize=1)
def _metadata():
    result = {}
    for line in (ASSETS / "navdy_meta.txt").read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        name,pitch,index,w,h,ax,ay,ppm = line.split()
        if name == "navdy_top":
            continue
        key = name if name == "navdy_ego" else f"{name}_car_{int(index):02d}"
        result[key] = tuple(map(float,(pitch,w,h,ax,ay,ppm)))
    return result


def traffic_pose(f,lat,curve=0.):
    # Original drawRoad3d center placement and far-distance display compression.
    distance=2.2+(f if f<=30 else 30+(f-30)*.7)
    th=curve*distance
    return (.5*curve*distance*distance+lat*math.cos(th),-distance+lat*math.sin(th),th)


def sprite_angles(wx,wz,th):
    eye=_CAMERA.get().eye
    dx,dz=wx-eye[0],wz-eye[2]
    elevation=math.degrees(math.atan2(eye[1],max(.1,math.hypot(dx,dz))))
    yaw=(math.degrees(math.atan2(dx,-dz)-th)+180)%360-180
    return yaw,elevation


def _sprite_for(yaw,pitch):
    index=max(0,min(24,math.floor(yaw/2+.5)+12))
    name=min(((2.5,"p25"),(5.,"p5"),(8.,"p8")),key=lambda p:abs(p[0]-pitch))[1]
    return f"navdy_{name}_car_{index:02d}"


def sprite_key(f,lat):
    return _sprite_for(*sprite_angles(*traffic_pose(f,lat)))


def _shadow(rl,x,y,radius,squash,alpha):
    # Canvas RadialGradient center->transparent mapped to vertex-color fan.
    if radius<=.5:return
    rl.rl_set_texture(rl.rl_get_texture_id_default());rl.rl_begin(rl.RL_TRIANGLES)
    for j in range(32):
        a,b=2*math.pi*j/32,2*math.pi*(j+1)/32
        for px,py,al in ((x,y,round(alpha*255)),(x+radius*math.cos(b),y+radius*squash*math.sin(b),0),
                         (x+radius*math.cos(a),y+radius*squash*math.sin(a),0)):
            rl.rl_color4ub(0,0,0,al);rl.rl_tex_coord2f(0.,0.);rl.rl_vertex2f(px,py)
    rl.rl_end()


def _car(rl,r,f,lat,lead=False,curve=0.,lamp=0):
    wx,wz,th=traffic_pose(f,lat,curve)
    cam=_CAMERA.get();cw=cam.clip(wx,0,wz)[2]
    if cw<1.2:return
    alpha=max(.35,min(1.,1.25-cw/115))
    yaw,pitch=sprite_angles(wx,wz,th)
    if abs(yaw)>25 or pitch>19:
        _faceted_car(rl,r,wx,wz,th,lead,lamp,alpha)
        return
    # Source navdy rear-ground point is2.10m behind the object center.
    point=project_world(wx-2.10*math.sin(th),0.,wz+2.10*math.cos(th))
    if point is None:return
    x,y,scale=point
    key=_sprite_for(yaw,pitch)
    _,w,h,ax,ay,ppm=_metadata()[key]
    tex=r._ego_brake_texture(ASSETS/(key+".png"))
    if tex is None:
        _faceted_car(rl,r,wx,wz,th,lead,lamp,alpha);return
    k=scale/ppm
    dst=rl.Rectangle(x-ax*k,y-ay*k,w*k,h*k)
    if hasattr(r,"_k9l_cars"):
        r._k9l_cars.append((dst.x,dst.y,dst.x+dst.width,dst.y+dst.height))
    tint=tuple(round(c*g) for c,g in zip((214,219,227) if lead else (150,156,166),(1.08,)*3 if lead else (.92,)*3))
    _shadow(rl,dst.x+dst.width/2,dst.y+dst.height*.96,dst.width*.56,.18,.55*alpha)
    rl.draw_texture_pro(tex,rl.Rectangle(0,0,tex.width,tex.height),dst,rl.Vector2(0,0),0.,rl.Color(*tint,round(255*alpha)))
    if lamp>0:
        mask=r._ego_brake_texture(ASSETS/(key+"_lamp.png"))
        if mask is not None:
            # Verified upstream mask is white RGB with alpha: tint implements
            # Android SRC_IN red exactly; additive passes preserve source glow.
            rl.begin_blend_mode(rl.BLEND_ADDITIVE)
            try:
                def paint(rect,a):
                    rl.draw_texture_pro(mask,rl.Rectangle(0,0,mask.width,mask.height),rect,rl.Vector2(0,0),0.,rl.Color(255,38,56,round(255*a)))
                if lamp>=2:
                    g=.06
                    paint(rl.Rectangle(dst.x-dst.width*g,dst.y-dst.height*g,dst.width*(1+2*g),dst.height*(1+2*g)),alpha*.55)
                    paint(dst,alpha);paint(dst,alpha)
                else:paint(dst,alpha*.6)
            finally:rl.end_blend_mode()


def _faceted_car(rl,r,cx,cz,th,lead,lamp,alpha):
    """HudRenderer.drawFacetedCar3d, original body/cabin vertices and face test."""
    co,si=math.cos(th),math.sin(th);eye=_CAMERA.get().eye
    def visible(x,y,z,nx,ny,nz):
        px,pz=cx+x*co-z*si,cz+x*si+z*co
        return (eye[0]-px)*(nx*co-nz*si)+(eye[1]-y)*ny+(eye[2]-pz)*(nx*si+nz*co)>0
    def quad(vertices,color):
        _world_quad(rl,r,[(cx+x*co-z*si,y,cz+x*si+z*co) for x,y,z in vertices],(*color,round(alpha*255)))
    hw,zf,zr=.90,-2.18,2.18
    top=(210,215,226) if lead else (143,151,166)
    side=(156,163,178) if lead else (109,116,130)
    rear=(122,129,144) if lead else (86,92,105)
    glass=(22,27,38)
    _world_quad(rl,r,[(cx+x*co-z*si,.01,cz+x*si+z*co) for x,z in
                     ((-hw*1.12,zf*1.06),(hw*1.12,zf*1.06),(hw*1.12,zr*1.06),(-hw*1.12,zr*1.06))],(0,0,0,round(135*alpha)))
    if visible(0,.54,zf,0,0,-1):
        quad(((-hw,.20,zf),(hw,.20,zf),(hw,.86,zf+.14),(-hw,.86,zf+.14)),side)
        for sign in (-1,1):
            quad(((sign*hw*.88,.54,zf+.02),(sign*hw*.44,.54,zf+.02),(sign*hw*.44,.68,zf+.05),(sign*hw*.88,.68,zf+.05)),(230,241,255) if lamp else (156,163,175))
    if visible(0,.54,zr,0,0,1):
        quad(((-hw,.22,zr),(hw,.22,zr),(hw,.94,zr-.10),(-hw,.94,zr-.10)),rear)
        if lamp:
            for sign in (-1,1):
                quad(((sign*hw*.86,.66,zr-.01),(sign*hw*.36,.66,zr-.01),(sign*hw*.36,.80,zr-.03),(sign*hw*.86,.80,zr-.03)),(255,51,68) if lamp>=2 else (217,43,62))
    for sign in (-1,1):
        if visible(sign*hw,.54,0,sign,0,0):
            quad(((sign*hw,.20,zf),(sign*hw,.22,zr),(sign*hw,.94,zr-.10),(sign*hw,.86,zf+.14)),side)
    quad(((-hw,.86,zf+.14),(hw,.86,zf+.14),(hw,.94,zr-.10),(-hw,.94,zr-.10)),top)
    gw,gzf,gzr,gtop=.72,-.78,1.48,1.42
    if visible(0,1.15,-.98,0,.45,-.89):
        quad(((-hw*.90,.88,-1.16),(hw*.90,.88,-1.16),(gw,gtop,gzf),(-gw,gtop,gzf)),glass)
    if visible(0,1.15,1.72,0,.45,.89):
        quad(((-hw*.90,.94,1.94),(hw*.90,.94,1.94),(gw,gtop,gzr),(-gw,gtop,gzr)),glass)
    for sign in (-1,1):
        if visible(sign*.80,1.15,.35,sign*.92,.38,0):
            quad(((sign*hw*.90,.88,-1.16),(sign*hw*.90,.94,1.94),(sign*gw,gtop,gzr),(sign*gw,gtop,gzf)),glass)
    quad(((-gw,gtop,gzf),(gw,gtop,gzf),(gw,gtop,gzr),(-gw,gtop,gzr)),top)
    if hasattr(r,"_k9l_cars"):
        pts=[project_world(cx+x*co-z*si,y,cz+x*si+z*co) for x in (-hw,hw) for y in (0,gtop) for z in (zf,zr)]
        pts=[v for v in pts if v is not None]
        if pts:r._k9l_cars.append((min(v[0] for v in pts),min(v[1] for v in pts),max(v[0] for v in pts),max(v[1] for v in pts)))

def _ego(rl,r):
    """Existing mode-1 K9 photo/anchor and brake helper, on mode-2 ground.
    Preserve its own colours. Never replace a missing K9 with the imported ego.
    """
    from cluster_renderer import K9_NAVDY_MID, K9_NAVDY_EGO_BRAKE_PATH
    tex = r._k9_navdy_ego_texture()
    sprites = r._k9_navdy_sprites()
    if tex is None or K9_NAVDY_MID not in sprites:
        return
    _,w,h,ax,ay,ppm = sprites[K9_NAVDY_MID]
    x,y,scale = project(0.,0.)
    k = scale/ppm
    dst = rl.Rectangle(x-ax*k,y-ay*k,w*k,h*k)
    # Original projected rear-ground anchor, shadow fully on measured ground.
    radius = max(4.,h*k*.12)
    shadow_y = y-radius-6.
    rl.draw_ellipse(int(x),int(shadow_y),w*k*.60,radius,rl.Color(0,0,0,55))
    rl.draw_ellipse(int(x),int(shadow_y),w*k*.48,max(3.,h*k*.07),rl.Color(0,0,0,105))
    rl.draw_texture_pro(tex,rl.Rectangle(0,0,tex.width,tex.height),dst,rl.Vector2(0,0),0.,rl.Color(255,255,255,255))
    r._draw_ego_brake_light(K9_NAVDY_EGO_BRAKE_PATH,dst)


PATH_LAT_X = (-1.72,-1.46,-1.10,0.,1.10,1.46,1.72)
PATH_LAT_A = (0.,.30,.86,1.,.86,.30,0.)
PATH_LAT_C = (0.,.15,.55,1.,.55,.15,0.)


def path_strips(segments,lead_distance=None):
    """drawFeatheredPath3d alpha profile, bounded by measured data only.
    Source nominal/behind-ego geometry is deliberately not fabricated.
    Per-vertex values are retained (not segment-average fallback colours).
    """
    if not segments:
        return ()
    end=min(segments[-1][1][0],82.)
    if lead_distance is not None:
        lead=finite(lead_distance)
        if lead is not None:
            end=min(end,max(3.5,2.2+(lead if lead<=30 else 30+(lead-30)*.7)-1.6))
    span=max(3.5,min(9.,(end-2)*.32)) if lead_distance is not None else 28.
    fade=max(-2.,end-span)
    def alpha(f):
        if f<=-24 or f>=end:return 0.
        if f<-2:
            u=(f+24)/22
            return u*u*(3-2*u)
        return 1. if f<=fade else max(0.,(end-f)/max(.1,end-fade))**2
    out=[]
    for a,b in segments:
        if a[0]>=end: continue
        stop=min(b[0],end)
        stations=sorted(set((a[0],stop,*[f for f in (fade,(fade+end)*.5) if a[0]<f<stop])))
        for f0,f1 in zip(stations,stations[1:]):
            lat=lambda f:a[1]+(b[1]-a[1])*(f-a[0])/(b[0]-a[0])
            for i in range(6):
                out.append(((f0,lat(f0)+PATH_LAT_X[i]),(f0,lat(f0)+PATH_LAT_X[i+1]),
                            (f1,lat(f1)+PATH_LAT_X[i+1]),(f1,lat(f1)+PATH_LAT_X[i]),
                            (alpha(f0)*PATH_LAT_A[i],alpha(f0)*PATH_LAT_A[i+1],
                             alpha(f1)*PATH_LAT_A[i+1],alpha(f1)*PATH_LAT_A[i]),
                            (PATH_LAT_C[i],PATH_LAT_C[i+1],PATH_LAT_C[i+1],PATH_LAT_C[i])))
    return tuple(out)


def _gradient_quad(rl,points,alphas,cores,edge,core,maximum):
    """Canvas.drawVertices -> rlgl vertex colours, same two-triangle fan."""
    rl.rl_set_texture(rl.rl_get_texture_id_default())
    rl.rl_begin(rl.RL_TRIANGLES)
    for triangle in ((0,1,2),(0,2,3)):
        a,b,c=triangle
        if (points[b][0]-points[a][0])*(points[c][1]-points[a][1])-(points[b][1]-points[a][1])*(points[c][0]-points[a][0])>0:
            b,c=c,b
        for i in (a,b,c):
            col=tuple(round(a+(b-a)*cores[i]) for a,b in zip(edge,core))
            rl.rl_color4ub(*col,round(maximum*alphas[i]))
            rl.rl_tex_coord2f(0.,0.)
            rl.rl_vertex2f(points[i][0],points[i][1])
    rl.rl_end()


def _gradient_world(rl,vertices,alphas,cores,edge,core,maximum):
    """Source emitClippedGradientQuad: interpolate alpha/color in clip space."""
    cam=_CAMERA.get()
    points=[(*cam.clip(*p),a,c) for p,a,c in zip(vertices,alphas,cores)]
    out=[]
    for a,b in zip(points,points[1:]+points[:1]):
        ain,bin=a[2]>=.35,b[2]>=.35
        if ain:out.append(a)
        if ain!=bin:
            t=(.35-a[2])/(b[2]-a[2])
            out.append(tuple(a[i]+(b[i]-a[i])*t for i in range(5)))
    for i in range(1,len(out)-1):
        triangle=(out[0],out[i],out[i+1],out[i+1])
        _gradient_quad(rl,[cam.screen(p[:3]) for p in triangle],[p[3] for p in triangle],
                       [p[4] for p in triangle],edge,core,maximum)


def _distance_bands(segments):
    """Original drawRoad3d bands; split at boundaries, never bridge samples."""
    for a,b in segments:
        for lo,hi,fade in ((-48.,44.,1.),(44.,86.,.52),(86.,128.,.18)):
            f0,f1=max(a[0],lo),min(b[0],hi)
            if f1<=f0: continue
            lat=lambda f:a[1]+(b[1]-a[1])*(f-a[0])/(b[0]-a[0])
            yield ((f0,lat(f0)),(f1,lat(f1))),fade

def lane_segments(lane,phase=0.,curve=None):
    segments=ground_segments(getattr(lane,"model_points",()),getattr(lane,"model_lateral_shift_m",0.))
    if curve is not None:segments=_rear_continuation(segments,curve,-48.)
    if getattr(lane,"style","solid") != "dashed":
        return segments
    # Spatial 3.5 m on/off cadence from upstream; observed style only, no made-up lanes.
    out=[]
    for a,b in segments:
        f=a[0]
        while f<b[0]:
            stop=min(b[0],(math.floor((f+phase)/3.5)+1)*3.5-phase)
            if stop<=f:stop=min(b[0],f+1e-8)
            if int(math.floor((f+phase)/3.5))%2==0:
                lat=lambda d:a[1]+(b[1]-a[1])*(d-a[0])/(b[0]-a[0])
                out.append(((f,lat(f)),(stop,lat(stop))))
            f=stop
    return tuple(out)


def _rear_continuation(segments,curve,start):
    """Original behind-ego display only when a valid observed line reaches0m.
    This is decorative continuation, not additional measured points or lanes.
    Missing/invalid near data never acquires geometry. Forward samples unchanged.
    """
    if not segments or segments[0][0][0]!=0:return segments
    offset=segments[0][0][1]
    stations=tuple(f for f in (-48.,-36.,-26.,-24.,-18.,-15.,-12.,-8.,-7.,-3.,0.) if f>=start)
    points=tuple((f,offset+.5*curve*f*f) for f in stations)
    return tuple(zip(points,points[1:]))+tuple(segments)


def _paint_scene(r,state,motion,curve,objects):
    """Road-only hook. Caller owns scene transform, clip, outer HUD and alerts."""
    import pyray as rl
    x0,y0,x1,y1=VIEW
    # The retained label layout must see this scene, not stale mode-1 car boxes.
    r._k9l_cars=[]
    # Original nominal asphalt is decorative backdrop, NOT an observed edge.
    # Keep sensed lines/path independently gated, but stop floating road thumbnail.
    _road_band(rl,r,curve,-48.,44.,-8.8,8.8,(255,255,255,16),8.)
    _road_band(rl,r,curve,44.,118.,-8.8,8.8,(255,255,255,7),14.)
    lead=next((v for v in objects if v[2]),None)
    path=_rear_continuation(ground_segments(getattr(state,"model_path",())),curve,-24.)
    distance=None if lead is None else motion.lead
    critical=distance is not None and distance<=12
    close=distance is not None and distance<=18
    engaged=getattr(state,"cruise_display_state",None)=="engaged" or getattr(state,"lfa_active",False)
    rgb=(227,25,55) if critical else (245,166,35) if close else (62,106,225) if engaged else (200,205,215)
    core=tuple(round(a+(b-a)*.28) for a,b in zip(rgb,(116,179,255))) if engaged and not close else rgb
    maximum=185 if engaged or close else 58
    for a,b,c,d,alpha,cores in path_strips(path,distance):
        _gradient_world(rl,[(p[1],0.,-p[0]) for p in (a,b,c,d)],alpha,cores,rgb,core,maximum)
    for lane in islice(getattr(state,"lanes",()) or (),12):
        if not getattr(lane,"visible",True): continue
        dashed=getattr(lane,"style","solid")=="dashed"
        for segment,fade in _distance_bands(lane_segments(lane,motion.phase%7,curve)):
            _ribbon(rl,r,(segment,),.06 if dashed else .075,
                    (230,232,236,round(150*fade)) if dashed else (245,245,247,round(232*fade)))
    for name,shift in (("left_road_edge_points","left_road_edge_lateral_shift_m"),
                       ("right_road_edge_points","right_road_edge_lateral_shift_m")):
        for segment,fade in _distance_bands(_rear_continuation(ground_segments(getattr(state,name,()),getattr(state,shift,0.)),curve,-48.)):
            _ribbon(rl,r,(segment,),.06,(160,164,172,round(120*fade)))
    # Draw farthest first by actual clip depth, as source drawRoad3d.
    ordered=sorted(objects,key=lambda v:_CAMERA.get().clip(traffic_pose(v[0],v[1],curve)[0],0.,traffic_pose(v[0],v[1],curve)[1])[2],reverse=True)
    for f,lat,is_lead in ordered:
        observed=next((v for v in islice(getattr(state,"detected_vehicles",()) or (),128)
                       if finite(getattr(v,"longitudinal_m",None))==f and finite(getattr(v,"lateral_m",None))==lat),None)
        acceleration=finite(getattr(observed,"acceleration_mps2",None))
        lamp=2 if acceleration is not None and acceleration<-1.5 else 0
        _car(rl,r,motion.lead if is_lead else f,motion.lateral if is_lead else lat,is_lead,curve,lamp)
    _ego(rl,r)
    # Existing outer top helper keeps its lead/confidence labels; no duplicate tag.
    r._k9l_car_rec=None
    r._k9l_perc_pos=None


def draw_scene(r,state):
    from cluster_renderer import time
    objects=detected_objects(state)
    motion=advance_motion(getattr(r,"_hud2_motion",None),state,objects,time.monotonic())
    r._hud2_motion=motion
    curve=effective_curve(motion)
    # No touch/zoom input exists in this HUD. Source defaults, not invented controls.
    token=_CAMERA.set(camera(curve))
    # Fit source scene without changing outer HUD. Its original rectangular
    # viewport is uniformly scaled/letterboxed, not stretched to fill the HUD.
    restore_clip=None
    if hasattr(r,"width"):
        import pyray as rl
        from cluster_renderer import K9R_SCENE_S,K9R_SCENE_DY
        sx,sy=r.width/1920.,r.height/480.
        scaled=getattr(r,"k9_hud_res",False)
        k,dy=(K9R_SCENE_S,K9R_SCENE_DY) if scaled else (1.,0.)
        vx=SCENE[0]+(VIEW[0]-SCENE[0])*k
        rl.begin_scissor_mode(int(vx*sx),int(dy*sy),int((VIEW[2]-VIEW[0])*k*sx),int(round(480*k*sy)))
        restore_clip=(int(SCENE[0]*sx),int(dy*sy),int((SCENE[2]-SCENE[0])*k*sx),int(round(480*k*sy)))
    try:_paint_scene(r,state,motion,curve,objects)
    finally:
        if restore_clip is not None:rl.begin_scissor_mode(*restore_clip)
        _CAMERA.reset(token)


def draw(renderer,state,signal_lights):
    """Preserve the existing outer HUD literally; replace its road callback only.
    Restore the instance callback even on paint failure. No global monkeypatch,
    settings mutation, alternate telemetry, P/R implementation or Android runtime.
    """
    from cluster_renderer import K9L_ROAD_GEO,K9L_CX
    r=renderer
    r._k9_geo,r._k9_cx=K9L_ROAD_GEO,K9L_CX
    r._k9_exp1_cam=None
    sentinel=object()
    previous=r.__dict__.get("_k9l_draw_road",sentinel)
    r._k9l_draw_road=lambda current:draw_scene(r,current)
    try:
        r._k9l_draw_hud(state,signal_lights)
    finally:
        if previous is sentinel:
            del r.__dict__["_k9l_draw_road"]
        else:
            r._k9l_draw_road=previous
        r._k9l_car_rec=None
