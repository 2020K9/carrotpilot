"""Pinned original driving adaptation; geometry/assets and outer-HUD isolation."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest
from unittest.mock import patch

FILE=Path(__file__).resolve().parents[1]/'cluster'/'cluster_k9_video.py'
spec=importlib.util.spec_from_file_location('k9_video_tested',FILE)
hud=importlib.util.module_from_spec(spec);sys.modules[spec.name]=hud;spec.loader.exec_module(hud)
def point(f,lat): return NS(forward_m=f,lateral_m=lat)
def vehicle(f,lat,**kw): return NS(longitudinal_m=f,lateral_m=lat,**kw)
FAKE_RENDERER=NS(K9L_ROAD_GEO=(1,2,3,4),K9L_CX=972,K9_NAVDY_MID=12,
                 K9_NAVDY_EGO_BRAKE_PATH=Path('k9_navdy_ego_brake.png'),time=NS(monotonic=lambda:100.))

class GeometryTests(unittest.TestCase):
    def test_reject_nonfinite(self):
        for v in (None,math.nan,math.inf,-math.inf,'30',True,object()):
            self.assertIsNone(hud.finite(v));self.assertIsNone(hud.project(v,0));self.assertIsNone(hud.project(0,v))
    def test_projection_upstream_formula_true_metres(self):
        x,y,s=hud.project(0,0)
        self.assertAlmostEqual(x,hud.CENTER_X-hud.TARGET_X*s);self.assertAlmostEqual(y,hud.EGO_Y)
        self.assertAlmostEqual(s,hud.EGO_WIDTH/1.9)
        # Independent analytic setLookAt/perspective calculation from Java D camera.
        for f in (5,30,50,100):
            x,y,s=hud.project(f,3.5)
            pitch=math.radians(3.60)
            depth=23+3.10*math.sin(pitch)+f*math.cos(pitch)
            focal=240/math.tan(math.radians(10))
            self.assertAlmostEqual(s,focal/depth)
            self.assertAlmostEqual(x,972+(3.5+.24)*s)
            self.assertAlmostEqual(y,240-(-3.10*math.cos(pitch)+f*math.sin(pitch))*focal/depth)
            self.assertGreater(y,hud.HORIZON_Y)
        for f,l in ((-1,0),(141,0),(0,31)): self.assertIsNone(hud.project(f,l))
    def test_clip(self):
        self.assertEqual(hud.clip_segment((-10,5),(20,5),(0,0,10,10)),((0,5),(10,5)))
        self.assertIsNone(hud.clip_segment((0,math.nan),(3,2)))
        self.assertIsNone(hud.clip_segment((-10,-5),(20,-5),(0,0,10,10)))
    def test_gaps_not_bridged(self):
        pts=(point(0,0),point(10,0),point(20,math.nan),point(30,1),point(40,2))
        self.assertEqual(hud.ground_segments(pts),(((0,0),(10,0)),((30,1),(40,2))))
        self.assertEqual(hud.ground_segments(None),())
        self.assertEqual(hud.ground_segments(pts,math.inf),())
        self.assertEqual(hud.ground_segments((point(20,0),point(10,0))),())
        self.assertEqual(hud.ground_segments((point(0,0),point(60,0))),())
    def test_real_objects_only(self):
        st=NS(left_blindspot=True,right_blindspot=True)
        self.assertEqual(hud.detected_objects(st),())
        st.detected_vehicles=(vehicle(2,.2,label='L1',primary=True),vehicle(2.5,.3),vehicle(30,3.5),vehicle(2,3.5,source='carState BSD'),vehicle(-2,-3),vehicle(math.inf,0))
        self.assertEqual(hud.detected_objects(st),((30,3.5,False),(2,.2,True)))
    def test_asset_angles_and_anchors(self):
        meta=hud.sprite_metadata()
        self.assertEqual(len(meta),76)
        self.assertEqual(meta['navdy_ego'][3:6],(100,194,100))
        for f,l in ((0,0),(10,-3.5),(30,3.5),(140,25)):
            self.assertIn(hud.sprite_key(f,l),meta)
        for key in meta:
            self.assertTrue((hud.ASSETS/(key+'.png')).is_file())
            self.assertTrue((hud.ASSETS/(key+'_lamp.png')).is_file())
    def test_path_feather_and_measured_end(self):
        self.assertEqual(hud.path_strips(()),())
        strips=hud.path_strips((((0,0),(10,1)),((10,1),(20,2))),12)
        self.assertTrue(strips)
        for a,b,c,d,alpha,core in strips:
            self.assertLessEqual(c[0],20)
            self.assertTrue(all(0<=v<=1 for v in alpha));self.assertTrue(all(0<=v<=1 for v in core))
        self.assertLess(strips[0][4][0],strips[2][4][0])
        self.assertLessEqual(max(s[2][0] for s in strips),12.6)
    def test_source_aspect_uniform_fit_and_sprite_scale(self):
        self.assertEqual((hud.VIEW[2]-hud.VIEW[0])/(hud.VIEW[3]-hud.VIEW[1]),864/576)
        cam=hud.camera(.001)
        center=cam.screen((0.,0.,30.))
        x=cam.screen((1.,0.,30.));y=cam.screen((0.,1.,30.))
        self.assertAlmostEqual(x[0]-center[0],center[1]-y[1])
        self.assertAlmostEqual(x[0]-center[0],center[2])
        self.assertGreaterEqual(hud.VIEW[0],hud.SCENE[0])
        self.assertLessEqual(hud.VIEW[2],hud.SCENE[2])
    def test_original_camera_constants_and_lane_bands(self):
        self.assertEqual(hud.CAMERA_D,23.)
        self.assertEqual(hud.TARGET_Y,3.10)
        self.assertAlmostEqual(hud.PITCH,math.radians(3.60))
        bands=list(hud._distance_bands((((40,0),(90,0)),)))
        self.assertEqual([v[1] for v in bands],[1.,.52,.18])
        self.assertEqual([v[0][1][0] for v in bands],[44.,86.,90.])
    def test_vertex_gradient_not_segment_average(self):
        rl=RaylibDouble()
        hud._gradient_quad(rl,((0,0),(1,0),(1,1),(0,1)),(0.,1.,.5,0.),(0.,1.,1.,0.),(0,0,0),(100,100,100),185)
        colours=[args for name,args in rl.calls if name=='rl_color4ub']
        self.assertEqual(colours[0],(0,0,0,0))
        self.assertIn((100,100,100,185),colours)
    def test_observed_dash_only(self):
        solid=NS(model_points=(point(0,0),point(14,0)))
        self.assertEqual(len(hud.lane_segments(solid)),1)
        solid.style='dashed'
        self.assertEqual(hud.lane_segments(solid),(((0,0),(3.5,0)),((7,0),(10.5,0))))

class RaylibDouble:
    def __init__(self): self.calls=[]
    def Vector2(self,x,y): return NS(x=x,y=y)
    def Rectangle(self,x,y,width,height): return NS(x=x,y=y,width=width,height=height)
    def Color(self,*rgba): return rgba
    def __getattr__(self,name):
        def call(*args): self.calls.append((name,args))
        return call
class RendererDouble:
    def __init__(self): self.assets=[];self.outer=[];self.brakes=[]
    def _ego_brake_texture(self,path):
        self.assets.append(path);return NS(width=200,height=177)
    def _k9_navdy_ego_texture(self):
        self.assets.append(Path('k9_navdy_ego.png'));return NS(width=254,height=200)
    def _k9_navdy_sprites(self): return {12:(None,127.,100.,63.5,100.4,61.6)}
    def _draw_ego_brake_light(self,path,dst): self.brakes.append((path,dst))
    def _k9_turzx_quad(self,*args): pass
    def _k9l_draw_road(self,state): raise AssertionError('old road must not run')
    def _k9l_draw_hud(self,state,signals):
        self.outer.append((state,signals));self._k9l_draw_road(state)

class DrawTests(unittest.TestCase):
    def test_scene_no_outer_cards_no_fabricated_traffic(self):
        rl,r=RaylibDouble(),RendererDouble()
        with patch.dict(sys.modules,{'pyray':rl,'cluster_renderer':FAKE_RENDERER}): hud.draw_scene(r,NS())
        self.assertEqual([p.name for p in r.assets],['k9_navdy_ego.png'])
        self.assertEqual(sum(n=='draw_texture_pro' for n,a in rl.calls),1)
        self.assertFalse(any(n in ('begin_scissor_mode','end_scissor_mode','rl_push_matrix','rl_pop_matrix') for n,a in rl.calls))
    def test_outer_reused_and_callback_restored(self):
        rl,r,st=RaylibDouble(),RendererDouble(),NS()
        fake=FAKE_RENDERER
        with patch.dict(sys.modules,{'pyray':rl,'cluster_renderer':fake}): hud.draw(r,st,(True,False))
        self.assertEqual(r.outer,[(st,(True,False))]);self.assertNotIn('_k9l_draw_road',r.__dict__)
        self.assertEqual(r._k9_cx,972)
    def test_restore_on_failure(self):
        r=RendererDouble();old=lambda s:None;r._k9l_draw_road=old
        r._k9l_draw_hud=lambda *args:(_ for _ in ()).throw(RuntimeError('paint'))
        with patch.dict(sys.modules,{'cluster_renderer':NS(K9L_ROAD_GEO=(),K9L_CX=972)}):
            with self.assertRaises(RuntimeError): hud.draw(r,NS(),(False,False))
        self.assertIs(r._k9l_draw_road,old)
    def test_ego_shadow_has_ground_margin_without_moving_sprite(self):
        r,rl=RendererDouble(),RaylibDouble()
        with patch.dict(sys.modules,{'cluster_renderer':FAKE_RENDERER}): hud._ego(rl,r)
        _,ellipse=next(c for c in rl.calls if c[0]=='draw_ellipse')
        _,texture=next(c for c in rl.calls if c[0]=='draw_texture_pro')
        x,y,scale=hud.project(0,0)
        self.assertLessEqual(ellipse[1]+ellipse[3],y-6.0)
        _,w,h,ax,ay,ppm=r._k9_navdy_sprites()[12]
        self.assertAlmostEqual(texture[2].y+ay*scale/ppm,y)
    def test_label_keepout_uses_current_scene_not_stale_mode_one(self):
        r,rl=RendererDouble(),RaylibDouble();r._k9l_cars=[(-999,-999,999,999)]
        with patch.dict(sys.modules,{'pyray':rl,'cluster_renderer':FAKE_RENDERER}):
            hud.draw_scene(r,NS(detected_vehicles=(vehicle(22,.2),)))
        self.assertEqual(len(r._k9l_cars),1)
        _,args=next(c for c in rl.calls if c[0]=='draw_texture_pro')
        dst=args[2]
        self.assertEqual(r._k9l_cars[0],(dst.x,dst.y,dst.x+dst.width,dst.y+dst.height))
        with patch.dict(sys.modules,{'pyray':rl,'cluster_renderer':FAKE_RENDERER}): hud.draw_scene(r,NS())
        self.assertEqual(r._k9l_cars,[])
    def test_k9_ego_uses_existing_asset_colours_and_brake_helper(self):
        r,rl=RendererDouble(),RaylibDouble()
        with patch.dict(sys.modules,{'cluster_renderer':FAKE_RENDERER}): hud._ego(rl,r)
        self.assertEqual([p.name for p in r.assets],['k9_navdy_ego.png'])
        _,args=next(c for c in rl.calls if c[0]=='draw_texture_pro')
        self.assertEqual(args[-1],(255,255,255,255))
        self.assertEqual(r.brakes,[(FAKE_RENDERER.K9_NAVDY_EGO_BRAKE_PATH,args[2])])
    def test_missing_k9_never_falls_back_to_imported_ego(self):
        r,rl=RendererDouble(),RaylibDouble();r._k9_navdy_ego_texture=lambda:None
        with patch.dict(sys.modules,{'cluster_renderer':FAKE_RENDERER}): hud._ego(rl,r)
        self.assertEqual(rl.calls,[])
    def test_nonfinite_scene_and_bounded_collections(self):
        bad=(point(0,0),point(3,0),point(6,math.nan),point(9,0),point(12,0))
        state=NS(model_path=bad,left_road_edge_points=bad,right_road_edge_points=bad,
                 lanes=(NS(model_points=bad),),
                 detected_vehicles=(vehicle(math.nan,0),vehicle(5,math.inf)))
        r,rl=RendererDouble(),RaylibDouble()
        with patch.dict(sys.modules,{'pyray':rl,'cluster_renderer':FAKE_RENDERER}): hud.draw_scene(r,state)
        self.assertEqual([p.name for p in r.assets],['k9_navdy_ego.png'])
        for name,args in rl.calls:
            if name=='draw_line_ex':
                self.assertTrue(all(math.isfinite(v) for p in args[:2] for v in (p.x,p.y)))
        self.assertEqual(len(hud.ground_segments(point(i/10,0) for i in range(10000))),255)
        self.assertLessEqual(len(hud.detected_objects(NS(detected_vehicles=(vehicle(1+i,0) for i in range(10000))))),128)
    def test_sprite_measured_position_not_shifted(self):
        r,rl=RendererDouble(),RaylibDouble()
        hud._car(rl,r,3,.2,lead=True)
        name,args=next(c for c in rl.calls if c[0]=='draw_texture_pro')
        dst=args[2];key=r.assets[0].stem;_,w,h,ax,ay,ppm=hud.sprite_metadata()[key]
        wx,wz,th=hud.traffic_pose(3,.2)
        x,y,s=hud.project_world(wx-2.10*math.sin(th),0.,wz+2.10*math.cos(th))
        self.assertAlmostEqual(dst.x+ax*s/ppm,x);self.assertAlmostEqual(dst.y+ay*s/ppm,y)
class SourceTemporalTests(unittest.TestCase):
    def test_source_exponential_smoothing_and_clock_bounds(self):
        old=hud.Motion(10.,20.,0.,0.,10.,0.)
        st=NS(speed_kph=40.,steering_angle_deg=50.,planned_curvature_m_inv=.002)
        out=hud.advance_motion(old,st,((30.,1.,True),),10.05)
        fast,med=1-math.exp(-.05*14),1-math.exp(-.05*9)
        self.assertAlmostEqual(out.speed,20+20*fast)
        self.assertAlmostEqual(out.curve,.002*med)
        self.assertAlmostEqual(out.lead,10+20*med)
        self.assertAlmostEqual(out.phase,out.speed/3.6*.05)
        self.assertEqual(hud.advance_motion(out,st,((30.,1.,True),),10.05),out)
        fresh=hud.advance_motion(out,st,(),20.)
        self.assertEqual((fresh.speed,fresh.lead,fresh.phase),(40.,0.,0.))
    def test_chase_camera_and_orbit_formula(self):
        cam=hud.camera(.001)
        self.assertLess(cam.eye[0],-.24)
        self.assertAlmostEqual(cam.eye[0],-.24+23*math.cos(math.radians(3.6))*math.sin(math.radians(-4.2)))
        orbit=hud.camera(0.,35.,18.,1.2)
        self.assertAlmostEqual(orbit.focal,240/math.tan(math.radians(16)))
        self.assertNotEqual(orbit.eye,cam.eye)
        self.assertEqual(hud.effective_curve(hud.Motion(0,0,100,0,0,0)),-.0025)
    def test_world_quad_near_plane_clipped_finite(self):
        class R(RendererDouble):
            def __init__(self):super().__init__();self.quads=[]
            def _k9_turzx_quad(self,*args):self.quads.append(args)
        r,rl=R(),RaylibDouble()
        hud._world_quad(rl,r,((-8,0,48),(8,0,48),(8,0,0),(-8,0,0)),(255,255,255,16))
        self.assertTrue(r.quads)
        self.assertTrue(all(math.isfinite(v) for q in r.quads for p in q[:-1] for v in (p.x,p.y)))
    def test_original_traffic_center_compression_and_rear_anchor(self):
        self.assertEqual(hud.traffic_pose(50,0),(0.,-46.2,0.))
        wx,wz,th=hud.traffic_pose(30,3.5,.001)
        self.assertAlmostEqual(th,.0322)
        self.assertAlmostEqual(wx,.5*.001*32.2**2+3.5*math.cos(th))
    def test_faceted_fallback_and_brake_mask(self):
        class R(RendererDouble):
            def __init__(self):super().__init__();self.quads=[]
            def _k9_turzx_quad(self,*args):self.quads.append(args)
        r,rl=R(),RaylibDouble()
        hud._car(rl,r,1,25)
        self.assertTrue(r.quads);self.assertEqual(r.assets,[])
        r,rl=R(),RaylibDouble()
        hud._car(rl,r,22,0,True,lamp=2)
        self.assertTrue(any(p.name.endswith('_lamp.png') for p in r.assets))
        self.assertEqual(sum(n=='draw_texture_pro' for n,_ in rl.calls),4)
        self.assertEqual(sum(n=='end_blend_mode' for n,_ in rl.calls),1)
    def test_decorative_rear_requires_valid_near_measurement(self):
        self.assertEqual(hud._rear_continuation((),.001,-24),())
        late=(((3,0),(6,0)),)
        self.assertEqual(hud._rear_continuation(late,.001,-24),late)
        near=(((0,0),(3,0)),)
        extended=hud._rear_continuation(near,.001,-24)
        self.assertEqual(extended[0][0][0],-24)
        self.assertEqual(extended[-1],near[0])
        self.assertEqual(hud.path_strips(extended)[0][4][0],0.)
    def test_dash_phase_and_camera_restoration(self):
        lane=NS(model_points=(point(0,0),point(14,0)),style='dashed')
        self.assertNotEqual(hud.lane_segments(lane,0),hud.lane_segments(lane,1))
        before=hud._CAMERA.get();r,rl=RendererDouble(),RaylibDouble()
        with patch.dict(sys.modules,{'pyray':rl,'cluster_renderer':FAKE_RENDERER}):
            hud.draw_scene(r,NS(planned_curvature_m_inv=.001))
        self.assertIs(hud._CAMERA.get(),before)

if __name__=='__main__': unittest.main()
